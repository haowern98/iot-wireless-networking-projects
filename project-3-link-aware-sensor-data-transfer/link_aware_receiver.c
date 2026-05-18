/*
 * CS4222/5422: Assignment 3b - Task 2
 * Node B: receiver and backhaul collector
 *
 * Print requirement preserved:
 *  - After successful full data reception:
 *      Light: Reading1, Reading2, ... , Reading60
 *      Motion: Reading1, Reading2, ... , Reading60
 */

#include "contiki.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include "net/packetbuf.h"
#include "net/linkaddr.h"
#include "node-id.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

/*---------------------------------------------------------------------------*/
#define NUM_READINGS            60
#define CHUNK_SIZE              3
#define NUM_CHUNKS              (NUM_READINGS / CHUNK_SIZE)

#define CONTACT_LOST_TIMEOUT    (CLOCK_SECOND * 3)
#define TRANSFER_IDLE_TIMEOUT   (CLOCK_SECOND * 8)
#define SESSION_SIGNAL_INTERVAL (CLOCK_SECOND / 10)

#define SLOT_TIME               (RTIMER_SECOND / 20)  /* 50 ms */
#define PRIME_P                 11
#define PRIME_Q                 17

/*---------------------------------------------------------------------------*/
#define PKT_PROBE           0x01
#define PKT_REPLY           0x02
#define PKT_TRANSFER_START  0x03
#define PKT_DATA            0x04
#define PKT_ACK             0x05
#define PKT_TRANSFER_READY  0x06
#define PKT_TRANSFER_END    0x07
#define PKT_TRANSFER_ABORT  0x08

typedef struct __attribute__((packed)) {
  uint8_t  pkt_type;
  uint16_t src_id;
  uint16_t seq;
} probe_pkt_t;

typedef struct __attribute__((packed)) {
  uint8_t  pkt_type;
  uint16_t src_id;
  uint16_t reply_to_seq;
} reply_pkt_t;

typedef struct __attribute__((packed)) {
  uint8_t  pkt_type;
  uint16_t src_id;
  uint16_t session_id;
  uint8_t  start_chunk;
  uint8_t  total_chunks;
} transfer_start_pkt_t;

typedef struct __attribute__((packed)) {
  uint8_t  pkt_type;
  uint16_t src_id;
  uint16_t session_id;
  uint8_t  chunk_id;
  uint8_t  total_chunks;
  uint16_t light[CHUNK_SIZE];
  uint16_t motion[CHUNK_SIZE];
} data_pkt_t;

typedef struct __attribute__((packed)) {
  uint8_t  pkt_type;
  uint16_t src_id;
  uint16_t session_id;
  uint8_t  chunk_id;
} ack_pkt_t;

typedef struct __attribute__((packed)) {
  uint8_t  pkt_type;
  uint16_t src_id;
  uint16_t session_id;
} transfer_end_pkt_t;

typedef struct __attribute__((packed)) {
  uint8_t  pkt_type;
  uint16_t src_id;
  uint16_t session_id;
  uint8_t  start_chunk;
} transfer_ready_pkt_t;

typedef struct __attribute__((packed)) {
  uint8_t  pkt_type;
  uint16_t src_id;
  uint16_t session_id;
} transfer_abort_pkt_t;

/*---------------------------------------------------------------------------*/
static uint16_t light_readings[NUM_READINGS];
static uint16_t motion_readings[NUM_READINGS];
static uint8_t chunk_received[NUM_CHUNKS];

static linkaddr_t broadcast_addr;
static uint16_t peer_id = 0;
static uint16_t current_session_id = 0;
static uint8_t peer_present = 0;
static uint8_t transfer_active = 0;
static uint8_t all_data_printed = 0;
static uint8_t pending_reply = 0;
static uint16_t pending_reply_seq = 0;
static uint8_t pending_transfer_ready = 0;
static uint8_t pending_transfer_start_chunk = 0;
static uint8_t pending_ack = 0;
static uint8_t pending_ack_chunk = 0;
static uint8_t waiting_for_first_chunk = 0;
static uint8_t last_acked_chunk_valid = 0;
static uint8_t last_acked_chunk = 0;

static clock_time_t last_probe_time = 0;
static clock_time_t last_transfer_activity = 0;
static clock_time_t last_session_signal = 0;

static struct rtimer discovery_rt;
static struct pt discovery_pt;
static unsigned long discovery_slot_number = 0;

static reply_pkt_t reply_pkt;
static ack_pkt_t ack_pkt;
static transfer_ready_pkt_t transfer_ready_pkt;

PROCESS(node_b_process, "Task 2 Node B");
AUTOSTART_PROCESSES(&node_b_process);

/*---------------------------------------------------------------------------*/
static unsigned long
seconds_now(void)
{
  return (unsigned long)(clock_time() / CLOCK_SECOND);
}

/*---------------------------------------------------------------------------*/
static void
clear_transfer_session(void)
{
  transfer_active = 0;
  pending_transfer_ready = 0;
  pending_ack = 0;
  waiting_for_first_chunk = 0;
  last_acked_chunk_valid = 0;
  current_session_id = 0;
  last_session_signal = 0;
}

/*---------------------------------------------------------------------------*/
static uint8_t
all_chunks_received(void)
{
  uint8_t i;

  for(i = 0; i < NUM_CHUNKS; i++) {
    if(!chunk_received[i]) {
      return 0;
    }
  }
  return 1;
}

/*---------------------------------------------------------------------------*/
static void
send_reply(uint16_t reply_to_seq)
{
  reply_pkt.pkt_type = PKT_REPLY;
  reply_pkt.src_id = (uint16_t)node_id;
  reply_pkt.reply_to_seq = reply_to_seq;

  nullnet_buf = (uint8_t *)&reply_pkt;
  nullnet_len = sizeof(reply_pkt);
  NETSTACK_NETWORK.output(&broadcast_addr);
}

/*---------------------------------------------------------------------------*/
static void
send_ack(uint8_t chunk_id)
{
  ack_pkt.pkt_type = PKT_ACK;
  ack_pkt.src_id = (uint16_t)node_id;
  ack_pkt.session_id = current_session_id;
  ack_pkt.chunk_id = chunk_id;

  nullnet_buf = (uint8_t *)&ack_pkt;
  nullnet_len = sizeof(ack_pkt);
  NETSTACK_NETWORK.output(&broadcast_addr);
  printf("TX ACK chunk %u\n", (unsigned)chunk_id);
}

/*---------------------------------------------------------------------------*/
static void
send_transfer_ready(void)
{
  transfer_ready_pkt.pkt_type = PKT_TRANSFER_READY;
  transfer_ready_pkt.src_id = (uint16_t)node_id;
  transfer_ready_pkt.session_id = current_session_id;
  transfer_ready_pkt.start_chunk = pending_transfer_start_chunk;

  nullnet_buf = (uint8_t *)&transfer_ready_pkt;
  nullnet_len = sizeof(transfer_ready_pkt);
  NETSTACK_NETWORK.output(&broadcast_addr);
}

/*---------------------------------------------------------------------------*/
static char
discovery_scheduler(struct rtimer *t, void *ptr)
{
  PT_BEGIN(&discovery_pt);

  while(1) {
    if(transfer_active || pending_transfer_ready || pending_ack) {
      NETSTACK_RADIO.on();
      rtimer_set(t, RTIMER_TIME(t) + SLOT_TIME, 1,
                 (rtimer_callback_t)discovery_scheduler, ptr);
      PT_YIELD(&discovery_pt);
      process_poll(&node_b_process);
      continue;
    }

    {
      uint8_t wake_now =
        ((discovery_slot_number % PRIME_P) == 0) ||
        ((discovery_slot_number % PRIME_Q) == 0);

      if(wake_now) {
        NETSTACK_RADIO.on();
        process_poll(&node_b_process);

        rtimer_set(t, RTIMER_TIME(t) + SLOT_TIME, 1,
                   (rtimer_callback_t)discovery_scheduler, ptr);
        PT_YIELD(&discovery_pt);

        if(!(transfer_active || pending_transfer_ready || pending_ack)) {
          NETSTACK_RADIO.off();
        }
      } else {
        NETSTACK_RADIO.off();
        rtimer_set(t, RTIMER_TIME(t) + SLOT_TIME, 1,
                   (rtimer_callback_t)discovery_scheduler, ptr);
        PT_YIELD(&discovery_pt);
      }

      discovery_slot_number++;
      process_poll(&node_b_process);
    }
  }

  PT_END(&discovery_pt);
}

/*---------------------------------------------------------------------------*/
static void
print_all_readings(void)
{
  uint8_t i;

  printf("Light: ");
  for(i = 0; i < NUM_READINGS; i++) {
    printf("%u", (unsigned)light_readings[i]);
    if(i + 1 < NUM_READINGS) {
      printf(", ");
    }
  }
  printf("\n");

  printf("Motion: ");
  for(i = 0; i < NUM_READINGS; i++) {
    printf("%u", (unsigned)motion_readings[i]);
    if(i + 1 < NUM_READINGS) {
      printf(", ");
    }
  }
  printf("\n");
}

/*---------------------------------------------------------------------------*/
static void
handle_probe(const probe_pkt_t *probe)
{
  if(peer_id == 0) {
    peer_id = probe->src_id;
  } else if(probe->src_id != peer_id) {
    return;
  }

  if(transfer_active) {
    last_probe_time = clock_time();
    pending_reply = 1;
    pending_reply_seq = probe->seq;
    process_poll(&node_b_process);
    return;
  }

  if(!peer_present) {
    peer_present = 1;
    printf("%lu DETECT %u\n", seconds_now(), (unsigned)peer_id);
  }

  last_probe_time = clock_time();
  pending_reply = 1;
  pending_reply_seq = probe->seq;
  process_poll(&node_b_process);
}

/*---------------------------------------------------------------------------*/
static void
handle_data(const data_pkt_t *pkt)
{
  uint8_t chunk_id;
  uint8_t base;

  if(pkt->src_id != peer_id ||
     pkt->session_id != current_session_id ||
     pkt->total_chunks != NUM_CHUNKS) {
    return;
  }

  chunk_id = pkt->chunk_id;
  if(chunk_id >= NUM_CHUNKS) {
    return;
  }

  waiting_for_first_chunk = 0;
  pending_transfer_ready = 0;
  base = chunk_id * CHUNK_SIZE;

  if(!chunk_received[chunk_id]) {
    memcpy(&light_readings[base], pkt->light, CHUNK_SIZE * sizeof(uint16_t));
    memcpy(&motion_readings[base], pkt->motion, CHUNK_SIZE * sizeof(uint16_t));
    chunk_received[chunk_id] = 1;
    printf("RX chunk %u/%u\n", (unsigned)(chunk_id + 1), (unsigned)NUM_CHUNKS);
  } else {
    printf("Duplicate chunk %u, re-ACKing\n", (unsigned)chunk_id);
  }

  transfer_active = 1;
  last_transfer_activity = clock_time();
  pending_ack = 1;
  pending_ack_chunk = chunk_id;
  last_acked_chunk_valid = 1;
  last_acked_chunk = chunk_id;
  process_poll(&node_b_process);

  if(!all_data_printed && all_chunks_received()) {
    all_data_printed = 1;
    print_all_readings();
  }
}

/*---------------------------------------------------------------------------*/
static void
input_callback(const void *data, uint16_t len,
               const linkaddr_t *src, const linkaddr_t *dest)
{
  uint8_t pkt_type;

  (void)src;
  (void)dest;

  if(len == 0) {
    return;
  }

  pkt_type = ((const uint8_t *)data)[0];

  if(pkt_type == PKT_PROBE && len == sizeof(probe_pkt_t)) {
    probe_pkt_t probe;
    memcpy(&probe, data, sizeof(probe));
    if(probe.src_id == (uint16_t)node_id) {
      return;
    }
    handle_probe(&probe);
  } else if(pkt_type == PKT_TRANSFER_START && len == sizeof(transfer_start_pkt_t)) {
    transfer_start_pkt_t start_pkt;
    memcpy(&start_pkt, data, sizeof(start_pkt));
    if(peer_id == 0) {
      peer_id = start_pkt.src_id;
    }
    if(start_pkt.src_id != peer_id || start_pkt.total_chunks != NUM_CHUNKS) {
      return;
    }
    peer_present = 1;
    transfer_active = 1;
    current_session_id = start_pkt.session_id;
    pending_transfer_start_chunk = start_pkt.start_chunk;
    waiting_for_first_chunk = 1;
    last_acked_chunk_valid = 0;
    last_session_signal = 0;
    last_transfer_activity = clock_time();
    pending_transfer_ready = 1;
    process_poll(&node_b_process);
  } else if(pkt_type == PKT_DATA && len == sizeof(data_pkt_t)) {
    data_pkt_t data_pkt;
    memcpy(&data_pkt, data, sizeof(data_pkt));
    handle_data(&data_pkt);
  } else if(pkt_type == PKT_TRANSFER_END && len == sizeof(transfer_end_pkt_t)) {
    transfer_end_pkt_t end_pkt;
    memcpy(&end_pkt, data, sizeof(end_pkt));
    if(end_pkt.src_id == peer_id && end_pkt.session_id == current_session_id) {
      clear_transfer_session();
      peer_present = 0;
      printf("Node B: back in slot discovery\n");
      process_poll(&node_b_process);
    }
  } else if(pkt_type == PKT_TRANSFER_ABORT && len == sizeof(transfer_abort_pkt_t)) {
    transfer_abort_pkt_t abort_pkt;
    memcpy(&abort_pkt, data, sizeof(abort_pkt));
    if(abort_pkt.src_id == peer_id && abort_pkt.session_id == current_session_id) {
      clear_transfer_session();
      peer_present = 0;
      printf("Node B: transfer aborted, back in slot discovery\n");
      process_poll(&node_b_process);
    }
  }
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(node_b_process, ev, data)
{
  static struct etimer timer;

  PROCESS_BEGIN();

  memset(chunk_received, 0, sizeof(chunk_received));
  linkaddr_copy(&broadcast_addr, &linkaddr_null);
  nullnet_set_input_callback(input_callback);

  printf("Node B: ready for discovery and transfer\n");
  printf("Node B: slot-based discovery | slot=%lu ms primes=%u,%u\n",
         (1000UL * SLOT_TIME) / RTIMER_SECOND,
         (unsigned)PRIME_P,
         (unsigned)PRIME_Q);

  rtimer_set(&discovery_rt, RTIMER_NOW() + (RTIMER_SECOND / 1000), 1,
             (rtimer_callback_t)discovery_scheduler, NULL);

  etimer_set(&timer, SESSION_SIGNAL_INTERVAL);
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer) || ev == PROCESS_EVENT_POLL);

    if(pending_reply) {
      send_reply(pending_reply_seq);
      pending_reply = 0;
    }

    if(pending_transfer_ready) {
      send_transfer_ready();
      pending_transfer_ready = 0;
      last_session_signal = clock_time();
    }

    if(pending_ack) {
      send_ack(pending_ack_chunk);
      pending_ack = 0;
      last_transfer_activity = clock_time();
      last_session_signal = clock_time();
    }

    if(transfer_active && waiting_for_first_chunk &&
       (clock_time() - last_session_signal) >= SESSION_SIGNAL_INTERVAL) {
      send_transfer_ready();
      last_session_signal = clock_time();
    }

    if(transfer_active && !waiting_for_first_chunk &&
       last_acked_chunk_valid &&
       !pending_ack &&
       (clock_time() - last_session_signal) >= SESSION_SIGNAL_INTERVAL) {
      send_ack(last_acked_chunk);
      last_session_signal = clock_time();
    }

    if(peer_present &&
       !transfer_active &&
       (clock_time() - last_probe_time) > CONTACT_LOST_TIMEOUT) {
      peer_present = 0;
    }

    if(transfer_active &&
       (clock_time() - last_transfer_activity) > TRANSFER_IDLE_TIMEOUT) {
      clear_transfer_session();
      peer_present = 0;
      printf("Node B: transfer timed out, back in slot discovery\n");
    }

    etimer_reset(&timer);
  }

  PROCESS_END();
}