/*
 * CS4222/5422: Assignment 3b - Task 2
 * Node A: sensing node and transfer initiator
 *
 * Two-run metric comparison:
 *  - Run 1: RSSI-only   -> set LINK_METRIC_MODE to LINK_METRIC_RSSI
 *  - Run 2: PRR-only    -> set LINK_METRIC_MODE to LINK_METRIC_PRR
 *
 * Task 2 print requirements preserved:
 *  - <timestamp> DETECT <nodeID>
 *  - <timestamp> TRANSFER <nodeID> RSSI:<value>   OR   PRR:<value>
 */

#include "contiki.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include "net/packetbuf.h"
#include "net/linkaddr.h"
#include "node-id.h"
#include "board-peripherals.h"

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/*---------------------------------------------------------------------------*/
#define NUM_READINGS                60
#define CHUNK_SIZE                  3
#define NUM_CHUNKS                  (NUM_READINGS / CHUNK_SIZE)

#define SENSOR_SAMPLE_PERIOD        CLOCK_SECOND
#define FIRST_LIGHT_WAIT            CLOCK_SECOND
#define LIGHT_READY_TIME            (CLOCK_SECOND / 4)

#define CONTACT_LOST_TIMEOUT        (CLOCK_SECOND * 3)
#define RECOVERY_COOLDOWN           CLOCK_SECOND
#define REDISCOVERY_STATUS_INTERVAL (CLOCK_SECOND * 5)

#define LINK_EVAL_PROBES            8

/* Thresholds for experiments: tune */
#define RSSI_THRESHOLD_DBM          (-75)
#define PRR_THRESHOLD_PERCENT       12

#define ACK_TIMEOUT                 CLOCK_SECOND
#define ACK_RETRY_LIMIT             12
#define TRANSFER_START_INTERVAL     (CLOCK_SECOND / 10)
#define TRANSFER_START_RETRY_LIMIT  20

#define SLOT_TIME                   (RTIMER_SECOND / 20)  /* 50 ms */
#define PRIME_P                     11
#define PRIME_Q                     17

/* Link metric mode selection for comparison purposes*/
#define LINK_METRIC_RSSI            1
#define LINK_METRIC_PRR             2

/* Link metric chosen is RSSI*/
#define LINK_METRIC_MODE            LINK_METRIC_RSSI

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
typedef enum {
  STATE_SENSING = 0,
  STATE_DISCOVERY,
  STATE_LINK_EVAL,
  STATE_TRANSFER,
  STATE_DONE
} node_a_state_t;

/*---------------------------------------------------------------------------*/
static uint16_t light_readings[NUM_READINGS];
static uint16_t motion_readings[NUM_READINGS];
static uint8_t chunk_sent[NUM_CHUNKS];

static linkaddr_t broadcast_addr;
static node_a_state_t state = STATE_SENSING;

static uint16_t peer_id = 0;
static uint16_t probe_seq = 0;
static uint16_t session_id = 0;
static uint8_t current_chunk = 0;

static uint8_t contact_active = 0;
static uint8_t chunk_acked = 0;
static uint8_t transfer_started = 0;
static uint8_t transfer_complete = 0;
static uint8_t transfer_ready = 0;
static uint8_t retry_count = 0;

static clock_time_t last_reply_time = 0;
static clock_time_t recovery_block_until = 0;

static uint8_t probes_sent_window = 0;
static uint8_t replies_received_window = 0;
static int16_t rssi_sum_window = 0;
static uint8_t rssi_count_window = 0;
static int8_t avg_rssi_dbm = -128;
static uint8_t prr_percent = 0;

static struct rtimer discovery_rt;
static struct pt discovery_pt;
static unsigned long discovery_slot_number = 0;

static probe_pkt_t probe_pkt;
static transfer_start_pkt_t transfer_start_pkt;
static data_pkt_t data_pkt;
static transfer_ready_pkt_t transfer_ready_pkt;
static transfer_end_pkt_t transfer_end_pkt;
static transfer_abort_pkt_t transfer_abort_pkt;

PROCESS(node_a_process, "Task 2 Node A");
AUTOSTART_PROCESSES(&node_a_process);

/*---------------------------------------------------------------------------*/
static unsigned long
seconds_now(void)
{
  return (unsigned long)(clock_time() / CLOCK_SECOND);
}

/*---------------------------------------------------------------------------*/
static void
reset_transfer_state(void)
{
  transfer_started = 0;
  transfer_ready = 0;
  chunk_acked = 0;
  retry_count = 0;
}

/*---------------------------------------------------------------------------*/
static uint8_t
recovery_block_active(void)
{
  return (recovery_block_until != 0) && (clock_time() < recovery_block_until);
}

/*---------------------------------------------------------------------------*/
static void
reset_link_window(void)
{
  probes_sent_window = 0;
  replies_received_window = 0;
  rssi_sum_window = 0;
  rssi_count_window = 0;
  avg_rssi_dbm = -128;
  prr_percent = 0;
}

static uint8_t
link_quality_good(void)
{
#if LINK_METRIC_MODE == LINK_METRIC_RSSI
  return (rssi_count_window > 0 && avg_rssi_dbm >= RSSI_THRESHOLD_DBM);
#elif LINK_METRIC_MODE == LINK_METRIC_PRR
  return (probes_sent_window > 0 && prr_percent >= PRR_THRESHOLD_PERCENT);
#else
  return 0;
#endif
}

/*---------------------------------------------------------------------------*/
static void
print_transfer_ready_line(void)
{
#if LINK_METRIC_MODE == LINK_METRIC_RSSI
  printf("%lu TRANSFER %u RSSI:%d\n",
         seconds_now(),
         (unsigned)peer_id,
         (int)avg_rssi_dbm);
#elif LINK_METRIC_MODE == LINK_METRIC_PRR
  printf("%lu TRANSFER %u PRR:%u\n",
         seconds_now(),
         (unsigned)peer_id,
         (unsigned)prr_percent);
#endif
}

/*---------------------------------------------------------------------------*/
static void
print_threshold_not_reached_line(void)
{
#if LINK_METRIC_MODE == LINK_METRIC_RSSI
  printf("%lu THRESHOLD_NOT_REACHED %u RSSI:%d\n",
         seconds_now(),
         (unsigned)peer_id,
         (int)avg_rssi_dbm);
#elif LINK_METRIC_MODE == LINK_METRIC_PRR
  printf("%lu THRESHOLD_NOT_REACHED %u PRR:%u\n",
         seconds_now(),
         (unsigned)peer_id,
         (unsigned)prr_percent);
#endif
}

/*---------------------------------------------------------------------------*/
static uint8_t
all_chunks_sent(void)
{
  uint8_t i;

  for(i = 0; i < NUM_CHUNKS; i++) {
    if(!chunk_sent[i]) {
      return 0;
    }
  }
  return 1;
}

/*---------------------------------------------------------------------------*/
static uint8_t
first_unsent_chunk(void)
{
  uint8_t i;

  for(i = 0; i < NUM_CHUNKS; i++) {
    if(!chunk_sent[i]) {
      return i;
    }
  }
  return NUM_CHUNKS;
}

/*---------------------------------------------------------------------------*/
static void
send_probe(void)
{
  probe_seq++;
  probe_pkt.pkt_type = PKT_PROBE;
  probe_pkt.src_id = (uint16_t)node_id;
  probe_pkt.seq = probe_seq;

  if(state == STATE_LINK_EVAL && probes_sent_window < 255) {
    probes_sent_window++;
  }

  nullnet_buf = (uint8_t *)&probe_pkt;
  nullnet_len = sizeof(probe_pkt);
  NETSTACK_NETWORK.output(&broadcast_addr);
}

/*---------------------------------------------------------------------------*/
static void
send_transfer_start(void)
{
  uint8_t start_chunk = first_unsent_chunk();

  transfer_start_pkt.pkt_type = PKT_TRANSFER_START;
  transfer_start_pkt.src_id = (uint16_t)node_id;
  transfer_start_pkt.session_id = session_id;
  transfer_start_pkt.start_chunk = start_chunk;
  transfer_start_pkt.total_chunks = NUM_CHUNKS;

  nullnet_buf = (uint8_t *)&transfer_start_pkt;
  nullnet_len = sizeof(transfer_start_pkt);
  NETSTACK_NETWORK.output(&broadcast_addr);
}

/*---------------------------------------------------------------------------*/
static void
send_chunk(uint8_t chunk_id)
{
  uint8_t base = chunk_id * CHUNK_SIZE;

  data_pkt.pkt_type = PKT_DATA;
  data_pkt.src_id = (uint16_t)node_id;
  data_pkt.session_id = session_id;
  data_pkt.chunk_id = chunk_id;
  data_pkt.total_chunks = NUM_CHUNKS;
  memcpy(data_pkt.light, &light_readings[base], CHUNK_SIZE * sizeof(uint16_t));
  memcpy(data_pkt.motion, &motion_readings[base], CHUNK_SIZE * sizeof(uint16_t));

  nullnet_buf = (uint8_t *)&data_pkt;
  nullnet_len = sizeof(data_pkt);
  NETSTACK_NETWORK.output(&broadcast_addr);

  printf("TX chunk %u/%u\n", (unsigned)(chunk_id + 1), (unsigned)NUM_CHUNKS);
}

/*---------------------------------------------------------------------------*/
static void
send_transfer_end(void)
{
  transfer_end_pkt.pkt_type = PKT_TRANSFER_END;
  transfer_end_pkt.src_id = (uint16_t)node_id;
  transfer_end_pkt.session_id = session_id;

  nullnet_buf = (uint8_t *)&transfer_end_pkt;
  nullnet_len = sizeof(transfer_end_pkt);
  NETSTACK_NETWORK.output(&broadcast_addr);
}

/*---------------------------------------------------------------------------*/
static void
send_transfer_abort(void)
{
  transfer_abort_pkt.pkt_type = PKT_TRANSFER_ABORT;
  transfer_abort_pkt.src_id = (uint16_t)node_id;
  transfer_abort_pkt.session_id = session_id;

  nullnet_buf = (uint8_t *)&transfer_abort_pkt;
  nullnet_len = sizeof(transfer_abort_pkt);
  NETSTACK_NETWORK.output(&broadcast_addr);
}

/*---------------------------------------------------------------------------*/
static void
send_transfer_abort_burst(void)
{
  uint8_t i;

  for(i = 0; i < 3; i++) {
    send_transfer_abort();
  }
}

/*---------------------------------------------------------------------------*/
static char
discovery_scheduler(struct rtimer *t, void *ptr)
{
  PT_BEGIN(&discovery_pt);

  while(1) {
    if(state == STATE_DISCOVERY || state == STATE_LINK_EVAL) {
      uint8_t wake_now =
        ((discovery_slot_number % PRIME_P) == 0) ||
        ((discovery_slot_number % PRIME_Q) == 0);

      if(wake_now) {
        NETSTACK_RADIO.on();
        send_probe();
        process_poll(&node_a_process);

        rtimer_set(t, RTIMER_TIME(t) + SLOT_TIME, 1,
                   (rtimer_callback_t)discovery_scheduler, ptr);
        PT_YIELD(&discovery_pt);

        if(state == STATE_DISCOVERY || state == STATE_LINK_EVAL) {
          send_probe();
        }
        NETSTACK_RADIO.off();
      } else {
        NETSTACK_RADIO.off();
        rtimer_set(t, RTIMER_TIME(t) + SLOT_TIME, 1,
                   (rtimer_callback_t)discovery_scheduler, ptr);
        PT_YIELD(&discovery_pt);
      }

      discovery_slot_number++;
      process_poll(&node_a_process);
    } else {
      rtimer_set(t, RTIMER_TIME(t) + SLOT_TIME, 1,
                 (rtimer_callback_t)discovery_scheduler, ptr);
      PT_YIELD(&discovery_pt);
    }
  }

  PT_END(&discovery_pt);
}

/*---------------------------------------------------------------------------*/
static uint16_t
read_motion_value(void)
{
  int values[6];
  uint8_t i;
  uint32_t motion_sum = 0;
  uint8_t valid_count = 0;

  values[0] = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_X);
  values[1] = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Y);
  values[2] = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Z);
  values[3] = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
  values[4] = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
  values[5] = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);

  for(i = 0; i < 6; i++) {
    if(values[i] != CC26XX_SENSOR_READING_ERROR) {
      motion_sum += (uint32_t)abs(values[i]);
      valid_count++;
    }
  }

  if(valid_count == 0) {
    return 0;
  }

  if(motion_sum > UINT16_MAX) {
    motion_sum = UINT16_MAX;
  }

  return (uint16_t)motion_sum;
}

/*---------------------------------------------------------------------------*/
static void
handle_reply(const reply_pkt_t *reply)
{
  int8_t rssi = (int8_t)packetbuf_attr(PACKETBUF_ATTR_RSSI);

  if(peer_id == 0) {
    peer_id = reply->src_id;
  } else if(reply->src_id != peer_id) {
    return;
  }

  if(recovery_block_active()) {
    return;
  }

  if(!contact_active) {
    contact_active = 1;
    printf("%lu DETECT %u\n", seconds_now(), (unsigned)peer_id);

    state = STATE_LINK_EVAL;
    reset_link_window();

    /* Count this received reply as the first successful sample in the window */
    probes_sent_window = 1;
    replies_received_window = 1;
    rssi_sum_window = rssi;
    rssi_count_window = 1;
  } else if(state == STATE_LINK_EVAL) {
    if(replies_received_window < 255) {
      replies_received_window++;
    }
    rssi_sum_window += rssi;
    if(rssi_count_window < 255) {
      rssi_count_window++;
    }
  }

  last_reply_time = clock_time();
  process_poll(&node_a_process);
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

  if(pkt_type == PKT_REPLY && len == sizeof(reply_pkt_t)) {
    reply_pkt_t reply;
    memcpy(&reply, data, sizeof(reply));
    if(reply.src_id == (uint16_t)node_id) {
      return;
    }
    handle_reply(&reply);
  } else if(pkt_type == PKT_ACK && len == sizeof(ack_pkt_t)) {
    ack_pkt_t ack;
    memcpy(&ack, data, sizeof(ack));

    if(ack.src_id != peer_id) {
      return;
    }

    if(state == STATE_TRANSFER &&
       ack.session_id == session_id &&
       ack.chunk_id == current_chunk) {
      printf("ACK chunk %u\n", (unsigned)ack.chunk_id);
      chunk_acked = 1;
      last_reply_time = clock_time();
      process_poll(&node_a_process);
    }
  } else if(pkt_type == PKT_TRANSFER_READY && len == sizeof(transfer_ready_pkt_t)) {
    memcpy(&transfer_ready_pkt, data, sizeof(transfer_ready_pkt));
    if(transfer_ready_pkt.src_id != peer_id ||
       transfer_ready_pkt.session_id != session_id) {
      return;
    }
    if(state == STATE_TRANSFER &&
       !transfer_ready &&
       transfer_ready_pkt.start_chunk == first_unsent_chunk()) {
      transfer_ready = 1;
      last_reply_time = clock_time();
      process_poll(&node_a_process);
    }
  }
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(node_a_process, ev, data)
{
  static struct etimer timer;
  static struct etimer discovery_status_timer;
  static uint8_t sample_idx;
  static int light_value;
  static clock_time_t light_wait;

  PROCESS_BEGIN();

  memset(chunk_sent, 0, sizeof(chunk_sent));
  linkaddr_copy(&broadcast_addr, &linkaddr_null);
  nullnet_set_input_callback(input_callback);

  printf("Node A: sensing phase (%u readings at 1Hz)\n", (unsigned)NUM_READINGS);

#if LINK_METRIC_MODE == LINK_METRIC_RSSI
  printf("Node A: link metric mode = RSSI-only\n");
#elif LINK_METRIC_MODE == LINK_METRIC_PRR
  printf("Node A: link metric mode = PRR-only\n");
#endif

  mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);

  sample_idx = 0;
  while(sample_idx < NUM_READINGS) {
    SENSORS_ACTIVATE(opt_3001_sensor);
    light_wait = (sample_idx == 0) ? FIRST_LIGHT_WAIT : LIGHT_READY_TIME;
    etimer_set(&timer, light_wait);
    PROCESS_WAIT_EVENT_UNTIL((ev == sensors_event && data == &opt_3001_sensor) ||
                             etimer_expired(&timer));

    light_value = opt_3001_sensor.value(0);
    if(light_value == CC26XX_SENSOR_READING_ERROR) {
      light_readings[sample_idx] = (sample_idx == 0) ? 0 : light_readings[sample_idx - 1];
    } else {
      light_readings[sample_idx] = (uint16_t)light_value;
    }
    SENSORS_DEACTIVATE(opt_3001_sensor);

    motion_readings[sample_idx] = read_motion_value();

    printf("Sample %u: light=%u motion=%u\n",
           (unsigned)sample_idx,
           (unsigned)light_readings[sample_idx],
           (unsigned)motion_readings[sample_idx]);

    sample_idx++;
    if(sample_idx < NUM_READINGS) {
      etimer_set(&timer, SENSOR_SAMPLE_PERIOD - light_wait);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
    }
  }

  SENSORS_DEACTIVATE(opt_3001_sensor);
  SENSORS_DEACTIVATE(mpu_9250_sensor);

  state = STATE_DISCOVERY;
  printf("Node A: sensing done, starting discovery\n");
  printf("Node A: slot-based discovery | slot=%lu ms primes=%u,%u\n",
         (1000UL * SLOT_TIME) / RTIMER_SECOND,
         (unsigned)PRIME_P,
         (unsigned)PRIME_Q);
  etimer_set(&discovery_status_timer, REDISCOVERY_STATUS_INTERVAL);
  rtimer_set(&discovery_rt, RTIMER_NOW() + (RTIMER_SECOND / 1000), 1,
             (rtimer_callback_t)discovery_scheduler, NULL);

  while(1) {
    if(state == STATE_DISCOVERY || state == STATE_LINK_EVAL) {
      PROCESS_WAIT_EVENT();

      if(!contact_active &&
         ev == PROCESS_EVENT_TIMER &&
         data == &discovery_status_timer) {
        printf("Node A: rediscovering in slot mode\n");
        etimer_reset(&discovery_status_timer);
      }

      if(contact_active && (clock_time() - last_reply_time) > CONTACT_LOST_TIMEOUT) {
        contact_active = 0;
        state = STATE_DISCOVERY;
        reset_link_window();
        etimer_set(&discovery_status_timer, REDISCOVERY_STATUS_INTERVAL);
      }

      if(state == STATE_LINK_EVAL && probes_sent_window >= LINK_EVAL_PROBES) {
        if(rssi_count_window > 0) {
          avg_rssi_dbm = (int8_t)(rssi_sum_window / (int16_t)rssi_count_window);
        }
        prr_percent = (uint8_t)((100U * replies_received_window) / probes_sent_window);

#if LINK_METRIC_MODE == LINK_METRIC_RSSI
        printf("LINK %u RSSI:%d\n",
               (unsigned)peer_id,
               (int)avg_rssi_dbm);
#elif LINK_METRIC_MODE == LINK_METRIC_PRR
        printf("LINK %u PRR:%u\n",
               (unsigned)peer_id,
               (unsigned)prr_percent);
#endif

        if(link_quality_good()) {
          session_id++;
          print_transfer_ready_line();
          state = STATE_TRANSFER;
          reset_transfer_state();
          etimer_stop(&discovery_status_timer);
        } else {
          print_threshold_not_reached_line();
          state = STATE_DISCOVERY;
        }
        reset_link_window();
      }
    } else if(state == STATE_TRANSFER) {
      if(!transfer_ready) {
        NETSTACK_RADIO.on();
        transfer_started = 1;

        while(!transfer_ready && retry_count < TRANSFER_START_RETRY_LIMIT) {
          send_transfer_start();
          etimer_set(&timer, TRANSFER_START_INTERVAL);
          PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer) || transfer_ready);

          if(!transfer_ready) {
            retry_count++;
          }
        }

        if(!transfer_ready) {
          send_transfer_abort_burst();
          NETSTACK_RADIO.off();
          printf("Node A: transfer start failed, resuming discovery\n");
          contact_active = 0;
          reset_transfer_state();
          reset_link_window();
          state = STATE_DISCOVERY;
          recovery_block_until = clock_time() + RECOVERY_COOLDOWN;
          etimer_set(&discovery_status_timer, REDISCOVERY_STATUS_INTERVAL);
          printf("Node A: back in slot discovery\n");
        } else {
          retry_count = 0;
        }
        continue;
      }

      current_chunk = first_unsent_chunk();
      if(current_chunk >= NUM_CHUNKS) {
        transfer_complete = 1;
        send_transfer_end();
        NETSTACK_RADIO.off();
        printf("Node A: all unsent readings transferred\n");
        state = STATE_DONE;
        continue;
      }

      chunk_acked = 0;
      send_chunk(current_chunk);

      etimer_set(&timer, ACK_TIMEOUT);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer) || ev == PROCESS_EVENT_POLL);

      if(chunk_acked) {
        chunk_sent[current_chunk] = 1;
        retry_count = 0;
        if(all_chunks_sent()) {
          transfer_complete = 1;
        }
      } else {
        retry_count++;
        if(retry_count >= ACK_RETRY_LIMIT) {
          send_transfer_abort_burst();
          NETSTACK_RADIO.off();
          printf("Node A: contact lost during transfer, resuming discovery\n");
          contact_active = 0;
          reset_transfer_state();
          reset_link_window();
          state = STATE_DISCOVERY;
          recovery_block_until = clock_time() + RECOVERY_COOLDOWN;
          etimer_set(&discovery_status_timer, REDISCOVERY_STATUS_INTERVAL);
          printf("Node A: back in slot discovery\n");
        }
      }
    } else {
      if(!transfer_complete) {
        state = STATE_DISCOVERY;
        etimer_set(&discovery_status_timer, REDISCOVERY_STATUS_INTERVAL);
      } else {
        etimer_set(&timer, CLOCK_SECOND);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
      }
    }
  }

  PROCESS_END();
}
