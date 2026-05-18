#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "contiki.h"
#include "sys/etimer.h"
#include "board-peripherals.h"
#include "buzzer.h"

PROCESS(task2_process, "Task 2 - Delta Monitoring & Timer");
AUTOSTART_PROCESSES(&task2_process);

typedef enum
{
    STATE_IDLE,
    STATE_BUZZ,
    STATE_WAIT
} state_t;

#define LIGHT_THRESHOLD 30000 // opt_3001.value(0) returns centilux
#define MOTION_THRESHOLD 5000
#define FREQUENCY 2093

static state_t current_state = STATE_IDLE;
static int cycle_count = 0;
static int last_lux = 0;
static int starting_pos = 0;
static int light_delta = 0;
static int motion_delta = 0;
static int calibrated = 0;

static void init_sensors_activators(void)
{
    buzzer_init();
    mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);
}

static int get_motion_reading(void)
{
    int gx = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_X);
    int gy = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Y);
    int gz = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Z);
    int ax = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
    int ay = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
    int az = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);

    if (gx != CC26XX_SENSOR_READING_ERROR &&
        gy != CC26XX_SENSOR_READING_ERROR &&
        gz != CC26XX_SENSOR_READING_ERROR &&
        ax != CC26XX_SENSOR_READING_ERROR &&
        ay != CC26XX_SENSOR_READING_ERROR &&
        az != CC26XX_SENSOR_READING_ERROR)
    {
        return abs(gx) + abs(gy) + abs(gz) + abs(ax) + abs(ay) + abs(az);
    }

    // If sensor fails, return last valid read
    return starting_pos;
}

static int get_light_reading(void)
{
    int lux = opt_3001_sensor.value(0);

    if(lux != CC26XX_SENSOR_READING_ERROR) {
        return lux;
    }

    // If sensor fail, return last valid read
    return last_lux;
}

static void calibrate_sensors(int cur_lux, int cur_pos)
{
    starting_pos = cur_pos;
    last_lux = cur_lux;

    light_delta = 0;
    motion_delta = 0;
    cycle_count = 0;
    printf("[CALIBRATION] Baselines set. Entering IDLE...\n");

    calibrated = 1;
}

PROCESS_THREAD(task2_process, ev, data)
{
    static struct etimer buzz_timer;
    static struct etimer sensor_timer;
    static int cur_lux;
    static int cur_pos;

    PROCESS_BEGIN();

    printf("===========================================\n");
    printf("  TASK 2 - DELTA MONITORING & PHASE TIMERS\n");
    printf("===========================================\n");

    init_sensors_activators();

    while(1)
    {

        if(current_state == STATE_IDLE) {
            /*
                In order to take a new reading, the caller has to use SENSORS_ACTIVATE again.
                https://docs.contiki-ng.org/en/master/_api/group__sensortag-cc26xx-opt-sensor.html
            */
            SENSORS_ACTIVATE(opt_3001_sensor);
            etimer_set(&sensor_timer, CLOCK_SECOND / 4);
            PROCESS_WAIT_UNTIL(etimer_expired(&sensor_timer));

            cur_lux = get_light_reading();
            cur_pos = get_motion_reading();

            if(!calibrated) {
                calibrate_sensors(cur_lux, cur_pos);
                continue;
            }

            light_delta = abs(last_lux - cur_lux);
            motion_delta = abs(starting_pos - cur_pos);

            last_lux = cur_lux;
            starting_pos = cur_pos;

            printf("[IDLE] Light: %6d.%02d lux | Light Delta: %6d.%02d lux | Motion Delta: %6d\n",
                last_lux / 100, abs(last_lux) % 100,
                light_delta / 100, light_delta % 100,
                motion_delta);

            if (light_delta > LIGHT_THRESHOLD) {
                printf("*** Light change detected - switching to BUZZ ***\n");
                current_state = STATE_BUZZ;
            } else if (motion_delta > MOTION_THRESHOLD) {
                printf("*** MOTION change detected - switching to BUZZ ***\n");
                current_state = STATE_BUZZ;
            }

        } else if(current_state == STATE_BUZZ) {
            buzzer_start(FREQUENCY);

            // If cycle count == 4, indicate its final buzz
            if(cycle_count < 4) {
                printf("[BUZZ] Cycle %d/4 : ", cycle_count + 1);
            } else {
                printf("[FINAL BUZZ]: ");
            }

            etimer_set(&buzz_timer, CLOCK_SECOND);
            PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&buzz_timer));
            printf("... 1");

            etimer_set(&buzz_timer, CLOCK_SECOND);
            PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&buzz_timer));
            printf("... 2\n");

            buzzer_stop();
            
            // If cycle_count == 4, proceed to STATE_IDLE, and reset baseline
            if(cycle_count < 4) {
                current_state = STATE_WAIT;
            } else {
                current_state = STATE_IDLE;
                calibrated = 0;
                printf("Sequence Complete. Resetting Baselines.\n");
            }

        } else if(current_state == STATE_WAIT) {
            printf("[WAIT] Cycle %d/4 : ", cycle_count + 1);

            etimer_set(&buzz_timer, CLOCK_SECOND);
            PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&buzz_timer));
            printf("... 1");

            etimer_set(&buzz_timer, CLOCK_SECOND);
            PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&buzz_timer));
            printf("... 2\n");

            cycle_count++;
            current_state = STATE_BUZZ;
        }
    }
    PROCESS_END();
}
