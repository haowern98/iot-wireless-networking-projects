#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "contiki.h"
#include "sys/etimer.h"
#include "board-peripherals.h"
#include "buzzer.h"

PROCESS(task3_process, "Task 3 - Complex State Machine");
AUTOSTART_PROCESSES(&task3_process);

typedef enum
{
    STATE_IDLE,
    STATE_INTERIM,
    STATE_BUZZ,
    STATE_WAIT
} state_t;

#define LIGHT_THRESHOLD 30000 // opt_3001.value(0) returns centilux
#define MOTION_THRESHOLD 5000
#define BUZZ_CLOCK 8
#define WAIT_CLOCK 16
#define FREQUENCY 2093

static state_t current_state = STATE_IDLE;
static int cycle_count = 0;

static int last_lux = 0;
static int starting_pos = 0;
static int light_delta = 0;
static int motion_delta = 0;

static int motion_calibrated = 0;
static int light_calibrated = 0;

static int buzz_ticks = 0;
static int wait_ticks = 0;
static int cycle_interrupted = 0;

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

    if (lux != CC26XX_SENSOR_READING_ERROR)
    {
        return lux;
    }

    // If sensor fail, return last valid read
    return last_lux;
}

static void reset_to_idle(void)
{
    current_state = STATE_IDLE;
    cycle_count = 0;

    motion_calibrated = 0;
    light_calibrated = 0;

    buzz_ticks = 0;
    wait_ticks = 0;
    cycle_interrupted = 0;

    light_delta = 0;
    motion_delta = 0;

    printf("[RESET] Returning to IDLE. Resetting baselines...\n");
}

PROCESS_THREAD(task3_process, ev, data)
{
    static struct etimer sensors_timer;
    static int cur_lux;
    static int cur_pos;

    PROCESS_BEGIN();

    printf("===========================================\n");
    printf("  TASK 3 - COMPLEX STATE MACHINE\n");
    printf("===========================================\n");
    ;

    init_sensors_activators();

    while (1)
    {

        // Activate Light Sensor only if not in IDLE state
        if(current_state != STATE_IDLE) {
            SENSORS_ACTIVATE(opt_3001_sensor);
        }

        etimer_set(&sensors_timer, CLOCK_SECOND / 4);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&sensors_timer));

        if (current_state == STATE_IDLE) {
            cur_pos = get_motion_reading();

            if(!motion_calibrated) {
                starting_pos = cur_pos;
                motion_delta = 0;
                motion_calibrated = 1;
                printf("[IDLE] Setting Motion Baseline...\n");
                continue;
            }

            motion_delta = abs(starting_pos - cur_pos);
            starting_pos = cur_pos;

            printf("[IDLE] Motion Delta: %d\n", motion_delta);

            if(motion_delta > MOTION_THRESHOLD) {
                printf("[IDLE] Motion detected - switching to INTERIM...\n");
                current_state = STATE_INTERIM;
                light_calibrated = 0;
            }
        } else if (current_state == STATE_INTERIM) {
            cur_lux = get_light_reading();

            if(!light_calibrated) {
                last_lux = cur_lux;
                light_delta = 0;
                light_calibrated = 1;
                printf("[INTERIM] Setting Light Baseline...\n");
                continue;
            }

            light_delta = abs(last_lux - cur_lux);
            last_lux = cur_lux;

            printf("[INTERIM] Light Delta: %6d.%02d lux\n",
                    light_delta / 100, abs(light_delta) % 100);

            if(light_delta > LIGHT_THRESHOLD) {
                printf("[INTERIM] Light Change Detected - switching to BUZZ...\n");
                current_state = STATE_BUZZ;
                light_calibrated = 0;
            }

            
        } else if (current_state == STATE_BUZZ) {
            if(buzz_ticks == 0) {
                buzzer_start(FREQUENCY);
                printf("[BUZZ] Starting 2s Buzz...\n");
            }

            cur_lux = get_light_reading();

            if(!light_calibrated) {
                last_lux = cur_lux;
                light_delta = 0;
                light_calibrated = 1;
                printf("[BUZZ] Setting Light Baseline...\n");
            } else {
                light_delta = abs(last_lux - cur_lux);
                last_lux = cur_lux;

                printf("[BUZZ] tick=%3d/8 | Light Delta: %6d.%02d lux\n",
                        buzz_ticks + 1,
                        light_delta / 100, abs(light_delta) % 100);
                
                if(!cycle_interrupted && light_delta > LIGHT_THRESHOLD) {
                    cycle_interrupted = 1;
                    printf("[BUZZ] Significant light change detected during BUZZ.\n");
                }
            }

            buzz_ticks++;

            if(buzz_ticks >= BUZZ_CLOCK) {
                buzzer_stop();
                if(cycle_interrupted) {
                    printf("[BUZZ] Completed final 2s buzz. Returning to IDLE.\n");
                    reset_to_idle();
                } else {
                    printf("[BUZZ] Finished, transitioning to WAIT\n");
                    current_state = STATE_WAIT;
                    wait_ticks = 0;
                }
            }
        } else if (current_state == STATE_WAIT) {

            if (wait_ticks == 0) {
                printf("[WAIT] Starting 4s wait...\n");
            }

            cur_lux = get_light_reading();
            if (!light_calibrated) { // Safety, should never occur
                last_lux = cur_lux;
                light_delta = 0;
                light_calibrated = 1;
            } else {
                light_delta = abs(last_lux - cur_lux);
                last_lux = cur_lux;

                printf("[WAIT tick=%3d/16 | Light Delta: %6d.%02d lux\n",
                        wait_ticks + 1,
                        light_delta / 100, abs(light_delta) % 100);

                if (!cycle_interrupted && light_delta > LIGHT_THRESHOLD) {
                    cycle_interrupted = 1;
                    printf("[WAIT] Significant light change detected during WAIT.\n");
                }
            }

            wait_ticks++;

            if(wait_ticks >= WAIT_CLOCK) {
                if(cycle_interrupted) {
                    printf("[WAIT] Interrupted. Proceeding to final BUZZ.\n");
                } else {
                    printf("[WAIT] Cycle complete. Returning to BUZZ.\n");
                }
                current_state = STATE_BUZZ;
                buzz_ticks = 0;
            }
        }
    }
    PROCESS_END();
}
