#include <stdio.h>
#include "pico/stdlib.h"
#include <cmath>
#include "sd_logger.h"
#include "arm_control.h"
#include <algorithm>
#include "sensors.h"

//Prototypes
void send_feedback();


// Controller command input
const size_t COMMAND_BUFFER_SIZE = 100;
const uint64_t COMMAND_TIMEOUT_US = 250000;   // 250 ms


// Main control loop
const uint64_t CONTROL_INTERVAL_US = 20000;   // 20 ms = 50 Hz


//Logger Timing
const uint64_t SD_LOG_INTERVAL_US = 100000;   // 10 Hz
const uint64_t SD_SYNC_INTERVAL_US = 1000000; // 1 second


// Telemetry and sensor update rates
const uint64_t SENSOR_INTERVAL_US = 100000;   // 100 ms = 10 Hz
const uint64_t FEEDBACK_INTERVAL_US = 100000; // 100 ms = 10 Hz

//Values passed
const int CONTROLLER_COMMAND_VALUES = 6;


//Logging
bool sd_logging_available = false;

const uint LED_PIN = 25;



int main() {
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    sleep_ms(10000);

    initialize_sensors();

    sd_logging_available = initialize_sd_logging();
    if (!sd_logging_available) {
        printf("SD logging unavailable\n");
    }

    initialize_arm();

    char command_buffer[COMMAND_BUFFER_SIZE]; //update this size based on expected command length, remember to leave space for the null terminator
    size_t buffer_index = 0;
    bool buffer_overflow = false;
    float joint0_command = 0.0f;
    float joint1_command = 0.0f;
    float joint2_command = 0.0f;
    float joint3_command = 0.0f;
    float clawOpen_command = 0.0f;
    float clawClose_command = 0.0f;
    
    uint64_t last_feedback_time = time_us_64();
    uint64_t last_sensor_time = time_us_64();
    uint64_t last_control_time = time_us_64();
    uint64_t last_command_time = time_us_64();
    uint64_t last_sd_log_time = time_us_64();
    uint64_t last_sd_sync_time = time_us_64();
    
    
    while (true) {
        int command = getchar_timeout_us(0);
        if (command != PICO_ERROR_TIMEOUT) {
            if (command == '\n') {
                command_buffer[buffer_index] = '\0'; // Null-terminate the string

                if (!buffer_overflow) {
                    float new_joint0, new_joint1, new_joint2, new_joint3, new_clawOpen, new_clawClose;

                    // Pull out each value between the commas and store them in the corresponding variables
                    int parsed = sscanf(
                        command_buffer, 
                        "%f,%f,%f,%f,%f,%f", 
                        &new_joint0, &new_joint1, &new_joint2, &new_joint3, &new_clawOpen, &new_clawClose
                        );

                    if (parsed == CONTROLLER_COMMAND_VALUES &&
                        std::isfinite(new_joint0) &&   
                        std::isfinite(new_joint1) && 
                        std::isfinite(new_joint2) && 
                        std::isfinite(new_joint3) && 
                        std::isfinite(new_clawOpen) && 
                        std::isfinite(new_clawClose)
                    ) {

                        joint0_command = std::clamp(new_joint0, -1.0f, 1.0f);
                        joint1_command = std::clamp(new_joint1, -1.0f, 1.0f);
                        joint2_command = std::clamp(new_joint2, -1.0f, 1.0f);
                        joint3_command = std::clamp(new_joint3, -1.0f, 1.0f);
                        clawOpen_command = std::clamp(new_clawOpen, 0.0f, 1.0f);
                        clawClose_command = std::clamp(new_clawClose, 0.0f, 1.0f);
                        last_command_time = time_us_64();
                    } else {
                        // Handle parsing error
                        printf("Error: Invalid controller command\n");
                    }
                } else {
                    printf("Error: Controller command too long\n");
                }

                buffer_index = 0; // Reset buffer index for next command
                buffer_overflow = false; // Reset overflow flag
            } else {
                if (buffer_index < COMMAND_BUFFER_SIZE - 1) { // Prevent buffer overflow
                    command_buffer[buffer_index++] = command;
                } else {
                    buffer_overflow = true;
                }
            }
        }

        uint64_t current_time = time_us_64();

        if (current_time - last_command_time >= COMMAND_TIMEOUT_US) {
            joint0_command = 0.0f;
            joint1_command = 0.0f;
            joint2_command = 0.0f;
            joint3_command = 0.0f;
            clawOpen_command = 0.0f;
            clawClose_command = 0.0f;
        }

        if (current_time - last_control_time >= CONTROL_INTERVAL_US) {
            float delta_time =
                (current_time - last_control_time) / 1000000.0f;

            update_joint_positions(
                joint0_command,
                joint1_command,
                joint2_command,
                joint3_command,
                clawOpen_command,
                clawClose_command,
                delta_time
            );

            last_control_time = current_time;
        }

        if (time_us_64() - last_sensor_time >= SENSOR_INTERVAL_US) {

            update_sensors();

            last_sensor_time = time_us_64();
        }
        if (time_us_64() - last_feedback_time >= FEEDBACK_INTERVAL_US) { // 100ms
                send_feedback();
                last_feedback_time = time_us_64();
        }
        if (
            sd_logging_available &&
            time_us_64() - last_sd_log_time >= SD_LOG_INTERVAL_US
        ) {
            log_telemetry_to_sd(
                time_us_64(),
                get_joint0_motor_command(),
                get_joint1_target(),
                get_joint2_target(),
                get_joint3_target(),
                get_claw_target(),
                get_accel_x(),
                get_accel_y(),
                get_accel_z(),
                get_temperature(),
                get_pressure(),
                get_altitude()
            );

            last_sd_log_time = time_us_64();
        }
        if (
            sd_logging_available &&
            time_us_64() - last_sd_sync_time >= SD_SYNC_INTERVAL_US
        ) {
            sync_sd_log();
            last_sd_sync_time = time_us_64();
}       
    }

    return 0;
}


void send_feedback() {
    //printf("FW_TEST_10SEC_DIAGNOSTIC\n");
    printf(
        "TEL,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
        get_joint0_motor_command(),
        get_joint1_target(),
        get_joint2_target(),
        get_joint3_target(),
        get_claw_target(),
        get_accel_x(),
        get_accel_y(),
        get_accel_z(),
        get_temperature(),
        get_pressure(),
        get_altitude()
    );
}



