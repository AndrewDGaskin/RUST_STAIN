#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

void update_joint_positions(float joint0_command, float joint1_command, float joint2_command, float joint3_command, float clawOpen_command, float clawClose_command);
void joint0_update(float command);
void joint1_update(float command);
void joint2_update(float command);
void joint3_update(float command);
void claw_update(float clawOpen_command, float clawClose_command);

void initialize_servo(uint gpio);
void set_servo_position(uint gpio, float angle, float servo_max_angle);

void send_feedback();

const uint JOINT0_GPIO = 12;
const uint JOINT1_GPIO = 18;
const uint JOINT2_GPIO = 19;
const uint JOINT3_GPIO = 20;
const uint CLAW_GPIO = 6;



float joint0_target = 90.0;
float joint1_target = 135.0;
float joint2_target = 135.0;
float joint3_target = 90.0;
float claw_target = 90.0;

int main() {
    stdio_init_all();
    initialize_servo(JOINT0_GPIO);
    initialize_servo(JOINT1_GPIO);
    initialize_servo(JOINT2_GPIO);
    initialize_servo(JOINT3_GPIO);
    initialize_servo(CLAW_GPIO);

    //enter safe state
    set_servo_position(JOINT0_GPIO, joint0_target, 180.0f);
    set_servo_position(JOINT1_GPIO, joint1_target, 270.0f);
    set_servo_position(JOINT2_GPIO, joint2_target, 270.0f);
    set_servo_position(JOINT3_GPIO, joint3_target, 180.0f);
    set_servo_position(CLAW_GPIO, claw_target, 180.0f);

    /*
    while (true) {
        printf("Data from Pico!\n");
        sleep_ms(1000);
    }
    */

    char command_buffer[100]; //update this size based on expected command length, remember to leave space for the null terminator
    int buffer_index = 0;
    float joint0_command, joint1_command, joint2_command, joint3_command, clawOpen_command, clawClose_command;
    
    uint64_t last_feedback_time = time_us_64();
    
    
    while (true) {
        int command = getchar_timeout_us(0);
        if (command != PICO_ERROR_TIMEOUT) {
            if (command == '\n') {
                command_buffer[buffer_index] = '\0'; // Null-terminate the string

                
                //printf("Received command: %s\n", command_buffer);
                

                // Pull out each value between the commas and store them in the corresponding variables
                int parsed = sscanf(
                    command_buffer, 
                    "%f,%f,%f,%f,%f,%f", 
                    &joint0_command, &joint1_command, &joint2_command, &joint3_command, &clawOpen_command, &clawClose_command
                    );

                if (parsed == 6) {
                    // Successfully parsed all values, proceed with processing
                    update_joint_positions(joint0_command, joint1_command, joint2_command, joint3_command, clawOpen_command, clawClose_command);
                } else {
                    // Handle parsing error
                    printf("Error: Expected 6 values but got %d\n", parsed);
                }

                buffer_index = 0; // Reset buffer index for next command
            } else {
                if (buffer_index <99) { // Prevent buffer overflow
                    command_buffer[buffer_index++] = command;
                }
            }
        }
        if (time_us_64() - last_feedback_time >= 100000) { // 100ms
                send_feedback();
                last_feedback_time = time_us_64();
        }
    }


    return 0;
}

void update_joint_positions(float joint0_command, float joint1_command, float joint2_command, float joint3_command, float clawOpen_command, float clawClose_command) {
    
    //printf("Updating joints: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", joint0, joint1, joint2, joint3, clawOpen, clawClose);

    joint0_update(joint0_command);
    joint1_update(joint1_command);
    joint2_update(joint2_command);
    joint3_update(joint3_command);
    claw_update(clawOpen_command, clawClose_command);

    set_servo_position(JOINT0_GPIO, joint0_target, 180.0f);
    set_servo_position(JOINT1_GPIO, joint1_target, 270.0f);
    set_servo_position(JOINT2_GPIO, joint2_target, 270.0f);
    set_servo_position(JOINT3_GPIO, joint3_target, 180.0f);
    set_servo_position(CLAW_GPIO, claw_target, 180.0f);
}

/*
!!! remember to update the limits when you get the actual limits of the joints and claw, these are just placeholders for now
*/

void joint0_update(float command) {
    joint0_target = joint0_target + command; // Update the target position based on the command

    if (joint0_target >= 180.0) {
        joint0_target = 180.0; // Limit the target position to a maximum of 180 degrees
    } else if (joint0_target <= 0.0) {
        joint0_target = 0.0; // Limit the target position to a minimum of 0 degrees
    }
}

void joint1_update(float command) {
    joint1_target = joint1_target + command; // Update the target position based on the command
    if (joint1_target >= 270.0) {
        joint1_target = 270.0;
    } else if (joint1_target <= 0.0) {
        joint1_target = 0.0;
    }
}

void joint2_update(float command) {
    joint2_target = joint2_target + command; // Update the target position based on the command
    if (joint2_target >= 270.0) {
        joint2_target = 270.0;
    } else if (joint2_target <= 0.0) {
        joint2_target = 0.0;
    }
}

void joint3_update(float command) {
    joint3_target = joint3_target + command; // Update the target position based on the command
    if (joint3_target >= 180.0) {
        joint3_target = 180.0;
    } else if (joint3_target <= 0.0) {
        joint3_target = 0.0;
    }
}

void claw_update(float clawOpen_command, float clawClose_command) {
    claw_target = claw_target + (clawOpen_command - clawClose_command);
    if (claw_target >= 180.0) {
        claw_target = 180.0;
    } else if (claw_target <= 0.0) {
        claw_target = 0.0;
    }
}

void initialize_servo(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_clkdiv(slice_num, 125.0f);
    pwm_set_wrap(slice_num, 19999);
    uint channel = pwm_gpio_to_channel(gpio);
    pwm_set_chan_level(slice_num, channel, 0);
    pwm_set_enabled(slice_num, true);
}

void set_servo_position(uint gpio, float angle, float servo_max_angle) {
    if (angle < 0.0f) angle = 0.0f;
    if (angle > servo_max_angle) angle = servo_max_angle;

    uint slice_num = pwm_gpio_to_slice_num(gpio);
    uint channel = pwm_gpio_to_channel(gpio);

    // Convert angle to pulse width in microseconds
    float pulse_width_us = 1000.0f + (angle / servo_max_angle) * 1000.0f; // 1000us to 2000us for 0 to servo_max_angle degrees

    // Convert pulse width to PWM level
    uint16_t level = (uint16_t)pulse_width_us;

    pwm_set_chan_level(slice_num, channel, level);
}

void send_feedback() {
    // Send the current target positions of the joints and claw back to the host
    printf("%.2f,%.2f,%.2f,%.2f,%.2f\n", joint0_target, joint1_target, joint2_target, joint3_target, claw_target);
}