#include "arm_control.h"

#include "pico/stdlib.h"
#include "hardware/pwm.h"

#include <cmath>

#include <algorithm>

// Internal arm-control functions
void initialize_joint0_motor();
void set_joint0_motor(float command);

void initialize_servo(uint gpio);
void set_servo_position(uint gpio, float angle, float servo_max_angle);

void joint1_update(float command, float delta_time);
void joint2_update(float command, float delta_time);
void joint3_update(float command, float delta_time);
void claw_update(
    float clawOpen_command,
    float clawClose_command,
    float delta_time
);

//Axis0 DC Motor
const uint JOINT0_IN1_GPIO = 11;
const uint JOINT0_IN2_GPIO = 14;
const uint JOINT0_PWM_GPIO = 15;
const uint JOINT1_GPIO = 13;
const uint JOINT2_GPIO = 19;
const uint JOINT3_GPIO = 20;
const uint CLAW_GPIO = 6;

//Current commanded target positions
float joint0_motor_command = 0.0f;
float joint1_target = 135.0;
float joint2_target = 135.0;
float joint3_target = 90.0;
float claw_target = 90.0;

//Maximum commanded motion speed in degrees per second
const float JOINT1_SPEED_DPS = 10.0f; //provisional value, will need to be tuned based on actual joint speed and desired responsiveness
const float JOINT2_SPEED_DPS = 10.0f; //provisional value, will need to be tuned based on actual joint speed and desired responsiveness
const float JOINT3_SPEED_DPS = 10.0f; //provisional value, will need to be tuned based on actual joint speed and desired responsiveness
const float CLAW_SPEED_DPS   = 10.0f; //provisional value, will need to be tuned based on actual joint speed and desired responsiveness

const float JOINT1_SERVO_MAX_ANGLE = 270.0f;
const float JOINT2_SERVO_MAX_ANGLE = 270.0f;
const float JOINT3_SERVO_MAX_ANGLE = 180.0f;
const float CLAW_SERVO_MAX_ANGLE   = 180.0f;

const float JOINT0_PWM_CLOCK_DIV = 1.0f;
const uint16_t JOINT0_PWM_WRAP = 6249;

//
void initialize_joint0_motor() {
    gpio_init(JOINT0_IN1_GPIO);
    gpio_set_dir(JOINT0_IN1_GPIO, GPIO_OUT);

    gpio_init(JOINT0_IN2_GPIO);
    gpio_set_dir(JOINT0_IN2_GPIO, GPIO_OUT);

    // Start with the motor stopped
    gpio_put(JOINT0_IN1_GPIO, 0);
    gpio_put(JOINT0_IN2_GPIO, 0);

    gpio_set_function(JOINT0_PWM_GPIO, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(JOINT0_PWM_GPIO);
    uint channel = pwm_gpio_to_channel(JOINT0_PWM_GPIO);

    pwm_set_clkdiv(slice_num, JOINT0_PWM_CLOCK_DIV);
    pwm_set_wrap(slice_num, JOINT0_PWM_WRAP);

    pwm_set_chan_level(slice_num, channel, 0);
    pwm_set_enabled(slice_num, true);
}

//
void set_joint0_motor(float command) {
    uint slice_num = pwm_gpio_to_slice_num(JOINT0_PWM_GPIO);
    uint channel = pwm_gpio_to_channel(JOINT0_PWM_GPIO);

    command = std::clamp(command, -1.0f, 1.0f);

    uint16_t pwm_level = (uint16_t)(std::fabs(command) * JOINT0_PWM_WRAP);

    joint0_motor_command = command;

    if (command > 0.0f) {

        gpio_put(JOINT0_IN1_GPIO, 1);
        gpio_put(JOINT0_IN2_GPIO, 0);

    } else if (command < 0.0f) {

        gpio_put(JOINT0_IN1_GPIO, 0);
        gpio_put(JOINT0_IN2_GPIO, 1);

    } else {

        gpio_put(JOINT0_IN1_GPIO, 0);
        gpio_put(JOINT0_IN2_GPIO, 0);
        pwm_level = 0;
    }

    pwm_set_chan_level(slice_num, channel, pwm_level);
}

//Prepares GPIO pin to drive servo
void initialize_servo(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);

    uint slice_num = pwm_gpio_to_slice_num(gpio);

    pwm_set_clkdiv(slice_num, 125.0f);
    pwm_set_wrap(slice_num, 19999);

    uint channel = pwm_gpio_to_channel(gpio);

    pwm_set_chan_level(slice_num, channel, 0);
    pwm_set_enabled(slice_num, true);
}



//Converts degree angles into pulse width for servos
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

//
void update_joint_positions(float joint0_command, float joint1_command, float joint2_command, float joint3_command, float clawOpen_command, float clawClose_command, float delta_time) {

    set_joint0_motor(joint0_command);
    joint1_update(joint1_command, delta_time);
    joint2_update(joint2_command, delta_time);
    joint3_update(joint3_command, delta_time);
    claw_update(clawOpen_command, clawClose_command, delta_time);

    set_servo_position(JOINT1_GPIO, joint1_target, JOINT1_SERVO_MAX_ANGLE);
    set_servo_position(JOINT2_GPIO, joint2_target, JOINT2_SERVO_MAX_ANGLE);
    set_servo_position(JOINT3_GPIO, joint3_target, JOINT3_SERVO_MAX_ANGLE);
    set_servo_position(CLAW_GPIO, claw_target, CLAW_SERVO_MAX_ANGLE);
}



void joint1_update(float command, float delta_time) {
    joint1_target = joint1_target + command * JOINT1_SPEED_DPS * delta_time; // Update the target position based on the command
    if (joint1_target >= JOINT1_SERVO_MAX_ANGLE) {
        joint1_target = JOINT1_SERVO_MAX_ANGLE;
    } else if (joint1_target <= 0.0) {
        joint1_target = 0.0;
    }
}



void joint2_update(float command, float delta_time) {
    joint2_target = joint2_target + command * JOINT2_SPEED_DPS * delta_time; // Update the target position based on the command
    if (joint2_target >= JOINT2_SERVO_MAX_ANGLE) {
        joint2_target = JOINT2_SERVO_MAX_ANGLE;
    } else if (joint2_target <= 0.0) {
        joint2_target = 0.0;
    }
}



void joint3_update(float command, float delta_time) {
    joint3_target = joint3_target + command * JOINT3_SPEED_DPS * delta_time; // Update the target position based on the command
    if (joint3_target >= JOINT3_SERVO_MAX_ANGLE) {
        joint3_target = JOINT3_SERVO_MAX_ANGLE;
    } else if (joint3_target <= 0.0) {
        joint3_target = 0.0;
    }
}



void claw_update(float clawOpen_command, float clawClose_command, float delta_time) {
    claw_target = claw_target + (clawOpen_command - clawClose_command) * CLAW_SPEED_DPS * delta_time;
    if (claw_target >= CLAW_SERVO_MAX_ANGLE) {
        claw_target = CLAW_SERVO_MAX_ANGLE;
    } else if (claw_target <= 0.0) {
        claw_target = 0.0;
    }
}



float get_joint0_motor_command() {
    return joint0_motor_command;
}

float get_joint1_target() {
    return joint1_target;
}

float get_joint2_target() {
    return joint2_target;
}

float get_joint3_target() {
    return joint3_target;
}

float get_claw_target() {
    return claw_target;
}

void initialize_arm() {

    initialize_joint0_motor();

    initialize_servo(JOINT1_GPIO);
    initialize_servo(JOINT2_GPIO);
    initialize_servo(JOINT3_GPIO);
    initialize_servo(CLAW_GPIO);

    set_servo_position(
        JOINT1_GPIO,
        joint1_target,
        JOINT1_SERVO_MAX_ANGLE
    );

    set_servo_position(
        JOINT2_GPIO,
        joint2_target,
        JOINT2_SERVO_MAX_ANGLE
    );

    set_servo_position(
        JOINT3_GPIO,
        joint3_target,
        JOINT3_SERVO_MAX_ANGLE
    );

    set_servo_position(
        CLAW_GPIO,
        claw_target,
        CLAW_SERVO_MAX_ANGLE
    );
}