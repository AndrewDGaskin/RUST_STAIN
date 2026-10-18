#pragma once

void initialize_arm();

void update_joint_positions(
    float joint0_command,
    float joint1_command,
    float joint2_command,
    float joint3_command,
    float clawOpen_command,
    float clawClose_command,
    float delta_time
);

float get_joint0_motor_command();
float get_joint1_target();
float get_joint2_target();
float get_joint3_target();
float get_claw_target();