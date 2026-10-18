#pragma once

void initialize_sensors();
void update_sensors();

float get_accel_x();
float get_accel_y();
float get_accel_z();

float get_temperature();
float get_pressure();
float get_altitude();