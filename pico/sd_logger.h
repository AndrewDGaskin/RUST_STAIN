#pragma once

#include <stdint.h>

bool initialize_sd_logging();

void log_telemetry_to_sd(
    uint64_t timestamp_us,
    float joint0,
    float joint1,
    float joint2,
    float joint3,
    float claw,
    float accel_x,
    float accel_y,
    float accel_z,
    float temperature,
    float pressure,
    float altitude
);

void sync_sd_log();

void close_sd_log();