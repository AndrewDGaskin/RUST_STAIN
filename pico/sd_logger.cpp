#include "sd_logger.h"
#include "pico/stdlib.h"

const uint SD_SCK_GPIO  = 2;
const uint SD_MOSI_GPIO = 4;
const uint SD_MISO_GPIO = 3;
const uint SD_CS_GPIO   = 5;

bool initialize_sd_logging() {
    return false;
}

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
) {
}

void sync_sd_log() {
}

void close_sd_log() {
}