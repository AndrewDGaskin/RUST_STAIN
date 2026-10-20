#include "sd_logger.h"
#include "pico/stdlib.h"
#include "ff.h"
#include "f_util.h"
#include <cstdio>

const uint SD_SCK_GPIO  = 2;
const uint SD_MOSI_GPIO = 3;
const uint SD_MISO_GPIO = 4;
const uint SD_CS_GPIO   = 5;

bool initialize_sd_logging() {

    FATFS fs;

    FRESULT result = f_mount(&fs, "", 1);

    if (result != FR_OK) {
        printf("SD mount failed: %d\n", result);
        return false;
    }

    FIL file;

    result = f_open(
        &file,
        "sd_test.txt",
        FA_CREATE_ALWAYS | FA_WRITE
    );

    if (result != FR_OK) {
        printf("SD file open failed: %d\n", result);
        f_unmount("");
        return false;
    }

    if (f_printf(&file, "RUST_STAIN SD test successful\n") < 0) {
        printf("SD write failed\n");
        f_close(&file);
        f_unmount("");
        return false;
    }

    result = f_close(&file);

    if (result != FR_OK) {
        printf("SD close failed: %d\n", result);
        f_unmount("");
        return false;
    }

    f_unmount("");

    printf("SD test file written successfully\n");

    return true;
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