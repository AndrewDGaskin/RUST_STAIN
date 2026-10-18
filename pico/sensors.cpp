#include "sensors.h"

#include <stdio.h>
#include <cmath>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

//I2C controls | Sensor comms
void initialize_i2c();
bool i2c_read_register_byte(i2c_inst_t *i2c, uint8_t device_address, uint8_t register_address, uint8_t*value);
bool i2c_write_register_byte(i2c_inst_t *i2c, uint8_t device_address, uint8_t register_address, uint8_t value);
bool i2c_read_register_block(i2c_inst_t *i2c, uint8_t device_address, uint8_t start_register, uint8_t *buffer, size_t length);
void detect_sensors();
void configure_bno055();
bool read_bno055_acceleration(float *x, float *y, float *z);
bool read_bmp388_calibration();
bool configure_bmp388();
bool read_bmp388_raw(uint32_t *raw_pressure, uint32_t *raw_temperature);
float compensate_bmp388_temperature(uint32_t raw_temperature);
float compensate_bmp388_pressure(uint32_t raw_pressure);
float calculate_relative_altitude(float pressure_pa, float reference_pressure_pa);

//BMP388
const uint BMP_SDA_GPIO = 8;
const uint BMP_SCL_GPIO = 9;

//BMP388
const uint8_t BMP_CHIP_ID_REG = 0x00;
const uint8_t BMP_EXPECTED_ID = 0x50;
uint8_t bmp_address = 0;
const uint8_t BMP_ADDRESS_1 = 0x76;
const uint8_t BMP_ADDRESS_2 = 0x77;
bool bmp_connected = false;
const uint8_t BMP_CALIB_DATA_START_REG = 0x31;
const size_t BMP_CALIB_DATA_LENGTH = 21;
const uint8_t BMP_PWR_CTRL_REG = 0x1B;
const uint8_t BMP_OSR_REG = 0x1C;
const uint8_t BMP_ODR_REG = 0x1D;
const uint8_t BMP_OSR_STANDARD = 0x02;
const uint8_t BMP_ODR_12_5_HZ = 0x04;
const uint8_t BMP_PWR_NORMAL_PRESS_TEMP = 0x33;
const uint8_t BMP_DATA_START_REG = 0x04;
const size_t BMP_DATA_LENGTH = 6;
uint32_t bmp_raw_pressure = 0;
uint32_t bmp_raw_temperature = 0;
bool bmp_configured = false;
float bmp_temperature_c = 0.0f;
float bmp_t_lin = 0.0f;
float bmp_pressure_pa = 0.0f;
float bmp_reference_pressure_pa = 0.0f;
float bmp_altitude_m = 0.0f;
bool bmp_altitude_reference_set = false;
struct BMP388Calibration {
    uint16_t par_t1;
    uint16_t par_t2;
    int8_t par_t3;

    int16_t par_p1;
    int16_t par_p2;
    int8_t par_p3;
    int8_t par_p4;
    uint16_t par_p5;
    uint16_t par_p6;
    int8_t par_p7;
    int8_t par_p8;
    int16_t par_p9;
    int8_t par_p10;
    int8_t par_p11;
};
BMP388Calibration bmp_calibration;
bool bmp_calibration_loaded = false;

//BNO055
const uint8_t BNO_CHIP_ID_REG = 0x00;
const uint8_t BNO_EXPECTED_ID = 0xA0;
uint8_t bno_address = 0;
const uint8_t BNO_ADDRESS_1 = 0x28;
const uint8_t BNO_ADDRESS_2 = 0x29;
bool bno_connected = false;
const uint8_t BNO_OPR_MODE_REG = 0x3D;
const uint8_t BNO_UNIT_SEL_REG = 0x3B;
const uint8_t BNO_MODE_CONFIG = 0x00;
const uint8_t BNO_MODE_ACCONLY = 0x01;
const uint8_t BNO_ACCEL_DATA_START_REG = 0x08;
const float BNO_ACCEL_SCALE = 100.0f;
float bno_accel_x = 0.0f;
float bno_accel_y = 0.0f;
float bno_accel_z = 0.0f;


// I2C communication
const uint32_t I2C_FREQUENCY_HZ = 100000;     // 100 kHz
const uint32_t I2C_TIMEOUT_US = 10000;          // 10 ms

//I2C startup
const uint32_t SENSOR_STARTUP_DELAY_MS = 700;


float get_accel_x() {
    return bno_accel_x;
}

float get_accel_y() {
    return bno_accel_y;
}

float get_accel_z() {
    return bno_accel_z;
}

float get_temperature() {
    return bmp_temperature_c;
}

float get_pressure() {
    return bmp_pressure_pa;
}

float get_altitude() {
    return bmp_altitude_m;
}

//Starts up sensors, sets there values
void initialize_i2c() {
    i2c_init(i2c0, I2C_FREQUENCY_HZ);

    gpio_set_function(BMP_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(BMP_SCL_GPIO, GPIO_FUNC_I2C);

    gpio_pull_up(BMP_SDA_GPIO);
    gpio_pull_up(BMP_SCL_GPIO);
}

void update_sensors() {

    read_bno055_acceleration(
        &bno_accel_x,
        &bno_accel_y,
        &bno_accel_z
    );

    if (read_bmp388_raw(
            &bmp_raw_pressure,
            &bmp_raw_temperature)) {

        bmp_temperature_c =
            compensate_bmp388_temperature(
                bmp_raw_temperature
            );

        bmp_pressure_pa =
            compensate_bmp388_pressure(
                bmp_raw_pressure
            );

        if (!bmp_altitude_reference_set &&
            bmp_pressure_pa > 0.0f) {

            bmp_reference_pressure_pa =
                bmp_pressure_pa;

            bmp_altitude_reference_set = true;
            bmp_altitude_m = 0.0f;

        } else if (bmp_altitude_reference_set) {

            bmp_altitude_m =
                calculate_relative_altitude(
                    bmp_pressure_pa,
                    bmp_reference_pressure_pa
                );
        }
    }
}


//detects if sensors are connected and, if so, assigns their addresses
void detect_sensors() {
    uint8_t chip_id = 0;

    // Check BMP388 on I2C0
    if (i2c_read_register_byte(i2c0, BMP_ADDRESS_1, BMP_CHIP_ID_REG, &chip_id)
        && chip_id == BMP_EXPECTED_ID) {

        bmp_address = BMP_ADDRESS_1;
        bmp_connected = true;
        printf("BMP388 confirmed at 0x%02X\n", (unsigned int)bmp_address);
    }
    else if (i2c_read_register_byte(i2c0, BMP_ADDRESS_2, BMP_CHIP_ID_REG, &chip_id)
             && chip_id == BMP_EXPECTED_ID) {

        bmp_address = BMP_ADDRESS_2;
        bmp_connected = true;
        printf("BMP388 confirmed at 0x%02X\n", (unsigned int)bmp_address);
    }
    else {
        printf("BMP388 not detected\n");
    }

    // Check BNO055 on i2c0
    if (i2c_read_register_byte(i2c0, BNO_ADDRESS_1, BNO_CHIP_ID_REG, &chip_id)
        && chip_id == BNO_EXPECTED_ID) {

        bno_address = BNO_ADDRESS_1;
        bno_connected = true;
        printf("BNO055 confirmed at 0x%02X\n", (unsigned int)bno_address);
    }
    else if (i2c_read_register_byte(i2c0, BNO_ADDRESS_2, BNO_CHIP_ID_REG, &chip_id)
             && chip_id == BNO_EXPECTED_ID) {

        bno_address = BNO_ADDRESS_2;
        bno_connected = true;
        printf("BNO055 confirmed at 0x%02X\n", (unsigned int)bno_address);
    }
    else {
        printf("BNO055 not detected\n");
    }
}



//
    void configure_bno055() {

    if (!bno_connected) {
        return;
    }

    // Enter configuration mode before changing settings
    i2c_write_register_byte(
        i2c0,
        bno_address,
        BNO_OPR_MODE_REG,
        BNO_MODE_CONFIG
    );

    sleep_ms(20);

    // Explicitly use m/s^2 for acceleration
    i2c_write_register_byte(
        i2c0,
        bno_address,
        BNO_UNIT_SEL_REG,
        0x00
    );

    // Start accelerometer-only mode
    i2c_write_register_byte(
        i2c0,
        bno_address,
        BNO_OPR_MODE_REG,
        BNO_MODE_ACCONLY
    );

    sleep_ms(10);
}



//
bool read_bno055_acceleration(float *x, float *y, float *z) {

    if (!bno_connected) {
        return false;
    }

    uint8_t data[6];

    if (!i2c_read_register_block(
            i2c0,
            bno_address,
            BNO_ACCEL_DATA_START_REG,
            data,
            6)) {

        return false;
    }

    int16_t raw_x =
        (int16_t)(((uint16_t)data[1] << 8) | data[0]);

    int16_t raw_y =
        (int16_t)(((uint16_t)data[3] << 8) | data[2]);

    int16_t raw_z =
        (int16_t)(((uint16_t)data[5] << 8) | data[4]);

    *x = raw_x / BNO_ACCEL_SCALE;
    *y = raw_y / BNO_ACCEL_SCALE;
    *z = raw_z / BNO_ACCEL_SCALE;

    return true;
}



//
bool read_bmp388_calibration() {

    if (!bmp_connected) {
        return false;
    }

    uint8_t data[BMP_CALIB_DATA_LENGTH];

    if (!i2c_read_register_block(
            i2c0,
            bmp_address,
            BMP_CALIB_DATA_START_REG,
            data,
            BMP_CALIB_DATA_LENGTH)) {

        return false;
    }

    bmp_calibration.par_t1 =
        (uint16_t)(((uint16_t)data[1] << 8) | data[0]);

    bmp_calibration.par_t2 =
        (uint16_t)(((uint16_t)data[3] << 8) | data[2]);

    bmp_calibration.par_t3 = (int8_t)data[4];

    bmp_calibration.par_p1 =
        (int16_t)(((uint16_t)data[6] << 8) | data[5]);

    bmp_calibration.par_p2 =
        (int16_t)(((uint16_t)data[8] << 8) | data[7]);

    bmp_calibration.par_p3 = (int8_t)data[9];
    bmp_calibration.par_p4 = (int8_t)data[10];

    bmp_calibration.par_p5 =
        (uint16_t)(((uint16_t)data[12] << 8) | data[11]);

    bmp_calibration.par_p6 =
        (uint16_t)(((uint16_t)data[14] << 8) | data[13]);

    bmp_calibration.par_p7 = (int8_t)data[15];
    bmp_calibration.par_p8 = (int8_t)data[16];

    bmp_calibration.par_p9 =
        (int16_t)(((uint16_t)data[18] << 8) | data[17]);

    bmp_calibration.par_p10 = (int8_t)data[19];
    bmp_calibration.par_p11 = (int8_t)data[20];

    bmp_calibration_loaded = true;

    return true;
}



//
bool configure_bmp388() {

    if (!bmp_connected || !bmp_calibration_loaded) {
        return false;
    }

    if (!i2c_write_register_byte(
            i2c0,
            bmp_address,
            BMP_OSR_REG,
            BMP_OSR_STANDARD)) {

        return false;
    }

    if (!i2c_write_register_byte(
            i2c0,
            bmp_address,
            BMP_ODR_REG,
            BMP_ODR_12_5_HZ)) {

        return false;
    }

    if (!i2c_write_register_byte(
            i2c0,
            bmp_address,
            BMP_PWR_CTRL_REG,
            BMP_PWR_NORMAL_PRESS_TEMP)) {

        return false;
    }

    return true;
}



//
bool read_bmp388_raw(
    uint32_t *raw_pressure,
    uint32_t *raw_temperature
) {

    if (!bmp_connected || !bmp_configured) {
        return false;
    }

    uint8_t data[BMP_DATA_LENGTH];

    if (!i2c_read_register_block(
            i2c0,
            bmp_address,
            BMP_DATA_START_REG,
            data,
            BMP_DATA_LENGTH)) {

        return false;
    }

    *raw_pressure =
        ((uint32_t)data[2] << 16) |
        ((uint32_t)data[1] << 8) |
        data[0];

    *raw_temperature =
        ((uint32_t)data[5] << 16) |
        ((uint32_t)data[4] << 8) |
        data[3];

    return true;
}



//
float compensate_bmp388_temperature(uint32_t raw_temperature) {

    float par_t1 =
        (float)bmp_calibration.par_t1 * 256.0f;

    float par_t2 =
        (float)bmp_calibration.par_t2 / 1073741824.0f;

    float par_t3 =
        (float)bmp_calibration.par_t3 / 281474976710656.0f;

    float partial1 =
        (float)raw_temperature - par_t1;

    float partial2 =
        partial1 * par_t2;

    bmp_t_lin =
        partial2 +
        (partial1 * partial1) * par_t3;

    return bmp_t_lin;
}



//
float compensate_bmp388_pressure(uint32_t raw_pressure) {

    float par_p1 =
        ((float)bmp_calibration.par_p1 - 16384.0f) / 1048576.0f;

    float par_p2 =
        ((float)bmp_calibration.par_p2 - 16384.0f) / 536870912.0f;

    float par_p3 =
        (float)bmp_calibration.par_p3 / 4294967296.0f;

    float par_p4 =
        (float)bmp_calibration.par_p4 / 137438953472.0f;

    float par_p5 =
        (float)bmp_calibration.par_p5 * 8.0f;

    float par_p6 =
        (float)bmp_calibration.par_p6 / 64.0f;

    float par_p7 =
        (float)bmp_calibration.par_p7 / 256.0f;

    float par_p8 =
        (float)bmp_calibration.par_p8 / 32768.0f;

    float par_p9 =
        (float)bmp_calibration.par_p9 / 281474976710656.0f;

    float par_p10 =
        (float)bmp_calibration.par_p10 / 281474976710656.0f;

    float par_p11 =
        (float)bmp_calibration.par_p11 / 36893488147419103232.0f;


    float t2 = bmp_t_lin * bmp_t_lin;
    float t3 = t2 * bmp_t_lin;

    float pressure = (float)raw_pressure;
    float pressure2 = pressure * pressure;
    float pressure3 = pressure2 * pressure;


    float partial_out1 =
        par_p5 +
        par_p6 * bmp_t_lin +
        par_p7 * t2 +
        par_p8 * t3;

    float partial_out2 =
        pressure *
        (
            par_p1 +
            par_p2 * bmp_t_lin +
            par_p3 * t2 +
            par_p4 * t3
        );

    float partial_out3 =
        pressure2 *
        (par_p9 + par_p10 * bmp_t_lin) +
        pressure3 * par_p11;

    return partial_out1 + partial_out2 + partial_out3;
}



//
float calculate_relative_altitude(
    float pressure_pa,
    float reference_pressure_pa
) {

    if (pressure_pa <= 0.0f || reference_pressure_pa <= 0.0f) {
        return 0.0f;
    }

    return 44330.0f *
        (1.0f -
        std::pow(
            pressure_pa / reference_pressure_pa,
            0.1903f
        ));
}

bool i2c_read_register_byte(i2c_inst_t *i2c, uint8_t device_address, uint8_t register_address, uint8_t *value) {
    int write_result = i2c_write_timeout_us(
        i2c,
        device_address,
        &register_address,
        1,
        true,
        I2C_TIMEOUT_US
    );

    if (write_result != 1) {
        return false;
    }

    int read_result = i2c_read_timeout_us(
        i2c,
        device_address,
        value,
        1,
        false,
        I2C_TIMEOUT_US
    );

    return read_result == 1;
}



bool i2c_write_register_byte(i2c_inst_t *i2c, uint8_t device_address, uint8_t register_address, uint8_t value) {
    uint8_t data[2] = {
        register_address,
        value
    };

    int result = i2c_write_timeout_us(
        i2c,
        device_address,
        data,
        2,
        false,
        I2C_TIMEOUT_US
    );

    return result == 2;
}



bool i2c_read_register_block(i2c_inst_t *i2c, uint8_t device_address, uint8_t start_register, uint8_t *buffer, size_t length) {
    int write_result = i2c_write_timeout_us(
        i2c,
        device_address,
        &start_register,
        1,
        true,
        I2C_TIMEOUT_US
    );

    if (write_result != 1) {
        return false;
    }

    int read_result = i2c_read_timeout_us(
        i2c,
        device_address,
        buffer,
        length,
        false,
        I2C_TIMEOUT_US
    );

    return read_result == (int)length;
}

void initialize_sensors() {

    initialize_i2c();

    sleep_ms(SENSOR_STARTUP_DELAY_MS);

    detect_sensors();

    if (bmp_connected) {
        if (read_bmp388_calibration()) {
            bmp_configured = configure_bmp388();
        }
    }

    configure_bno055();
}