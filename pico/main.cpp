#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include <cmath>
/*
=================================================================================================================================================================
FUNCTION PROTOTYPES
=================================================================================================================================================================
*/

//Arm controls
void update_joint_positions(float joint0_command, float joint1_command, float joint2_command, float joint3_command, float clawOpen_command, float clawClose_command, float delta_time);
void joint1_update(float command, float delta_time);
void joint2_update(float command, float delta_time);
void joint3_update(float command, float delta_time);
void claw_update(float clawOpen_command, float clawClose_command, float delta_time);

//Servo controls
void initialize_servo(uint gpio);
void set_servo_position(uint gpio, float angle, float servo_max_angle);
void initialize_joint0_motor();
void set_joint0_motor(float command);


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
void scan_i2c_bus(i2c_inst_t* i2c, const char* bus_name);

//Feedback
void send_feedback();

//Utility
float clamp_value(float value, float min_value, float max_value);

/*
=================================================================================================================================================================
GPIO Configuration
=================================================================================================================================================================
*/

//Axis0 DC Motor
const uint JOINT0_IN1_GPIO = 11;
const uint JOINT0_IN2_GPIO = 14;
const uint JOINT0_PWM_GPIO = 15;
const uint JOINT1_GPIO = 18;
const uint JOINT2_GPIO = 19;
const uint JOINT3_GPIO = 20;
const uint CLAW_GPIO = 6;

//BMP388
const uint BMP_SDA_GPIO = 8;
const uint BMP_SCL_GPIO = 9;

//BNO055
const uint BNO_SDA_GPIO = 2;
const uint BNO_SCL_GPIO = 3;

/*
=================================================================================================================================================================
Sensor Configuration
=================================================================================================================================================================
*/

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

/*
=================================================================================================================================================================
Arm State and Control Configuration
=================================================================================================================================================================
*/

//Current commanded target positions
float joint0_motor_command = 0.0f;
float joint1_target = 135.0;
float joint2_target = 135.0;
float joint3_target = 90.0;
float claw_target = 90.0;

//Maximum commanded motion speed in degrees per second
const float JOINT1_SPEED_DPS = 30.0f; //provisional value, will need to be tuned based on actual joint speed and desired responsiveness
const float JOINT2_SPEED_DPS = 30.0f; //provisional value, will need to be tuned based on actual joint speed and desired responsiveness
const float JOINT3_SPEED_DPS = 30.0f; //provisional value, will need to be tuned based on actual joint speed and desired responsiveness
const float CLAW_SPEED_DPS   = 30.0f; //provisional value, will need to be tuned based on actual joint speed and desired responsiveness

const float JOINT1_SERVO_MAX_ANGLE = 270.0f;
const float JOINT2_SERVO_MAX_ANGLE = 270.0f;
const float JOINT3_SERVO_MAX_ANGLE = 180.0f;
const float CLAW_SERVO_MAX_ANGLE   = 180.0f;

/*
=================================================================================================================================================================
Timing and Communication Configuration
=================================================================================================================================================================
*/

// Controller command input
const size_t COMMAND_BUFFER_SIZE = 100;
const uint64_t COMMAND_TIMEOUT_US = 250000;   // 250 ms

// Main control loop
const uint64_t CONTROL_INTERVAL_US = 20000;   // 20 ms = 50 Hz

// Telemetry and sensor update rates
const uint64_t SENSOR_INTERVAL_US = 100000;   // 100 ms = 10 Hz
const uint64_t FEEDBACK_INTERVAL_US = 100000; // 100 ms = 10 Hz

// I2C communication
const uint32_t I2C_FREQUENCY_HZ = 100000;     // 100 kHz
const uint32_t I2C_TIMEOUT_US = 2000;          // 2 ms

//I2C startup
const uint32_t SENSOR_STARTUP_DELAY_MS = 700;

//Values passed
const int CONTROLLER_COMMAND_VALUES = 6;




const float JOINT0_PWM_CLOCK_DIV = 1.0f;
const uint16_t JOINT0_PWM_WRAP = 6249;













int main() {
    stdio_init_all();
    initialize_i2c();

    sleep_ms(SENSOR_STARTUP_DELAY_MS); // Wait for I2C devices to power up
    detect_sensors(); //launch function below

    if (bmp_connected) {
        if (read_bmp388_calibration()) {
            bmp_configured = configure_bmp388();
        }
    }   

    configure_bno055();

    sleep_ms(3000);

    scan_i2c_bus(i2c0, "i2c0");
    scan_i2c_bus(i2c1, "i2c1");

    initialize_joint0_motor();
    initialize_servo(JOINT1_GPIO);
    initialize_servo(JOINT2_GPIO);
    initialize_servo(JOINT3_GPIO);
    initialize_servo(CLAW_GPIO);

    //enter safe state
    set_servo_position(JOINT1_GPIO, joint1_target, JOINT1_SERVO_MAX_ANGLE);
    set_servo_position(JOINT2_GPIO, joint2_target, JOINT2_SERVO_MAX_ANGLE);
    set_servo_position(JOINT3_GPIO, joint3_target, JOINT3_SERVO_MAX_ANGLE);
    set_servo_position(CLAW_GPIO, claw_target, CLAW_SERVO_MAX_ANGLE);

    /*
    while (true) {
        printf("Data from Pico!\n");
        sleep_ms(1000);
    }
    */

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
    
    
    while (true) {
        int command = getchar_timeout_us(0);
        if (command != PICO_ERROR_TIMEOUT) {
            if (command == '\n') {
                command_buffer[buffer_index] = '\0'; // Null-terminate the string

                if (!buffer_overflow) {
                    float new_joint0, new_joint1, new_joint2, new_joint3, new_clawOpen, new_clawClose;
                
                    //printf("Received command: %s\n", command_buffer);

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

                        joint0_command = clamp_value(new_joint0, -1.0f, 1.0f);
                        joint1_command = clamp_value(new_joint1, -1.0f, 1.0f);
                        joint2_command = clamp_value(new_joint2, -1.0f, 1.0f);
                        joint3_command = clamp_value(new_joint3, -1.0f, 1.0f);
                        clawOpen_command = clamp_value(new_clawOpen, 0.0f, 1.0f);
                        clawClose_command = clamp_value(new_clawClose, 0.0f, 1.0f);
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
                if (!bmp_altitude_reference_set && bmp_pressure_pa > 0.0f) {

                    bmp_reference_pressure_pa = bmp_pressure_pa;
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

            last_sensor_time = time_us_64();
        }
        if (time_us_64() - last_feedback_time >= FEEDBACK_INTERVAL_US) { // 100ms
                send_feedback();
                last_feedback_time = time_us_64();
        }
    }

    return 0;
}








/*
=================================================================================================================================================================
Functions
=================================================================================================================================================================
*/



//Starts up sensors, sets there values
void initialize_i2c() {
    //BMP bus
    i2c_init(i2c0, I2C_FREQUENCY_HZ);
    gpio_set_function(BMP_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(BMP_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(BMP_SDA_GPIO);
    gpio_pull_up(BMP_SCL_GPIO);

    //BNO bus
    i2c_init(i2c1, I2C_FREQUENCY_HZ);
    gpio_set_function(BNO_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(BNO_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(BNO_SDA_GPIO);
    gpio_pull_up(BNO_SCL_GPIO);
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

    // Check BNO055 on I2C1
    if (i2c_read_register_byte(i2c1, BNO_ADDRESS_1, BNO_CHIP_ID_REG, &chip_id)
        && chip_id == BNO_EXPECTED_ID) {

        bno_address = BNO_ADDRESS_1;
        bno_connected = true;
        printf("BNO055 confirmed at 0x%02X\n", (unsigned int)bno_address);
    }
    else if (i2c_read_register_byte(i2c1, BNO_ADDRESS_2, BNO_CHIP_ID_REG, &chip_id)
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
        i2c1,
        bno_address,
        BNO_OPR_MODE_REG,
        BNO_MODE_CONFIG
    );

    sleep_ms(20);

    // Explicitly use m/s^2 for acceleration
    i2c_write_register_byte(
        i2c1,
        bno_address,
        BNO_UNIT_SEL_REG,
        0x00
    );

    // Start accelerometer-only mode
    i2c_write_register_byte(
        i2c1,
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
            i2c1,
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

    command = clamp_value(command, -1.0f, 1.0f);

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
    
    //printf("Updating joints: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", joint0, joint1, joint2, joint3, clawOpen, clawClose);

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



void send_feedback() {
    printf(
        "TEL,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
        joint0_motor_command,
        joint1_target,
        joint2_target,
        joint3_target,
        claw_target,
        bno_accel_x,
        bno_accel_y,
        bno_accel_z,
        bmp_temperature_c,
        bmp_pressure_pa,
        bmp_altitude_m
    );
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



float clamp_value(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}



//
void scan_i2c_bus(i2c_inst_t* i2c, const char* bus_name)
{
    printf("Scanning %s...\n", bus_name);

    for (int address = 0x08; address <= 0x77; address++)
    {
        uint8_t data;

        int result = i2c_read_timeout_us(
            i2c,
            address,
            &data,
            1,
            false,
            2000
        );

        if (result >= 0)
        {
            printf("%s found device at 0x%02X\n",
                   bus_name,
                   address);
        }
    }
}