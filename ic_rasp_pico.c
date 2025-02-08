#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define TRIGGER 19
#define ECHO 18

#define MPU6050 0x68
#define SDA 20
#define SCL 21
#define I2C i2c0

void i2C_init() {
    i2c_init(I2C, 400 * 1000);
    gpio_set_function(SDA, GPIO_FUNC_I2C);
    gpio_set_function(SCL, GPIO_FUNC_I2C);
    gpio_pull_up(SDA);
    gpio_pull_up(SCL);
}

void mpu6050_init() {
    uint8_t buffer[2];

    buffer[0] = 0x6B;  //registrador de energia
    buffer[1] = 0x00; //acorda o sensor

    i2c_write_blocking(I2C, MPU6050, buffer, 2, false);
}

void mpu6050_read_accel(int16_t* ax, int16_t* ay, int16_t* az) {
    uint8_t reg = 0x3B;
    uint8_t buffer[6];

    i2c_write_blocking(I2C, MPU6050, &reg, 1, true);
    i2c_read_blocking(I2C, MPU6050, buffer, 6, false);

    *ax = (buffer[0] << 8) | buffer[1];
    *ay = (buffer[2] << 8) | buffer[3];
    *az = (buffer[4] << 8) | buffer[5];
}

void mpu6050_read_gyro(int16_t* gx, int16_t* gy, int16_t* gz) {
    uint8_t reg = 0x43;
    uint8_t buffer[6];

    i2c_write_blocking(I2C, MPU6050, &reg, 1, true);
    i2c_read_blocking(I2C, MPU6050, buffer, 6, false);

    *gx = (buffer[0] << 8) | buffer[1];
    *gy = (buffer[2] << 8) | buffer[3];
    *gz = (buffer[4] << 8) | buffer[5];
}

float hypotenuse(float a, float b) {
    return sqrt((a*a) + (b*b));

}

float get_x_rotation(float x, float y, float z) {
    float radians = atan2(y, hypotenuse(x, z));
    return radians * (180.0 / M_PI);
}

float get_y_rotation(float x, float y, float z) {
    float radians = atan2(x, hypotenuse(y, z));
    return - radians * (180.0 / M_PI);
}

void hcsr04_init() {
    gpio_init(TRIGGER);
    gpio_set_dir(TRIGGER, GPIO_OUT);

    gpio_init(ECHO);
    gpio_set_dir(ECHO, GPIO_IN);
}

uint32_t hcsr04_read() {
    uint32_t signal_off, signal_on, time_passed = 0;

    gpio_put(TRIGGER, 0);
    sleep_us(2);
    gpio_put(TRIGGER, 1);
    sleep_us(10);
    gpio_put(TRIGGER, 0);

    while (gpio_get(ECHO) == 0) {
        signal_off = time_us_32();
    }
    
    while (gpio_get(ECHO) == 1) {
        signal_on = time_us_32();
    } 
    
    time_passed = signal_on - signal_off;
    return time_passed;
    
}

int main() {
    stdio_init_all();

    i2C_init();
    mpu6050_init();
    hcsr04_init();

    uint32_t measured_time;
    float distance_cm;

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float x_angle, y_angle;

    while (true) {
        mpu6050_read_accel(&ax, &ay, &az);
        mpu6050_read_gyro(&gx, &gy, &gz);

        accel_x = ax/16384.0;
        accel_y = ay/16384.0;
        accel_z = az/16384.0;

        gyro_x = gx/131.0;
        gyro_y = gy/131.0;
        gyro_z = gz/131.0;

        x_angle = get_x_rotation(accel_x, accel_y, accel_z);
        y_angle = get_y_rotation(accel_x, accel_y, accel_z);

        printf("Acelerômetro (g): X=%.2f Y=%.2f Z=%.2f\n", accel_x, accel_y, accel_z);
        printf("Giroscópio (°/s): X=%.2f Y=%.2f Z=%.2f\n", gyro_x, gyro_y, gyro_z);
        printf("Inclinação: X=%.2f° Y=%.2f°\n\n", x_angle, y_angle);

        measured_time = hcsr04_read();
        distance_cm = (measured_time * 0.0343) / 2;
        printf("distance_cm: %.2f\n", distance_cm);
        
        sleep_ms(1000);
    }
    }