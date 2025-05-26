#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "pico/bootrom.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include <math.h>

#define DEBOUNCE_TIME_US 200000
#define BUTTON 18
#define TRIG_PIN 17
#define ECHO_PIN 16
#define IR_SENSOR 26


#define MPU6050 0x68
#define SDA 4
#define SCL 5
#define I2C i2c0

volatile uint64_t start_time = 0;
volatile uint64_t end_time = 0;
static volatile uint64_t last_press = 0;

static uint32_t adc_value= 0; 

bool measure_done = false;

uint32_t duration; 
float distance;

void sensor_read(uint32_t* adc_value) {
    adc_select_input(0); // Canal 0 para GPIO 26
    *adc_value = adc_read();
}

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

void mpu6050_read_gyro(int16_t* gx, int16_t* gy, int16_t* gz) {
    uint8_t reg = 0x43;
    uint8_t buffer[6];

    i2c_write_blocking(I2C, MPU6050, &reg, 1, true);
    i2c_read_blocking(I2C, MPU6050, buffer, 6, false);

    *gx = (buffer[0] << 8) | buffer[1];
    *gy = (buffer[2] << 8) | buffer[3];
    *gz = (buffer[4] << 8) | buffer[5];
}

void gpio_irq_handler(uint gpio, uint32_t events) {
    uint32_t current_time = to_us_since_boot(get_absolute_time());

    if (gpio == BUTTON && (events & GPIO_IRQ_EDGE_FALL)) {
        if (current_time - last_press > DEBOUNCE_TIME_US) {
            last_press = current_time;
            reset_usb_boot(0, 0); 
        }
    } 
    else if (gpio == ECHO_PIN && (events & GPIO_IRQ_EDGE_RISE)) {
        start_time = to_us_since_boot(get_absolute_time());
        end_time = 0;
    }
    else if(gpio == ECHO_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
        end_time = to_us_since_boot(get_absolute_time());
        duration = end_time - start_time;
        distance = (duration * 0.0343f) / 2.0f;
        //printf("Distância: %.2f cm\n", distance);
        measure_done = true;
        
    }
}

int64_t trig_off_callback(alarm_id_t id, void *user_data) {
    gpio_put(TRIG_PIN, 0);
    return 0; 
}

bool trig_on_callback(struct repeating_timer *t) {
    gpio_put(TRIG_PIN, 1);
    add_alarm_in_us(10, trig_off_callback, NULL, false);
    return true;  
}

bool monitor_timer_callback(struct repeating_timer *t) {

    //usar struct
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float x_angle, y_angle;

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

    sensor_read(&adc_value);
    float tensao = adc_value * 3.3 / 4095.0;
    float distancia_cm = (27.86 * (1.0 / tensao)) - 0.42;

    if(distancia_cm < 10) distancia_cm = 10;   // limite inferior
    if(distancia_cm > 80) distancia_cm = 80;   // limite superior

    printf("Distância IR: %.2f cm\n", distancia_cm);


    if(measure_done) {
            printf("Distância: %.2f cm\n", distance);
            measure_done = false;
    } 
    return true;  
}

void setup() {
    gpio_init(BUTTON);
    gpio_set_dir(BUTTON, GPIO_IN);
    gpio_pull_up(BUTTON);

    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, 0);

    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    gpio_pull_down(ECHO_PIN);

    adc_init();
    adc_gpio_init(IR_SENSOR);

    gpio_set_irq_enabled_with_callback(BUTTON, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(ECHO_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true,  &gpio_irq_handler);

}

int main() {
    stdio_init_all();
    setup();
    i2C_init();
    mpu6050_init();

    uint32_t measured_time;
    float distance_cm;

    struct repeating_timer trigger_timer;
    struct repeating_timer monitor_timer;

    add_repeating_timer_us(-100000, trig_on_callback, NULL, &trigger_timer);
    add_repeating_timer_us(-1000000, monitor_timer_callback, NULL, &monitor_timer);

    while (1) {      

    }
}