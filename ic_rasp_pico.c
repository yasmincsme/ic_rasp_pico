#include <stdio.h>
#include "pico/stdlib.h"

#define TRIGGER 19
#define ECHO 18

void setup_all() {
    gpio_init(TRIGGER);
    gpio_set_dir(TRIGGER, GPIO_OUT);

    gpio_init(ECHO);
    gpio_set_dir(ECHO, GPIO_IN);
}

uint32_t ultrasonic() {
    uint32_t signal_off, singal_on, time_passed = 0;

    gpio_put(TRIGGER, 0);
    sleep_us(2);
    gpio_put(TRIGGER, 1);
    sleep_us(10);
    gpio_put(TRIGGER, 0);

    while (gpio_get(ECHO) == 0) {
        signal_off = time_us_32();
    }
    
    while (gpio_get(ECHO) == 1) {
        singal_on = time_us_32();
    } 
    
    time_passed = singal_on - signal_off;
    return time_passed;
    
}

int main() {
  stdio_init_all();
  setup_all();

  uint32_t measured_time;

  while (true) {
    
    sleep_ms(1000);
  }
}