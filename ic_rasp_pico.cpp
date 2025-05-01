#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "pico/bootrom.h"
#include "hardware/sync.h"

#define DEBOUNCE_TIME_US 200000
#define BUTTON 18
#define TRIG_PIN 17
#define ECHO_PIN 16

volatile uint64_t start_time = 0;
volatile uint64_t end_time = 0;
static volatile uint64_t last_press = 0;

bool measure_done = false;

uint32_t duration; 
float distance;

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
        gpio_set_irq_enabled(ECHO_PIN, GPIO_IRQ_EDGE_RISE, false);
        gpio_set_irq_enabled(ECHO_PIN, GPIO_IRQ_EDGE_FALL, true);
    }
    else if(gpio == ECHO_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
        end_time = to_us_since_boot(get_absolute_time());
        gpio_set_irq_enabled(ECHO_PIN, GPIO_IRQ_EDGE_FALL, false);
        gpio_set_irq_enabled(ECHO_PIN, GPIO_IRQ_EDGE_RISE, true);
        measure_done = true;
        
    }
}

void setup() {
    stdio_init_all();
    sleep_ms(2000); 

    gpio_init(BUTTON);
    gpio_set_dir(BUTTON, GPIO_IN);
    gpio_pull_up(BUTTON);

    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, 0);

    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    gpio_pull_down(ECHO_PIN);

    gpio_set_irq_enabled_with_callback(BUTTON, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(ECHO_PIN, GPIO_IRQ_EDGE_RISE, true,  &gpio_irq_handler);
}

int main() {
    setup();
    
    while (1) {
        gpio_put(TRIG_PIN, 1);
        sleep_us(10);
        gpio_put(TRIG_PIN, 0);

        if(measure_done) {
            duration = end_time - start_time;
            distance = (duration * 0.0343f) / 2.0f;
            printf("Distância: %.2f cm\n", distance);
            measure_done = false;
        }       

        sleep_ms(100); 
    }
}