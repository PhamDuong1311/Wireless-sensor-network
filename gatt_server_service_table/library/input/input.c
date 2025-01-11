#include "stdio.h"
#include "esp_log.h"
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "rom/gpio.h"
#include "input_header.h"

input_callback_t input_callback = NULL;

static void IRAM_ATTR gpio_input_hanler(void *arg){
          int gpio_num = (uint32_t) arg;
          input_callback(gpio_num);
          }


void input_io_create(gpio_num_t gpio_num, type_interrupt_e type){
     gpio_set_direction(gpio_num, GPIO_MODE_INPUT);
     gpio_set_pull_mode(gpio_num,GPIO_PULLUP_ONLY);
     gpio_set_intr_type(gpio_num,type);
     gpio_install_isr_service(0);
     gpio_isr_handler_add(gpio_num,gpio_input_hanler,(void*) gpio_num);
}

void input_callback_register(void *cb){
    input_callback = cb;
}