#ifndef __INPUT_HEADER_H_
#define __INPUT_HEADER_H_
#include "driver/gpio.h"


typedef void (*input_callback_t )(int);

typedef enum {
    GPIO_INTR_rising= 1,     /*!< GPIO interrupt type : rising edge                  */
    GPIO_INTR_falling = 2,     /*!< GPIO interrupt type : falling edge                 */
    GPIO_INTR_ANY = 3,     /*!< GPIO interrupt type : both rising and falling edge */
} type_interrupt_e;

void input_io_create(gpio_num_t gpio_num, type_interrupt_e type);
void input_get_level(gpio_num_t gpio_num );
void input_callback_register(void *cb);

#endif