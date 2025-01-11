#ifndef __DHT22_HEADER_H_
#define __DHT22_HEADER_H_

#include "esp_err.h"

void DHT22_de_init(void);
esp_err_t DHT22_read_data(float *temperature, float *humidity);
void DHT22_init(void);

#endif