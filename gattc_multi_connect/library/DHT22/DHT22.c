// DHT22.c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "DHT22_header.h"

// Sử dụng cùng chân GPIO với LM35 để tương thích
#define DHT_GPIO GPIO_NUM_4  
static const char *TAG = "DHT22";

// Cấu hình chân GPIO
void DHT22_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL<<DHT_GPIO),
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,  // Đảm bảo pull-up được bật
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(DHT_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(1000)); // Thêm delay khởi tạo
    ESP_LOGI(TAG, "DHT22 initialized on GPIO %d", DHT_GPIO);
}

// Tạo tín hiệu bắt đầu giao tiếp với DHT22
static bool dht22_start_signal(void) {
    ESP_LOGD("DHT22", "Sending start signal...");
    
    gpio_set_direction(DHT_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(20));  // Pull down for 20ms
    gpio_set_level(DHT_GPIO, 1);
    esp_rom_delay_us(40);
    gpio_set_direction(DHT_GPIO, GPIO_MODE_INPUT);
    
    // Wait for DHT22 response
    int timeout = 0;
    while(gpio_get_level(DHT_GPIO) == 1) {
        if(timeout++ > 200) {
            ESP_LOGW("DHT22", "No response - line still high");
            return false;
        }
        esp_rom_delay_us(1);
    }
    
    ESP_LOGD("DHT22", "Got initial response");
    
    timeout = 0;
    while(gpio_get_level(DHT_GPIO) == 0) {
        if(timeout++ > 200) {
            ESP_LOGW("DHT22", "No response - line stuck low");
            return false;
        }
        esp_rom_delay_us(1);
    }
    
    ESP_LOGD("DHT22", "Start signal complete");
    return true;
}

// Đọc dữ liệu từ DHT22
static esp_err_t dht22_read_bits(uint8_t *data, size_t length) {
    uint8_t current_byte = 0;
    uint8_t bit_count = 0;
    
    for(int i = 0; i < length * 8; i++) {
        // Đợi cạnh lên
        int timeout = 0;
        while(gpio_get_level(DHT_GPIO) == 0) {
            if(timeout++ > 100) return ESP_ERR_TIMEOUT;
            esp_rom_delay_us(1);
        }
        
        // Đo độ rộng xung cao (26-28us = 0, 70us = 1)
        esp_rom_delay_us(40);  // Đợi và lấy mẫu sau 40us
        current_byte <<= 1;
        if(gpio_get_level(DHT_GPIO)) {
            current_byte |= 1;
        }
        
        bit_count++;
        if(bit_count == 8) {
            data[i/8] = current_byte;
            bit_count = 0;
            current_byte = 0;
        }
        
        // Đợi cạnh xuống
        timeout = 0;
        while(gpio_get_level(DHT_GPIO) == 1) {
            if(timeout++ > 100) return ESP_ERR_TIMEOUT;
            esp_rom_delay_us(1);
        }
    }
    
    return ESP_OK;
}

// Giải phóng DHT22
void DHT22_de_init(void) {
    // Không cần thao tác đặc biệt để giải phóng
}

// Đọc nhiệt độ và độ ẩm từ DHT22
esp_err_t DHT22_read_data(float *temperature, float *humidity) {
    const int MAX_RETRIES = 3;  // Số lần thử lại tối đa
    
    for(int retry = 0; retry < MAX_RETRIES; retry++) {
        uint8_t data[5] = {0};
        
        if (!dht22_start_signal()) {
            ESP_LOGW(TAG, "Retry %d: Failed to start signal", retry + 1);
            vTaskDelay(pdMS_TO_TICKS(1000)); // Đợi 1s trước khi thử lại
            continue;
        }
        
        esp_err_t err = dht22_read_bits(data, 5);
        if (err == ESP_OK) {
            // Kiểm tra checksum
            if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
                *humidity = ((data[0] << 8) + data[1]) / 10.0;
                int16_t temp16 = (data[2] << 8) + data[3];
                *temperature = temp16 / 10.0;
                return ESP_OK;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    return ESP_ERR_TIMEOUT;
}