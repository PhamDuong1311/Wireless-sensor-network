set(pre_req DHT22 driver esp_adc)
idf_component_register(SRCS "DHT22.c"
                    INCLUDE_DIRS "."
                    REQUIRES ${pre_req})
                    