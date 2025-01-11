idf_component_register(SRCS "gattc_multi_connect.c"
                    INCLUDE_DIRS "." "../library/DHT22" "../library/input"
                    REQUIRES esp_wifi mqtt nvs_flash input bt)
