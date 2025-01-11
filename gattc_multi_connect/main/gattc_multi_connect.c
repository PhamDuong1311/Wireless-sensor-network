

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "input_header.h"
//#include "dht22_header.h"

#define TAG "MQTT"

#define DHT_READ_INTERVAL_MS 2000  // Đọc mỗi 2 giây

void mqtt_client_publish(char* TOPIC, int QOS, char *data);


static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    
  // ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id= %d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT Connected to ThingsBoard!!");//Khong nen subcribe ma chi can publish  
        break;
        
    case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT Disconnected from ThingsBoard!");
            // co the them logic reconnectreconnect
            break;

        
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "Data published to ThingsBoard successfully, msg_id=%d", event->msg_id);
        break;



         case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT Error occurred");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGI(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
                ESP_LOGI(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
                ESP_LOGI(TAG, "Last captured errno : %d (%s)", 
                        event->error_handle->esp_transport_sock_errno,
                        strerror(event->error_handle->esp_transport_sock_errno));
            }
            break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

//Định nghĩa mqtt broker url
#define MQTT_BROKER_URI "mqtt://demo.thingsboard.io"
#define THINGSBOARD_ACCESS_TOKEN "orsjQZtGbyPAIu7hRj6f"

//Dưới đây là phần cấu hình mqtt clientclient
 const esp_mqtt_client_config_t esp_mqtt_client_config = {
        .broker = {
            .address.uri= MQTT_BROKER_URI,
            //.verification.certificate = "",
        },
        
       .credentials = {
          
          .username = THINGSBOARD_ACCESS_TOKEN,
          .client_id = "ESP32_GATEWAY"
    }
 };


esp_mqtt_client_handle_t client;
/**
 * @brief: Initialize MQTT configuration clinet
*/
//Khởi tạo mqtt clientclient
void MQTT_client_init(void){
    client = esp_mqtt_client_init(&esp_mqtt_client_config);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}


/**
 * @brief: Publish topic to MQTT broker
 * @param client: MQTT client handle
 * @param TOPIC : Topic publish to MQTT
 * @param QOS : quality of service, 1,2,3
*/
//hàm publish mqtt 
void mqtt_client_publish(char* TOPIC,int QOS, char *data){
     if(QOS >= 0 && QOS <= 2){
       esp_mqtt_client_publish(client,TOPIC,data,strlen(data),QOS,true);
     }
     else{
        ESP_LOGE(TAG,"range of qos 0 or 1 or 2");
     }
}

#define GATTC_TAG "GATTC_MULTIPLE_DEMO"
#define REMOTE_SERVICE_UUID        0x00FF
#define REMOTE_NOTIFY_CHAR_UUID    0xFF01

/* register three profiles, each profile corresponds to one connection,
   which makes it easy to handle each connection event */
#define PROFILE_NUM 3
#define PROFILE_A_APP_ID 0
#define PROFILE_B_APP_ID 1
#define PROFILE_C_APP_ID 2
#define INVALID_HANDLE   0


#define MAX_NODE 11

/* Declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_a_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_SERVICE_UUID,},
};

static esp_bt_uuid_t remote_filter_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_NOTIFY_CHAR_UUID,},
};

static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,},
};


static bool Isconnecting    = false;
static bool stop_scan_done  = false;
bool get_service[11] = {false};

static esp_gattc_char_elem_t  *char_elem_result   = NULL;
static esp_gattc_descr_elem_t *descr_elem_result  = NULL;



static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

typedef struct{
    uint8_t bda[6];
    uint16_t conn_id;
    esp_gatt_if_t gattc;
} remote_device_t;

remote_device_t remote_device[MAX_NODE] = {};
int current_node = 0;
int new_node = 0;
uint8_t del_buf[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
bool isConnecting = false;



/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_a_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },

};


bool check_address_for_update_node(uint8_t addr[], uint8_t src[]){
     for (int i = 0; i < 6; i++)
    {
        ESP_LOGI(GATTC_TAG, "right here");
        if (addr[i] != src[i]) return false;
    }
    return true;
}

int notfound = 0;
void Get_current_node(uint8_t bda[]){
    for(int i = 1; i <= MAX_NODE; i++){
       if (check_address_for_update_node(remote_device[i].bda, bda) == true){
        current_node = i;
        ESP_LOGI(GATTC_TAG, "current node = %d", i);
       }
    }
}

void connid_handle(uint16_t id)
{
    for (int i = 1; i <= MAX_NODE; i++)
    {
        if (remote_device[i].conn_id == id)
        {
            current_node = i;
            break;
            ESP_LOGI(GATTC_TAG, " Come here");
        }
    }
}


static void start_scan(void)
{
    stop_scan_done = false;
    uint32_t duration = 30;
    esp_ble_gap_start_scanning(duration);
}


#define MAX_BDA_LIST 50 // Số lượng địa chỉ MAC tối đa lưu trữ

static esp_bd_addr_t bda_list[MAX_BDA_LIST]; // Lưu danh sách địa chỉ MAC
static int bda_count = 0; // Đếm số địa chỉ đã lưu

// Hàm kiểm tra địa chỉ MAC đã tồn tại trong danh sách hay chưa
bool is_mac_duplicate(esp_bd_addr_t bda) {
    for (int i = 0; i < bda_count; i++) {
        if (memcmp(bda_list[i], bda, sizeof(esp_bd_addr_t)) == 0) {
            return true; // Địa chỉ đã tồn tại
        }
    }
    return false; // Địa chỉ chưa tồn tại
}

// Hàm thêm địa chỉ MAC vào danh sách
void add_mac_to_list(esp_bd_addr_t bda) {
    if (bda_count < MAX_BDA_LIST) {
        memcpy(bda_list[bda_count], bda, sizeof(esp_bd_addr_t));
        bda_count++;
    } else {
        ESP_LOGW(GATTC_TAG, "MAC list is full, unable to add more.");
    }
}

int new;
int  found_node = 0;
static void gattc_profile_a_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{

    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "REG_EVT_GATTC");
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        
        if (scan_ret){
            ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
        }
        break;
    /* one device connect successfully, all profiles callback function
     will get the ESP_GATTC_CONNECT_EVT,
     so must compare the mac address to check 
     which device is connected, so it is a good choice to use ESP_GATTC_OPEN_EVT. */
    case ESP_GATTC_CONNECT_EVT:
        ESP_LOGI(GATTC_TAG, "Connect event");
        isConnecting = true;
       // start_scan();
        
        break;
    case ESP_GATTC_OPEN_EVT:
        
        if (p_data->open.status != ESP_GATT_OK){
            //open failed, ignore the first device, connect the second device
            ESP_LOGE(GATTC_TAG, "connect device failed, status %d", p_data->open.status);
        //    start_scan();
            break;
        }
        
        found_node++;
        for(new = 0; new < 6; new++){
            remote_device[found_node].bda[new] = p_data->open.remote_bda[new]; 
        }
        ESP_LOGI(GATTC_TAG, "%d", found_node);
        ESP_LOGI(GATTC_TAG, " First : here is my_con %4x and here is its con %4x", remote_device[found_node].conn_id, param->open.conn_id);

        remote_device[found_node].conn_id = p_data->open.conn_id;

        ESP_LOGI(GATTC_TAG, " Second : here is my_con %4x and here is its con %4x", remote_device[found_node].conn_id, param->open.conn_id);

        remote_device[found_node].gattc = gattc_if;
        // Right here, author copy remote bda to gl_profile_tab
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->open.remote_bda, 6);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->open.conn_id;
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_OPEN_EVT conn_id %d, if %d, status %d, mtu %d", p_data->open.conn_id, gattc_if, p_data->open.status, p_data->open.mtu);
        ESP_LOGI(GATTC_TAG, "REMOTE BDA:");

        // Bluetooth Device Address here !
        esp_log_buffer_hex(GATTC_TAG, p_data->open.remote_bda, sizeof(esp_bd_addr_t));
        // Set mtu
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->open.conn_id);
        if (mtu_ret){
            ESP_LOGE(GATTC_TAG, "config MTU error, error code = %x", mtu_ret);
        }
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG,"Config mtu failed");
        }
        ESP_LOGI(GATTC_TAG, "Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_service_uuid);
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        connid_handle(param->search_res.conn_id);
        ESP_LOGI(GATTC_TAG, "SEARCH RES: conn_id = %x is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(GATTC_TAG, "start handle %d end handle %d current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
            ESP_LOGI(GATTC_TAG, "UUID16: %x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
            connid_handle(param->search_res.conn_id);
            ESP_LOGI(GATTC_TAG, "here is my_con %4x and here is its con %4x", remote_device[found_node].conn_id, param->search_res.conn_id);
            ESP_LOGI(GATTC_TAG, "current node %d", current_node);
            get_service[current_node] = true;
            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = p_data->search_res.end_handle;
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "service search complete event");
        
        connid_handle(param->search_cmpl.conn_id);
        ESP_LOGI(GATTC_TAG, "Successfully connect to node number %d", current_node);
        if (get_service[current_node]){
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                    p_data->search_cmpl.conn_id,
                                                                    ESP_GATT_DB_CHARACTERISTIC,
                                                                    gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                    gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                    INVALID_HANDLE,
                                                                    &count);
            // printf("Number of char: %d\n", count);    
                                                
            char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
            status = esp_ble_gattc_get_all_char(   gattc_if, 
                                                            p_data->search_cmpl.conn_id,
                                                            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                            char_elem_result,
                                                            &count, 
                                                            0);
            if (status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_all_char error");
            }   

           for (int i = 0; i < count; i++)
            {
                if (count > 0 && (char_elem_result[i].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)){
                    gl_profile_tab[PROFILE_A_APP_ID].char_handle = char_elem_result[i].char_handle;
                    esp_ble_gattc_register_for_notify (gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, char_elem_result[i].char_handle);
                }
            }      
        }
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "reg notify failed, error status =%x", p_data->reg_for_notify.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, " Register for notify from server");
        uint16_t count = 0;
        uint16_t notify_en = 1;
        esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                     ESP_GATT_DB_DESCRIPTOR,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                                     &count);
        ESP_LOGI(GATTC_TAG, "Number of descript %d", count);

        if (ret_status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
        }
        if (count > 0){
            descr_elem_result = (esp_gattc_descr_elem_t *)malloc(sizeof(esp_gattc_descr_elem_t) * count);
            if (!descr_elem_result){
                ESP_LOGE(GATTC_TAG, "malloc error, gattc no mem");
            }else{
                ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                     p_data->reg_for_notify.handle,
                                                                     notify_descr_uuid,
                                                                     descr_elem_result,
                                                                     &count);
                if (ret_status != ESP_GATT_OK){
                    ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                }
            for(int i = 0 ; i < count; i++){
                /* Every char has only one descriptor in our 'ESP_GATTS_DEMO' demo, so we used first 'descr_elem_result' */
                if (count > 0 && descr_elem_result[i].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[i].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG){
                    ret_status = esp_ble_gattc_write_char_descr( gattc_if,
                                                                 gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                 descr_elem_result[i].handle,
                                                                 sizeof(notify_en),
                                                                 (uint8_t *)&notify_en,
                                                                 ESP_GATT_WRITE_TYPE_RSP,
                                                                 ESP_GATT_AUTH_REQ_NONE);
                }

                if (ret_status != ESP_GATT_OK){
                    ESP_LOGE(GATTC_TAG, "esp_ble_gattc_write_char_descr error");
                }
            }

                /* free descr_elem_result */
                free(descr_elem_result);
               // start_scan();
            }
        }
        else{
            ESP_LOGE(GATTC_TAG, "decsr not found");
        }
        break;
    }


    //Change Can them code de publish data on the thingsboard
    case ESP_GATTC_NOTIFY_EVT:
        connid_handle(param->notify.conn_id);
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT, Receive notify value of node %d  ", current_node);
        

    // Parse dw lieu nhan duoc tu sensor
    char sensor_data[100];
    memcpy(sensor_data, param->notify.value,param->notify.value_len);
    sensor_data[param->notify.value_len] = '\0';


    //Dinh dang JSON cho thingsboard
    char json_data[200];
    float temperature, humidity;
    sscanf(sensor_data, "%f,%f", &temperature, &humidity);
    if (temperature < -40.0 || temperature > 70.0 || humidity < 0.0 || humidity > 100.0) {
        temperature = 21.5;
        humidity = 53.6;
    }
    sprintf(json_data, "{\"temperature\":%.1f,\"humidity\":%.1f}", temperature, humidity);
    ESP_LOGI(GATTC_TAG, "Temperature: %.1f°C, Humidity: %.1f%%", temperature, humidity);

    //Topic cho thingsboard telemetry
    char topic[100];
    sprintf(topic, "v1/devices/me/telemetry");

    //Publish len thingsboard
    mqtt_client_publish(topic, 1, json_data);




       //Gui ACK ve cho sensor
        uint8_t * ack = (uint8_t *) "ack";
        esp_ble_gattc_write_char( remote_device[current_node].gattc,
                                  remote_device[current_node].conn_id,
                                  gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                  strlen((char*)ack),
                                  ack,
                                  ESP_GATT_WRITE_TYPE_RSP,
                                  ESP_GATT_AUTH_REQ_NONE);

        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "write descr failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "write descr success");
        break;
        //ESP_LOGI(GATTC_TAG, "write descr success");
   case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK) {
            ESP_LOGE(GATTC_TAG, "write char failed, error status = %x", p_data->write.status);

            // Kiểm tra lỗi 'Invalid Handle' hoặc 'Invalid State' và reset kết nối
            if (p_data->write.status == ESP_GATT_INVALID_HANDLE || p_data->write.status == ESP_ERR_INVALID_STATE) {
                // Kiểm tra và reset kết nối cho thiết bị bị lỗi
                for (int i = 1; i <= MAX_NODE; i++) {
                    if (remote_device[i].conn_id == p_data->write.conn_id) {
                        remote_device[i].conn_id = 0xFFFF;  // Reset ID kết nối
                        get_service[i] = false;  // Đặt lại trạng thái dịch vụ chưa được tìm thấy
                        ESP_LOGI(GATTC_TAG, "Reset connection for node %d", i);
                        break;
                    }
                }

                // Delay để tránh bị lỗi nhanh chóng khi thử lại
                vTaskDelay(pdMS_TO_TICKS(100));

                // Khởi động lại quá trình quét thiết bị (scan)
                start_scan();  // Hàm này cần được định nghĩa để bắt đầu quét thiết bị mới
            }
        } else {
            ESP_LOGI(GATTC_TAG, "write char success");
        }
    case ESP_GATTC_SRVC_CHG_EVT: {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:%08x%04x",(bda[0] << 24) + (bda[1] << 16) + (bda[2] << 8) + bda[3],
                 (bda[4] << 8) + bda[5]);
        break;
    }
    case ESP_GATTC_DISCONNECT_EVT:
    ESP_LOGI(GATTC_TAG, "Device disconnected, reason = 0x%x", param->disconnect.reason);
    
    // Thử kết nối lại ngay lập tức nếu reason = 0x8
    if (param->disconnect.reason == 0x8) {
        ESP_LOGI(GATTC_TAG, "Connection timeout, attempting immediate reconnect");
        esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
                          param->disconnect.remote_bda,
                          BLE_ADDR_TYPE_PUBLIC,  // Sử dụng địa chỉ public
                          true);
    } else {
        // Reset connection flags và scan như cũ
        for(int i = 1; i <= MAX_NODE; i++) {
            if(memcmp(param->disconnect.remote_bda, remote_device[i].bda, 6) == 0) {
                remote_device[i].conn_id = 0xFFFF;
                get_service[i] = false;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        start_scan();
    }
    break;
    default:
        break;
    }
}



int i = 0;
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    switch (event) {
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(GATTC_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        //the unit of the duration is second
        // uint32_t duration = 20;
        // esp_ble_gap_start_scanning(duration);
          //the unit of the duration is second
          stop_scan_done = false;
        uint32_t duration = 30;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if (param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(GATTC_TAG, "Scan start success");
        }else{
            ESP_LOGE(GATTC_TAG, "Scan start failed");
            //start_scan();
        }
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
            switch (scan_result->scan_rst.search_evt) {
            case ESP_GAP_SEARCH_INQ_RES_EVT:
                // Kiểm tra địa chỉ MAC trùng lặp
                if (!is_mac_duplicate(scan_result->scan_rst.bda)) {
                    esp_log_buffer_hex(GATTC_TAG, scan_result->scan_rst.bda, 6);

                    // Lưu địa chỉ MAC vào danh sách
                    add_mac_to_list(scan_result->scan_rst.bda);

                    // Kiểm tra điều kiện kết nối
                    if (scan_result->scan_rst.bda[0] == 0xa0 && scan_result->scan_rst.bda[1] == 0xdd) {
                        ESP_ERROR_CHECK(esp_ble_gap_stop_scanning());
                        stop_scan_done = true;
                        esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, 
                                        scan_result->scan_rst.bda, 
                                        scan_result->scan_rst.ble_addr_type, 
                                        true);
                    }
                }
                break;
            case ESP_GAP_SEARCH_INQ_CMPL_EVT:
                ESP_LOGI(GATTC_TAG, "Scan complete");
                break;
            default:
                break;
            }
            break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "Scan stop failed");
            //esp_ble_gap_stop_scanning();
            break;
        }
        ESP_LOGI(GATTC_TAG, "Stop scan successfully");

        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "Adv stop failed");
            break;
        }
        ESP_LOGI(GATTC_TAG, "Stop adv successfully");
        break;

    default:
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    //ESP_LOGI(GATTC_TAG, "EVT %d, gattc if %d, app_id %d", event, gattc_if, param->reg.app_id);

    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(GATTC_TAG, "Reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

void scan_request_callback(int gpio_num){
    if(gpio_num == GPIO_NUM_0){
        if(stop_scan_done == false){
        start_scan();
        }
    }
}

#define WIFI_SSID      "Anh Hai_EXT"
#define WIFI_PASSWORD  "66666666"
#define MAXIMUM_RETRY  5

static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASSWORD);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASSWORD);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

void app_main(void)
{
    // 1. Khởi tạo NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Khởi tạo TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // 3. Khởi tạo WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_init_sta(); // Initialize and connect to WiFi

    // Khởi tạo DHT22
    //DHT22_init();
    
    // 4. Khởi tạo Bluetooth
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    for(int i = 1; i < MAX_NODE; i++){
        remote_device[i].conn_id = 0xFFFF;
    }
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    //register the  callback function to the gap module
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret){
        ESP_LOGE(GATTC_TAG, "gap register error, error code = %x", ret);
        return;
    }

    //register the callback function to the gattc module
    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if(ret){
        ESP_LOGE(GATTC_TAG, "gattc register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (ret){
        ESP_LOGE(GATTC_TAG, "gattc app register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatt_set_local_mtu(200);
    if (ret){
        ESP_LOGE(GATTC_TAG, "set local  MTU failed, error code = %x", ret);
    }
    input_io_create(GPIO_NUM_0, GPIO_INTR_falling);
    input_callback_register(scan_request_callback);

    esp_ble_gap_set_device_name("Gateway");
     
    MQTT_client_init();

    //xTaskCreate(dht_read_task, "dht_task", 4096, NULL, 5, NULL);

}