/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "tcpip_adapter.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

//#include "esp_gatt_defs.h"
#include "esp_bt.h"
#include "esp_log.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"

//#include "esp_gattc_api.h"
#include "esp_gap_ble_api.h" //esp_ble_gap_register_callback(function), 


static const char *TAG = "MQTT_EXAMPLE";

static const char* DEMO_TAG = "Beacon_DEMO";

/* declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);


static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x50,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

typedef struct {
    int16_t sensor_id;
    uint8_t seq_no;
    int16_t sensor_rssi;
    int16_t sensor_acc[3];
    int16_t sensor_gyro[3];
    int16_t sensor_mag[3];
    float acc[3];
    float gyro[3];
    float mag[3];
} sensor_data;

sensor_data data_t;

esp_mqtt_client_handle_t client;

uint8_t mac[6];

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    // int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            // char cPayload[200];
            // sprintf(cPayload,
			// 	"{\"temp_c\": %d, \"pressure_hpa\": %d}",
			// 	data_t.sensor_rssi, data_t.sensor_id);
            // msg_id = esp_mqtt_client_publish(client, "/topic/qos1", cPayload, 0, 1, 0);
            // ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);

            // msg_id = esp_mqtt_client_subscribe(client, "/topic/qos0", 0);
            // ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            // msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
            // ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            // msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
            // ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        // case MQTT_EVENT_SUBSCRIBED:
        //     ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        //     msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
        //     ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        //     break;
        // case MQTT_EVENT_UNSUBSCRIBED:
        //     ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        //     break;
        // case MQTT_EVENT_PUBLISHED:
        //     ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        //     break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = "mqtt://mqtt_server:1234@192.168.43.187:1883", //lahiru
        //.uri = "mqtt://mqtt_server:1234@192.168.8.102:1883", //thameera

    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

// bluetooth

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param)
{
    esp_err_t err;

    switch(event)
    {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
            uint32_t duration = 0;
            esp_ble_gap_start_scanning(duration);
            break;
        }
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT: {
            if((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(DEMO_TAG,"Scan start failed: %s", esp_err_to_name(err));
            }
            else {
                ESP_LOGI(DEMO_TAG,"Start scanning...");
            }
            break;
        }
        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            esp_ble_gap_cb_param_t* scan_result = (esp_ble_gap_cb_param_t*)param;
            switch(scan_result->scan_rst.search_evt)
            {
                case ESP_GAP_SEARCH_INQ_RES_EVT: {
                    int ret = 0;
                    if (ret) {
                        // error:The received data is not an eddystone frame packet or a correct eddystone frame packet.
                        // just return
                        return;
                    } else {   
                        
                        // int nework_id;
                        int data_len;
                        uint16_t nework_id;

                        // The received adv data is a correct eddystone frame packet.
                        // Here, we get the eddystone infomation in eddystone_res, we can use the data in res to do other things.
                        // For example, just print them:
                        // ESP_LOGI(DEMO_TAG, "--------Beacon Found----------");
                        // esp_log_buffer_hex("Beacon_DEMO: Device address:", scan_result->scan_rst.bda, ESP_BD_ADDR_LEN);
                        // ESP_LOGI(DEMO_TAG, "RSSI of packet:%d dbm", scan_result->scan_rst.rssi);
                        // ESP_LOGI(DEMO_TAG, "Data length: %d ", (int) scan_result->scan_rst.adv_data_len);
                        // esp_log_buffer_hex("Beacon_DEMO: data:", scan_result->scan_rst.ble_adv, (int) scan_result->scan_rst.adv_data_len);
                        // nework_id = scan_result->scan_rst.ble_adv[3];
                        data_len = scan_result->scan_rst.adv_data_len;
                        nework_id = (uint16_t) scan_result->scan_rst.ble_adv[1] | ((uint16_t) (scan_result->scan_rst.ble_adv[0])) << 8;
                        if( data_len == 23 && nework_id == 0x1621){
                            ESP_LOGI(DEMO_TAG, "--------Beacon Found----------");
                            //esp_log_buffer_hex("Beacon_DEMO: Device address:", scan_result->scan_rst.bda, ESP_BD_ADDR_LEN);
                            //ESP_LOGI(DEMO_TAG, "RSSI of packet:%d dbm", scan_result->scan_rst.rssi);
                            //ESP_LOGI(DEMO_TAG, "Data length: %d ", (int) scan_result->scan_rst.adv_data_len);
                            //esp_log_buffer_hex("Beacon_DEMO: data:", scan_result->scan_rst.ble_adv, (int) scan_result->scan_rst.adv_data_len);
                            data_t.sensor_rssi = scan_result->scan_rst.rssi;
                            data_t.seq_no = scan_result->scan_rst.ble_adv[2];
                            data_t.sensor_id = (uint16_t)scan_result->scan_rst.ble_adv[4] | ((uint16_t)(scan_result->scan_rst.ble_adv[3])) << 8;
                            int i;
                            for( i=0; i<3; i++){
                                data_t.sensor_acc[i] = (uint16_t)(scan_result->scan_rst.ble_adv[i*2+5]) <<8 | (uint16_t)scan_result->scan_rst.ble_adv[i*2+6];
                                //-- calculate acceleration, unit G, range -16, +16    
                                data_t.acc[i] = (data_t.sensor_acc[i] * 1.0) / (32768.0/16.0);

                                data_t.sensor_gyro[i] = (uint16_t)(scan_result->scan_rst.ble_adv[i*2+11]) <<8 | (uint16_t)scan_result->scan_rst.ble_adv[i*2+12];
                                //-- calculate rotation, unit deg/s, range -250, +250
                                data_t.gyro[i] = (data_t.sensor_gyro[i] * 1.0) / (65536.0 / 500.0);

                                data_t.sensor_mag[i] = (uint16_t)(scan_result->scan_rst.ble_adv[i*2+17]) <<8 | (uint16_t)scan_result->scan_rst.ble_adv[i*2+18];
                                //-- calculate magnetic field, unit microtesla, range -250, +250
                                data_t.mag[i] = (data_t.sensor_mag[i] * 1.0) / (65536.0 / 500.0);
                            }

                            
                            char cPayload[500];
                            sprintf(cPayload,
                                "{\"seq_no\": %d, \"esp_mac\": \"%02X:%02X:%02X:%02X:%02X:%02X\", \"sensor_id\": %d, \"rssi\": %d, \"sensor_acc_x\": %f, \"sensor_acc_y\": %f, \"sensor_acc_z\": %f, \"sensor_gyro_x\": %f, \"sensor_gyro_y\": %f, \"sensor_gyro_z\": %f, \"sensor_mag_x\": %f, \"sensor_mag_y\": %f, \"sensor_mag_z\": %f}",
                                data_t.seq_no, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], data_t.sensor_id, data_t.sensor_rssi, 
                                data_t.acc[0], data_t.acc[1], data_t.acc[2], 
                                data_t.gyro[0], data_t.gyro[1], data_t.gyro[2],
                                data_t.mag[0], data_t.mag[1], data_t.mag[2]);
                            esp_mqtt_client_publish(client, "/topic/qos1", cPayload, 0, 1, 0);
                            //esp_log_buffer_hex("JSON data",cPayload,500);
                        }

                    }
                    break;
                }
                default:
                    break;
            }
            break;
        }
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:{
            if((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(DEMO_TAG,"Scan stop failed: %s", esp_err_to_name(err));
            }
            else {
                ESP_LOGI(DEMO_TAG,"Stop scan successfully");
            }
            break;
        }
        default:
            break;
    }
}

void esp_device_appRegister(void)
{
    esp_err_t status;
    
    ESP_LOGI(DEMO_TAG,"Register callback");

    /*<! register the scan callback function to the gap module */
    if((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        ESP_LOGE(DEMO_TAG,"gap register error: %s", esp_err_to_name(status));
        return;
    }
}

void esp_device_init(void)
{
    esp_bluedroid_init(); //essential
    esp_bluedroid_enable(); //essential
    esp_device_appRegister();
}


void app_main()
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT_EXAMPLE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg); //edit callback as neccessary
    esp_bt_controller_enable(ESP_BT_MODE_BLE); // above functions are essential.
 
    esp_device_init();
    
    esp_wifi_get_mac(ESP_MAC_WIFI_STA, mac);

    mqtt_app_start();

    /*<! set scan parameters */
    esp_ble_gap_set_scan_params(&ble_scan_params);
}
