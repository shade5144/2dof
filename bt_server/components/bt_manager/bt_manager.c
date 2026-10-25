#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#include "esp_bt_device.h"
#include "bt_manager.h"

#define SPP_TAG "SPP_ACCEPTOR_AUX"
#define SPP_SERVER_NAME "SPP_SERVER_AUX"

static const char local_device_name[] = CONFIG_EXAMPLE_LOCAL_DEVICE_NAME;
static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const bool esp_spp_enable_l2cap_ertm = true;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static uint32_t write_handle = 0;

static char rx_buffer = '\0';

int btman_servo_1_angle_write = 0;
int btman_servo_2_angle_write = 0;

extern bool move_servo;

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    char bda_str[18] = {0};

    switch (event) {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
            esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_INIT_EVT status:%d", param->init.status);
        }
        break;
    case ESP_SPP_START_EVT:
        if (param->start.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT handle:%"PRIu32" sec_id:%d scn:%d", param->start.handle, param->start.sec_id,
                     param->start.scn);
            esp_bt_gap_set_device_name(local_device_name);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_START_EVT status:%d", param->start.status);
        }
        break;
    case ESP_SPP_DATA_IND_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT");

        if(param->data_ind.len > 0) 
        {
            ESP_LOGI(SPP_TAG, "opcode: %d", param->data_ind.data[0]); 
            switch(param->data_ind.data[0])
            {
                // Get ACK for sending BT data 
                case 0:
                {
                    rx_buffer = 'A';
                    break;
                }
                case 1:
                {
                    ESP_LOGI(SPP_TAG, "Command received to move servos");
                    int16_t *angle_ptr = (int16_t *)param->data_ind.data;
                    btman_servo_1_angle_write = angle_ptr[1];
                    btman_servo_2_angle_write = angle_ptr[2];
                    ESP_LOGI(SPP_TAG, "Move to: %d %d", angle_ptr[1], angle_ptr[2]);
                    move_servo = true;

                    break;
                }
                case 2:
                {
                    break;
                }

            }
        }

        break;
    case ESP_SPP_SRV_OPEN_EVT:
        write_handle = param->srv_open.handle;

        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT status:%d handle:%"PRIu32", rem_bda:[%s]", param->srv_open.status,
                 param->srv_open.handle, bda2str(param->srv_open.rem_bda, bda_str, sizeof(bda_str)));
        break;

    case ESP_SPP_CLOSE_EVT:
        write_handle = 0;
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT status:%d handle:%"PRIu32" close_by_remote:%d", param->close.status,
                 param->close.handle, param->close.async);
        break;
    default:
        break;
    }
}

static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    char bda_str[18] = {0};

    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(SPP_TAG, "authentication success: %s bda:[%s]", param->auth_cmpl.device_name,
                     bda2str(param->auth_cmpl.bda, bda_str, sizeof(bda_str)));
        } else {
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:{
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(SPP_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }

    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d bda:[%s]", param->mode_chg.mode,
                 bda2str(param->mode_chg.bda, bda_str, sizeof(bda_str)));
        break;

    default: {
        ESP_LOGI(SPP_TAG, "event: %d", event);
        break;
    }
    }
    return;
}

char *bda2str(uint8_t * bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

bool init_bluetooth(char *bda_str)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return false;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return false;
    }

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    bluedroid_cfg.ssp_en = false;

    if ((ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return false;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return false;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s", __func__, esp_err_to_name(ret));
        return false;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s", __func__, esp_err_to_name(ret));
        return false;
    }

    esp_spp_cfg_t bt_spp_cfg = {
        .mode = esp_spp_mode,
        .enable_l2cap_ertm = esp_spp_enable_l2cap_ertm,
        .tx_buffer_size = 0, /* Only used for ESP_SPP_MODE_VFS mode */
    };
    if ((ret = esp_spp_enhanced_init(&bt_spp_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s", __func__, esp_err_to_name(ret));
        return false;
    }

    return true;
}

bool bt_send_data(uint8_t *data, int length)
{
    esp_err_t ret = esp_spp_write(write_handle, length, data);
    if (ret != ESP_OK) {
            ESP_LOGE(SPP_TAG, "Failed to send data via bt(stream): %s", esp_err_to_name(ret));
            return false;
    }

    return true;
}

bool client_connected()
{
    return write_handle != 0;
}


