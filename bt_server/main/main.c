/*
 * SPDX-FileCopyrightText: 2020-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "esp_log.h"

#include "bt_manager.h"

#include "oneshot_read.h"

#include "mcpwm_manager.h"

#define SPP_TAG "SPP_ACCEPTOR_MAIN"
#define SPP_SERVER_NAME "SPP_SERVER_MAIN"

#define ADC_PIN1 ADC_CHANNEL_4
#define ADC_PIN2 ADC_CHANNEL_6

static const int NUM_SAMPLES = 96;

static bool block_transmission = false;
static uint16_t data[120];

// Extern variables for servo actuation via BT 
extern int btman_servo_1_angle_write;
extern int btman_servo_2_angle_write;

int move_servo = 0;

static void move_servo_task(void *params)
{
    move_servo = 0;

    ESP_LOGI(SPP_TAG, "Moving servo 1..");
    smooth_move_motor(0, btman_servo_1_angle_write);  
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(SPP_TAG, "Moving servo 2..");
    smooth_move_motor(1, btman_servo_2_angle_write);
    vTaskDelay(pdMS_TO_TICKS(100));

    block_transmission = true;

    // Wait for transmission to actually be blocked
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(SPP_TAG, "Sending some data");
    memset(data, 0, sizeof(uint16_t) * NUM_SAMPLES);
    bt_send_data((uint8_t *)data, sizeof(uint16_t) * NUM_SAMPLES);

    block_transmission = false;

    vTaskDelete(NULL);
}

static void transmit_task(void *params)
{
    adc_oneshot_unit_handle_t adc_handle;
    adc_cali_handle_t cali_chan0, cali_chan1;
    adc_init(&adc_handle, &cali_chan0, &cali_chan1);

    while(1)
    {
        while(!client_connected())
            vTaskDelay(pdMS_TO_TICKS(10));

        ESP_LOGI(SPP_TAG, "Client connected!");
        char buf[2];

        while(client_connected())
        {
            if(block_transmission)
            {
                ESP_LOGI(SPP_TAG, "Blocking transmission...");
                while(block_transmission)
                    vTaskDelay(pdMS_TO_TICKS(10));
            }

            if(move_servo)
            {
                ESP_LOGI(SPP_TAG, "Preparing to move the servos....");
                xTaskCreate(move_servo_task, "MoveServoTask", 4 * 1024, NULL, configMAX_PRIORITIES - 12, NULL);

                // Wait 
                vTaskDelay(pdMS_TO_TICKS(100));
            }

            int delay_counter = 1;
            
            for(int i = 0; i < NUM_SAMPLES / 2; i++)
            {
                data[i] = oneshot_read_voltage(adc_handle, ADC_PIN1, cali_chan0);
                data[i + (NUM_SAMPLES / 2)] = oneshot_read_voltage(adc_handle, ADC_PIN2, cali_chan1);
                delay_counter++;

                vTaskDelay(pdMS_TO_TICKS(10));
            }

            bt_send_data((uint8_t *)data, sizeof(uint16_t) * NUM_SAMPLES);
            // ESP_LOGI(SPP_TAG, "Data sent...");

            // Wait for ACK. Blocking
            // ESP_LOGI(SPP_TAG, "Waiting for ACK...");
            // bt_recv_ack();
            // ESP_LOGI(SPP_TAG, "ACKed, next iteration...");
        }

    }

    /* Virtually useless, but maybe in the future */
    // adc_deinit(adc_handle, cali_chan0, cali_chan1);
}

void app_main(void)
{
    /* BT init */
    char bda_str[18] = {0};

    if( !init_bluetooth(bda_str) )
    {
        ESP_LOGE(SPP_TAG, "Bluetooth stack init failed!");
        return;
    }

    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    ESP_LOGI(SPP_TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));

    mcpwm_init();

    xTaskCreate(transmit_task, "BtSendAdcTask", 4 * 1024, NULL, configMAX_PRIORITIES - 10, NULL);
}
