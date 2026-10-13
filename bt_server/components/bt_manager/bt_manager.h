#ifndef BT_MANAGER_H
#define BT_MANAGER_H

#include <stdbool.h>
#include <inttypes.h>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"
#include "esp_bt_device.h"

char *bda2str(uint8_t * bda, char *str, size_t size);
bool init_bluetooth(char *bda_str);

bool bt_send_data(uint8_t *data, int length);
bool bt_recv_ack();
bool client_connected();

#endif
