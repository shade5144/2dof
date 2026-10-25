#ifndef ONESHOT_READ_H
#define ONESHOT_READ_H

#include <stdint.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

void adc_init(adc_oneshot_unit_handle_t *adc_handle, adc_cali_handle_t *cali_chan0, adc_cali_handle_t *cali_chan1);
int oneshot_read_voltage(adc_oneshot_unit_handle_t handle, adc_channel_t chan, adc_cali_handle_t cali);

#endif
