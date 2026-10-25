 /*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/mcpwm_prelude.h"

#include "esp_rom_sys.h"

#include "mcpwm_manager.h"

static const char *TAG = "example";

extern int init_servo1, init_servo2;

// -5 has proper straight alignment, not 0. So the bounds must be adjusted accordingly
static struct ServoDrive servos[SERVO_COUNT] =
{
    {
        // .min_pw = 1000,
        // .max_pw = 2000,
        .min_pw = 600,
        .max_pw = 2550,
        .min_degree = -90,
        .max_degree = 90,
        .gen_cfg = { .gen_gpio_num = SERVO_GPIO_1},
    },
    {
        // .min_pw = 1000,
        // .max_pw = 2000,
        .min_pw = 500,
        .max_pw = 2400,
        .min_degree = -90,
        .max_degree = 90,
        .gen_cfg = { .gen_gpio_num = SERVO_GPIO_2},
    },
};

static struct ServoPos servo_position[SERVO_COUNT] = 
{
    {0, 1}, 
    {0, 1}
};

// Please consult the datasheet of your servo before changing the following parameters

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

static int convert_voltage_to_angle_servo1(int x)
{
    return ((x - 384) * (90 - (-90)) / (3094-384)) + (-90);
}

static int convert_voltage_to_angle_servo2(int x)
{
    return ((x - 292) * (90 - (-90)) / (2924-292)) + (-90);
}

static inline uint32_t example_angle_to_compare(int angle, struct ServoDrive *servo)
{
    int min_degree = servo->min_degree;
    int max_degree = servo->max_degree;

    return (uint32_t)((angle - min_degree) * (servo->max_pw - servo->min_pw) / (max_degree - min_degree) + servo->min_pw);
}

void smooth_move_motor(int servo_num, int to_move)
{
    struct ServoDrive *servo = &servos[servo_num];
    struct ServoPos *servo_pos = &servo_position[servo_num];
    mcpwm_cmpr_handle_t comparator = servo->cmp_handle;

    int delay = 4;

    /*
    if(!servo_num)
    {
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(to_move, servo)));
        return;
    }
    */

    int step = servo_pos->step;
    int angle = servo_pos->angle;

    int to_move_cpy = to_move;

    to_move = example_angle_to_compare(to_move, servo); 
    ESP_LOGI(TAG, "BEFORE: %d", angle);
    angle = example_angle_to_compare(angle, servo); 
    ESP_LOGI(TAG, "AFTER: %d", angle);

    int counter = 1;
    int delay_amt = 1000;
    int counter_limit = 0xffffffff; // Infinity pretty much

    if(delay_amt >= 2000)
    {
        // This is a little jank as I have only tested for 2000, 4000, 8000 and 16000 while this formula generalizes too much
        counter_limit = (0x800 >> (delay_amt / 2000 - 1)) - 1;
    }

    if(to_move < angle)
    {
        step *= -1;

        while((angle + step) > to_move)
        {
            angle += step;
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angle));
            esp_rom_delay_us(delay_amt);

            counter = (counter + 1) & 0x7ff;
            if(!counter)
              vTaskDelay(1);
        }
    }
    else
    {
        while((angle + step) < to_move)
        {
            angle += step;
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angle));
            esp_rom_delay_us(delay_amt);

            counter = (counter + 1) & 0x7ff;
            if(!counter)
              vTaskDelay(1);
        }
    }

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, to_move));

    servo_pos->angle = to_move_cpy;
}

void mcpwm_init()
{
    ESP_LOGI(TAG, "Create timer and operator");

    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    ESP_LOGI(TAG, "Servo Voltages: %d %d", init_servo1, init_servo2);
    for(int i = 0; i < SERVO_COUNT; i++)
    {
        ESP_LOGI(TAG, "One time init?");

        mcpwm_generator_config_t *generator_config = &servos[i].gen_cfg;

        mcpwm_oper_handle_t oper = NULL;
        mcpwm_operator_config_t operator_config = {
            .group_id = 0, // operator must be in the same group to the timer
        };
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));


        ESP_LOGI(TAG, "Connect timer and operator");
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

        ESP_LOGI(TAG, "Create comparator and generator from the operator");
        mcpwm_cmpr_handle_t comparator = NULL;
        mcpwm_comparator_config_t comparator_config = {
            .flags.update_cmp_on_tez = true,
        };
        ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

        mcpwm_gen_handle_t generator = NULL;
        ESP_ERROR_CHECK(mcpwm_new_generator(oper, generator_config, &generator));

        if(!i)
        {
            servo_position[i].angle = convert_voltage_to_angle_servo1(init_servo1);
        }
        else
        {
            servo_position[i].angle = convert_voltage_to_angle_servo2(init_servo2);
        }

        ESP_LOGI(TAG, "ANGLE: Servo%d: %d", i+1, servo_position[i].angle);

        // set the initial compare value, so that the servo will spin to the center position
        // ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(initial, &servos[i])));

        ESP_LOGI(TAG, "Set generator action on timer and compare event");
        // go high on counter empty
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                                                                  MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        // go low on compare threshold
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));
        servos[i].op_handle = oper;
        servos[i].cmp_handle = comparator;
        servos[i].gen_handle = generator;
    }

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    float initial = -5;
    smooth_move_motor(0, initial);
    vTaskDelay(1);
    smooth_move_motor(1, initial);
    vTaskDelay(1);
}

