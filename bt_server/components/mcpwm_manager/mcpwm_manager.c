 /*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/mcpwm_prelude.h"

#include "mcpwm_manager.h"

static const char *TAG = "example";

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
        .initial_degree = -5,
        .gen_cfg = { .gen_gpio_num = SERVO_GPIO_1},
    },
    {
        // .min_pw = 1000,
        // .max_pw = 2000,
        .min_pw = 500,
        .max_pw = 2400,
        .min_degree = -90,
        .max_degree = 90,
        .initial_degree = -5,
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

static inline uint32_t example_angle_to_compare(int angle, struct ServoDrive *servo)
{
    int min_degree = servo->min_degree;
    int max_degree = servo->max_degree;

    ESP_LOGI(TAG, "min_degree: %d max_degree: %d angle: %d", min_degree, max_degree, angle);
    ESP_LOGI(TAG, "angle - min_degree: %d", angle - min_degree);
    ESP_LOGI(TAG, "max_pw - min_pw: %d", servo->max_pw - servo->min_pw);
    ESP_LOGI(TAG, "(angle - min_degree) * (max_pw - min_pw): %d", (angle - min_degree) * (servo->max_pw - servo->min_pw));
    ESP_LOGI(TAG, "max_degree - min_degree: %d", max_degree - min_degree);
    ESP_LOGI(TAG, "(angle - min_degree) * (max_pw - min_pw) / (max_degree - min_degree): %d", (angle - min_degree) * (servo->max_pw - servo->min_pw)/ (max_degree - min_degree));
    ESP_LOGI(TAG, "min_pw: %u", servo->min_pw);
    ESP_LOGI(TAG, "Actual PW: %u", (angle - min_degree) * (servo->max_pw - servo->min_pw)/ (max_degree - min_degree) + servo->min_pw);

    return (uint32_t)((angle - min_degree) * (servo->max_pw - servo->min_pw) / (max_degree - min_degree) + servo->min_pw);
}

void smooth_move_motor(int servo_num, int to_move)
{
    struct ServoDrive *servo = &servos[servo_num];
    struct ServoPos *servo_pos = &servo_position[servo_num];
    mcpwm_cmpr_handle_t comparator = servo->cmp_handle;

    ESP_LOGI(TAG, "Signal has PW: %d", example_angle_to_compare(to_move, servo));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(to_move, servo)));

    return;

    /*
    int angle = servo_pos->angle;
    int step = servo_pos->step;
    mcpwm_cmpr_handle_t comparator = servo->cmp_handle;

    if (to_move < angle)
    {
        step *= -1;
    }

    // Should be fine considering the unit step size currently being employed
    while ( (step > 0 && angle < to_move) || (step < 0 && angle > to_move) )
    {
        ESP_LOGI(TAG, "Angle of rotation of servo: %d", to_move);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(angle, servo)));

        angle += step;
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    servo_pos->angle = angle;
    */
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

    for(int i = 0; i < SERVO_COUNT; i++)
    {
        ESP_LOGI(TAG, "One time init?");

        float initial = servos[i].initial_degree;
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

        // set the initial compare value, so that the servo will spin to the center position
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(initial, &servos[i])));

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
}

