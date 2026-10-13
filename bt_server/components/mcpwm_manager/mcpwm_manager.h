#ifndef MCPWM_MANAGER_H
#define MCPWM_MANAGER_H

#define SERVO_COUNT 2
#define SERVO_GPIO_1 14
#define SERVO_GPIO_2 12

#include "driver/mcpwm_prelude.h"

struct ServoPos
{
    int angle;
    int step;
};

// Individual Generator, Operator and Comparator for each servo, which share the same timer 
struct ServoDrive 
{
    int min_pw;
    int max_pw;
    int min_degree;
    int max_degree;
    int initial_degree;

    mcpwm_oper_handle_t op_handle;
    mcpwm_cmpr_handle_t cmp_handle;
    mcpwm_gen_handle_t gen_handle;

    mcpwm_generator_config_t gen_cfg;
};

void mcpwm_init();
void smooth_move_motor(int servo_num, int to_move);

#endif
