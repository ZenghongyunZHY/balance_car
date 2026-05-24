#include "get_target.h"
#include "main.h"
#include "balance_controller.h"

void get_new_target(Balance_Target_t *target)
{
    //占位函数，实际应用中可以通过串口、蓝牙等方式接收新的目标状态，并更新全局变量 target
    target->target_speed = 0.0f; // 设置目标前后速度为 0.0 m/s
    target->target_turn = 0.0f;  // 设置目标转向差
}
