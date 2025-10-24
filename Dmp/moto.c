#include "moto.h"
#include "stm32f4xx_hal.h"
#include "tim.h"


extern TIM_HandleTypeDef htim2;  // 供其他文件引用
/**
 * @brief 电机控制函数
 * @param direction 方向：0-后退，1-前进
 * @param speed 速度：0-100
 * @retval None
 */
void Motor_Control(uint8_t left_pwm_1, uint8_t left_pwm_2, uint8_t right_pwm_1, uint8_t right_pwm_2)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, left_pwm_1);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, left_pwm_2);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, right_pwm_1);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, right_pwm_2);
}

