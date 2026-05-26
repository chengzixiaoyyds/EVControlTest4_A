#include "DRV8871.h"

#define PWM_PERIOD 800

void DRV8871_Init(DRV8871_HandleTypeDef *hmotor,
                  TIM_HandleTypeDef *htim,
                  uint32_t in1_channel,
                  uint32_t in2_channel)
{
    hmotor->htim = htim;
    hmotor->in1_channel = in1_channel;
    hmotor->in2_channel = in2_channel;
}

void DRV8871_Start(DRV8871_HandleTypeDef *hmotor)
{
    HAL_TIM_PWM_Start(hmotor->htim, hmotor->in1_channel);
    HAL_TIM_PWM_Start(hmotor->htim, hmotor->in2_channel);
}

void DRV8871_Stop(DRV8871_HandleTypeDef *hmotor)
{
    HAL_TIM_PWM_Stop(hmotor->htim, hmotor->in1_channel);
    HAL_TIM_PWM_Stop(hmotor->htim, hmotor->in2_channel);
}

void DRV8871_SetSpeed(DRV8871_HandleTypeDef *hmotor, int32_t speed)
{
    if (speed > PWM_PERIOD) speed = PWM_PERIOD;
    if (speed < -PWM_PERIOD) speed = -PWM_PERIOD;

    if (speed > 0) {
        /* Forward: IN1=HIGH, IN2=PWM  → ON=Brake, OFF=Forward */
        /* speed↑ → CCR↓ → Brake↓ Forward↑ → faster */
        __HAL_TIM_SET_COMPARE(hmotor->htim, hmotor->in1_channel, PWM_PERIOD);
        __HAL_TIM_SET_COMPARE(hmotor->htim, hmotor->in2_channel, PWM_PERIOD - (uint16_t)speed);
    } else if (speed < 0) {
        /* Reverse: IN1=PWM, IN2=HIGH  → ON=Brake, OFF=Reverse */
        /* |speed|↑ → CCR↓ → Brake↓ Reverse↑ → faster */
        __HAL_TIM_SET_COMPARE(hmotor->htim, hmotor->in1_channel, PWM_PERIOD - (uint16_t)(-speed));
        __HAL_TIM_SET_COMPARE(hmotor->htim, hmotor->in2_channel, PWM_PERIOD);
    } else {
        /* Brake: IN1=HIGH, IN2=HIGH */
        __HAL_TIM_SET_COMPARE(hmotor->htim, hmotor->in1_channel, PWM_PERIOD);
        __HAL_TIM_SET_COMPARE(hmotor->htim, hmotor->in2_channel, PWM_PERIOD);
    }
}

void DRV8871_Forward(DRV8871_HandleTypeDef *hmotor, uint16_t speed)
{
    if (speed > PWM_PERIOD) speed = PWM_PERIOD;
    /* Forward: IN1=HIGH, IN2=PWM  → ON=Brake, OFF=Forward */
    __HAL_TIM_SET_COMPARE(hmotor->htim, hmotor->in1_channel, PWM_PERIOD);
    __HAL_TIM_SET_COMPARE(hmotor->htim, hmotor->in2_channel, PWM_PERIOD - speed);
}

void DRV8871_Reverse(DRV8871_HandleTypeDef *hmotor, uint16_t speed)
{
    if (speed > PWM_PERIOD) speed = PWM_PERIOD;
    /* Reverse: IN1=PWM, IN2=HIGH  → ON=Brake, OFF=Reverse */
    __HAL_TIM_SET_COMPARE(hmotor->htim, hmotor->in1_channel, PWM_PERIOD - speed);
    __HAL_TIM_SET_COMPARE(hmotor->htim, hmotor->in2_channel, PWM_PERIOD);
}

void DRV8871_Brake(DRV8871_HandleTypeDef *hmotor)
{
    __HAL_TIM_SET_COMPARE(hmotor->htim, hmotor->in1_channel, PWM_PERIOD);
    __HAL_TIM_SET_COMPARE(hmotor->htim, hmotor->in2_channel, PWM_PERIOD);
}

void DRV8871_Coast(DRV8871_HandleTypeDef *hmotor)
{
    __HAL_TIM_SET_COMPARE(hmotor->htim, hmotor->in1_channel, 0);
    __HAL_TIM_SET_COMPARE(hmotor->htim, hmotor->in2_channel, 0);
}
