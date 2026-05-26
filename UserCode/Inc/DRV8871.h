#ifndef _DRV8871_H_
#define _DRV8871_H_

#include "stm32f1xx_hal.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t in1_channel;
    uint32_t in2_channel;
} DRV8871_HandleTypeDef;

void DRV8871_Init(DRV8871_HandleTypeDef *hmotor,
                  TIM_HandleTypeDef *htim,
                  uint32_t in1_channel,
                  uint32_t in2_channel);

void DRV8871_Start(DRV8871_HandleTypeDef *hmotor);
void DRV8871_Stop(DRV8871_HandleTypeDef *hmotor);

void DRV8871_SetSpeed(DRV8871_HandleTypeDef *hmotor, int32_t speed);
void DRV8871_Forward(DRV8871_HandleTypeDef *hmotor, uint16_t speed);
void DRV8871_Reverse(DRV8871_HandleTypeDef *hmotor, uint16_t speed);
void DRV8871_Brake(DRV8871_HandleTypeDef *hmotor);
void DRV8871_Coast(DRV8871_HandleTypeDef *hmotor);

#endif /* _DRV8871_H_ */
