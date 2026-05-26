#ifndef __THRUST_CURVE_H__
#define __THRUST_CURVE_H__

/* 推力曲线参数: F = a * (PWM - 中性点)^2  (二次拟合 + 死区) */
#define THRUST_NEUTRAL       7.48f    /* PWM 中性点 */
#define THRUST_DZ_HALF       0.12f    /* 死区半宽 */
#define THRUST_DZ_LOW        (THRUST_NEUTRAL - THRUST_DZ_HALF)   /* 7.36 死区下界 */
#define THRUST_DZ_HIGH       (THRUST_NEUTRAL + THRUST_DZ_HALF)   /* 7.60 死区上界 */
#define THRUST_COEFF_FWD     639.88f  /* 正向系数: F = a * (PWM - 中性点)^2 */
#define THRUST_COEFF_REV     362.95f  /* 反向系数: F = -a * (中性点 - PWM)^2 */

float Thrust_FromPWM(float pwm_duty);
float PWM_FromThrust(float force);

#endif /* __THRUST_CURVE_H__ */
