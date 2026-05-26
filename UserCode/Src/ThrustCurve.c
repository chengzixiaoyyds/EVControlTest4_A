#include "ThrustCurve.h"
#include "math.h"

/**
  * @brief  PWM 占空比 -> 推力(N)，含死区处理
  * @param  pwm_duty: PWM 占空比 (典型范围 6.0 ~ 8.9)
  * @retval 推力值(N)，死区内返回 0
  */
float Thrust_FromPWM(float pwm_duty)
{
    if (pwm_duty >= THRUST_DZ_HIGH)
    {
        /* 正转: F = a * (PWM - 中性点)^2 */
        float diff = pwm_duty - THRUST_NEUTRAL;
        return THRUST_COEFF_FWD * diff * diff;
    }
    else if (pwm_duty <= THRUST_DZ_LOW)
    {
        /* 反转: F = -a * (中性点 - PWM)^2 */
        float diff = THRUST_NEUTRAL - pwm_duty;
        return -THRUST_COEFF_REV * diff * diff;
    }
    else
    {
        /* 死区: 推力为零 */
        return 0.0f;
    }
}

/**
  * @brief  目标推力(N) -> PWM 占空比 (反函数)
  * @param  force: 目标推力(N)，正值=正转，负值=反转
  * @retval PWM 占空比，force=0 时返回死区中性点
  */
float PWM_FromThrust(float force)
{
    if (force > 0.0f)
    {
        /* 正转: PWM = 中性点 + sqrt(F / a) */
        return THRUST_NEUTRAL + sqrtf(force / THRUST_COEFF_FWD);
    }
    else if (force < 0.0f)
    {
        /* 反转: PWM = 中性点 - sqrt(-F / a) */
        return THRUST_NEUTRAL - sqrtf(-force / THRUST_COEFF_REV);
    }
    else
    {
        /* 返回死区中心 */
        return THRUST_NEUTRAL;
    }
}
