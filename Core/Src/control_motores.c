#include "control_motores.h"

static TIM_HandleTypeDef *tim_derecho;
static TIM_HandleTypeDef *tim_izquierdo;

static void IniciarCanales(void)
{
    HAL_TIM_PWM_Start(tim_derecho, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(tim_derecho, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(tim_izquierdo, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(tim_izquierdo, TIM_CHANNEL_2);
}

void ControlMotores_Init(TIM_HandleTypeDef *timer_derecho,
                         TIM_HandleTypeDef *timer_izquierdo)
{
    tim_derecho = timer_derecho;
    tim_izquierdo = timer_izquierdo;
    IniciarCanales();

    /* Duty inicial 0%: PB4/PB5 para derecho, PB6/PB7 para izquierdo. */
    __HAL_TIM_SET_COMPARE(tim_derecho, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(tim_derecho, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(tim_izquierdo, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(tim_izquierdo, TIM_CHANNEL_2, 0);
}

void ControlMotores_Aplicar(int16_t motor_derecho, int16_t motor_izquierdo)
{
    if (motor_derecho == 0 && motor_izquierdo == 0) {
        HAL_TIM_PWM_Stop(tim_derecho, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(tim_derecho, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(tim_izquierdo, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(tim_izquierdo, TIM_CHANNEL_2);
        return;
    }

    IniciarCanales();

    if (motor_derecho > 100)  motor_derecho = 100;
    if (motor_derecho < -100) motor_derecho = -100;
    if (motor_izquierdo > 100)  motor_izquierdo = 100;
    if (motor_izquierdo < -100) motor_izquierdo = -100;

    uint32_t arr_derecho = __HAL_TIM_GET_AUTORELOAD(tim_derecho);
    uint32_t arr_izquierdo = __HAL_TIM_GET_AUTORELOAD(tim_izquierdo);
    uint32_t duty_derecho = (motor_derecho >= 0)
            ? (uint32_t)motor_derecho : (uint32_t)(-motor_derecho);
    uint32_t duty_izquierdo = (motor_izquierdo >= 0)
            ? (uint32_t)motor_izquierdo : (uint32_t)(-motor_izquierdo);

    /* El timer cuenta 0..ARR: CCR=(ARR+1)*duty/100. Se conserva el
     * recorte historico a ARR, incluso para el comando de 100%.
     */
    uint32_t ccr_derecho = ((arr_derecho + 1U) * duty_derecho) / 100U;
    uint32_t ccr_izquierdo = ((arr_izquierdo + 1U) * duty_izquierdo) / 100U;
    if (ccr_derecho > arr_derecho) ccr_derecho = arr_derecho;
    if (ccr_izquierdo > arr_izquierdo) ccr_izquierdo = arr_izquierdo;

    if (motor_derecho >= 0) {
        __HAL_TIM_SET_COMPARE(tim_derecho, TIM_CHANNEL_1, ccr_derecho);
        __HAL_TIM_SET_COMPARE(tim_derecho, TIM_CHANNEL_2, 0);
    } else {
        __HAL_TIM_SET_COMPARE(tim_derecho, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(tim_derecho, TIM_CHANNEL_2, ccr_derecho);
    }

    /* El sentido fisico del izquierdo esta invertido respecto del derecho. */
    if (motor_izquierdo >= 0) {
        __HAL_TIM_SET_COMPARE(tim_izquierdo, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(tim_izquierdo, TIM_CHANNEL_2, ccr_izquierdo);
    } else {
        __HAL_TIM_SET_COMPARE(tim_izquierdo, TIM_CHANNEL_1, ccr_izquierdo);
        __HAL_TIM_SET_COMPARE(tim_izquierdo, TIM_CHANNEL_2, 0);
    }
}
