/* Prueba nativa del modulo real, sin tocar hardware:
 * gcc -std=c11 -Wall -Wextra -Werror -Itests/motores_stubs -ICore/Inc
 *     tests/test_control_motores.c Core/Src/control_motores.c -o test_motores
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "control_motores.h"

static TIM_HandleTypeDef derecho = {.autoreload=959};
static TIM_HandleTypeDef izquierdo = {.autoreload=799};
static unsigned arranques, paradas, escrituras;

int HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
    assert((timer == &derecho || timer == &izquierdo) && channel <= TIM_CHANNEL_2);
    timer->running[channel] = 1;
    ++arranques;
    return 0;
}

int HAL_TIM_PWM_Stop(TIM_HandleTypeDef *timer, uint32_t channel)
{
    assert((timer == &derecho || timer == &izquierdo) && channel <= TIM_CHANNEL_2);
    timer->running[channel] = 0;
    ++paradas;
    return 0;
}

void MotoresTest_SetCompare(TIM_HandleTypeDef *timer, uint32_t channel,
                            uint32_t compare)
{
    assert((timer == &derecho || timer == &izquierdo) && channel <= TIM_CHANNEL_2);
    timer->compare[channel] = compare;
    ++escrituras;
}

static int16_t limitar(int value)
{
    if (value > 100) return 100;
    if (value < -100) return -100;
    return (int16_t)value;
}

static uint32_t comparar(uint32_t arr, int16_t command)
{
    uint32_t duty = (command >= 0) ? (uint32_t)command : (uint32_t)(-command);
    uint32_t result = ((arr + 1U) * duty) / 100U;
    return result > arr ? arr : result;
}

static void verificar_salida(int input_derecho, int input_izquierdo)
{
    int16_t cmd_derecho = limitar(input_derecho);
    int16_t cmd_izquierdo = limitar(input_izquierdo);
    unsigned start_before = arranques;
    unsigned stop_before = paradas;
    unsigned writes_before = escrituras;

    ControlMotores_Aplicar((int16_t)input_derecho, (int16_t)input_izquierdo);
    if (input_derecho == 0 && input_izquierdo == 0) {
        assert(arranques == start_before && paradas == stop_before + 4);
        assert(escrituras == writes_before); /* El firmware historico conserva CCR. */
        assert(!derecho.running[0] && !derecho.running[1]);
        assert(!izquierdo.running[0] && !izquierdo.running[1]);
        return;
    }

    assert(arranques == start_before + 4 && paradas == stop_before);
    assert(escrituras == writes_before + 4);
    assert(derecho.running[0] && derecho.running[1]);
    assert(izquierdo.running[0] && izquierdo.running[1]);
    uint32_t ccr_d = comparar(derecho.autoreload, cmd_derecho);
    uint32_t ccr_i = comparar(izquierdo.autoreload, cmd_izquierdo);
    assert(derecho.compare[0] == (cmd_derecho >= 0 ? ccr_d : 0));
    assert(derecho.compare[1] == (cmd_derecho >= 0 ? 0 : ccr_d));
    assert(izquierdo.compare[0] == (cmd_izquierdo >= 0 ? 0 : ccr_i));
    assert(izquierdo.compare[1] == (cmd_izquierdo >= 0 ? ccr_i : 0));
}

int main(void)
{
    derecho.compare[0]=derecho.compare[1]=123;
    izquierdo.compare[0]=izquierdo.compare[1]=456;
    ControlMotores_Init(&derecho, &izquierdo);
    assert(arranques == 4 && paradas == 0 && escrituras == 4);
    assert(derecho.running[0] && derecho.running[1]);
    assert(izquierdo.running[0] && izquierdo.running[1]);
    assert(!derecho.compare[0] && !derecho.compare[1]);
    assert(!izquierdo.compare[0] && !izquierdo.compare[1]);

    /* Todas las combinaciones alrededor de limites, signos y redondeos. */
    for (int right=-150; right<=150; ++right)
        for (int left=-150; left<=150; ++left)
            verificar_salida(right, left);

    verificar_salida(INT16_MIN, INT16_MAX);
    verificar_salida(INT16_MAX, INT16_MIN);
    verificar_salida(0, 0);
    verificar_salida(1, -1);
    verificar_salida(50, -50);
    verificar_salida(100, -100);
    puts("PASS motores: init, stop, signos, cableado, clamps y 90601 combinaciones PWM.");
    return 0;
}
