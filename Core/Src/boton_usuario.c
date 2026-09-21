#include "boton_usuario.h"

#define BOTON_CLICK_MIN_MS 20U
#define BOTON_SECUENCIA_MS 400U
#define BOTON_LARGO_MS 800U

static uint8_t nivel_previo = 1;
static uint32_t pulsacion_desde_ms;
static uint32_t ultimo_click_ms;
static uint8_t cantidad_clicks;

void BotonUsuario_Init(uint8_t nivel_inicial)
{
    nivel_previo = nivel_inicial;
    pulsacion_desde_ms = 0;
    ultimo_click_ms = 0;
    cantidad_clicks = 0;
}

BotonUsuarioEventos BotonUsuario_Actualizar(uint8_t nivel, uint32_t ahora_ms)
{
    BotonUsuarioEventos eventos = {0};

    if (nivel_previo == 1 && nivel == 0) {
        eventos.pulsacion_iniciada = 1;
        pulsacion_desde_ms = ahora_ms;
    }

    /* Se dispara una vez mientras sigue presionado; al soltar no suma click. */
    if (nivel == 0 && pulsacion_desde_ms != 0 &&
        (ahora_ms - pulsacion_desde_ms) > BOTON_LARGO_MS) {
        eventos.pulsacion_larga = 1;
        pulsacion_desde_ms = 0;
    }

    if (nivel_previo == 0 && nivel == 1) {
        uint32_t sostenido_ms = ahora_ms - pulsacion_desde_ms;
        if (pulsacion_desde_ms != 0 && sostenido_ms > BOTON_CLICK_MIN_MS) {
            uint32_t desde_ultimo_ms = ahora_ms - ultimo_click_ms;
            ultimo_click_ms = ahora_ms;
            if (desde_ultimo_ms < BOTON_SECUENCIA_MS && cantidad_clicks > 0)
                cantidad_clicks++;
            else
                cantidad_clicks = 1;
        }
        pulsacion_desde_ms = 0;
    }

    /* Mismo orden y umbrales estrictos del loop original. */
    if (cantidad_clicks > 0 && (ahora_ms - ultimo_click_ms) > BOTON_SECUENCIA_MS) {
        eventos.clicks = cantidad_clicks;
        cantidad_clicks = 0;
    }
    nivel_previo = nivel;
    return eventos;
}
