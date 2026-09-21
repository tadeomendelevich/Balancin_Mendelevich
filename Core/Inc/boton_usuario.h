#ifndef BOTON_USUARIO_H
#define BOTON_USUARIO_H

#include <stdint.h>

typedef struct {
    uint8_t pulsacion_iniciada;
    uint8_t pulsacion_larga;
    uint8_t clicks;  /* 0: secuencia sin resolver; 1..255: cantidad resuelta. */
} BotonUsuarioEventos;

/* KEY activo en bajo: 0 presionado, 1 libre. Si arranca presionado,
 * la primera liberacion no genera un click (seleccion SoftAP al encender).
 */
void BotonUsuario_Init(uint8_t nivel_inicial);
/* Llamar cada 10 ms desde el loop, con HAL_GetTick() como ahora_ms.
 * Devuelve eventos; no cambia modos, pantallas ni comandos de motor.
 */
BotonUsuarioEventos BotonUsuario_Actualizar(uint8_t nivel, uint32_t ahora_ms);

#endif
