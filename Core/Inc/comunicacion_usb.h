#ifndef COMUNICACION_USB_H
#define COMUNICACION_USB_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Conecta RX de CDC con UNER; llamar en el arranque, tras MX_USB_DEVICE_Init. */
void USB_Comunicacion_Init(void);
/* Avanza TX sin bloquear. Llamar en cada pasada del loop principal. */
void usb_service_tx(void);
/* Copia todos los bytes a la cola o rechaza el mensaje entero si no cabe.
 * Los buffers del llamador pueden reutilizarse al retornar. 1 = aceptado.
 * Dos segmentos permiten enviar una trama UNER que cruza su buffer circular.
 */
uint8_t usb_enqueue_tx(const uint8_t *data, uint16_t len);
uint8_t usb_enqueue_tx_segments(const uint8_t *first, uint16_t largo_primero,
                                const uint8_t *second, uint16_t largo_segundo);

/* Depuracion compartida. Usa la misma cola; no espera a la transferencia. */
void USB_DebugSend(const uint8_t *data, uint16_t len);
void USB_DebugStr(const char *s);
void USB_DebugHex(uint8_t b);
/* Mini-printf acotado: %s, %c, %d, %u, %X y %%; no admite floats. */
void USB_Debug(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif
