/* Transporte USB CDC de la aplicacion: RX hacia UNER, cola TX y debug.
 * El driver generado por CubeMX permanece en USB_DEVICE.
 */
#include "comunicacion_usb.h"
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include "UNER.h"
#include <stdarg.h>
#include <stddef.h>
#include <string.h>

#define USB_TX_BUFFER_TAMANO   512
#define USB_TX_BUFFER_MASCARA  (USB_TX_BUFFER_TAMANO-1)

extern USBD_HandleTypeDef hUsbDeviceFS;
static uint8_t usb_buffer_tx[USB_TX_BUFFER_TAMANO];
static volatile uint16_t tx_cabeza = 0;
static volatile uint16_t tx_cola = 0;
static volatile uint8_t usb_tx_ocupado = 0;
static volatile uint32_t usb_tx_descartados = 0;
static const char DIGITOS_HEX[] = "0123456789ABCDEF";	// Tabla de dígitos hex para USB

static void USBRxData(uint8_t *buf, int len) {
    if (buf == NULL || len <= 0) return;

    for (int i = 0; i < len; i++)
        UNER_PushByte(buf[i]);
}

// Encola el mensaje completo; usb_service_tx lo transmite en paquetes de hasta 64 bytes.
void USB_DebugSend(const uint8_t *data, uint16_t len) {
    if (data != NULL && len != 0U)
        (void)usb_enqueue_tx(data, len);
}

// Envía una cadena literal
void USB_DebugStr(const char *s) {
    if (s != NULL)
        USB_DebugSend((const uint8_t *)s, (uint16_t)strlen(s));
}

// Envía un byte como dos dígitos hex ASCII
void USB_DebugHex(uint8_t b) {
    char h[2] = { DIGITOS_HEX[b >> 4], DIGITOS_HEX[b & 0xF] };
    USB_DebugSend((const uint8_t *)h, sizeof(h));
}

static uint16_t USB_AppendUInt(char *dst, uint16_t pos, uint16_t capacity,
                               unsigned int value, unsigned int base) {
    char reversed[10];
    uint8_t count = 0;

    do {
        reversed[count++] = DIGITOS_HEX[value % base];
        value /= base;
    } while (value != 0U && count < sizeof(reversed));

    while (count != 0U && pos < capacity)
        dst[pos++] = reversed[--count];

    return pos;
}

// Mini-printf acotado: construye cada log completo antes de encolarlo.
void USB_Debug(const char *fmt, ...) {
    char out[192];
    uint16_t pos = 0;
    va_list ap;

    if (fmt == NULL) return;
    va_start(ap, fmt);

    while (*fmt && pos < sizeof(out)) {
        if (*fmt == '%') {
            fmt++;
            if (*fmt == '\0') break;
            switch (*fmt) {
                case 's': {
                    const char *s = va_arg(ap, const char *);
                    if (s == NULL) s = "(null)";
                    while (*s && pos < sizeof(out)) out[pos++] = *s++;
                    break;
                }
                case 'c': {
                    out[pos++] = (char)va_arg(ap, int);
                    break;
                }
                case 'u': {
                    pos = USB_AppendUInt(out, pos, sizeof(out),
                                         va_arg(ap, unsigned int), 10U);
                    break;
                }
                case 'd': {
                    int value = va_arg(ap, int);
                    unsigned int magnitude;
                    if (value < 0) {
                        if (pos < sizeof(out)) out[pos++] = '-';
                        magnitude = 0U - (unsigned int)value;
                    } else {
                        magnitude = (unsigned int)value;
                    }
                    pos = USB_AppendUInt(out, pos, sizeof(out), magnitude, 10U);
                    break;
                }
                case 'X': {
                    pos = USB_AppendUInt(out, pos, sizeof(out),
                                         va_arg(ap, unsigned int), 16U);
                    break;
                }
                case '%':
                    out[pos++] = '%';
                    break;
                default:
                    if (pos < sizeof(out)) out[pos++] = '%';
                    if (pos < sizeof(out)) out[pos++] = *fmt;
                    break;
            }
        } else {
            out[pos++] = *fmt;
        }
        fmt++;
    }

    va_end(ap);
    USB_DebugSend((const uint8_t *)out, pos);
}


uint8_t usb_enqueue_tx(const uint8_t *data, uint16_t len) {
    return usb_enqueue_tx_segments(data, len, NULL, 0U);
}

uint8_t usb_enqueue_tx_segments(const uint8_t *first, uint16_t largo_primero,
                                const uint8_t *second, uint16_t largo_segundo) {
    uint32_t primask;
    uint16_t used;
    uint16_t free_space;
    uint16_t largo_total = (uint16_t)(largo_primero + largo_segundo);

    if (largo_total == 0U) return 1;
    if ((largo_primero != 0U && first == NULL) || (largo_segundo != 0U && second == NULL))
        return 0;

    primask = __get_PRIMASK();
    __disable_irq();
    used = (uint16_t)((tx_cabeza - tx_cola) & USB_TX_BUFFER_MASCARA);   // Buffer circular por mascara: bytes pendientes de enviar
    free_space = (uint16_t)(USB_TX_BUFFER_MASCARA - used);   // Espacio libre = capacidad total - ocupado
    if (largo_total > free_space) {
        usb_tx_descartados++;
        if (!primask) __enable_irq();
        return 0;
    }

    for (uint16_t i = 0; i < largo_primero; i++) {
        usb_buffer_tx[tx_cabeza] = first[i];
        tx_cabeza = (uint16_t)((tx_cabeza + 1U) & USB_TX_BUFFER_MASCARA);
    }
    for (uint16_t i = 0; i < largo_segundo; i++) {
        usb_buffer_tx[tx_cabeza] = second[i];
        tx_cabeza = (uint16_t)((tx_cabeza + 1U) & USB_TX_BUFFER_MASCARA);
    }
    if (!primask) __enable_irq();

    if (usb_tx_ocupado == 0) {
        usb_service_tx();
    }
    return 1;
}

void usb_service_tx(void) {
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED || hcdc == NULL)
        return;

    if (usb_tx_ocupado && hcdc->TxState == 0) {
        usb_tx_ocupado = 0;
    }

    // 2) Si aún está ocupada la línea o no hay datos, no hacemos nada
    if (usb_tx_ocupado || tx_cabeza == tx_cola) {
        return;
    }

    // 3) Preparamos el siguiente chunk y lo enviamos.
    // static: CDC_Transmit_FS solo guarda el puntero y la transferencia USB ocurre
    // después de retornar — un buffer de stack ya liberado corrompería los datos.
    static uint8_t chunk[64];
    uint16_t cnt = 0;
    uint16_t read_index = tx_cola;
    while (cnt < sizeof(chunk) && read_index != tx_cabeza) {
        chunk[cnt++] = usb_buffer_tx[read_index];
        read_index = (uint16_t)((read_index + 1U) & USB_TX_BUFFER_MASCARA);
    }

    // Solo consumir bytes si el driver aceptó realmente la transferencia.
    if (cnt && CDC_Transmit_FS(chunk, cnt) == USBD_OK) {
        tx_cola = read_index;
        usb_tx_ocupado = 1;
    }
}

void USB_Comunicacion_Init(void)
{
    CDC_Attach_Rx(USBRxData);
}
