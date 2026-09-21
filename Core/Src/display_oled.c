#include "display_oled.h"
#include "ssd1306.h"
#include "i2c_manager.h"
#include "ESP01.h"
#include "UNER.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define PANTALLA_ANCHO SSD1306_WIDTH
#define PANTALLA_ALTO SSD1306_HEIGHT
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define DISPLAY_INTERVALO_IDLE_MS    100U
#define DISPLAY_INTERVALO_ACTIVO_MS  60U

static volatile uint32_t wifi_splash_hasta_ms = 0;
static uint32_t ultimo_display_ms = 0;

/* Depuracion USB compartida con el resto del firmware. */
extern void USB_Debug(const char *fmt, ...);
extern void USB_DebugHex(uint8_t b);

static void my_ssd1306_init(void *ctx);
static int my_ssd1306_write_cmd(void *ctx, uint8_t cmd);
static int my_ssd1306_write_data(void *ctx, const uint8_t *data, uint16_t len);
static int my_ssd1306_write_data_async(void *ctx, const uint8_t *data, uint16_t len);
static uint8_t my_ssd1306_is_busy(void *ctx);
static void my_ssd1306_errorCb(void *ctx, int err);
static void my_ssd1306_delay_ms(void *ctx, uint32_t ms);

static SSD1306_Ctx_t ssd_ctx;

static const SSD1306_Platform_t SSD1306_plat = {
  .ctx              = &ssd_ctx,
  .init             = my_ssd1306_init,
  .write_cmd        = my_ssd1306_write_cmd,
  .write_data       = my_ssd1306_write_data,
  .write_data_async = my_ssd1306_write_data_async,
  .is_busy          = my_ssd1306_is_busy,
  .delay_ms         = my_ssd1306_delay_ms,
  .onError          = my_ssd1306_errorCb
};

static void my_ssd1306_init(void *ctx) {
    (void)ctx;  // I2C1 ya fue inicializado por main.
}

static int my_ssd1306_write_cmd(void *ctx, uint8_t cmd) {
    SSD1306_Ctx_t *c = (SSD1306_Ctx_t*)ctx;
    uint8_t buf[2] = {0x00, cmd};

    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(
        c->hi2c,
        SSD1306_I2C_ADDR,
        buf,
        2,
        HAL_MAX_DELAY
    );

    if (st != HAL_OK) {
        SSD1306_plat.onError(ctx, (int)st);
        return -1;
    }
    return 0;
}

// Callback datos bloqueante
static int my_ssd1306_write_data(void *ctx, const uint8_t *data, uint16_t len) {
    // Desempaquetar el contexto
    SSD1306_Ctx_t *c = (SSD1306_Ctx_t*)ctx;
    // Preparo buffer con control byte + datos
    uint8_t buf[1 + SSD1306_WIDTH];
    buf[0] = 0x40;
    memcpy(&buf[1], data, len);
    // Transmisión bloqueante
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(
        c->hi2c,
        SSD1306_I2C_ADDR,
        buf,
        len + 1,
        HAL_MAX_DELAY
    );
    if (st != HAL_OK) {
        // Notifico el fallo (p.ej. por USB y LED)
        SSD1306_plat.onError(ctx, (int)st);
        return -1;
    }
    return 0;
}

// Callback datos no bloqueante (DMA)
static void ssd1306_dma_done_cb(void *context, HAL_StatusTypeDef status)
{
    SSD1306_Ctx_t *c = (SSD1306_Ctx_t*)context;
    (void)c;

    if (status != HAL_OK) {
        SSD1306_plat.onError(context, (int)status);
        *ssd_ctx.busy_flag = I2C_Manager_IsBusy();
        return;
    }

    *ssd_ctx.busy_flag = I2C_Manager_IsBusy();
}

static int my_ssd1306_write_data_async(void *ctx, const uint8_t *data, uint16_t len)
{
    SSD1306_Ctx_t *c = (SSD1306_Ctx_t*)ctx;

    static uint8_t dmaBufPool[4][1 + SSD1306_WIDTH];
    static uint8_t dmaBufIndex = 0;

    uint8_t *dmaBuf = dmaBufPool[dmaBufIndex];
    dmaBufIndex = (dmaBufIndex + 1) & 0x03;

    dmaBuf[0] = 0x40;
    memcpy(&dmaBuf[1], data, len);

    I2C_Request_t req = {
        .type       = I2C_REQ_MASTER_TX_DMA,
        .hi2c       = c->hi2c,
        .devAddr    = SSD1306_I2C_ADDR,
        .memAddr    = 0,
        .memAddSize = 0,
        .data       = dmaBuf,
        .len        = len + 1,
        .callback   = ssd1306_dma_done_cb,
        .context    = c
    };

    if (!I2C_Manager_Enqueue(&req)) {
        SSD1306_plat.onError(ctx, -1);
        return -1;
    }

    I2C_Manager_Process();
    *ssd_ctx.busy_flag = I2C_Manager_IsBusy();
    return 0;
}


// Callback busy-check
static uint8_t my_ssd1306_is_busy(void *ctx) {
    (void)ctx;
    return I2C_Manager_IsBusy() ? 1 : 0;
}

static void my_ssd1306_errorCb(void *ctx, int err) {
	USB_Debug("ERROR SSD1306: 0x");
	USB_DebugHex(err);
	USB_Debug("\r\n");
	*ssd_ctx.busy_flag = I2C_Manager_IsBusy();
	SSD1306_ResetUpdateState();
}

// Callback delay
static void my_ssd1306_delay_ms(void *ctx, uint32_t ms) {
    HAL_Delay(ms);
}

void Pantalla_Init(I2C_HandleTypeDef *hi2c, uint8_t *busy_flag)
{
    ssd_ctx.hi2c = hi2c;
    ssd_ctx.busy_flag = busy_flag;
    SSD1306_RegisterPlatform(&SSD1306_plat);
    SSD1306_Init();
    SSD1306_DrawBitmap(0, 0, unerLogo, 128, 64, SSD1306_COLOR_WHITE);
    SSD1306_UpdateScreen_Blocking();
}

void Pantalla_MostrarWifi(uint32_t duracion_ms)
{
    wifi_splash_hasta_ms = HAL_GetTick() + duracion_ms;
}

void Pantalla_ForzarActualizacion(void)
{
    ultimo_display_ms = 0;
}

uint8_t Pantalla_Procesar(uint8_t reposo_o_caida,
                         const volatile uint8_t *lectura_imu_pendiente,
                         const volatile uint8_t *dato_imu_listo)
{
    SSD1306_UpdateScreen();
    if (SSD1306_IsUpdateDone() && !*ssd_ctx.busy_flag &&
        !*lectura_imu_pendiente && !*dato_imu_listo) {
        uint32_t now = HAL_GetTick();
        uint32_t intervalo = reposo_o_caida
                ? DISPLAY_INTERVALO_IDLE_MS : DISPLAY_INTERVALO_ACTIVO_MS;
        if ((now - ultimo_display_ms) >= intervalo) {
            ultimo_display_ms = now;
            return 1;
        }
    }
    return 0;
}

static float clampf_local(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;   // Saturacion (clamp): recorta a la banda [min,max]
    if (value < min_value) return min_value;
    return value;
}

// Zona muerta: devuelve cuanto se PASO del umbral, no el valor entero.
// Asi la correccion arranca en 0 al cruzarlo, sin escalon.
static void FormatSignedFixed(char *buf, size_t buf_size, float value, uint8_t decimals)
{
    int32_t scale = 1;
    for (uint8_t i = 0; i < decimals; i++) {
        scale *= 10;
    }

    int32_t scaled = (value >= 0.0f)                       // Aritmetica de punto fijo: se corre la coma multiplicando por 10^decimales
                   ? (int32_t)(value * (float)scale + 0.5f)  // El +/-0.5 antes de truncar = redondeo al entero mas cercano
                   : (int32_t)(value * (float)scale - 0.5f);

    uint32_t escalado_abs = (scaled < 0) ? (uint32_t)(-scaled) : (uint32_t)scaled;
    uint32_t int_part   = escalado_abs / (uint32_t)scale;   // Division entera: parte entera
    uint32_t frac_part  = escalado_abs % (uint32_t)scale;   // Resto (modulo): parte decimal
    char sign           = (scaled < 0) ? '-' : '+';

    // Acotar rangos explícitamente: el valor mostrado nunca necesita más de
    // 5 dígitos enteros, y frac_part ya es < scale — el módulo redundante
    // le deja claro el rango al compilador (evita -Wformat-truncation).
    if (int_part > 99999U) int_part = 99999U;

    switch (decimals) {
        case 0:
            snprintf(buf, buf_size, "%c%lu", sign, (unsigned long)int_part);
            break;
        case 1:
            snprintf(buf, buf_size, "%c%lu.%01lu", sign,
                     (unsigned long)int_part, (unsigned long)(frac_part % 10U));
            break;
        case 2:
            snprintf(buf, buf_size, "%c%lu.%02lu", sign,
                     (unsigned long)int_part, (unsigned long)(frac_part % 100U));
            break;
        default:
            snprintf(buf, buf_size, "%c%lu.%03lu", sign,
                     (unsigned long)int_part, (unsigned long)(frac_part % 1000U));
            break;
    }
}

// ─────────────────────────────────────────────────────────────────────
// Helpers de dibujo del display (usados solo por updateDisplay)
// ─────────────────────────────────────────────────────────────────────
static uint8_t oled_alive_fase = 0;   // parpadeo del punto "vivo" del header

// Dibuja un string en 5x7. OJO: SSD1306_DrawChar5x7 solo tiene glifos para
// 0-9 y A-Z — acá se mapean las minúsculas a mayúsculas y se dibuja la
// puntuación común a mano (píxeles), si no esos caracteres desaparecen.
static void OLED_Str5(uint16_t x, uint16_t y, const char *s)
{
    for (; *s; s++) {
        char c = *s;
        if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
        switch (c) {
            case ':':
                SSD1306_DrawPixel(x + 2, y + 2, SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(x + 2, y + 5, SSD1306_COLOR_WHITE);
                break;
            case '.':
                SSD1306_DrawPixel(x + 2, y + 6, SSD1306_COLOR_WHITE);
                break;
            case '-':
                SSD1306_DrawLine(x + 1, y + 3, x + 3, y + 3, SSD1306_COLOR_WHITE);
                break;
            case '+':
                SSD1306_DrawLine(x,     y + 3, x + 4, y + 3, SSD1306_COLOR_WHITE);
                SSD1306_DrawLine(x + 2, y + 1, x + 2, y + 5, SSD1306_COLOR_WHITE);
                break;
            case '*':
                SSD1306_DrawLine(x,     y + 1, x + 4, y + 5, SSD1306_COLOR_WHITE);
                SSD1306_DrawLine(x + 4, y + 1, x,     y + 5, SSD1306_COLOR_WHITE);
                break;
            case '/':
                SSD1306_DrawLine(x, y + 6, x + 4, y, SSD1306_COLOR_WHITE);
                break;
            case '\\':
                SSD1306_DrawLine(x, y, x + 4, y + 6, SSD1306_COLOR_WHITE);
                break;
            case '|':
                SSD1306_DrawLine(x + 2, y, x + 2, y + 6, SSD1306_COLOR_WHITE);
                break;
            case '<':
                SSD1306_DrawLine(x + 3, y + 1, x + 1, y + 3, SSD1306_COLOR_WHITE);
                SSD1306_DrawLine(x + 1, y + 3, x + 3, y + 5, SSD1306_COLOR_WHITE);
                break;
            case '>':
                SSD1306_DrawLine(x + 1, y + 1, x + 3, y + 3, SSD1306_COLOR_WHITE);
                SSD1306_DrawLine(x + 3, y + 3, x + 1, y + 5, SSD1306_COLOR_WHITE);
                break;
            case '^':
                SSD1306_DrawLine(x,     y + 3, x + 2, y + 1, SSD1306_COLOR_WHITE);
                SSD1306_DrawLine(x + 2, y + 1, x + 4, y + 3, SSD1306_COLOR_WHITE);
                break;
            case '=':
                SSD1306_DrawLine(x + 1, y + 2, x + 3, y + 2, SSD1306_COLOR_WHITE);
                SSD1306_DrawLine(x + 1, y + 4, x + 3, y + 4, SSD1306_COLOR_WHITE);
                break;
            case '!':
                SSD1306_DrawLine(x + 2, y, x + 2, y + 4, SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(x + 2, y + 6, SSD1306_COLOR_WHITE);
                break;
            default:
                SSD1306_DrawChar5x7(c, x, y);   // 0-9, A-Z; espacio y otros: nada
                break;
        }
        x += (uint16_t)(Font_5x7.FontWidth + 1);
    }
}

static uint16_t OLED_Str5W(const char *s)
{
    uint16_t w = 0;
    for (; *s; s++) w += (uint16_t)(Font_5x7.FontWidth + 1);
    return w;
}

static void OLED_Str5Centered(uint16_t y, const char *s)
{
    uint16_t w = OLED_Str5W(s);
    OLED_Str5((uint16_t)((w < PANTALLA_ANCHO) ? (PANTALLA_ANCHO - w) / 2 : 0), y, s);
}

static void OLED_Puts7CenteredX(const char *s, uint16_t x0, uint16_t x1, uint16_t y)
{
    uint16_t len = 0;
    for (const char *p = s; *p; p++) len++;
    uint16_t sw   = (len > 0) ? (uint16_t)(len * 8 - 1) : 0;
    uint16_t span = (uint16_t)(x1 - x0);
    uint16_t sx   = x0 + ((span > sw) ? (span - sw) / 2 : 0);
    SSD1306_GotoXY(sx, y);
    SSD1306_Puts(s, &Font_7x10, SSD1306_COLOR_WHITE);
}

static void OLED_WifiIcon7(const PantallaDatos *datos, uint16_t x, uint16_t y)   // 7x6 px
{
    if (datos->f_wifi_conectado) {
        SSD1306_DrawLine(x + 1, y,     x + 5, y,     SSD1306_COLOR_WHITE);
        SSD1306_DrawPixel(x,     y + 1, SSD1306_COLOR_WHITE);
        SSD1306_DrawPixel(x + 6, y + 1, SSD1306_COLOR_WHITE);
        SSD1306_DrawLine(x + 2, y + 2, x + 4, y + 2, SSD1306_COLOR_WHITE);
        SSD1306_DrawPixel(x + 1, y + 3, SSD1306_COLOR_WHITE);
        SSD1306_DrawPixel(x + 5, y + 3, SSD1306_COLOR_WHITE);
        SSD1306_DrawPixel(x + 3, y + 5, SSD1306_COLOR_WHITE);
    } else {
        SSD1306_DrawLine(x, y, x + 6, y + 5, SSD1306_COLOR_WHITE);
        SSD1306_DrawLine(x + 6, y, x, y + 5, SSD1306_COLOR_WHITE);
    }
}

// Header común a todas las pantallas: título a la izquierda; "F!" centrado
// si está caído; a la derecha modo actual, icono WiFi y spinner de actividad
// que gira mientras el display se sigue refrescando (loop vivo).
static void OLED_Header(const PantallaDatos *datos, const char *title)
{
    OLED_Str5(1, 1, title);

    const char *modo_texto;
    switch (datos->estado_robot) {
        case ROBOT_STATE_BALANCE_ONLY:      modo_texto = "BAL"; break;
        case ROBOT_STATE_BALANCE_AND_SPEED: modo_texto = "SPD"; break;
        case ROBOT_STATE_LINE_FOLLOWING:    modo_texto = "LIN"; break;
        case ROBOT_STATE_MANUAL_CONTROL:    modo_texto = "MAN"; break;
        case ROBOT_STATE_MOTOR_TEST:        modo_texto = "TST"; break;
        default:                            modo_texto = "IDL"; break;
    }

    // Spinner de actividad tamaño carácter (| / - \): mucho más visible que
    // un punto — si deja de girar, el loop/display está congelado.
    // (vía OLED_Str5: DrawChar5x7 no tiene glifos para estos caracteres)
    static const char spin_chars[4] = { '|', '/', '-', '\\' };
    char spin_buf[2] = { spin_chars[oled_alive_fase & 0x03], '\0' };
    uint16_t x = PANTALLA_ANCHO - 7;
    OLED_Str5(x, 1, spin_buf);

    x -= 10;                                  // icono wifi (7px + aire)
    OLED_WifiIcon7(datos, x, 1);

    x -= (uint16_t)(OLED_Str5W(modo_texto) + 3);
    OLED_Str5(x, 1, modo_texto);

    // Indicador de caída centrado en el header (antes seguía la pila de la
    // derecha y se mezclaba con el modo/icono WiFi).
    if (datos->f_caido) {
        OLED_Str5Centered(1, "F!");
    }

    SSD1306_DrawLine(0, 9, PANTALLA_ANCHO - 1, 9, SSD1306_COLOR_WHITE);
}

// Nombre corto del sub-estado del seguidor de línea (pantallas 1 y 6).
static const char *LineStateStr(const PantallaDatos *datos)
{
    if (datos->estado_robot != ROBOT_STATE_LINE_FOLLOWING) return "OFF";
    switch (datos->linea_estado) {
        case LINE_STATE_FOLLOWING:          return "SIGUE";
        case LINE_STATE_LOST:               return "PERDI";
        case LINE_STATE_SEARCHING:          return "BUSCA";
        case LINE_STATE_LOST_BRAKE:         return "FRENA";
        case LINE_STATE_LOST_ROTATE:        return "GIRO";
        case LINE_STATE_LOST_ASENTAR:        return "ESTAB";
        case LINE_STATE_LOST_AVANZA:           return "VUELVE";
        case LINE_STATE_EDGE_WAIT:          return "EFRENA";
        case LINE_STATE_EDGE_ROTATE:        return "EGIRO";
        case LINE_STATE_EDGE_ASENTAR:        return "EESTAB";
        case LINE_STATE_EDGE_AVANZA:           return "EVUELV";
        case LINE_STATE_GIVEN_UP:           return "PARADO";
        case LINE_STATE_PERPENDICULAR_ROTATE:        return "PGIRO";
        case LINE_STATE_OBJ_FRENO_REVERSA:  return "STOP";
        case LINE_STATE_OBJ_GIRO_ESQUIVE:   return "ESQUIV";
        case LINE_STATE_OBJ_PAUSA_GIRO:     return "PAUSA";
        case LINE_STATE_OBJ_BUSCAR_PARED:   return "APRCH";
        case LINE_STATE_OBJ_BORDEAR_PARED:  return "PARED";
        case LINE_STATE_OBJ_PARED_LIBRE:    return "LIBRE";
        case LINE_STATE_OBJ_GIRO_PARED:     return "GIRAP";
        default:                            return "UNK";
    }
}

// Acción vigente del wall-following (pantalla 6), calculada directo de los
// umbrales crudos de ADC7 — exactamente lo que deciden los 3 estados de pared
// cada ciclo. Solo tiene sentido durante PARED/LIBRE/GIRAP.
static const char *ObjWallActionStr(const PantallaDatos *datos)
{
    float pared_adc = (float)datos->adc_mediana[datos->obj_pared_adc_indice];
    if (pared_adc < datos->obj_pared_reversa_umbral)   return "REV";
    if (pared_adc < datos->obj_pared_muy_cerca_umbral) return "PIV";
    return "AVZ";
}

// Tiempo transcurrido en el sub-estado actual del seguidor. Se mide acá,
// al ritmo de refresco del display — suficiente para debug visual.
static uint32_t OLED_LineStateElapsedMs(const PantallaDatos *datos)
{
    static eLineState oled_prev_lstate = LINE_STATE_FOLLOWING;
    static uint32_t   oled_lstate_t0   = 0;
    if (datos->linea_estado != oled_prev_lstate) {
        oled_prev_lstate = datos->linea_estado;
        oled_lstate_t0   = HAL_GetTick();
    }
    return HAL_GetTick() - oled_lstate_t0;
}

// Barra horizontal con marco, relleno proporcional (escala ADC 0..4095) y
// ticks de umbral dibujados en color invertido respecto al relleno para
// que se vean tanto sobre la parte llena como sobre la vacía.
static void OLED_HBar(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                      uint16_t value, const float *ticks, uint8_t n_ticks)
{
    if (value > 4095) value = 4095;
    SSD1306_DrawRectangle(x, y, w, h, SSD1306_COLOR_WHITE);
    uint16_t fill = (uint16_t)((uint32_t)value * (uint32_t)(w - 2) / 4095U);   // Regla de tres: cuentas de ADC -> pixeles de ancho
    if (fill > 0)
        SSD1306_DrawFilledRectangle(x + 1, y + 1, fill, h - 2, SSD1306_COLOR_WHITE);
    for (uint8_t i = 0; i < n_ticks; i++) {
        uint16_t tx = x + 1 + (uint16_t)((uint32_t)ticks[i] * (uint32_t)(w - 2) / 4095U);
        SSD1306_COLOR_t tc = (tx <= x + fill) ? SSD1306_COLOR_BLACK : SSD1306_COLOR_WHITE;
        SSD1306_DrawLine(tx, y + 1, tx, y + h - 2, tc);
    }
}

void Pantalla_Dibujar(const PantallaDatos *datos) {
    SSD1306_Fill(SSD1306_COLOR_BLACK);
    oled_alive_fase++;

    char nbuf[16];
    char nbuf2[16];
    char lbuf[40];
    uint8_t fall_alert_visible = datos->f_caido && datos->caida_alerta_hasta_ms != 0 &&
        ((int32_t)(datos->caida_alerta_hasta_ms - HAL_GetTick()) > 0);

    // Máxima prioridad visual: confirmar la conexión WiFi aunque el robot
    // esté caído o haya disparado el límite de velocidad.
    if (wifi_splash_hasta_ms != 0) {
        if ((int32_t)(wifi_splash_hasta_ms - HAL_GetTick()) > 0) {
            OLED_Header(datos, ESP01_GetMode() == ESP01_MODE_SOFTAP ? "WIFI DIRECTO" : "WIFI ROUTER");
            OLED_Puts7CenteredX(ESP01_StateWIFI() != ESP01_WIFI_CONNECTED ? "INICIANDO" :
                (ESP01_HasPeer() ? "QT CONECTADO" : "ESPERANDO QT"), 0, PANTALLA_ANCHO - 1, 14);

            char ssid_buf[22];
            uint8_t si = 0;
            for (; wifiSSID[si] != '\0' && si < sizeof(ssid_buf) - 1; si++)
                ssid_buf[si] = wifiSSID[si];
            ssid_buf[si] = '\0';
            OLED_Str5Centered(30, "RED:");
            OLED_Str5Centered(38, ssid_buf);

            const char *lip = ESP01_GetLocalIP();
            char ip_buf[24];
            uint8_t bi = 0;
            const char *pfx = "IP: ";
            for (; *pfx; pfx++) ip_buf[bi++] = *pfx;
            for (const char *p = (lip != NULL) ? lip : "...";
                 *p != '\0' && bi < sizeof(ip_buf) - 1; p++)
                ip_buf[bi++] = *p;
            ip_buf[bi] = '\0';
            OLED_Str5Centered(48, ip_buf);

            bi = 0;
            pfx = "PC: ";
            for (; *pfx; pfx++) ip_buf[bi++] = *pfx;
            for (const char *p = ESP01_HasPeer() ? ESP01_GetPeerIP() : "SIN CLIENTE"; *p != '\0' && bi < sizeof(ip_buf) - 1; p++)
                ip_buf[bi++] = *p;
            ip_buf[bi] = '\0';
            OLED_Str5Centered(56, ip_buf);

            SSD1306_RequestUpdate();
            return;
        }
        wifi_splash_hasta_ms = 0;
    }

    // La seguridad queda enclavada, pero la PANTALLA de alarma es temporal y
    // descartable con KEY para no impedir seleccionar otro modo.
    if (datos->vel_limite_falla && fall_alert_visible) {
        const char *titulo_velocidad = (datos->estado_robot == ROBOT_STATE_BALANCE_ONLY)   ? "BALANCE" :
                                  (datos->estado_robot == ROBOT_STATE_MANUAL_CONTROL) ? "MANUAL"  :
                                  (datos->estado_robot == ROBOT_STATE_LINE_FOLLOWING) ? "LINEA"   :
                                                                                "ALARMA";
        OLED_Header(datos, titulo_velocidad);
        OLED_Puts7CenteredX("LIMITE DE", 0, PANTALLA_ANCHO - 1, 14);
        OLED_Puts7CenteredX("VELOCIDAD", 0, PANTALLA_ANCHO - 1, 27);
        OLED_Puts7CenteredX("EXCEDIDO", 0, PANTALLA_ANCHO - 1, 40);
        FormatSignedFixed(nbuf, sizeof(nbuf), datos->vel_limite_disparo, 2);
        snprintf(lbuf, sizeof(lbuf), "V:%s M/S", nbuf);
        OLED_Str5Centered(55, lbuf);
        SSD1306_RequestUpdate();
        return;
    }

    // Caída por inclinación: muestra una foto de los valores ANTERIORES al
    // reset del controlador. Tiene prioridad sobre las pantallas normales.
    if (fall_alert_visible) {
        const char *reason = (datos->caida_motivo == FALL_REASON_UPSIDE_DOWN)   ? "BOCA ABAJO" :
                             (datos->caida_motivo == FALL_REASON_CRITICAL_ZONE) ? "ANGULO CRITICO" :
                             (datos->caida_motivo == FALL_REASON_SPEED)         ? "VELOCIDAD" :
                                                                              "INCLINACION";
        OLED_Header(datos, "CAIDA");
        OLED_Puts7CenteredX("ROBOT CAIDO", 0, PANTALLA_ANCHO - 1, 14);
        snprintf(lbuf, sizeof(lbuf), "CAUSA:%s", reason);
        OLED_Str5Centered(29, lbuf);

        FormatSignedFixed(nbuf, sizeof(nbuf), datos->caida_angulo, 1);
        FormatSignedFixed(nbuf2, sizeof(nbuf2), datos->caida_giro, 0);
        snprintf(lbuf, sizeof(lbuf), "ANG:%s G:%s", nbuf, nbuf2);
        OLED_Str5Centered(41, lbuf);

        FormatSignedFixed(nbuf, sizeof(nbuf), datos->caida_velocidad, 2);
        FormatSignedFixed(nbuf2, sizeof(nbuf2), datos->caida_pwm, 0);
        snprintf(lbuf, sizeof(lbuf), "V:%s PWM:%s", nbuf, nbuf2);
        OLED_Str5Centered(53, lbuf);
        SSD1306_RequestUpdate();
        return;
    }

    if (datos->f_cambiar_pantalla == 0) {
        // ───────────────────────────────────────────────────────────
        // PANTALLA 0 — BALANCE: gauge de roll + números clave del PID
        // ───────────────────────────────────────────────────────────
        OLED_Header(datos, "BALANCE");
        SSD1306_DrawLine(63, 11, 63, PANTALLA_ALTO - 1, SSD1306_COLOR_WHITE);

        // ── Izquierda: ángulo grande + inclinómetro horizontal ±15° ──
        OLED_Str5(2, 13, "ANG");
        FormatSignedFixed(nbuf, sizeof(nbuf), datos->roll_filtrado_grados, 1);
        SSD1306_GotoXY(2, 21);
        SSD1306_Puts(nbuf, &Font_7x10, SSD1306_COLOR_WHITE);

        {
            const uint16_t gx0 = 2, gy0 = 34, gw = 58, gh = 9;
            SSD1306_DrawRectangle(gx0, gy0, gw, gh, SSD1306_COLOR_WHITE);
            // tick central (0°) por encima del marco
            SSD1306_DrawLine(gx0 + gw / 2, gy0 - 2, gx0 + gw / 2, gy0 - 1, SSD1306_COLOR_WHITE);
            // marcador de roll actual (±15° a fondo de escala)
            float rr = clampf_local(datos->roll_filtrado_grados, -15.0f, 15.0f);
            int16_t moff = (int16_t)(rr * (float)(gw / 2 - 3) / 15.0f);   // Regla de tres: grados -> pixeles de desplazamiento
            uint16_t mx = (uint16_t)((int16_t)(gx0 + gw / 2) + moff);   // Centro del marco + offset con signo
            SSD1306_DrawFilledRectangle(mx - 1, gy0 + 2, 3, gh - 4, SSD1306_COLOR_WHITE);
            // marcador del setpoint (tick corto debajo del marco)
            float ss = clampf_local(datos->setpoint_dinamico_final, -15.0f, 15.0f);
            int16_t soff = (int16_t)(ss * (float)(gw / 2 - 3) / 15.0f);
            uint16_t sxp = (uint16_t)((int16_t)(gx0 + gw / 2) + soff);
            SSD1306_DrawLine(sxp, gy0 + gh + 1, sxp, gy0 + gh + 2, SSD1306_COLOR_WHITE);
        }

        FormatSignedFixed(nbuf, sizeof(nbuf), datos->setpoint_dinamico_final, 2);
        snprintf(lbuf, sizeof(lbuf), "SP:%s", nbuf);
        OLED_Str5(2, 49, lbuf);
        FormatSignedFixed(nbuf, sizeof(nbuf), datos->setpoint_trim, 1);
        snprintf(lbuf, sizeof(lbuf), "TR:%s", nbuf);
        OLED_Str5(2, 57, lbuf);

        // ── Derecha: ganancias PID + velocidad + motores ──
        {
            const uint16_t rx = 67;
            FormatSignedFixed(nbuf, sizeof(nbuf), datos->KP_value, 3);
            snprintf(lbuf, sizeof(lbuf), "KP:%s", nbuf + 1);  OLED_Str5(rx, 12, lbuf);
            FormatSignedFixed(nbuf, sizeof(nbuf), datos->KI_value, 3);
            snprintf(lbuf, sizeof(lbuf), "KI:%s", nbuf + 1);  OLED_Str5(rx, 21, lbuf);
            FormatSignedFixed(nbuf, sizeof(nbuf), datos->KD_value, 3);
            snprintf(lbuf, sizeof(lbuf), "KD:%s", nbuf + 1);  OLED_Str5(rx, 30, lbuf);
            FormatSignedFixed(nbuf, sizeof(nbuf), datos->velocidad_est_ema, 2);
            snprintf(lbuf, sizeof(lbuf), "VE:%s", nbuf);      OLED_Str5(rx, 39, lbuf);

            if (datos->estado_robot == ROBOT_STATE_MANUAL_CONTROL) {
                uint8_t cmd = UNER_GetLastManualCmd();
                char c = (cmd == MOVE_FORWARD)  ? '^'
                       : (cmd == MOVE_BACKWARD) ? 'v'
                       : (cmd == MOVE_LEFT)     ? '<'
                       : (cmd == MOVE_RIGHT)    ? '>' : 'o';
                snprintf(lbuf, sizeof(lbuf), "CMD:%c", c);
            } else {
                // comando de motores (PWM firmado, R y L)
                snprintf(lbuf, sizeof(lbuf), "M%+03d%+03d",
                         (int)datos->motor_derecho_velocidad, (int)datos->motor_izquierdo_velocidad);
            }
            OLED_Str5(rx, 57, lbuf);
        }

    }  else if (datos->f_cambiar_pantalla == 1) {
        // ───────────────────────────────────────────────────────────
        // PANTALLA 1 — LINEA: barras ADC + posición de línea + estado
        // ───────────────────────────────────────────────────────────
        OLED_Header(datos, "LINEA");

        // ── Izquierda: 8 barras (1-4 línea con indicador B/N, 5-8 objeto) ──
        {
            const uint16_t bar_top   = 12;
            const uint16_t ind_h     = 4;
            const uint16_t digit_y   = 47;
            const uint16_t bar_width = 7;
            const uint16_t spacing   = 1;
            const uint16_t bar_line_top = bar_top + ind_h;

            for (uint8_t i = 0; i < 8; i++) {
                // ADC 1-4 en orden invertido (coincide con la vista del robot)
                uint8_t adc_idx = (i < 4) ? (3 - i) : i;
                uint16_t v  = datos->adc_mediana[adc_idx] > 4095 ? 4095 : datos->adc_mediana[adc_idx];
                uint16_t x0 = spacing + i * (bar_width + spacing);

                if (i < 4) {
                    uint16_t hmax = digit_y - 1 - bar_line_top;
                    uint16_t h = (uint32_t)v * hmax / 4095;
                    if (h > 0)
                        SSD1306_DrawFilledRectangle(x0, digit_y - 1 - h, bar_width, h, SSD1306_COLOR_WHITE);
                    // indicador negro/blanco: relleno = ve cinta negra
                    if (datos->adc_mediana[adc_idx] > (uint16_t)datos->LINEA_UMBRAL_ADC)
                        SSD1306_DrawFilledRectangle(x0, bar_top, bar_width, ind_h - 1, SSD1306_COLOR_WHITE);
                    else
                        SSD1306_DrawRectangle(x0, bar_top, bar_width, ind_h - 1, SSD1306_COLOR_WHITE);
                } else {
                    uint16_t hmax = digit_y - 1 - bar_top;
                    uint16_t h = (uint32_t)v * hmax / 4095;
                    if (h > 0)
                        SSD1306_DrawFilledRectangle(x0, digit_y - 1 - h, bar_width, h, SSD1306_COLOR_WHITE);
                    // tick del umbral de objeto, en color invertido si la barra lo cubre
                    uint16_t hth = (uint16_t)((uint32_t)datos->obj_deteccion_umbral_adc * hmax / 4095U);
                    uint16_t yth = digit_y - 1 - hth;
                    SSD1306_COLOR_t tc = (h >= hth) ? SSD1306_COLOR_BLACK : SSD1306_COLOR_WHITE;
                    SSD1306_DrawLine(x0, yth, x0 + bar_width - 1, yth, tc);
                }
                if (adc_idx < 4 && datos->linea_canal_cuarentena[adc_idx])
                    SSD1306_DrawChar5x7('X', x0 + 1, digit_y);   // sensor en cuarentena
                else
                    SSD1306_DrawChar5x7('1' + adc_idx, x0 + 1, digit_y);
            }

            // separador punteado entre grupo línea (1-4) y objeto (5-8)
            uint16_t sep_x = spacing + 4 * (bar_width + spacing) - 1;
            for (uint16_t py = bar_top; py < digit_y - 1; py += 3)
                SSD1306_DrawPixel(sep_x, py, SSD1306_COLOR_WHITE);
        }

        // ── Franja de posición de línea (centroide, ±0.6 a fondo de escala) ──
        {
            const uint16_t fx = 0, fy = 56, fw = 70, fh = 8;
            SSD1306_DrawRectangle(fx, fy, fw, fh, SSD1306_COLOR_WHITE);
            // tick central
            SSD1306_DrawLine(fx + fw / 2, fy + 2, fx + fw / 2, fy + fh - 3, SSD1306_COLOR_WHITE);
            uint8_t det = 0;
            for (uint8_t ch = 0; ch < 4; ch++)
                if (!datos->linea_canal_cuarentena[ch] &&
                    datos->adc_mediana[ch] > (uint16_t)datos->LINEA_UMBRAL_ADC) det = 1;
            if (det) {
                float e = clampf_local(datos->linea_error_display, -0.6f, 0.6f);
                int16_t off = (int16_t)(e * (float)(fw / 2 - 4) / 0.6f);
                uint16_t cx = (uint16_t)((int16_t)(fx + fw / 2) + off);
                SSD1306_DrawFilledRectangle(cx - 1, fy + 1, 3, fh - 2, SSD1306_COLOR_WHITE);
            }
        }

        SSD1306_DrawLine(71, 11, 71, PANTALLA_ALTO - 1, SSD1306_COLOR_WHITE);

        // ── Derecha: estado + tiempo en estado + números clave ──
        OLED_Puts7CenteredX(LineStateStr(datos), 73, 127, 12);
        {
            uint32_t e = OLED_LineStateElapsedMs(datos);
            snprintf(lbuf, sizeof(lbuf), "t:%lu.%01lus",
                     (unsigned long)(e / 1000U), (unsigned long)((e % 1000U) / 100U));
            OLED_Str5(74, 24, lbuf);
        }
        FormatSignedFixed(nbuf, sizeof(nbuf), datos->linea_error_display, 2);
        snprintf(lbuf, sizeof(lbuf), "E:%s", nbuf);   OLED_Str5(74, 33, lbuf);
        FormatSignedFixed(nbuf, sizeof(nbuf), datos->velocidad_est_ema, 2);
        snprintf(lbuf, sizeof(lbuf), "V:%s", nbuf);   OLED_Str5(74, 42, lbuf);
        snprintf(lbuf, sizeof(lbuf), "A%c:%u",
                 (datos->obj_esquive_dir > 0) ? '7' : '5',
                 (unsigned)datos->adc_mediana[datos->obj_pared_adc_indice]);
        OLED_Str5(74, 51, lbuf);

    } else if (datos->f_cambiar_pantalla == 2) {
        // ───────────────────────────────────────────────────────────
        // PANTALLA 2 — ADC BARRAS: 8 canales con umbral y disparo
        // ───────────────────────────────────────────────────────────
        OLED_Header(datos, "ADC BARRAS");

        const uint16_t bar_top   = 12;
        const uint16_t digit_y   = 56;
        const uint16_t bar_max_h = digit_y - bar_top - 1;
        const uint16_t spacing   = 2;
        const uint16_t bar_width = (PANTALLA_ANCHO - (PANTALLA_ADC_CANTIDAD + 1) * spacing) / PANTALLA_ADC_CANTIDAD;

        for (uint8_t i = 0; i < PANTALLA_ADC_CANTIDAD; i++) {
            uint16_t v  = datos->adc_mediana[i] > 4095 ? 4095 : datos->adc_mediana[i];
            uint16_t h  = (uint32_t)v * bar_max_h / 4095;   // Regla de tres: cuentas de ADC -> pixeles de alto
            uint16_t x0 = spacing + i * (bar_width + spacing);
            if (h > 0)
                SSD1306_DrawFilledRectangle(x0, digit_y - 1 - h, bar_width, h, SSD1306_COLOR_WHITE);

            // tick de umbral (1-4: LINEA_UMBRAL_ADC, 5-8: OBJ_DETECCION_UMBRAL_ADC),
            // en color invertido si la barra ya lo cubre
            float th = (i < 4) ? datos->LINEA_UMBRAL_ADC : datos->obj_deteccion_umbral_adc;
            uint16_t hth = (uint16_t)((uint32_t)th * bar_max_h / 4095U);
            uint16_t yth = digit_y - 1 - hth;
            SSD1306_COLOR_t tc = (h >= hth) ? SSD1306_COLOR_BLACK : SSD1306_COLOR_WHITE;
            SSD1306_DrawLine(x0, yth, x0 + bar_width - 1, yth, tc);

            // indicador de disparo: cuadradito arriba de la columna
            // (línea 1-4: v > umbral = cinta negra; objeto 5-8: v < umbral = objeto)
            uint8_t trig = (i < 4) ? (datos->adc_mediana[i] > (uint16_t)datos->LINEA_UMBRAL_ADC)
                                   : ((float)datos->adc_mediana[i] < datos->obj_deteccion_umbral_adc);
            if (trig) {
                SSD1306_COLOR_t ic = (h >= bar_max_h - 7) ? SSD1306_COLOR_BLACK
                                                          : SSD1306_COLOR_WHITE;
                SSD1306_DrawFilledRectangle(x0 + bar_width / 2 - 2, bar_top + 1, 5, 5, ic);
            }

            uint16_t tx = x0 + (bar_width - Font_5x7.FontWidth) / 2;
            SSD1306_DrawChar5x7('1' + i, tx, digit_y);
        }
        SSD1306_DrawLine(0, digit_y - 1, PANTALLA_ANCHO - 1, digit_y - 1, SSD1306_COLOR_WHITE);

    } else if (datos->f_cambiar_pantalla == 3) {
        // ───────────────────────────────────────────────────────────
        // PANTALLA 3 — PARAM: ganancias de balance y de línea
        // ───────────────────────────────────────────────────────────
        OLED_Header(datos, "PARAM");
        SSD1306_DrawLine(63, 11, 63, PANTALLA_ALTO - 1, SSD1306_COLOR_WHITE);

        OLED_Str5(2, 12, "BALANCE");
        FormatSignedFixed(nbuf, sizeof(nbuf), datos->KP_value, 3);
        snprintf(lbuf, sizeof(lbuf), "P:%s", nbuf + 1);   OLED_Str5(2, 21, lbuf);
        FormatSignedFixed(nbuf, sizeof(nbuf), datos->KI_value, 3);
        snprintf(lbuf, sizeof(lbuf), "I:%s", nbuf + 1);   OLED_Str5(2, 30, lbuf);
        FormatSignedFixed(nbuf, sizeof(nbuf), datos->KD_value, 3);
        snprintf(lbuf, sizeof(lbuf), "D:%s", nbuf + 1);   OLED_Str5(2, 39, lbuf);
        FormatSignedFixed(nbuf, sizeof(nbuf), datos->setpoint_trim, 1);
        snprintf(lbuf, sizeof(lbuf), "TR:%s", nbuf);      OLED_Str5(2, 48, lbuf);
        FormatSignedFixed(nbuf, sizeof(nbuf), datos->KV_brake_value, 1);
        snprintf(lbuf, sizeof(lbuf), "KV:%s", nbuf + 1);  OLED_Str5(2, 57, lbuf);

        OLED_Str5(66, 12, "LINEA");
        FormatSignedFixed(nbuf, sizeof(nbuf), datos->KP_LINE, 2);
        snprintf(lbuf, sizeof(lbuf), "P:%s", nbuf + 1);   OLED_Str5(66, 21, lbuf);
        FormatSignedFixed(nbuf, sizeof(nbuf), datos->KI_LINE, 2);
        snprintf(lbuf, sizeof(lbuf), "I:%s", nbuf + 1);   OLED_Str5(66, 30, lbuf);
        FormatSignedFixed(nbuf, sizeof(nbuf), datos->KD_LINE, 2);
        snprintf(lbuf, sizeof(lbuf), "D:%s", nbuf + 1);   OLED_Str5(66, 39, lbuf);
        snprintf(lbuf, sizeof(lbuf), "TH:%lu", (unsigned long)(uint32_t)datos->LINEA_UMBRAL_ADC);
        OLED_Str5(66, 48, lbuf);
        FormatSignedFixed(nbuf, sizeof(nbuf), datos->LINE_SPEED_TARGET, 2);
        snprintf(lbuf, sizeof(lbuf), "VT:%s", nbuf + 1);  OLED_Str5(66, 57, lbuf);

    } else if (datos->f_cambiar_pantalla == 4) {
        // ───────────────────────────────────────────────────────────
        // PANTALLA 4 — ADC VALORES: 8 canales numéricos + disparo (*)
        // ───────────────────────────────────────────────────────────
        OLED_Header(datos, "ADC VALORES");
        SSD1306_DrawLine(63, 11, 63, PANTALLA_ALTO - 1, SSD1306_COLOR_WHITE);

        const uint16_t rows4[4] = { 13, 26, 39, 52 };
        for (uint8_t i = 0; i < 4; i++) {
            uint8_t trig_l = (datos->adc_mediana[i] > (uint16_t)datos->LINEA_UMBRAL_ADC);
            snprintf(lbuf, sizeof(lbuf), "%u:%4u%c", i + 1, datos->adc_mediana[i], trig_l ? '*' : ' ');
            OLED_Str5(2, rows4[i], lbuf);

            uint8_t trig_o = ((float)datos->adc_mediana[i + 4] < datos->obj_deteccion_umbral_adc);
            snprintf(lbuf, sizeof(lbuf), "%u:%4u%c", i + 5, datos->adc_mediana[i + 4], trig_o ? '*' : ' ');
            OLED_Str5(66, rows4[i], lbuf);
        }

    } else if (datos->f_cambiar_pantalla == 5) {
        // ───────────────────────────────────────────────────────────
        // PANTALLA 5 — IMU: gyro/accel crudos + roll/omega/movimiento
        // ───────────────────────────────────────────────────────────
        OLED_Header(datos, "IMU");
        SSD1306_DrawLine(63, 11, 63, 45, SSD1306_COLOR_WHITE);

        OLED_Str5(2, 12, "GYRO");
        snprintf(lbuf, sizeof(lbuf), "X:%+d", (int)datos->giro_x);  OLED_Str5(2, 20, lbuf);
        snprintf(lbuf, sizeof(lbuf), "Y:%+d", (int)datos->giro_y);  OLED_Str5(2, 28, lbuf);
        snprintf(lbuf, sizeof(lbuf), "Z:%+d", (int)datos->giro_z);  OLED_Str5(2, 36, lbuf);

        OLED_Str5(66, 12, "ACEL");
        snprintf(lbuf, sizeof(lbuf), "X:%+d", (int)datos->accel_x);  OLED_Str5(66, 20, lbuf);
        snprintf(lbuf, sizeof(lbuf), "Y:%+d", (int)datos->accel_y);  OLED_Str5(66, 28, lbuf);
        snprintf(lbuf, sizeof(lbuf), "Z:%+d", (int)datos->accel_z);  OLED_Str5(66, 36, lbuf);

        SSD1306_DrawLine(0, 46, PANTALLA_ANCHO - 1, 46, SSD1306_COLOR_WHITE);

        FormatSignedFixed(nbuf, sizeof(nbuf), datos->roll_filtrado_grados, 1);
        snprintf(lbuf, sizeof(lbuf), "ROLL:%s", nbuf);   OLED_Str5(2, 49, lbuf);
        FormatSignedFixed(nbuf, sizeof(nbuf), datos->giro_dps_clampeado, 1);
        snprintf(lbuf, sizeof(lbuf), "W:%s", nbuf);      OLED_Str5(70, 49, lbuf);

        snprintf(lbuf, sizeof(lbuf), "MOV:%lu", (unsigned long)(uint32_t)datos->accel_movimiento_ema);
        OLED_Str5(2, 57, lbuf);
        FormatSignedFixed(nbuf, sizeof(nbuf), (float)datos->giro_z / 100.0f, 0);
        snprintf(lbuf, sizeof(lbuf), "YAW:%s", nbuf);    OLED_Str5(70, 57, lbuf);

    } else if (datos->f_cambiar_pantalla == 6) {
        // ───────────────────────────────────────────────────────────
        // PANTALLA 6 — OBJETO: estado de evasión + sensores con umbrales
        // ───────────────────────────────────────────────────────────
        OLED_Header(datos, "OBJETO");

        // En los estados de pared se agrega la acción vigente (AVZ/PIV/REV)
        // al nombre del sub-estado: "PARED>REV".
        {
            uint8_t en_pared = (datos->estado_robot == ROBOT_STATE_LINE_FOLLOWING) &&
                              (datos->linea_estado == LINE_STATE_OBJ_BORDEAR_PARED ||
                               datos->linea_estado == LINE_STATE_OBJ_PARED_LIBRE  ||
                               datos->linea_estado == LINE_STATE_OBJ_GIRO_PARED);
            if (en_pared) {
                snprintf(lbuf, sizeof(lbuf), "%s>%s",
                         LineStateStr(datos), ObjWallActionStr(datos));
                OLED_Puts7CenteredX(lbuf, 0, 127, 12);
            } else {
                OLED_Puts7CenteredX(LineStateStr(datos), 0, 127, 12);
            }
        }
        {
            uint32_t e = OLED_LineStateElapsedMs(datos);
            snprintf(lbuf, sizeof(lbuf), "t:%lu.%01lus",
                     (unsigned long)(e / 1000U), (unsigned long)((e % 1000U) / 100U));
            uint16_t w = OLED_Str5W(lbuf);
            OLED_Str5((PANTALLA_ANCHO - w) / 2, 24, lbuf);
        }

        // A6 (frontal, corte de la reversa inicial): ticks en detección (3200)
        // y en el corte de reversa (OBJ_DISTANCIA_BANDA_PISO_ADC=3600)
        {
            const float ticks6[2] = { datos->obj_deteccion_umbral_adc,
                                             datos->obj_distancia_banda_piso_adc };
            snprintf(lbuf, sizeof(lbuf), "A6:%4u", (unsigned)datos->adc_mediana[5]);
            OLED_Str5(0, 35, lbuf);
            OLED_HBar(44, 34, 83, 9, datos->adc_mediana[5], ticks6, 2);
        }
        // Lateral de pared activo (A7 esquivando a derecha, A5 a izquierda):
        // ticks en reversa / muy-cerca / pared visible
        {
            const float ticks7[3] = { datos->obj_pared_reversa_umbral,
                                             datos->obj_pared_muy_cerca_umbral,
                                             datos->obj_pared_umbral };
            snprintf(lbuf, sizeof(lbuf), "A%c:%4u",
                     (datos->obj_esquive_dir > 0) ? '7' : '5',
                     (unsigned)datos->adc_mediana[datos->obj_pared_adc_indice]);
            OLED_Str5(0, 46, lbuf);
            OLED_HBar(44, 45, 83, 9, datos->adc_mediana[datos->obj_pared_adc_indice], ticks7, 3);
        }

        // A8: el corte de la reversa exige A6 Y A8 ≥ 3600 — si la reversa se
        // pasa de largo, mirar acá cuál de los dos es el que no despeja.
        FormatSignedFixed(nbuf, sizeof(nbuf), datos->velocidad_est_ema, 2);
        snprintf(lbuf, sizeof(lbuf), "VE:%s", nbuf);     OLED_Str5(0, 57, lbuf);
        snprintf(lbuf, sizeof(lbuf), "A8:%4u", (unsigned)datos->adc_mediana[7]);
        OLED_Str5(70, 57, lbuf);

    } else if (datos->f_cambiar_pantalla == 7) {
        // ───────────────────────────────────────────────────────────
        // PANTALLA 7 — ODOMETRIA: mapa de pose + números
        // Mapa: X odométrico hacia arriba, Y hacia la IZQUIERDA (θ positivo
        // antihorario, convención estándar — espejar Y aquí haría que la
        // flecha gire al revés de la realidad), origen al
        // centro. Autoescala para que la pose (y el punto de pérdida de
        // línea, si existe) siempre entren en el recuadro.
        // ───────────────────────────────────────────────────────────
        OLED_Header(datos, "ODOMETRIA");

        {
            const int16_t bx = 0, by = 11, bw = 53, bh = 53;
            const int16_t cx = bx + bw / 2, cy = by + bh / 2;
            SSD1306_DrawRectangle(bx, by, bw, bh, SSD1306_COLOR_WHITE);
            // cruz del origen
            SSD1306_DrawLine(cx - 2, cy, cx + 2, cy, SSD1306_COLOR_WHITE);
            SSD1306_DrawLine(cx, cy - 2, cx, cy + 2, SSD1306_COLOR_WHITE);

            float rng = 0.5f;
            if (fabsf(datos->odom_x_m) > rng) rng = fabsf(datos->odom_x_m);
            if (fabsf(datos->odom_y_m) > rng) rng = fabsf(datos->odom_y_m);
            if (datos->linea_perdida_pose_valida) {
                if (fabsf(datos->linea_perdida_x_m) > rng) rng = fabsf(datos->linea_perdida_x_m);
                if (fabsf(datos->linea_perdida_y_m) > rng) rng = fabsf(datos->linea_perdida_y_m);
            }
            float k = (float)(bw / 2 - 3) / rng;   // Escala del mapa: pixeles por metro (autoajuste al rango visible)

            // punto de pérdida de línea (cuadradito hueco)
            if (datos->linea_perdida_pose_valida) {
                int16_t lx = cx - (int16_t)(datos->linea_perdida_y_m * k);
                int16_t ly = cy - (int16_t)(datos->linea_perdida_x_m * k);
                SSD1306_DrawRectangle(lx - 1, ly - 1, 3, 3, SSD1306_COLOR_WHITE);
            }

            // pose actual: punto lleno + rayo de rumbo (θ=0 → +X → arriba)
            int16_t px = cx - (int16_t)(datos->odom_y_m * k);
            int16_t py = cy - (int16_t)(datos->odom_x_m * k);
            float th = datos->odom_theta_grados * ((float)M_PI / 180.0f);
            int16_t hx = px - (int16_t)(sinf(th) * 7.0f);   // Punta de la flecha: descomposicion polar->cartesiano del rumbo
            int16_t hy = py - (int16_t)(cosf(th) * 7.0f);
            if (hx < bx + 1)      hx = bx + 1;
            if (hx > bx + bw - 2) hx = bx + bw - 2;
            if (hy < by + 1)      hy = by + 1;
            if (hy > by + bh - 2) hy = by + bh - 2;
            SSD1306_DrawLine(px, py, hx, hy, SSD1306_COLOR_WHITE);
            SSD1306_DrawFilledRectangle(px - 1, py - 1, 3, 3, SSD1306_COLOR_WHITE);
        }

        // ── Derecha: números ──
        {
            const uint16_t rx = 57;
            FormatSignedFixed(nbuf, sizeof(nbuf), datos->odom_x_m, 2);
            snprintf(lbuf, sizeof(lbuf), "X:%sm", nbuf);   OLED_Str5(rx, 12, lbuf);
            FormatSignedFixed(nbuf, sizeof(nbuf), datos->odom_y_m, 2);
            snprintf(lbuf, sizeof(lbuf), "Y:%sm", nbuf);   OLED_Str5(rx, 21, lbuf);
            FormatSignedFixed(nbuf, sizeof(nbuf), datos->odom_theta_grados, 0);
            snprintf(lbuf, sizeof(lbuf), "TH:%s", nbuf);   OLED_Str5(rx, 30, lbuf);

            if (datos->linea_perdida_pose_valida) {
                float ddx = datos->linea_perdida_x_m - datos->odom_x_m;   // Vector desde la pose actual hasta el punto guardado
                float ddy = datos->linea_perdida_y_m - datos->odom_y_m;
                FormatSignedFixed(nbuf, sizeof(nbuf), sqrtf(ddx * ddx + ddy * ddy), 2);
                snprintf(lbuf, sizeof(lbuf), "D:%sm", nbuf + 1);
            } else {
                snprintf(lbuf, sizeof(lbuf), "D:----");
            }
            OLED_Str5(rx, 39, lbuf);

            FormatSignedFixed(nbuf, sizeof(nbuf), datos->velocidad_est_ema, 2);
            snprintf(lbuf, sizeof(lbuf), "V:%s", nbuf);    OLED_Str5(rx, 48, lbuf);
            FormatSignedFixed(nbuf, sizeof(nbuf), (float)datos->giro_z / 100.0f, 0);
            snprintf(lbuf, sizeof(lbuf), "GZ:%s", nbuf);   OLED_Str5(rx, 57, lbuf);
        }

    } else {
        SSD1306_GotoXY(30, 25);
        SSD1306_Puts("DISPLAY?", &Font_7x10, SSD1306_COLOR_WHITE);
    }
    SSD1306_RequestUpdate();
}

