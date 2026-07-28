/*
 * UNER.c
 *
 *  Created on: Apr 16, 2025
 *      Author: Tadeo Mendelevich
 */

#include "UNER.h"
#include <stddef.h>
#include <string.h>
#include <math.h>
#include "ESP01.h"
#include "usbd_core.h"    // para USBD_HandleTypeDef, USBD_STATE_CONFIGURED

extern void USB_Debug(const char *fmt, ...);
extern void USB_DebugStr(const char *dbgStr);

static const char firmware[] = "UNER V1.0";

static _sRx *unerRx;
static _sTx *unerTx;

// Prototipo externo de estado USB (ya lo tienes en main.c)
extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t usb_enqueue_tx_segments(const uint8_t *first, uint16_t first_len,
                                       const uint8_t *second, uint16_t second_len);

static void decodeCommand(_sRx *dataRx, _sTx *dataTx);
static void UNER_SendData(void);
static uint8_t putHeaderOnTx(_sTx *dataTx, _eCmd ID, uint8_t frameLength);
static uint8_t putByteOnTx(_sTx *dataTx, uint8_t byte);
static uint8_t putStrOntx(_sTx *dataTx, const char *str);
static uint8_t getByteFromRx(_sRx *dataRx, uint8_t iniPos, uint8_t finalPos);
static void putBytesOnTx(_sTx *dataTx, const void *bytes, uint8_t length);
static void putU16OnTx(_sTx *dataTx, uint16_t value);
static void putF32OnTx(_sTx *dataTx, float value);
static float getF32FromRx(_sRx *dataRx);
static int32_t getI32FromRx(_sRx *dataRx);
static void putPidValuesOnTx(_sTx *dataTx);
static uint8_t sensorsAreRegistered(void);
static void putAllSensorsOnTx(_sTx *dataTx);
static void UNER_SendWifiStruct(_eCmd command, const void *data, uint8_t payloadLen);

static int16_t *p_ax = NULL, *p_ay = NULL, *p_az = NULL;
static int16_t *p_gx = NULL, *p_gy = NULL, *p_gz = NULL;

volatile uint8_t sendAllSensorsFlag = 0;

static uint16_t *p_adcBuf = NULL;	// // Punteros a las variables de adc en main.c
static uint8_t   adcBufLen = 0;

static int16_t *p_motorRightVel = NULL;		// Punteros a las variables de velocidad en main.c
static int16_t *p_motorLeftVel  = NULL;

static float *p_KP = NULL;		// Punteros a las variables de control proporcional en main.c
static float *p_KD  = NULL;
static float *p_KI  = NULL;
static float *p_BETA_G  = NULL;
static float *p_BETA_A  = NULL;
static float *p_KV_BRAKE  = NULL;

static float *p_roll  = NULL;	// Punteros a las variables de inclinacion en main.c
static float *p_pitch = NULL;

static float *p_steering = NULL;


static uint8_t *p_robot_state = NULL;	// Puntero a la maquina de estados de los modos del robot
static uint8_t *p_resetMassCenter_flag = NULL;	// Bandera para resetear el centro de gravedad del auto, realizando la medicion de la mpu nuevamente
static uint8_t *p_send_csv_log_flag = NULL;	//
static uint8_t *p_send_wifi_log_flag = NULL;
static uint8_t *p_change_display = NULL;

static float *p_KP_LINE = NULL;
static float *p_KD_LINE = NULL;
static float *p_KI_LINE = NULL;
static float *p_LINE_THRES = NULL;
static float *p_LINE_SPEED = NULL;
static float *p_SP_LIMIT = NULL;

// Limites de operacion para el objetivo de velocidad recibido desde Qt.
// El corte de emergencia (10 m/s) sigue siendo una proteccion independiente;
// este rango es deliberadamente mas conservador para el comando de usuario.
#define LINE_SPEED_CMD_MIN_MPS  0.20f
// 2026-07-27: 4.00 -> 8.00 a pedido del usuario (queria pasar de 4 m/s desde
// Qt y el clamp del firmware lo recortaba en silencio). Sigue por debajo del
// corte de emergencia de 10 m/s: pedir un objetivo >= a ese corte seria
// autodestructivo (el robot cortaria motores al alcanzarlo).
#define LINE_SPEED_CMD_MAX_MPS  8.00f

// Limites del tope de inclinacion del setpoint dinamico (0xC9). Debajo de 1
// grado el robot no puede avanzar; arriba de 15 la zona muerta de motores
// (35 grados) todavia queda lejos pero el balance ya es muy agresivo.
#define SP_LIMIT_CMD_MIN_DEG    1.00f
#define SP_LIMIT_CMD_MAX_DEG    15.00f

// Limite del trim de setpoint recibido desde Qt (0xC8). El spinbox de Qt
// permite +-180 grados; mas alla de unos pocos grados el robot no puede
// sostener el equilibrio (zona muerta de motores arranca en 35 grados).
#define SETPOINT_TRIM_MAX_DEG   10.0f

// Limite para KV_BRAKE recibido desde Qt (0xBF); el dialogo de Qt ofrece 0..100.
#define KV_BRAKE_CMD_MAX        100.0f

// Limites de sanidad para el resto de los comandos de tuning. Generosos a
// proposito (un orden de magnitud sobre los valores de trabajo): filtran NaN,
// Inf y disparates, no molestan al tuneo real.
#define PID_GAIN_CMD_MAX        200.0f   // KP/KI/KD balance y linea (valores de trabajo: 0.1..10)
#define LINE_THRES_CMD_MAX      4095.0f  // umbral sobre cuentas crudas del ADC de 12 bits
#define STEERING_CMD_MAX        60.0f    // diferencial de PWM (el seguidor de linea se limita solo a +-20)
#define ROTATE_CUSTOM_MAX_DEG   360.0f   // giro remoto: una vuelta completa como techo

static float *p_manual_sp_cmd = NULL;
static float *p_manual_st_cmd = NULL;
static uint32_t *p_manual_tmo = NULL;

static float   *p_rot_target_deg = NULL;
static uint8_t *p_rot_trigger    = NULL;

static float *p_odom_x     = NULL;   // Punteros a la pose odométrica en main.c
static float *p_odom_y     = NULL;
static float *p_odom_theta = NULL;

static float *p_setpoint_trim = NULL;

static float *p_velocity_mps   = NULL;   // velocidad global encoders (m/s, interno: negativo = adelante)
static float *p_wheel_right_rps = NULL;  // velocidad rueda derecha (rps)
static float *p_wheel_left_rps  = NULL;  // velocidad rueda izquierda (rps)

static uint8_t last_manual_cmd = 0;
static volatile uint32_t rx_overflow_count = 0;

void UNER_Init(_sRx *rx, _sTx *tx) {
    unerRx = rx;
    unerTx = tx;
    unerRx->indexR = 0;
    unerRx->indexW = 0;
    unerRx->header = HEADER_U;
    unerRx->mask = RXBUFSIZE - 1;
    unerTx->indexR = 0;
    unerTx->indexW = 0;
    unerTx->mask = TXBUFSIZE - 1;
    unerRx->indexData = 0;
    unerRx->nBytes    = 0;
    unerRx->chk       = 0;
    unerRx->timeOut   = 0;
    unerRx->isComannd = false;
    unerTx->chk       = 0;

}

void UNER_PushByte(uint8_t byte) {
    uint8_t next;

    if (unerRx == NULL || unerRx->buff == NULL) return;
    next = (uint8_t)((unerRx->indexW + 1U) & unerRx->mask);
    if (next == unerRx->indexR) {
        rx_overflow_count++;
        return;
    }

    unerRx->buff[unerRx->indexW] = byte;
    unerRx->indexW = next;
}

void UNER_Task(void) {
    uint8_t auxIndex = unerRx->indexW;
    while (unerRx->indexR != auxIndex) {
        switch (unerRx->header) {
            case HEADER_U:
                if (unerRx->buff[unerRx->indexR] == 'U') {
                    unerRx->header = HEADER_N;
                    unerRx->timeOut = 5;
                }
                break;
            case HEADER_N:
                if (unerRx->buff[unerRx->indexR] == 'N') {
                    unerRx->header = HEADER_E;
                } else {
                    if (unerRx->buff[unerRx->indexR] != 'U') {
                        unerRx->header = HEADER_U;
                        unerRx->indexR--;
                    }
                }
                break;
            case HEADER_E:
                if (unerRx->buff[unerRx->indexR] == 'E') {
                    unerRx->header = HEADER_R;
                } else {
                    unerRx->header = HEADER_U;
                    unerRx->indexR--;
                }
                break;
            case HEADER_R:
                if (unerRx->buff[unerRx->indexR] == 'R') {
                    unerRx->header = NBYTES;
                } else {
                    unerRx->header = HEADER_U;
                    unerRx->indexR--;
                }
                break;
            case NBYTES:
                unerRx->nBytes = unerRx->buff[unerRx->indexR];
                // Mínimo 2 (cmd + checksum): con nBytes=0, el nBytes-- del estado
                // PAYLOAD underflowea a 255 y el parser se traga 255 bytes.
                unerRx->header = (unerRx->nBytes >= 2) ? TOKEN : HEADER_U;
                break;
            case TOKEN:
                if (unerRx->buff[unerRx->indexR] == ':') {
                    unerRx->header = PAYLOAD;
                    unerRx->indexData = unerRx->indexR + 1;
                    unerRx->indexData &= unerRx->mask;
                    unerRx->chk = 'U' ^ 'N' ^ 'E' ^ 'R' ^ unerRx->nBytes ^ ':';
                } else {
                    unerRx->header = HEADER_U;
                    unerRx->indexR--;
                }
                break;
            case PAYLOAD:
            	//UNER_Debug("  payload byte, remaining=%u chk=0x%02X\n", unerRx->nBytes, unerRx->chk);
                unerRx->nBytes--;
                if (unerRx->nBytes > 0) {
                    unerRx->chk ^= unerRx->buff[unerRx->indexR];
                } else {
                    unerRx->header = HEADER_U;
                    if (unerRx->buff[unerRx->indexR] == unerRx->chk) {
                        unerRx->isComannd = true;
                        decodeCommand(unerRx, unerTx);
                    }
                }
                break;
            default:
                unerRx->header = HEADER_U;
                break;
        }
        unerRx->indexR++;
        unerRx->indexR &= unerRx->mask;
    }
}

static uint8_t putHeaderOnTx(_sTx *dataTx, _eCmd ID, uint8_t frameLength)
{
    dataTx->chk = 0;
    dataTx->buff[dataTx->indexW++]='U';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]='N';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]='E';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]='R';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]=frameLength+1;
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]=':';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]=ID;
    dataTx->indexW &= dataTx->mask;
    dataTx->chk ^= (frameLength+1);
    dataTx->chk ^= ('U' ^'N' ^'E' ^'R' ^ID ^':') ;
    return  dataTx->chk;
}

static uint8_t putByteOnTx(_sTx *dataTx, uint8_t byte)
{
    dataTx->buff[dataTx->indexW++]=byte;
    dataTx->indexW &= dataTx->mask;
    dataTx->chk ^= byte;
    return dataTx->chk;
}

static uint8_t putStrOntx(_sTx *dataTx, const char *str)
{
    while (*str)
        putByteOnTx(dataTx, (uint8_t)*str++);

    return dataTx->chk ;
}

static uint8_t getByteFromRx(_sRx *dataRx, uint8_t iniPos, uint8_t finalPos){
    uint8_t getByte;
    dataRx->indexData += iniPos;
    dataRx->indexData &=dataRx->mask;
    getByte = dataRx->buff[dataRx->indexData];
    dataRx->indexData += finalPos;
    dataRx->indexData &=dataRx->mask;
    return getByte;
}

static void putBytesOnTx(_sTx *dataTx, const void *bytes, uint8_t length)
{
    const uint8_t *src = (const uint8_t *)bytes;
    for (uint8_t i = 0; i < length; i++)
        putByteOnTx(dataTx, src[i]);
}

static void putU16OnTx(_sTx *dataTx, uint16_t value)
{
    _uWord word = {0};
    word.ui16[0] = value;
    putBytesOnTx(dataTx, word.ui8, 2);
}

static void putF32OnTx(_sTx *dataTx, float value)
{
    _uWord word;
    word.f32 = value;
    putBytesOnTx(dataTx, word.ui8, 4);
}

static float getF32FromRx(_sRx *dataRx)
{
    _uWord word;
    for (uint8_t i = 0; i < 4; i++)
        word.ui8[i] = getByteFromRx(dataRx, 1, 0);
    return word.f32;
}

// Lee un float del payload y lo valida: NaN/Inf se rechaza (devuelve 0 y no
// toca *out), un valor fuera de [lo, hi] se recorta al borde. Centraliza la
// sanidad de TODOS los comandos de tuning: sin esto, un frame corrupto que
// pasara el checksum (o un bug en la UI) podía meter NaN en una ganancia PID
// y voltear el robot sin diagnóstico posible.
static uint8_t getF32BoundedFromRx(_sRx *dataRx, float *out, float lo, float hi)
{
    float v = getF32FromRx(dataRx);
    if (!isfinite(v)) return 0;
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    *out = v;
    return 1;
}

static int32_t getI32FromRx(_sRx *dataRx)
{
    _uWord word;
    for (uint8_t i = 0; i < 4; i++)
        word.ui8[i] = getByteFromRx(dataRx, 1, 0);
    return word.i32;
}

static void putPidValuesOnTx(_sTx *dataTx)
{
    putF32OnTx(dataTx, *p_KP);
    putF32OnTx(dataTx, *p_KD);
    putF32OnTx(dataTx, *p_KI);
}

static uint8_t sensorsAreRegistered(void)
{
    return p_adcBuf != NULL && adcBufLen >= 8U &&
           p_ax != NULL && p_ay != NULL && p_az != NULL &&
           p_gx != NULL && p_gy != NULL && p_gz != NULL &&
           p_roll != NULL && p_pitch != NULL;
}

static void putAllSensorsOnTx(_sTx *dataTx)
{
    for (uint8_t i = 0; i < 8U; i++)
        putU16OnTx(dataTx, p_adcBuf[i]);

    putU16OnTx(dataTx, (uint16_t)*p_ax);
    putU16OnTx(dataTx, (uint16_t)*p_ay);
    putU16OnTx(dataTx, (uint16_t)*p_az);
    putU16OnTx(dataTx, (uint16_t)*p_gx);
    putU16OnTx(dataTx, (uint16_t)*p_gy);
    putU16OnTx(dataTx, (uint16_t)*p_gz);
    putF32OnTx(dataTx, *p_roll);
    putF32OnTx(dataTx, *p_pitch);
}

static void decodeCommand(_sRx *dataRx, _sTx *dataTx)
{
    switch(dataRx->buff[dataRx->indexData]){
        case ALIVE:
        	USB_Debug("\n ALIVE RECIBIDO!\n");
            putHeaderOnTx(dataTx, ALIVE, 2);
            putByteOnTx(dataTx, ACK);
            putByteOnTx(dataTx, dataTx->chk);	// EL CHECKSUM SE AGREGA SOLO COMO SUMA LUEGO DEL CALCULO DEL PAYLOAD
        break;
        case FIRMWARE:
            // frameLength = cmd (1) + strlen(firmware); sizeof-1 lo calcula solo.
            // El 12 hardcodeado declaraba 2 bytes de mas y desincronizaba al
            // parser de Qt (la trama real era mas corta que su encabezado).
            putHeaderOnTx(dataTx, FIRMWARE, (uint8_t)(1U + sizeof(firmware) - 1U));
            putStrOntx(dataTx, firmware);
            putByteOnTx(dataTx, dataTx->chk);
        break;
        case GETADCVALUES:
			if (p_adcBuf != NULL && adcBufLen >= 8U) {
				putHeaderOnTx(dataTx, GETADCVALUES, 17);
				for (uint8_t i = 0; i < 8U; i++)
					putU16OnTx(dataTx, p_adcBuf[i]);
			} else {
				putHeaderOnTx(dataTx, GETADCVALUES, 2);
				putByteOnTx(dataTx, UNKNOWN);
			}
			putByteOnTx(dataTx, dataTx->chk);
        	break;
        case GETMPU6050VALUES:
			if (p_ax && p_ay && p_az && p_gx && p_gy && p_gz) {
				putHeaderOnTx(dataTx, GETMPU6050VALUES, 13);
				putU16OnTx(dataTx, (uint16_t)*p_ax);
				putU16OnTx(dataTx, (uint16_t)*p_ay);
				putU16OnTx(dataTx, (uint16_t)*p_az);
				putU16OnTx(dataTx, (uint16_t)*p_gx);
				putU16OnTx(dataTx, (uint16_t)*p_gy);
				putU16OnTx(dataTx, (uint16_t)*p_gz);
			} else {
				putHeaderOnTx(dataTx, GETMPU6050VALUES, 2);
				putByteOnTx(dataTx, UNKNOWN);
			}
			putByteOnTx(dataTx, dataTx->chk);
        	break;
        case GETANGLE:
        	if (p_roll && p_pitch) {
        		putHeaderOnTx(dataTx, GETANGLE, 9); // 1 byte cmd + 2*4 bytes for float angles

				putF32OnTx(dataTx, *p_roll);
				putF32OnTx(dataTx, *p_pitch);

				putByteOnTx(dataTx, dataTx->chk);
			} else {
				// si no está registrado, devolvemos sólo ACK
				putHeaderOnTx(dataTx, GETANGLE, 2);
				putByteOnTx(dataTx, ACK);
				putByteOnTx(dataTx, dataTx->chk);
			}
        	break;
        case GETSPEED:
            // Implementado 2026-07-24 (antes caia en default → UNKNOWN).
            // Respuesta: 3 floats LE = velocidad global [m/s], rueda derecha
            // [rps], rueda izquierda [rps]. La convencion interna del firmware
            // es "negativo = adelante" (ver vel_enc en main.c); para la UI se
            // envia con signo invertido: positivo = avanzando.
            if (p_velocity_mps && p_wheel_right_rps && p_wheel_left_rps) {
                putHeaderOnTx(dataTx, GETSPEED, 13); // 1 cmd + 3 floats
                putF32OnTx(dataTx, -(*p_velocity_mps));
                putF32OnTx(dataTx, *p_wheel_right_rps);
                putF32OnTx(dataTx, *p_wheel_left_rps);
            } else {
                putHeaderOnTx(dataTx, GETSPEED, 2);
                putByteOnTx(dataTx, UNKNOWN);
            }
            putByteOnTx(dataTx, dataTx->chk);
        break;

        case SETMOTORSPEED:
			putHeaderOnTx(dataTx, SETMOTORSPEED, 2);
			putByteOnTx(dataTx, ACK );
			putByteOnTx(dataTx, dataTx->chk);
			int32_t requestedLeft = getI32FromRx(dataRx);
			int16_t vLeft = (requestedLeft > 100) ? 100 :
			                (requestedLeft < -100) ? -100 : (int16_t)requestedLeft;
			// sólo si el puntero está registrado
			if (p_motorLeftVel) *p_motorLeftVel = vLeft;

			int32_t requestedRight = getI32FromRx(dataRx);
			int16_t vRight = (requestedRight > 100) ? 100 :
			                 (requestedRight < -100) ? -100 : (int16_t)requestedRight;
			if (p_motorRightVel) *p_motorRightVel = vRight;
		break;

        case MODIFYKP:
			float new_KP;
			if (p_KP && getF32BoundedFromRx(dataRx, &new_KP, 0.0f, PID_GAIN_CMD_MAX))
				*p_KP = new_KP;

			putHeaderOnTx(dataTx, MODIFYKP, 13);
			putPidValuesOnTx(dataTx);
			putByteOnTx(dataTx, dataTx->chk);
        break;

        case MODIFYKD:
			float new_KD;
			if (p_KD && getF32BoundedFromRx(dataRx, &new_KD, 0.0f, PID_GAIN_CMD_MAX))
				*p_KD = new_KD;

			putHeaderOnTx(dataTx, MODIFYKD, 13);
			putPidValuesOnTx(dataTx);
			putByteOnTx(dataTx, dataTx->chk);
		break;

        case MODIFYKI:
			float new_KI;
			if (p_KI && getF32BoundedFromRx(dataRx, &new_KI, 0.0f, PID_GAIN_CMD_MAX))
				*p_KI = new_KI;

			putHeaderOnTx(dataTx, MODIFYKI, 13);
			putPidValuesOnTx(dataTx);
			putByteOnTx(dataTx, dataTx->chk);
		break;

        case BALANCE:
		if (p_robot_state != NULL) {
				if (*p_robot_state == 0) { // IDLE -> BALANCE_ONLY
					*p_robot_state = 1;
				} else { // ALL OTHER STATES -> IDLE
					*p_robot_state = 0;
				}
				putHeaderOnTx(dataTx, BALANCE, 2);
				putByteOnTx(dataTx, ACK);
				putByteOnTx(dataTx, dataTx->chk);
			}
        break;

        case GETPIDVALUES:
        	putHeaderOnTx(dataTx, GETPIDVALUES, 13); // 1 byte cmd + 2*4 bytes for float values
			putPidValuesOnTx(dataTx);
			putByteOnTx(dataTx, dataTx->chk);
        break;

        case MODIFYSTEERING:
            if (p_steering) {
                float new_steer;
                if (getF32BoundedFromRx(dataRx, &new_steer, -STEERING_CMD_MAX, STEERING_CMD_MAX))
                    *p_steering = new_steer;
            }
            putHeaderOnTx(dataTx, MODIFYSTEERING, 2);
            putByteOnTx(dataTx, ACK);
            putByteOnTx(dataTx, dataTx->chk);
        break;

        case RESETMASSCENTER:
			if (p_resetMassCenter_flag != NULL) {
				*p_resetMassCenter_flag = !(*p_resetMassCenter_flag);
				putHeaderOnTx(dataTx, RESETMASSCENTER, 2);
				putByteOnTx(dataTx, ACK);
				putByteOnTx(dataTx, dataTx->chk);	// EL CHECKSUM SE AGREGA SOLO COMO SUMA LUEGO DEL CALCULO DEL PAYLOAD
			}
		break;

        case ACTIVATE_CSV_LOG:
			if (p_send_csv_log_flag!= NULL) {
				*p_send_csv_log_flag = !(*p_send_csv_log_flag);
				putHeaderOnTx(dataTx, ACTIVATE_CSV_LOG, 2);
				putByteOnTx(dataTx, ACK);
				putByteOnTx(dataTx, dataTx->chk);	// EL CHECKSUM SE AGREGA SOLO COMO SUMA LUEGO DEL CALCULO DEL PAYLOAD
			}
		break;

        case ACTIVATE_WIFI_LOG:
            if (p_send_wifi_log_flag != NULL) {
                *p_send_wifi_log_flag = !(*p_send_wifi_log_flag);
                putHeaderOnTx(dataTx, ACTIVATE_WIFI_LOG, 2);
                putByteOnTx(dataTx, ACK);
                putByteOnTx(dataTx, dataTx->chk);
            }
        break;

        // Los betas de los filtros EMA hoy son defines fijos en main.c y estos
        // punteros no se registran (NULL): en vez de un ACK mentiroso se
        // responde UNKNOWN para que Qt muestre que el comando no aplica.
        // Si algun dia vuelven a ser variables, registrarlas en unerBindings
        // y el ACK vuelve solo. Rango valido de un beta EMA: 0..1.
        case MODIFY_BETA_G:
			if (p_BETA_G) {
				float new_BETA_G;
				if (getF32BoundedFromRx(dataRx, &new_BETA_G, 0.0f, 1.0f))
					*p_BETA_G = new_BETA_G;
				putHeaderOnTx(dataTx, MODIFY_BETA_G, 2);
				putByteOnTx(dataTx, ACK);
			} else {
				putHeaderOnTx(dataTx, MODIFY_BETA_G, 2);
				putByteOnTx(dataTx, UNKNOWN);
			}
			putByteOnTx(dataTx, dataTx->chk);
		break;

        case MODIFY_BETA_A:
			if (p_BETA_A) {
				float new_BETA_A;
				if (getF32BoundedFromRx(dataRx, &new_BETA_A, 0.0f, 1.0f))
					*p_BETA_A = new_BETA_A;
				putHeaderOnTx(dataTx, MODIFY_BETA_A, 2);
				putByteOnTx(dataTx, ACK);
			} else {
				putHeaderOnTx(dataTx, MODIFY_BETA_A, 2);
				putByteOnTx(dataTx, UNKNOWN);
			}
			putByteOnTx(dataTx, dataTx->chk);
		break;

        case MODIFY_KV_BRAKE:
            // Hasta 2026-07-24 este comando NO tenia case: Qt lo enviaba y el
            // firmware respondia UNKNOWN sin tocar KV_brake_value.
            if (p_KV_BRAKE) {
                float new_KV;
                if (getF32BoundedFromRx(dataRx, &new_KV, 0.0f, KV_BRAKE_CMD_MAX))
                    *p_KV_BRAKE = new_KV;
            }
            putHeaderOnTx(dataTx, MODIFY_KV_BRAKE, 2);
            putByteOnTx(dataTx, ACK);
            putByteOnTx(dataTx, dataTx->chk);
        break;

        case CHANGE_DISPLAY:
            if (p_change_display != NULL) {
                // 8 pantallas, igual que el botón físico (main.c): 6=OBJ, 7=ODOM
                *p_change_display = (*p_change_display + 1) % 8;
                putHeaderOnTx(dataTx, CHANGE_DISPLAY, 2);
                putByteOnTx(dataTx, ACK);
                putByteOnTx(dataTx, dataTx->chk);
            }
        break;

        case MODIFY_KP_LINE:
			if (p_KP_LINE) {
				float new_kp_line;
				if (getF32BoundedFromRx(dataRx, &new_kp_line, 0.0f, PID_GAIN_CMD_MAX))
					*p_KP_LINE = new_kp_line;
			}
			putHeaderOnTx(dataTx, MODIFY_KP_LINE, 2);
			putByteOnTx(dataTx, ACK);
			putByteOnTx(dataTx, dataTx->chk);
		break;

        case MODIFY_KD_LINE:
			if (p_KD_LINE) {
				float new_kd_line;
				if (getF32BoundedFromRx(dataRx, &new_kd_line, 0.0f, PID_GAIN_CMD_MAX))
					*p_KD_LINE = new_kd_line;
			}
			putHeaderOnTx(dataTx, MODIFY_KD_LINE, 2);
			putByteOnTx(dataTx, ACK);
			putByteOnTx(dataTx, dataTx->chk);
		break;

        case MODIFY_KI_LINE:
			if (p_KI_LINE) {
				float new_ki_line;
				if (getF32BoundedFromRx(dataRx, &new_ki_line, 0.0f, PID_GAIN_CMD_MAX))
					*p_KI_LINE = new_ki_line;
			}
			putHeaderOnTx(dataTx, MODIFY_KI_LINE, 2);
			putByteOnTx(dataTx, ACK);
			putByteOnTx(dataTx, dataTx->chk);
		break;

        case MODIFY_LINE_THRES:
			if (p_LINE_THRES) {
				float new_thres;
				if (getF32BoundedFromRx(dataRx, &new_thres, 0.0f, LINE_THRES_CMD_MAX))
					*p_LINE_THRES = new_thres;
			}
			putHeaderOnTx(dataTx, MODIFY_LINE_THRES, 2);
			putByteOnTx(dataTx, ACK);
			putByteOnTx(dataTx, dataTx->chk);
		break;

        case MODIFY_LINE_SPEED:
			if (p_LINE_SPEED) {
				float requested_speed;
				if (getF32BoundedFromRx(dataRx, &requested_speed,
				                        LINE_SPEED_CMD_MIN_MPS, LINE_SPEED_CMD_MAX_MPS))
					*p_LINE_SPEED = requested_speed;
			}
			putHeaderOnTx(dataTx, MODIFY_LINE_SPEED, 2);
			putByteOnTx(dataTx, ACK);
			putByteOnTx(dataTx, dataTx->chk);
		break;

        // Tope de inclinacion del setpoint dinamico [grados] (2026-07-27).
        // Es el "sp_limit" de Ctrl_SetpointDinamico: cuanto puede inclinarse
        // el robot para acelerar o frenar. Responde UNKNOWN si no hay binding
        // (mismo patron que MODIFY_BETA_*), no un ACK mentiroso.
        case MODIFY_SP_LIMIT:
            if (p_SP_LIMIT) {
                float new_sp_limit;
                if (getF32BoundedFromRx(dataRx, &new_sp_limit,
                                        SP_LIMIT_CMD_MIN_DEG, SP_LIMIT_CMD_MAX_DEG))
                    *p_SP_LIMIT = new_sp_limit;
                putHeaderOnTx(dataTx, MODIFY_SP_LIMIT, 2);
                putByteOnTx(dataTx, ACK);
            } else {
                putHeaderOnTx(dataTx, MODIFY_SP_LIMIT, 2);
                putByteOnTx(dataTx, UNKNOWN);
            }
            putByteOnTx(dataTx, dataTx->chk);
        break;

        case ACTIVATE_LINE_FOLLOWING:
			if (p_robot_state != NULL) {
				if (*p_robot_state == 3) { // ROBOT_STATE_LINE_FOLLOWING -> BALANCE_ONLY
					*p_robot_state = 1;
				} else if (*p_robot_state != 0) { // IF NOT IDLE -> LINE_FOLLOWING
					*p_robot_state = 3;
				}
				putHeaderOnTx(dataTx, ACTIVATE_LINE_FOLLOWING, 2);
				putByteOnTx(dataTx, ACK);
				putByteOnTx(dataTx, dataTx->chk);
			}
		break;

        case ACTIVATE_POS_MAINTENANCE:
			if (p_robot_state != NULL) {
				if (*p_robot_state == 2) { // BALANCE_AND_SPEED -> BALANCE_ONLY
					*p_robot_state = 1;
				} else if (*p_robot_state == 1) { // BALANCE_ONLY -> BALANCE_AND_SPEED
					*p_robot_state = 2;
				}
				putHeaderOnTx(dataTx, ACTIVATE_POS_MAINTENANCE, 2);
				putByteOnTx(dataTx, ACK);
				putByteOnTx(dataTx, dataTx->chk);
			}
		break;

        case ACTIVATE_MANUAL_CONTROL:
            if (p_robot_state != NULL) {
                if (*p_robot_state == 4) { // MANUAL_CONTROL -> BALANCE_ONLY
                    *p_robot_state = 1;
                } else if (*p_robot_state != 0) { // IF NOT IDLE -> MANUAL_CONTROL
                    *p_robot_state = 4;
                }
                putHeaderOnTx(dataTx, ACTIVATE_MANUAL_CONTROL, 2);
                putByteOnTx(dataTx, ACK);
                putByteOnTx(dataTx, dataTx->chk);
            }
        break;

        case MOVE_FORWARD:
            if (p_robot_state != NULL && (*p_robot_state == 4 || *p_robot_state == 3)) {
                // ==4: MANUAL_CONTROL siempre. ==3: LINE_FOLLOWING también acepta estos
                // comandos, pero main.c los ignora si en ese momento SÍ ve la línea
                // (manual_line_override exige !line_detected) — acá no se puede saber
                // eso, así que se deja pasar siempre y el firmware filtra.
                if (p_manual_sp_cmd) *p_manual_sp_cmd = 1.0f; // m/s deseados hacia adelante (ver MANUAL_SPEED_MAX en main.c)
                if (p_manual_st_cmd) *p_manual_st_cmd = 0.0f;
                last_manual_cmd = MOVE_FORWARD;
                if (p_manual_tmo) *p_manual_tmo = HAL_GetTick();
            }
            putHeaderOnTx(dataTx, MOVE_FORWARD, 2);
            putByteOnTx(dataTx, ACK);
            putByteOnTx(dataTx, dataTx->chk);
        break;

        case MOVE_BACKWARD:
            if (p_robot_state != NULL && (*p_robot_state == 4 || *p_robot_state == 3)) {
                // ==4: MANUAL_CONTROL siempre. ==3: LINE_FOLLOWING también acepta estos
                // comandos, pero main.c los ignora si en ese momento SÍ ve la línea
                // (manual_line_override exige !line_detected) — acá no se puede saber
                // eso, así que se deja pasar siempre y el firmware filtra.
                if (p_manual_sp_cmd) *p_manual_sp_cmd = -1.0f; // m/s deseados hacia atrás
                if (p_manual_st_cmd) *p_manual_st_cmd = 0.0f;
                last_manual_cmd = MOVE_BACKWARD;
                if (p_manual_tmo) *p_manual_tmo = HAL_GetTick();
            }
            putHeaderOnTx(dataTx, MOVE_BACKWARD, 2);
            putByteOnTx(dataTx, ACK);
            putByteOnTx(dataTx, dataTx->chk);
        break;

        case MOVE_LEFT:
            if (p_robot_state != NULL && (*p_robot_state == 4 || *p_robot_state == 3)) {
                // ==4: MANUAL_CONTROL siempre. ==3: LINE_FOLLOWING también acepta estos
                // comandos, pero main.c los ignora si en ese momento SÍ ve la línea
                // (manual_line_override exige !line_detected) — acá no se puede saber
                // eso, así que se deja pasar siempre y el firmware filtra.
                if (p_manual_sp_cmd) *p_manual_sp_cmd = 0.0f;
                if (p_manual_st_cmd) *p_manual_st_cmd = -15.0f; // steering value for left (1/4 de 60, a pedido: giro demasiado fuerte)
                last_manual_cmd = MOVE_LEFT;
                if (p_manual_tmo) *p_manual_tmo = HAL_GetTick();
            }
            putHeaderOnTx(dataTx, MOVE_LEFT, 2);
            putByteOnTx(dataTx, ACK);
            putByteOnTx(dataTx, dataTx->chk);
        break;

        case MOVE_RIGHT:
            if (p_robot_state != NULL && (*p_robot_state == 4 || *p_robot_state == 3)) {
                // ==4: MANUAL_CONTROL siempre. ==3: LINE_FOLLOWING también acepta estos
                // comandos, pero main.c los ignora si en ese momento SÍ ve la línea
                // (manual_line_override exige !line_detected) — acá no se puede saber
                // eso, así que se deja pasar siempre y el firmware filtra.
                if (p_manual_sp_cmd) *p_manual_sp_cmd = 0.0f;
                if (p_manual_st_cmd) *p_manual_st_cmd = 15.0f; // steering value for right (1/4 de 60, a pedido: giro demasiado fuerte)
                last_manual_cmd = MOVE_RIGHT;
                if (p_manual_tmo) *p_manual_tmo = HAL_GetTick();
            }
            putHeaderOnTx(dataTx, MOVE_RIGHT, 2);
            putByteOnTx(dataTx, ACK);
            putByteOnTx(dataTx, dataTx->chk);
        break;

        case MOVE_STOP:
            if (p_robot_state != NULL && (*p_robot_state == 4 || *p_robot_state == 3)) {
                // ==4: MANUAL_CONTROL siempre. ==3: LINE_FOLLOWING también acepta estos
                // comandos, pero main.c los ignora si en ese momento SÍ ve la línea
                // (manual_line_override exige !line_detected) — acá no se puede saber
                // eso, así que se deja pasar siempre y el firmware filtra.
                if (p_manual_sp_cmd) *p_manual_sp_cmd = 0.0f;
                if (p_manual_st_cmd) *p_manual_st_cmd = 0.0f;
                last_manual_cmd = MOVE_STOP;
                if (p_manual_tmo) *p_manual_tmo = HAL_GetTick();
            }
            putHeaderOnTx(dataTx, MOVE_STOP, 2);
            putByteOnTx(dataTx, ACK);
            putByteOnTx(dataTx, dataTx->chk);
        break;

        case ROTATE_90_RIGHT:
            if (p_robot_state != NULL && *p_robot_state == 4 && p_rot_target_deg && p_rot_trigger) {
                *p_rot_target_deg = +90.0f;
                *p_rot_trigger    = 1;
            }
            putHeaderOnTx(dataTx, ROTATE_90_RIGHT, 2);
            putByteOnTx(dataTx, ACK);
            putByteOnTx(dataTx, dataTx->chk);
        break;

        case ROTATE_90_LEFT:
            if (p_robot_state != NULL && *p_robot_state == 4 && p_rot_target_deg && p_rot_trigger) {
                *p_rot_target_deg = -90.0f;
                *p_rot_trigger    = 1;
            }
            putHeaderOnTx(dataTx, ROTATE_90_LEFT, 2);
            putByteOnTx(dataTx, ACK);
            putByteOnTx(dataTx, dataTx->chk);
        break;

        case ROTATE_180_RIGHT:
            // Deshabilitado temporalmente: solo se permiten giros de 90°.
            putHeaderOnTx(dataTx, ROTATE_180_RIGHT, 2);
            putByteOnTx(dataTx, ACK);
            putByteOnTx(dataTx, dataTx->chk);
        break;

        case ROTATE_180_LEFT:
            // Deshabilitado temporalmente: solo se permiten giros de 90°.
            putHeaderOnTx(dataTx, ROTATE_180_LEFT, 2);
            putByteOnTx(dataTx, ACK);
            putByteOnTx(dataTx, dataTx->chk);
        break;

        case ROTATE_CUSTOM:
        {
            // Payload: 4 bytes = float en little-endian (grados; + = derecha, - = izquierda)
            if (p_robot_state != NULL && *p_robot_state == 4 && p_rot_target_deg && p_rot_trigger) {
                // Leído con getByteFromRx como el resto de los comandos — la
                // versión anterior indexaba desde indexR (que acá apunta al
                // checksum) y caía fuera del frame: leía basura.
                float requested_rotation;
                if (getF32BoundedFromRx(dataRx, &requested_rotation,
                                        -ROTATE_CUSTOM_MAX_DEG, ROTATE_CUSTOM_MAX_DEG) &&
                    requested_rotation != 0.0f) {
                    *p_rot_target_deg = requested_rotation;
                    *p_rot_trigger    = 1;
                }
            }
            putHeaderOnTx(dataTx, ROTATE_CUSTOM, 2);
            putByteOnTx(dataTx, ACK);
            putByteOnTx(dataTx, dataTx->chk);
        }
        break;

        case SENDALLSENSORS:
        	sendAllSensorsFlag = !sendAllSensorsFlag;	// Si esta activa desactivo, y sino, activo
			if (sensorsAreRegistered()) {
				putHeaderOnTx(dataTx, SENDALLSENSORS, 37);
				putAllSensorsOnTx(dataTx);
			} else {
				putHeaderOnTx(dataTx, SENDALLSENSORS, 2);
				putByteOnTx(dataTx, UNKNOWN);
			}
			putByteOnTx(dataTx, dataTx->chk);
        	break;
        case GET_ODOMETRY:
            if (p_odom_x && p_odom_y && p_odom_theta) {
                putHeaderOnTx(dataTx, GET_ODOMETRY, 13); // 1 cmd + 3 floats

                putF32OnTx(dataTx, *p_odom_x);
                putF32OnTx(dataTx, *p_odom_y);
                putF32OnTx(dataTx, *p_odom_theta);

                putByteOnTx(dataTx, dataTx->chk);
            } else {
                putHeaderOnTx(dataTx, GET_ODOMETRY, 2);
                putByteOnTx(dataTx, ACK);
                putByteOnTx(dataTx, dataTx->chk);
            }
        break;

        case RESET_ODOMETRY:
            if (p_odom_x)     *p_odom_x     = 0.0f;
            if (p_odom_y)     *p_odom_y     = 0.0f;
            if (p_odom_theta) *p_odom_theta = 0.0f;
            putHeaderOnTx(dataTx, RESET_ODOMETRY, 2);
            putByteOnTx(dataTx, ACK);
            putByteOnTx(dataTx, dataTx->chk);
        break;

        case MODIFY_SETPOINT:
            // El spinbox de Qt permite +-180 grados: sin este clamp un valor
            // grande se aplicaba directo al setpoint y volteaba el robot.
            if (p_setpoint_trim) {
                float new_trim;
                if (getF32BoundedFromRx(dataRx, &new_trim,
                                        -SETPOINT_TRIM_MAX_DEG, SETPOINT_TRIM_MAX_DEG))
                    *p_setpoint_trim = new_trim;
            }
            putHeaderOnTx(dataTx, MODIFY_SETPOINT, 2);
            putByteOnTx(dataTx, ACK);
            putByteOnTx(dataTx, dataTx->chk);
        break;

        default:
            putHeaderOnTx(dataTx, (_eCmd)dataRx->buff[dataRx->indexData], 2);
            putByteOnTx(dataTx,UNKNOWN );
            putByteOnTx(dataTx, dataTx->chk);
        break;
    }
    UNER_SendData();
}


void UNER_SendAlive(void) {
    USB_Debug(">>> UNER_SendAlive llamado\n");

    if (unerTx->indexR != unerTx->indexW) {
        USB_DebugStr(">>> NO envio ALIVE: buffer ocupado\n");
        return;
    }
    if (ESP01_IsSending()) {
        USB_DebugStr(">>> NO envio ALIVE: ESP01 enviando\n");
        return;
    }
    if (ESP01_StateUDPTCP() != ESP01_UDPTCP_CONNECTED) {
        USB_DebugStr(">>> NO envio ALIVE: sin conexion UDP\n");
        return;
    }

    putHeaderOnTx(unerTx, ALIVE, 2);
    putByteOnTx(unerTx, ACK);
    putByteOnTx(unerTx, unerTx->chk);

    UNER_SendData();
}

void UNER_SendAllSensors(void) {
	if (ESP01_IsSending() || !sensorsAreRegistered()) return;

	putHeaderOnTx(unerTx, SENDALLSENSORS, 37);
	putAllSensorsOnTx(unerTx);
	putByteOnTx(unerTx, unerTx->chk);
	UNER_SendData();
}

uint8_t UNER_ShouldSendAllSensors(void) {
    return sendAllSensorsFlag;
}

void UNER_RegisterBindings(const UNER_Bindings_t *b) {
    if (b == NULL) return;

    p_adcBuf = b->adc; adcBufLen = b->adc_len;
    p_motorRightVel = b->motor_right_velocity; p_motorLeftVel = b->motor_left_velocity;
    p_ax = b->ax; p_ay = b->ay; p_az = b->az;
    p_gx = b->gx; p_gy = b->gy; p_gz = b->gz;
    p_roll = b->roll; p_pitch = b->pitch;
    p_KP = b->kp; p_KD = b->kd; p_KI = b->ki; p_KV_BRAKE = b->kv_brake;
    p_BETA_G = b->beta_g; p_BETA_A = b->beta_a;
    p_steering = b->steering; p_robot_state = b->robot_state;
    p_resetMassCenter_flag = b->reset_mass_center;
    p_send_csv_log_flag = b->send_csv_log; p_send_wifi_log_flag = b->send_wifi_log;
    p_change_display = b->change_display;
    p_KP_LINE = b->kp_line; p_KD_LINE = b->kd_line; p_KI_LINE = b->ki_line;
    p_LINE_THRES = b->line_threshold; p_LINE_SPEED = b->line_speed;
    p_SP_LIMIT = b->sp_limit;
    p_manual_sp_cmd = b->manual_setpoint; p_manual_st_cmd = b->manual_steering;
    p_manual_tmo = b->manual_timeout_ms;
    p_rot_target_deg = b->rotation_target_deg; p_rot_trigger = b->rotation_trigger;
    p_odom_x = b->odom_x; p_odom_y = b->odom_y; p_odom_theta = b->odom_theta;
    p_setpoint_trim = b->setpoint_trim;
    p_velocity_mps = b->velocity_mps;
    p_wheel_right_rps = b->wheel_right_rps; p_wheel_left_rps = b->wheel_left_rps;
}

// Envía el ring-buffer por USB y por UDP
static void UNER_SendData(void) {
    uint16_t len = (unerTx->indexW + unerTx->mask + 1 - unerTx->indexR) & unerTx->mask;
    if (!len) return;

    // USB CDC — copiar con wrap: la trama puede cruzar el final del ring buffer,
    // pero ambas partes se reservan juntas para no encolar media trama.
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
        uint16_t first_len = (uint16_t)(TXBUFSIZE - unerTx->indexR);
        if (first_len > len) first_len = len;
        uint16_t second_len = (uint16_t)(len - first_len);
        usb_enqueue_tx_segments(&unerTx->buff[unerTx->indexR], first_len,
                                unerTx->buff, second_len);
    }

    // UDP (ESP01)
    if (ESP01_StateUDPTCP() == ESP01_UDPTCP_CONNECTED && !ESP01_IsSending()) {
        ESP01_Send(unerTx->buff, unerTx->indexR, len, TXBUFSIZE);
    }

    // Avanzar índice de lectura
    unerTx->indexR = (unerTx->indexR + len) & unerTx->mask;
}

static void UNER_SendWifiStruct(_eCmd command, const void *data, uint8_t payloadLen) {
    if (data == NULL || payloadLen == 0U)
        return;

    if (ESP01_StateUDPTCP() != ESP01_UDPTCP_CONNECTED || ESP01_IsSending()) {
        return;
    }

    putHeaderOnTx(unerTx, command, (uint8_t)(payloadLen + 1U));
    putBytesOnTx(unerTx, data, payloadLen);
    putByteOnTx(unerTx, unerTx->chk);
    UNER_SendData();
}

void UNER_SendWifiLogData(WifiLogData_t *data) {
    UNER_SendWifiStruct(CMD_WIFI_LOG_DATA, data, (uint8_t)sizeof(*data));
}

void UNER_SendWifiOdomData(WifiOdomData_t *data) {
    UNER_SendWifiStruct(CMD_WIFI_ODOM_DATA, data, (uint8_t)sizeof(*data));
}

uint8_t UNER_GetLastManualCmd(void) { return last_manual_cmd; }

/* END Private Functions*/
