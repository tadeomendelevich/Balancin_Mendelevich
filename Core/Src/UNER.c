/*
 * UNER.c
 *
 *  Created on: Apr 16, 2025
 *      Author: Tadeo Mendelevich
 */

#include "UNER.h"
#include <stddef.h>
#include <string.h>
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

static float *p_manual_sp_cmd = NULL;
static float *p_manual_st_cmd = NULL;
static uint32_t *p_manual_tmo = NULL;

static float   *p_rot_target_deg = NULL;
static uint8_t *p_rot_trigger    = NULL;

static float *p_odom_x     = NULL;   // Punteros a la pose odométrica en main.c
static float *p_odom_y     = NULL;
static float *p_odom_theta = NULL;

static float *p_setpoint_trim = NULL;

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
            putHeaderOnTx(dataTx, FIRMWARE, 12);
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
			float new_KP = getF32FromRx(dataRx);
			if (p_KP) *p_KP = new_KP;

			putHeaderOnTx(dataTx, MODIFYKP, 13);
			putPidValuesOnTx(dataTx);
			putByteOnTx(dataTx, dataTx->chk);
        break;

        case MODIFYKD:
			float new_KD = getF32FromRx(dataRx);
			if (p_KD) *p_KD = new_KD;

			putHeaderOnTx(dataTx, MODIFYKD, 13);
			putPidValuesOnTx(dataTx);
			putByteOnTx(dataTx, dataTx->chk);
		break;

        case MODIFYKI:
			float new_KI = getF32FromRx(dataRx);
			if (p_KI) *p_KI = new_KI;

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
            if (p_steering) *p_steering = getF32FromRx(dataRx);
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

        case MODIFY_BETA_G:
			float new_BETA_G = getF32FromRx(dataRx);
			if (p_BETA_G) *p_BETA_G = new_BETA_G;

			putHeaderOnTx(dataTx, MODIFY_BETA_G, 2);
			putByteOnTx(dataTx, ACK);
			putByteOnTx(dataTx, dataTx->chk);
		break;

        case MODIFY_BETA_A:
			float new_BETA_A = getF32FromRx(dataRx);
			if (p_BETA_A) *p_BETA_A = new_BETA_A;

			putHeaderOnTx(dataTx, MODIFY_BETA_A, 2);
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
			if (p_KP_LINE) *p_KP_LINE = getF32FromRx(dataRx);
			putHeaderOnTx(dataTx, MODIFY_KP_LINE, 2);
			putByteOnTx(dataTx, ACK);
			putByteOnTx(dataTx, dataTx->chk);
		break;

        case MODIFY_KD_LINE:
			if (p_KD_LINE) *p_KD_LINE = getF32FromRx(dataRx);
			putHeaderOnTx(dataTx, MODIFY_KD_LINE, 2);
			putByteOnTx(dataTx, ACK);
			putByteOnTx(dataTx, dataTx->chk);
		break;

        case MODIFY_KI_LINE:
			if (p_KI_LINE) *p_KI_LINE = getF32FromRx(dataRx);
			putHeaderOnTx(dataTx, MODIFY_KI_LINE, 2);
			putByteOnTx(dataTx, ACK);
			putByteOnTx(dataTx, dataTx->chk);
		break;

        case MODIFY_LINE_THRES:
			if (p_LINE_THRES) *p_LINE_THRES = getF32FromRx(dataRx);
			putHeaderOnTx(dataTx, MODIFY_LINE_THRES, 2);
			putByteOnTx(dataTx, ACK);
			putByteOnTx(dataTx, dataTx->chk);
		break;

        case MODIFY_LINE_SPEED:
			if (p_LINE_SPEED) *p_LINE_SPEED = getF32FromRx(dataRx);
			putHeaderOnTx(dataTx, MODIFY_LINE_SPEED, 2);
			putByteOnTx(dataTx, ACK);
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
                float requested_rotation = getF32FromRx(dataRx);
                if (requested_rotation != 0.0f) {
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
            if (p_setpoint_trim) *p_setpoint_trim = getF32FromRx(dataRx);
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
    p_manual_sp_cmd = b->manual_setpoint; p_manual_st_cmd = b->manual_steering;
    p_manual_tmo = b->manual_timeout_ms;
    p_rot_target_deg = b->rotation_target_deg; p_rot_trigger = b->rotation_trigger;
    p_odom_x = b->odom_x; p_odom_y = b->odom_y; p_odom_theta = b->odom_theta;
    p_setpoint_trim = b->setpoint_trim;
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
