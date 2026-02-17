/*
 * UNER.c
 *
 *  Created on: Apr 16, 2025
 *      Author: Tadeo Mendelevich
 */

#include "UNER.h"
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include "ESP01.h"

#include <stdarg.h>       // para va_list, va_start, va_end
#include "usbd_cdc_if.h"  // para CDC_Transmit_FS
#include "usbd_core.h"    // para USBD_HandleTypeDef, USBD_STATE_CONFIGURED
void USB_Debug(const char *fmt, ...);  // declaración implícita del prototipo


#include "ESP01.h"

char *firmware = "UNER V1.0";

uint16_t globalIndex = 0;

static _sRx *unerRx;
static _sTx *unerTx;

_uWord myWord;

// buffer temporal para formateo
//static char dbgBuf[128];

// Prototipo externo de estado USB (ya lo tienes en main.c)
extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t usb_enqueue_tx(const uint8_t *data, uint16_t len);

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

static float *p_roll  = NULL;	// Punteros a las variables de inclinacion en main.c
static float *p_pitch = NULL;

static float *p_steering = NULL;

static uint8_t *p_balance_flag = NULL;	// Bandera para activar o desctivar el balance del auto, si esta apagada, los motores estan desabilitados

static uint8_t *p_resetMassCenter_flag = NULL;

void UNER_Init(_sRx *rx, _sTx *tx, int16_t *ax_ptr, int16_t *ay_ptr, int16_t *az_ptr, int16_t *gx_ptr, int16_t *gy_ptr, int16_t *gz_ptr) {
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

    p_ax = ax_ptr; p_ay = ay_ptr; p_az = az_ptr;
    p_gx = gx_ptr; p_gy = gy_ptr; p_gz = gz_ptr;
}

void UNER_PushByte(uint8_t byte) {
    unerRx->buff[unerRx->indexW++] = byte;
    unerRx->indexW &= unerRx->mask;
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
                unerRx->header = TOKEN;
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

void UNER_Send(uint8_t cmd, const uint8_t *payload, uint8_t length) {
    uint8_t chk = 0;
    char header[] = { 'U', 'N', 'E', 'R' };

    for (int i = 0; i < 4; i++) {
        unerTx->buff[unerTx->indexW++] = header[i];
        unerTx->indexW &= unerTx->mask;
    }

    uint8_t len = length + 1;
    unerTx->buff[unerTx->indexW++] = len;
    unerTx->indexW &= unerTx->mask;
    unerTx->buff[unerTx->indexW++] = ':';
    unerTx->indexW &= unerTx->mask;
    unerTx->buff[unerTx->indexW++] = cmd;
    unerTx->indexW &= unerTx->mask;

    chk ^= ('U' ^ 'N' ^ 'E' ^ 'R' ^ len ^ ':' ^ cmd);
    for (uint8_t i = 0; i < length; i++) {
        unerTx->buff[unerTx->indexW++] = payload[i];
        unerTx->indexW &= unerTx->mask;
        chk ^= payload[i];
    }

    unerTx->buff[unerTx->indexW++] = chk;
    unerTx->indexW &= unerTx->mask;
}

uint8_t putHeaderOnTx(_sTx  *dataTx, _eCmd ID, uint8_t frameLength)
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

uint8_t putByteOnTx(_sTx *dataTx, uint8_t byte)
{
    dataTx->buff[dataTx->indexW++]=byte;
    dataTx->indexW &= dataTx->mask;
    dataTx->chk ^= byte;
    return dataTx->chk;
}

uint8_t putStrOntx(_sTx *dataTx, const char *str)
{
    globalIndex=0;
    while(str[globalIndex]){
        dataTx->buff[dataTx->indexW++]=str[globalIndex];
        dataTx->indexW &= dataTx->mask;
        dataTx->chk ^= str[globalIndex++];
    }
    return dataTx->chk ;
}

uint8_t getByteFromRx(_sRx *dataRx, uint8_t iniPos, uint8_t finalPos){
    uint8_t getByte;
    dataRx->indexData += iniPos;
    dataRx->indexData &=dataRx->mask;
    getByte = dataRx->buff[dataRx->indexData];
    dataRx->indexData += finalPos;
    dataRx->indexData &=dataRx->mask;
    return getByte;
}

void decodeCommand(_sRx *dataRx, _sTx *dataTx)
{
    switch(dataRx->buff[dataRx->indexData]){
        case ALIVE:
        	USB_Debug("\n ALIVE RECIBIDO!\n");
            putHeaderOnTx(dataTx, ALIVE, 2);
            putByteOnTx(dataTx, ACK);
            putByteOnTx(dataTx, dataTx->chk);
        break;
        case FIRMWARE:
            putHeaderOnTx(dataTx, FIRMWARE, 12);
            putStrOntx(dataTx, firmware);
            putByteOnTx(dataTx, dataTx->chk);
        break;
        case GETADCVALUES:
        	putHeaderOnTx(dataTx, GETADCVALUES, 17);

			myWord.ui16[0] =  (int16_t)p_adcBuf[0]; 		// ADC 1
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );
			myWord.ui16[0] =  (int16_t)p_adcBuf[1];			// ADC 2
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );
			myWord.ui16[0] =  (int16_t)p_adcBuf[2];			// ADC 3
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );
			myWord.ui16[0] =  (int16_t)p_adcBuf[3];			// ADC 4
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );
			myWord.ui16[0] =  (int16_t)p_adcBuf[4];			// ADC 5
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );
			myWord.ui16[0] =  (int16_t)p_adcBuf[5];			// ADC 6
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );
			myWord.ui16[0] =  (int16_t)p_adcBuf[6];			// ADC 7
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );
			myWord.ui16[0] =  (int16_t)p_adcBuf[7];			// ADC 8
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );

			putByteOnTx(dataTx, dataTx->chk);
        	break;
        case GETMPU6050VALUES:
        	putHeaderOnTx(dataTx, GETMPU6050VALUES, 13);

			myWord.ui16[0] =  (int16_t)*p_ax; 	// Envio datos de aceleracion
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );
			myWord.ui16[0] =  (int16_t)*p_ay;
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );
			myWord.ui16[0] =  (int16_t)*p_az;
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );

			myWord.ui16[0] =  (int16_t)*p_gx; 	// Envio datos de giroscopio
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );
			myWord.ui16[0] =  (int16_t)*p_gy;
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );
			myWord.ui16[0] =  (int16_t)*p_gz;
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );

			putByteOnTx(dataTx, dataTx->chk);
        	break;
        case GETANGLE:
        	if (p_roll && p_pitch) {
        		putHeaderOnTx(dataTx, GETANGLE, 9); // 1 byte cmd + 2*4 bytes for float angles

				myWord.f32 = *p_roll; 	// Envio datos de INCLINACION
				putByteOnTx(dataTx, myWord.ui8[0] );
				putByteOnTx(dataTx, myWord.ui8[1] );
				putByteOnTx(dataTx, myWord.ui8[2] );
				putByteOnTx(dataTx, myWord.ui8[3] );
				myWord.f32 = *p_pitch;
				putByteOnTx(dataTx, myWord.ui8[0] );
				putByteOnTx(dataTx, myWord.ui8[1] );
				putByteOnTx(dataTx, myWord.ui8[2] );
				putByteOnTx(dataTx, myWord.ui8[3] );

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
			myWord.ui8[0]=getByteFromRx(dataRx,1,0);
			myWord.ui8[1]=getByteFromRx(dataRx,1,0);
			myWord.ui8[2]=getByteFromRx(dataRx,1,0);
			myWord.ui8[3]=getByteFromRx(dataRx,1,0);
			int16_t vLeft = myWord.i32;
			// clamp al rango [-100, +100]
			if      (vLeft >  100) vLeft =  100;
			else if (vLeft < -100) vLeft = -100;
			// sólo si el puntero está registrado
			if (p_motorLeftVel) *p_motorLeftVel = vLeft;

			myWord.ui8[0]=getByteFromRx(dataRx,1,0);
			myWord.ui8[1]=getByteFromRx(dataRx,1,0);
			myWord.ui8[2]=getByteFromRx(dataRx,1,0);
			myWord.ui8[3]=getByteFromRx(dataRx,1,0);
			int16_t vRight = myWord.i32;
			if      (vRight >  100) vRight =  100;
			else if (vRight < -100) vRight = -100;
			if (p_motorRightVel) *p_motorRightVel = vRight;
		break;

        case MODIFYKP:
			myWord.ui8[0]=getByteFromRx(dataRx,1,0); // Extraigo datos de KP
			myWord.ui8[1]=getByteFromRx(dataRx,1,0);
			myWord.ui8[2]=getByteFromRx(dataRx,1,0);
			myWord.ui8[3]=getByteFromRx(dataRx,1,0);
			float new_KP = myWord.f32;
			if (p_KP) *p_KP = new_KP;

			putHeaderOnTx(dataTx, MODIFYKP, 13);
			myWord.f32 = *p_KP; 	// Envio datos de KP
			putByteOnTx(dataTx, myWord.ui8[0]);
			putByteOnTx(dataTx, myWord.ui8[1]);
			putByteOnTx(dataTx, myWord.ui8[2]);
			putByteOnTx(dataTx, myWord.ui8[3]);

			myWord.f32 = *p_KD; 	// Envio datos de KD
			putByteOnTx(dataTx, myWord.ui8[0]);
			putByteOnTx(dataTx, myWord.ui8[1]);
			putByteOnTx(dataTx, myWord.ui8[2]);
			putByteOnTx(dataTx, myWord.ui8[3]);

			myWord.f32 = *p_KI; 	// Envio datos de KI
			putByteOnTx(dataTx, myWord.ui8[0]);
			putByteOnTx(dataTx, myWord.ui8[1]);
			putByteOnTx(dataTx, myWord.ui8[2]);
			putByteOnTx(dataTx, myWord.ui8[3]);
			putByteOnTx(dataTx, dataTx->chk);
        break;

        case MODIFYKD:
			myWord.ui8[0]=getByteFromRx(dataRx,1,0);
			myWord.ui8[1]=getByteFromRx(dataRx,1,0);
			myWord.ui8[2]=getByteFromRx(dataRx,1,0);
			myWord.ui8[3]=getByteFromRx(dataRx,1,0);
			float new_KD = myWord.f32;
			if (p_KD) *p_KD = new_KD;

			putHeaderOnTx(dataTx, MODIFYKD, 13);
			myWord.f32 = *p_KP; 	// Envio datos de KP
			putByteOnTx(dataTx, myWord.ui8[0]);
			putByteOnTx(dataTx, myWord.ui8[1]);
			putByteOnTx(dataTx, myWord.ui8[2]);
			putByteOnTx(dataTx, myWord.ui8[3]);

			myWord.f32 = *p_KD; 	// Envio datos de KD
			putByteOnTx(dataTx, myWord.ui8[0]);
			putByteOnTx(dataTx, myWord.ui8[1]);
			putByteOnTx(dataTx, myWord.ui8[2]);
			putByteOnTx(dataTx, myWord.ui8[3]);

			myWord.f32 = *p_KI; 	// Envio datos de KI
			putByteOnTx(dataTx, myWord.ui8[0]);
			putByteOnTx(dataTx, myWord.ui8[1]);
			putByteOnTx(dataTx, myWord.ui8[2]);
			putByteOnTx(dataTx, myWord.ui8[3]);
			putByteOnTx(dataTx, dataTx->chk);
		break;

        case MODIFYKI:
			myWord.ui8[0]=getByteFromRx(dataRx,1,0);
			myWord.ui8[1]=getByteFromRx(dataRx,1,0);
			myWord.ui8[2]=getByteFromRx(dataRx,1,0);
			myWord.ui8[3]=getByteFromRx(dataRx,1,0);
			float new_KI = myWord.f32;
			if (p_KI) *p_KI = new_KI;

			putHeaderOnTx(dataTx, MODIFYKI, 13);
			myWord.f32 = *p_KP; 	// Envio datos de KP
			putByteOnTx(dataTx, myWord.ui8[0]);
			putByteOnTx(dataTx, myWord.ui8[1]);
			putByteOnTx(dataTx, myWord.ui8[2]);
			putByteOnTx(dataTx, myWord.ui8[3]);

			myWord.f32 = *p_KD; 	// Envio datos de KD
			putByteOnTx(dataTx, myWord.ui8[0]);
			putByteOnTx(dataTx, myWord.ui8[1]);
			putByteOnTx(dataTx, myWord.ui8[2]);
			putByteOnTx(dataTx, myWord.ui8[3]);

			myWord.f32 = *p_KI; 	// Envio datos de KI
			putByteOnTx(dataTx, myWord.ui8[0]);
			putByteOnTx(dataTx, myWord.ui8[1]);
			putByteOnTx(dataTx, myWord.ui8[2]);
			putByteOnTx(dataTx, myWord.ui8[3]);
			putByteOnTx(dataTx, dataTx->chk);
		break;

        case BALANCE:
        	if (p_balance_flag != NULL) {
				*p_balance_flag = !(*p_balance_flag);
				putHeaderOnTx(dataTx, BALANCE, 2);
				putByteOnTx(dataTx, ACK);
				putByteOnTx(dataTx, dataTx->chk);
			}
        break;

        case GETPIDVALUES:
        	putHeaderOnTx(dataTx, GETPIDVALUES, 13); // 1 byte cmd + 2*4 bytes for float values

			myWord.f32 = *p_KP; 	// Envio datos de INCLINACION
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );
			putByteOnTx(dataTx, myWord.ui8[2] );
			putByteOnTx(dataTx, myWord.ui8[3] );
			myWord.f32 = *p_KD;
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );
			putByteOnTx(dataTx, myWord.ui8[2] );
			putByteOnTx(dataTx, myWord.ui8[3] );
			myWord.f32 = *p_KI;
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );
			putByteOnTx(dataTx, myWord.ui8[2] );
			putByteOnTx(dataTx, myWord.ui8[3] );

			putByteOnTx(dataTx, dataTx->chk);
        break;

        case MODIFYSTEERING:
            myWord.ui8[0]=getByteFromRx(dataRx,1,0);
            myWord.ui8[1]=getByteFromRx(dataRx,1,0);
            myWord.ui8[2]=getByteFromRx(dataRx,1,0);
            myWord.ui8[3]=getByteFromRx(dataRx,1,0);
            if (p_steering) *p_steering = myWord.f32;
            putHeaderOnTx(dataTx, MODIFYSTEERING, 2);
            putByteOnTx(dataTx, ACK);
            putByteOnTx(dataTx, dataTx->chk);
        break;

        case RESETMASSCENTER:
			if (p_resetMassCenter_flag != NULL) {
				*p_resetMassCenter_flag = !(*p_resetMassCenter_flag);
				putHeaderOnTx(dataTx, RESETMASSCENTER, 2);
				putByteOnTx(dataTx, ACK);
				putByteOnTx(dataTx, dataTx->chk);
			}
		break;

        case SENDALLSENSORS:
        	sendAllSensorsFlag = !sendAllSensorsFlag;	// Si esta activa desactivo, y sino, activo

        	putHeaderOnTx(dataTx, SENDALLSENSORS, 37); // 1 cmd + 16 ADC + 12 MPU + 8 Angle

			myWord.ui16[0] =  (int16_t)p_adcBuf[0]; 		// ADC 1
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );
			myWord.ui16[0] =  (int16_t)p_adcBuf[1];			// ADC 2
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );
			myWord.ui16[0] =  (int16_t)p_adcBuf[2];			// ADC 3
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );
			myWord.ui16[0] =  (int16_t)p_adcBuf[3];			// ADC 4
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );
			myWord.ui16[0] =  (int16_t)p_adcBuf[4];			// ADC 5
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );
			myWord.ui16[0] =  (int16_t)p_adcBuf[5];			// ADC 6
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );
			myWord.ui16[0] =  (int16_t)p_adcBuf[6];			// ADC 7
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );
			myWord.ui16[0] =  (int16_t)p_adcBuf[7];			// ADC 8
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );

			myWord.ui16[0] =  (int16_t)*p_ax; 	// Envio datos de aceleracion
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );
			myWord.ui16[0] =  (int16_t)*p_ay;
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );
			myWord.ui16[0] =  (int16_t)*p_az;
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );

			myWord.ui16[0] =  (int16_t)*p_gx; 	// Envio datos de giroscopio
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );
			myWord.ui16[0] =  (int16_t)*p_gy;
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );
			myWord.ui16[0] =  (int16_t)*p_gz;
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );

			myWord.f32 = *p_roll; 	// Envio datos de INCLINACION
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );
			putByteOnTx(dataTx, myWord.ui8[2] );
			putByteOnTx(dataTx, myWord.ui8[3] );
			myWord.f32 = *p_pitch;
			putByteOnTx(dataTx, myWord.ui8[0] );
			putByteOnTx(dataTx, myWord.ui8[1] );
			putByteOnTx(dataTx, myWord.ui8[2] );
			putByteOnTx(dataTx, myWord.ui8[3] );

			putByteOnTx(dataTx, unerTx->chk);
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
	if (ESP01_IsSending())
	        return;

	unerTx->indexW = 0;
	unerTx->indexR = 0;
	unerTx->chk    = 0;

	putHeaderOnTx(unerTx, SENDALLSENSORS, 37); // 1 cmd + 16 ADC + 12 MPU + 8 Angle

	myWord.ui16[0] =  (int16_t)p_adcBuf[0]; 		// ADC 1
	putByteOnTx(unerTx, myWord.ui8[0] );
	putByteOnTx(unerTx, myWord.ui8[1] );
	myWord.ui16[0] =  (int16_t)p_adcBuf[1];			// ADC 2
	putByteOnTx(unerTx, myWord.ui8[0] );
	putByteOnTx(unerTx, myWord.ui8[1] );
	myWord.ui16[0] =  (int16_t)p_adcBuf[2];			// ADC 3
	putByteOnTx(unerTx, myWord.ui8[0] );
	putByteOnTx(unerTx, myWord.ui8[1] );
	myWord.ui16[0] =  (int16_t)p_adcBuf[3];			// ADC 4
	putByteOnTx(unerTx, myWord.ui8[0] );
	putByteOnTx(unerTx, myWord.ui8[1] );
	myWord.ui16[0] =  (int16_t)p_adcBuf[4];			// ADC 5
	putByteOnTx(unerTx, myWord.ui8[0] );
	putByteOnTx(unerTx, myWord.ui8[1] );
	myWord.ui16[0] =  (int16_t)p_adcBuf[5];			// ADC 6
	putByteOnTx(unerTx, myWord.ui8[0] );
	putByteOnTx(unerTx, myWord.ui8[1] );
	myWord.ui16[0] =  (int16_t)p_adcBuf[6];			// ADC 7
	putByteOnTx(unerTx, myWord.ui8[0] );
	putByteOnTx(unerTx, myWord.ui8[1] );
	myWord.ui16[0] =  (int16_t)p_adcBuf[7];			// ADC 8
	putByteOnTx(unerTx, myWord.ui8[0] );
	putByteOnTx(unerTx, myWord.ui8[1] );

	myWord.ui16[0] =  (int16_t)*p_ax; 	// Envio datos de aceleracion
	putByteOnTx(unerTx, myWord.ui8[0] );
	putByteOnTx(unerTx, myWord.ui8[1] );
	myWord.ui16[0] =  (int16_t)*p_ay;
	putByteOnTx(unerTx, myWord.ui8[0] );
	putByteOnTx(unerTx, myWord.ui8[1] );
	myWord.ui16[0] =  (int16_t)*p_az;
	putByteOnTx(unerTx, myWord.ui8[0] );
	putByteOnTx(unerTx, myWord.ui8[1] );

	myWord.ui16[0] =  (int16_t)*p_gx; 	// Envio datos de giroscopio
	putByteOnTx(unerTx, myWord.ui8[0] );
	putByteOnTx(unerTx, myWord.ui8[1] );
	myWord.ui16[0] =  (int16_t)*p_gy;
	putByteOnTx(unerTx, myWord.ui8[0] );
	putByteOnTx(unerTx, myWord.ui8[1] );
	myWord.ui16[0] =  (int16_t)*p_gz;
	putByteOnTx(unerTx, myWord.ui8[0] );
	putByteOnTx(unerTx, myWord.ui8[1] );

	myWord.f32 = *p_roll; 	// Envio datos de INCLINACION
	putByteOnTx(unerTx, myWord.ui8[0] );
	putByteOnTx(unerTx, myWord.ui8[1] );
	putByteOnTx(unerTx, myWord.ui8[2] );
	putByteOnTx(unerTx, myWord.ui8[3] );
	myWord.f32 = *p_pitch;
	putByteOnTx(unerTx, myWord.ui8[0] );
	putByteOnTx(unerTx, myWord.ui8[1] );
	putByteOnTx(unerTx, myWord.ui8[2] );
	putByteOnTx(unerTx, myWord.ui8[3] );

	putByteOnTx(unerTx, unerTx->chk);

	UNER_SendData();
}

uint8_t UNER_ShouldSendAllSensors(void) {
    return sendAllSensorsFlag;
}

void UNER_SendSerial(_sTx *tx)
{
    uint16_t len = (tx->indexW + tx->mask + 1 - tx->indexR) & tx->mask;
    if (!len) return;

    uint8_t tmp[TXBUFSIZE];
    for (uint16_t i = 0; i < len; i++) {
        tmp[i] = tx->buff[(tx->indexR + i) & tx->mask];
    }
    // ---> envío asíncrono y no bloqueante <---
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
        usb_enqueue_tx(tmp, len);
    }
    tx->indexR = tx->indexW;
}


void UNER_RegisterADCBuffer(uint16_t *buf, uint8_t len) {
    p_adcBuf   = buf;
    adcBufLen  = len;
}

void UNER_RegisterMotorSpeed(int16_t *rightPtr, int16_t *leftPtr) {
    p_motorRightVel = rightPtr;
    p_motorLeftVel  = leftPtr;
}

void UNER_RegisterProportionalControl(float *kpPtr, float *kdPtr, float *kiPtr) {
    p_KP  = kpPtr;
    p_KD  = kdPtr;
    p_KI  = kiPtr;
}

void UNER_RegisterAngle(float *rollPtr, float *pitchPtr)
{
    p_roll  = rollPtr;
    p_pitch = pitchPtr;
}

void UNER_RegisterSteering(float *steeringPtr) {
    p_steering = steeringPtr;
}

void UNER_RegisterFlags(uint8_t *flagPtr1, uint8_t *flagPtr2) {
    p_balance_flag = flagPtr1;
    p_resetMassCenter_flag = flagPtr2;
}


// Envía el ring-buffer por USB y por UDP
void UNER_SendData(void) {
    uint16_t len = (unerTx->indexW + unerTx->mask + 1 - unerTx->indexR) & unerTx->mask;
    if (!len) return;

    // USB CDC
    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
        usb_enqueue_tx(&unerTx->buff[unerTx->indexR], len);
    }

    // UDP (ESP01)
    if (ESP01_StateUDPTCP() == ESP01_UDPTCP_CONNECTED && !ESP01_IsSending()) {
        ESP01_Send(unerTx->buff, unerTx->indexR, len, TXBUFSIZE);
    }

    // Avanzar índice de lectura
    unerTx->indexR = (unerTx->indexR + len) & unerTx->mask;
}

/* END Private Functions*/
