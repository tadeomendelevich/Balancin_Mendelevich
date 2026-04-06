/**
  ******************************************************************************
  * @file    ESP01.h
  * @author  Tadeo Mendelevich
  * @brief   Header file containing functions prototypes of ESP01 library.
  ******************************************************************************
  * @attention
  *
  *
  * Copyright (c) 2023 HGE.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  * Version: 01b05 - 04/08/2024
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef UNER_H_
#define UNER_H_

#include <stdint.h>
#include <stdbool.h>

/* buffers circulares para protocolo UNER */
#define RXBUFSIZE  256
#define TXBUFSIZE  256

extern uint16_t globalIndex;

// Estructura para la recepción de datos
typedef struct {
    volatile uint8_t *buff;
    uint8_t indexR;
    uint8_t indexW;
    uint8_t indexData;
    uint8_t nBytes;
    uint8_t header;
    uint8_t chk;
    uint8_t mask;
    bool isComannd;
    uint8_t timeOut;
} _sRx;

// Estructura para la transmisión de datos
typedef struct {
    uint8_t *buff;
    uint8_t indexR;
    uint8_t indexW;
    uint8_t mask;
    uint8_t chk;	// EL CHECKSUM SE AGREGA SOLO COMO SUMA LUEGO DEL CALCULO DEL PAYLOAD
} _sTx;

// Enums para el estado de parsing
enum { HEADER_U, HEADER_N, HEADER_E, HEADER_R, NBYTES, TOKEN, PAYLOAD };

typedef union{                          //union para definir la bandera, y no ocupar una variable booleana
    struct{
        uint8_t b0:1;
        uint8_t b1:1;
        uint8_t b2:1;
        uint8_t b3:1;
        uint8_t b4:1;
        uint8_t b5:1;
        uint8_t b6:1;
        uint8_t b7:1;
    }bit;
    uint8_t byte;
}_flag;

/**
 *
 * @brief Unión ara la descomposición/composición de números mayores a 1 byte
 *
 */
typedef union{
    uint32_t    ui32;
    int32_t     i32;
    uint16_t    ui16[2];
    int16_t     i16[2];
    uint8_t     ui8[4];
    int8_t      i8[4];
    float       f32;
}_uWord;

/**
 * @brief Enumeración de los comandos del protocolo
 *
 */
typedef enum{
    ALIVE = 0xF0,
    FIRMWARE = 0xF1,
    SETMOTORSPEED = 0xA1,
    GETSPEED = 0xA4,
	GETADCVALUES = 0xA5,
	GETMPU6050VALUES = 0xA6,
	GETANGLE = 0XA7,
    RADAR = 0xA8,
	SENDALLSENSORS = 0xA9,
	MODIFYKP = 0xB1,
	MODIFYKD = 0xB2,
	MODIFYKI = 0xB3,
	BALANCE = 0xB4,
	GETPIDVALUES= 0xB5,
    MODIFYSTEERING = 0xB6,
	RESETMASSCENTER = 0xB7,
	CMD_LOG_DATA    = 0xB8,
	ACTIVATE_CSV_LOG = 0xB9,
    ACTIVATE_WIFI_LOG = 0xBA,
    CMD_WIFI_LOG_DATA = 0xBB,
	MODIFY_BETA_G = 0xBC,
	MODIFY_BETA_A = 0xBD,
	CHANGE_DISPLAY = 0xBE,
	MODIFY_KV_BRAKE = 0xBF,
    MODIFY_KP_LINE = 0xC0,
    MODIFY_KD_LINE = 0xC1,
    MODIFY_KI_LINE = 0xC2,
    MODIFY_LINE_THRES = 0xC3,
    MODIFY_LINE_SPEED = 0xC4,
    ACTIVATE_LINE_FOLLOWING = 0xC5,
    ACTIVATE_POS_MAINTENANCE = 0xC6,
    ACTIVATE_MANUAL_CONTROL = 0xC7,
    MOVE_FORWARD = 0xD0,
    MOVE_BACKWARD = 0xD1,
    MOVE_LEFT = 0xD2,
    MOVE_RIGHT = 0xD3,
    MOVE_STOP = 0xD4,
	ACK = 0x0D,
    UNKNOWN = 0xFF
}_eCmd;

typedef struct __attribute__((packed)) {
    uint32_t t_ms;
    uint32_t dt_us;
    float accel_roll;
    float gyro_y;
    float roll_filt;
    float error;
    float p_term;
    float i_term;
    float d_term;
    float output;
    float pwm_cmd;
    float pwm_sat;
    uint8_t sat_flag;
    int16_t mR;
    int16_t mL;
} LogData_t;

typedef struct __attribute__((packed)) {
    uint32_t t_ms;
    float roll_filt;
    float output;
    float p_term;
    float i_term;
    float d_term;
    int16_t mR;
    int16_t mL;
    uint32_t dt_ctrl_us;
    float dyn_sp;

    // Line Follower Telemetry
    float line_error;
    float p_line;
    float i_line;
    float d_line;
    float steering_adjustment;
    uint16_t adc1;
    uint16_t adc2;
    uint16_t adc3;
    uint16_t adc4;
} WifiLogData_t;

void UNER_Init(_sRx *rx, _sTx *tx, int16_t *ax_ptr, int16_t *ay_ptr, int16_t *az_ptr, int16_t *gx_ptr, int16_t *gy_ptr, int16_t *gz_ptr);

void UNER_PushByte(uint8_t byte);

void UNER_Task(void);

void UNER_Send(uint8_t cmd, const uint8_t *payload, uint8_t length);

uint8_t putHeaderOnTx(_sTx  *dataTx, _eCmd ID, uint8_t frameLength);

uint8_t putByteOnTx(_sTx *dataTx, uint8_t byte);

uint8_t putStrOntx(_sTx *dataTx, const char *str);

uint8_t getByteFromRx(_sRx *dataRx, uint8_t iniPos, uint8_t finalPos);

void decodeCommand(_sRx *dataRx, _sTx *dataTx);

void UNER_SendAlive(void);

void UNER_SendAllSensors(void);

uint8_t UNER_ShouldSendAllSensors(void);

void UNER_SendSerial(_sTx *tx);

/**
 * Registra el buffer de valores ADC para que UNER pueda leerlos.
 * @param buf  Puntero al array de uint16_t con las lecturas ADC.
 * @param len  Número de elementos (p. ej. 8).
 */
void UNER_RegisterADCBuffer(uint16_t *buf, uint8_t len);

/**< Registra dónde escribir la velocidad de los motores */
void UNER_RegisterMotorSpeed(int16_t *rightPtr, int16_t *leftPtr);

void UNER_RegisterAngle(float *rollPtr, float *pitchPtr);

void UNER_RegisterProportionalControl(float *kpPtr, float *kdPtr, float *kiPtr, float *BETA_G_Ptr, float *BETA_A_Ptr, float *KV_BRAKE_Ptr);

void UNER_RegisterSteering(float *steeringPtr);

void UNER_RegisterFlags(uint8_t *flagPtr1, uint8_t *flagPtr2, uint8_t *flagPtr3, uint8_t *flagPtr4, uint8_t *flagPtr5);

void UNER_RegisterLineControl(float *kpLinePtr, float *kdLinePtr, float *kiLinePtr, float *thresPtr, float *speedPtr, uint8_t *lineFollowFlagPtr);

void UNER_RegisterManualControl(float *spCmdPtr, float *stCmdPtr, uint32_t *tmoPtr);

void UNER_RegisterRobotState(uint8_t *robotStatePtr);

void UNER_SendLogData(LogData_t *data);

void UNER_SendWifiLogData(WifiLogData_t *data);

void UNER_SendData(void);

uint8_t UNER_GetLastManualCmd(void);

#endif /* ESP01_H_ */
