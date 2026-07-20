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

// Todas las variables de aplicación que el protocolo puede consultar/modificar.
// Se registran juntas para evitar una larga serie de funciones Register*.
typedef struct {
    uint16_t *adc;
    uint8_t adc_len;
    int16_t *motor_right_velocity;
    int16_t *motor_left_velocity;
    int16_t *ax;
    int16_t *ay;
    int16_t *az;
    int16_t *gx;
    int16_t *gy;
    int16_t *gz;
    float *roll;
    float *pitch;
    float *kp;
    float *kd;
    float *ki;
    float *kv_brake;
    float *beta_g;
    float *beta_a;
    float *steering;
    uint8_t *robot_state;
    uint8_t *reset_mass_center;
    uint8_t *send_csv_log;
    uint8_t *send_wifi_log;
    uint8_t *change_display;
    float *kp_line;
    float *kd_line;
    float *ki_line;
    float *line_threshold;
    float *line_speed;
    float *manual_setpoint;
    float *manual_steering;
    uint32_t *manual_timeout_ms;
    float *rotation_target_deg;
    uint8_t *rotation_trigger;
    float *odom_x;
    float *odom_y;
    float *odom_theta;
    float *setpoint_trim;
} UNER_Bindings_t;

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
    MODIFY_SETPOINT = 0xC8,
    MOVE_FORWARD = 0xD0,
    MOVE_BACKWARD = 0xD1,
    MOVE_LEFT = 0xD2,
    MOVE_RIGHT = 0xD3,
    MOVE_STOP = 0xD4,
    ROTATE_90_RIGHT  = 0xD5,   // giro 90° derecha (encoders + gyro)
    ROTATE_90_LEFT   = 0xD6,   // giro 90° izquierda
    ROTATE_180_RIGHT = 0xD7,   // giro 180° derecha
    ROTATE_180_LEFT  = 0xD8,   // giro 180° izquierda
    ROTATE_CUSTOM    = 0xD9,   // payload: float 4 bytes (°, + = derecha, − = izquierda)
    GET_ODOMETRY     = 0xDA,   // respuesta: 3 floats little-endian = x[m], y[m], theta[°]
    RESET_ODOMETRY   = 0xDB,   // pone la pose en (0,0,0°); responde ACK
    CMD_WIFI_ODOM_DATA = 0xDC, // push periódico (no pedido) de WifiOdomData_t, ver struct
	ACK = 0x0D,
    UNKNOWN = 0xFF
}_eCmd;

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

// Push periódico y liviano (2 Hz por defecto) de pose + posición de línea, para
// graficar mapa XY y franja de línea en Qt sin depender de ACTIVATE_WIFI_LOG.
// Se envía solo mientras hay conexión WiFi activa (ver f_wifi_connected en main.c).
typedef struct __attribute__((packed)) {
    uint16_t seq;              // incremental, uno por envío — para medir pérdida de paquetes en Qt
    uint32_t t_ms;
    float    x_m;             // odometría: X [m]
    float    y_m;              // odometría: Y [m]
    float    theta_deg;        // odometría: rumbo [-180..180]°
    float    line_error;       // posición del centroide de línea (0=centrado), 0 si no detectada
    uint8_t  line_detected;    // 1 si en este ciclo se vio la línea
    uint8_t  robot_state;      // eRobotState
    uint8_t  line_state;       // eLineState (solo válido si robot_state==LINE_FOLLOWING)
    uint16_t adc5;             // sensores de objeto (adcAvg[4..7]): menos = más cerca,
    uint16_t adc6;             // ~4095 = nada adelante. Para graficar en Qt una barrera/
    uint16_t adc7;             // cuerpo frente al robot cuando está viendo algo.
    uint16_t adc8;
    float    roll_deg;         // ángulo de balanceo (filtered_roll_deg) — alimenta la Vista 3D de Qt sin depender de ACTIVATE_WIFI_LOG
    float    lat_deg;          // inclinación LATERAL (banking alrededor del eje de avance), por
                               // acelerómetro con EMA — tercer eje de la Vista 3D de Qt (2026-07-10)
} WifiOdomData_t;

void UNER_Init(_sRx *rx, _sTx *tx);

void UNER_RegisterBindings(const UNER_Bindings_t *bindings);

void UNER_PushByte(uint8_t byte);

void UNER_Task(void);

void UNER_SendAlive(void);

void UNER_SendAllSensors(void);

uint8_t UNER_ShouldSendAllSensors(void);


void UNER_SendWifiLogData(WifiLogData_t *data);

void UNER_SendWifiOdomData(WifiOdomData_t *data);

uint8_t UNER_GetLastManualCmd(void);

#endif /* ESP01_H_ */
