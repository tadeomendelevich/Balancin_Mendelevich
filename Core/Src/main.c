/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include "usbd_cdc_if.h"
#include "ssd1306.h"
#include "fonts.h"
#include "MPU6050.h"
#include "ESP01.h"
#include "UNER.h"
#include "i2c_manager.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
    ROBOT_STATE_IDLE = 0,
    ROBOT_STATE_BALANCE_ONLY,
    ROBOT_STATE_BALANCE_AND_SPEED,
    ROBOT_STATE_LINE_FOLLOWING,
    ROBOT_STATE_MANUAL_CONTROL,
    ROBOT_STATE_MOTOR_TEST
} eRobotState;

// --- Line Search & Loss Control ---
typedef enum {
    LINE_STATE_FOLLOWING = 0,  // Siguiendo línea normalmente
    LINE_STATE_LOST,           // Línea perdida, frenando y buscando
    LINE_STATE_SEARCHING,      // Girando suavemente para buscar
    LINE_STATE_LOST_BRAKE,     // Línea perdida: frena hasta velocidad baja antes de girar
    LINE_STATE_LOST_ROTATE,    // Línea perdida: giro 180° para buscarla
    LINE_STATE_LOST_SETTLE,    // Post-180°: pausa de estabilización antes de avanzar
    LINE_STATE_LOST_FWD,       // Post-180°: avanza hacia adelante hasta encontrar la línea
    LINE_STATE_EDGE_WAIT,      // Línea perdida por un extremo (curva): espera a frenar antes de girar 90°
    LINE_STATE_EDGE_ROTATE,    // Línea perdida por un extremo: gira 90° hacia ese lado
    LINE_STATE_EDGE_SETTLE,    // Post-90°: pausa de estabilización antes de avanzar
    LINE_STATE_EDGE_FWD,       // Post-90°: avanza con velocidad controlada hasta encontrar la línea
    LINE_STATE_GIVEN_UP,       // Ni el giro de 180 ni el de 90 encontraron la línea: reposo total hasta reponerla a mano
    LINE_STATE_PERP_ROTATE,    // Los 4 ADC en negro sin manipulación: cruce perpendicular, gira 90° instantáneo y retoma FOLLOWING
    LINE_STATE_OBJ_PRE_REVERSE_HOLD, // Objeto detectado: balance estático 2s → luego OBJ_ROTATE
    LINE_STATE_OBJ_REVERSE,    // (deshabilitado) reversa breve antes del giro
    LINE_STATE_OBJ_BRAKE,      // (deshabilitado) frena hasta velocidad ≈ 0 antes de girar
    LINE_STATE_OBJ_ROTATE,     // Objeto detectado: girando 180° derecha por encoders
    LINE_STATE_OBJ_HOLD,          // Post-rotación: balance estático espera 2s
    LINE_STATE_OBJ_ARC,           // (no usado) reservado
    LINE_STATE_OBJ_WALL_APPROACH, // Avanza despacio (2°) hasta encontrar la pared en ADC7
    LINE_STATE_OBJ_WALL_FWD,      // Wall-following: avanza mientras ADC7 en rango 500-3750
    LINE_STATE_OBJ_WALL_TURN,     // Wall-following: pivot izquierda hasta re-ver objeto en ADC7
} eLineState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_Pin 			GPIO_PIN_10
#define LED_GPIO_Port 		GPIOB

#define INTEGRATED_LED_Pin 			GPIO_PIN_13
#define INTEGRATED_LED_GPIO_Port    GPIOC

#define KEY_Pin        		GPIO_PIN_0
#define KEY_GPIO_Port  		GPIOA

#define CH_PD_GPIO_Port  	GPIOB
#define CH_PD_Pin        	GPIO_PIN_2

#define UDP_RX_SIZE  	 	512
#define UDP_RX_MASK   		(UDP_RX_SIZE - 1)
#define USB_TX_BUF_SIZE 	512
#define USB_TX_BUF_MASK 	(USB_TX_BUF_SIZE-1)
#define UDP_BYTES_PER_CYCLE 8


#define MPU_AVERAGE_SIZE 	10
#define ADC_AVERAGE_SIZE 	4

#define BAR_COUNT    		8
#define BAR_SPACING  		2
#define SCREEN_W     		SSD1306_WIDTH
#define SCREEN_H    		SSD1306_HEIGHT

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define ESP_USB_BUF_SIZE	512

// PID
#define KP     		4.0f   // Ganancia proporcional en PWM directo
#define KD     		0.12f   // Ganancia derivativa en PWM/(deg/s)
#define KI    		0.1f   // Ganancia integral en PWM/(deg*s)

#define SETPOINT_ANGLE 	0.0f

#define SOFT_ZONE_ANGLE_DEG   1.50f   // error a partir del cual el PID va al 100%
#define SOFT_ZONE_MIN_SCALE   0.15f   // escala mínima cuando el error es ~0
// Zona de hold con histéresis: silencia PID en el punto dulce de equilibrio
#define BALANCE_HOLD_ENTER_ANGLE_DEG  0.50f  // entra en hold si |error| <= este valor
#define BALANCE_HOLD_EXIT_ANGLE_DEG   0.90f  // sale de hold si |error| >= este valor
#define BALANCE_HOLD_ENTER_GYRO_DPS   4.0f   // entra en hold si |gyro| <= este valor
#define BALANCE_HOLD_EXIT_GYRO_DPS    10.0f  // sale de hold si |gyro| >= este valor

// Complementary Filter / PID timing
#define ALPHA 0.98f
#define DT_CTRL_FIXED 0.010f

// LOGGING MACROS
#define LOG_ENABLE 1
#define LOG_DECIM  5		// Frecuencia de envio de log csv mediante USB
#define LOG_WIFI_DECIM 10	// Frecuencia de envio de log binario mediante WIFI

// Filter Control Parameters
#define I_MAX  100.0f       // Max Integral Term
#define DT_MIN 0.005f       // Min valid DT (5ms)
#define DT_MAX 0.05f        // Max valid DT (50ms)

// Fall detection (hysteresis)
#define FALL_ANGLE           	60.0f
#define RECOVER_ANGLE        	2.0f
#define UPSIDE_DOWN_ANGLE    	120.0f  // más agresivo para detectar boca abajo antes
#define DEAD_ZONE_ANGLE      	15.0f   // entre 35° y 120° → zona muerta, motores off

#define MOTOR_RIGHT_DEADBAND  	1   // offset sumado al motor derecho para compensar su mayor zona muerta (0 = sin compensación)
#define MOTOR_LEFT_DEADBAND   	1   // ídem motor izquierdo — ajustar si el izq. no arranca a PWM bajo
// 2026-07-01: freno más agresivo (0/5/1.5) por reportes de aceleración
// descontrolada. Bajado un poco después: con KV_BRAKE=2.0 base + deadband
// chico (0.05), el ruido de cuantización de la nueva velocidad por encoders
// alimentaba una corrección constante y el balance normal (parado, sin
// perturbación) oscilaba visiblemente. Valores intermedios entre el
// original y el primer ajuste agresivo.
#define KV_BRAKE                0.8f  // ganancia base (era 0.0 → 2.0 → 0.8)
#define KV_BRAKE_STRONG         6.0f  // ganancia extra por encima del umbral (era 5.0 → 8.0 → 6.0)
#define BRAKE_VEL_THRESHOLD     1.0f  // velocidad a partir de la cual se aplica el freno fuerte (era 1.5 → 0.6 → 1.0)
#define VEL_DECAY        		0.999f
#define VEL_DECAY_ACCEL  		0.97f   // decay del integrador del acelerómetro

#define VEL_CF_ALPHA     		0.0f    // peso del giroscopio en el filtro complementario (1=solo gyro, 0=solo accel)
#define VEL_ACCEL_SCALE  		30.0f   // escala para igualar m/s del accel con unidades del gyro
#define VEL_LPF_BETA     		0.35f
// BRAKE_VEL_DEADBAND calculado a partir del piso de cuantización real del
// encoder: con ENC_CPR=28 y ciclo de control de 10ms, UN solo count en una
// sola rueda ya calcula (0.5/(28*0.01))*ENC_VEL_SCALE(0.1775) ≈ 0.32 m/s;
// si tiquean ambas ruedas en el mismo ciclo, ≈ 0.63 m/s. 0.05-0.12 no
// filtraba nada de eso — cualquier pulso aislado se leía como "movimiento
// real" y disparaba freno, causando el bamboleo parado. 0.35 queda apenas
// por encima del piso mínimo (0.32) para filtrar el ruido de un solo pulso
// sin tapar una deriva sostenida real.
#define BRAKE_VEL_DEADBAND      0.35f
#define BRAKE_VEL_MAX           4.0f
#define BRAKE_TILT_MAX          4.0f  // era 3.0 → 5.0 → 4.0: compromiso entre frenar de verdad y no sobrecorregir en balance normal
#define BRAKE_TILT_MAX_MANUAL   4.0f  // ídem
#define BRAKE_TILT_STEP_BAL     0.5f
#define BRAKE_TILT_STEP_MAN     0.5f
// Multiplicador extra sobre ComputeBrakeSetpointTarget, solo para LOST_BRAKE/
// EDGE_WAIT (el frenado justo al perder la línea del todo). No se sube el
// freno estándar de balance en general porque reintroduciría la oscilación
// en balance común ya corregida. 2026-07-01, a pedido del usuario para no
// alejarse tanto de la línea antes de arrancar a buscarla.
#define LOST_BRAKE_BOOST        1.6f
#define INTEGRAL_DECAY   		0.990f

// Steering PID (lazo cerrado por encoders)
#define STEER_KP        5.0f    // ajustar según respuesta real
#define STEER_KI        0.2f
#define STEER_KD        0.1f
#define STEER_I_MAX    15.0f
#define STEER_OUT_MAX  20.0f
// Velocity PI (lazo externo de velocidad en seguimiento de línea)
#define LINE_VEL_KP             3.0f   // ganancia proporcional vel PI (acelerando)
#define LINE_VEL_KP_BRAKE   	3.0f   // ganancia proporcional cuando va sobrevelocidad (frenando) — 2026-07-01
#define LINE_VEL_KI             1.2f   // ganancia integral vel PI
#define LINE_VEL_I_MAX          2.5f   // anti-windup vel PI
// Avance post-giro (LOST_FWD / EDGE_FWD) — control P continuo en vez del
// bang-bang viejo (ángulo fijo + freno de -1° solo por encima de 0.90 m/s),
// que no frenaba nunca hasta cruzar el umbral y podía acelerar sin control
// si la lectura de velocidad no cruzaba el umbral limpiamente. 2026-07-01.
// 0.30 quedaba DENTRO del piso de cuantización del encoder (~0.32-0.63 m/s,
// ver BRAKE_VEL_DEADBAND arriba): el control nunca encontraba un punto
// estable y terminaba peleando entre acelerar a fondo y frenar a fondo,
// resultando en que el robot no avanzaba en absoluto. Subido a 0.45, claramente
// por encima del piso de un solo pulso (~0.32) para que el P tenga margen real.
#define LOST_FWD_SPEED_TARGET   0.45f  // m/s, avance cauteloso "a ciegas"
#define LOST_FWD_KP             6.0f   // ganancia P acelerando
#define LOST_FWD_KP_BRAKE       10.0f  // ganancia P frenando (sobrevelocidad)
#define LOST_FWD_ANGLE_MAX      2.0f   // ángulo máximo de avance (°)
#define LOST_FWD_BRAKE_MAX      2.0f   // ángulo máximo de freno activo (°, se aplica en negativo)
#define LINE_ENC_CORR_KP        8.5f   // ganancia P: grados por (m/s de deficit) normalizado
#define LINE_ENC_CORR_MAX       4.0f   // angulo extra maximo por deficit de velocidad (grados)
#define LINE_REV_BOOST_MAX      1.4f   // extra de inclinacion si los encoders muestran reversa
#define LINE_REV_BOOST_UP       0.06f  // subida max por ciclo de control
#define LINE_REV_BOOST_DOWN     0.10f  // bajada max por ciclo de control
#define LINE_REV_VEL_START      0.05f  // m/s de reversa desde donde empieza a actuar
#define LINE_REV_VEL_FULL       0.35f  // m/s de reversa para aplicar boost maximo
#define LINE_OBJ_REV_SPEED_MAX  0.30f  // m/s objetivo maximo durante reversa por obstaculo
#define LINE_OBJ_REV_SPEED_MIN  0.10f  // m/s minimo para que la reversa no se quede sin empuje
#define LINE_OBJ_REV_TILT_MAX   2.0f   // inclinacion maxima hacia atras durante reversa por obstaculo
#define LINE_OBJ_REV_STEER_GAIN 0.70f  // escala del PID de linea cuando corrige marcha atras
#define LINE_OBJ_REV_STEER_MAX  16.0f  // limite de steering durante reversa siguiendo linea
// Inner steering PI (lazo cerrado con encoder diferencial)
#define LINE_STEER_ENC_SCALE    0.12f  // escala steering_cmd [PWM] → diff_sp [rps]
#define LINE_STEER_MAX_RPS      2.0f   // límite del setpoint diferencial
#define LINE_STEER_FB_KP        5.0f   // inner steering PI — P
#define LINE_STEER_FB_KI        0.2f   // inner steering PI — I
#define LINE_STEER_FB_I_MAX     8.0f   // anti-windup inner steering

#define LINE_LOST_TIMEOUT_MS   2000    // ms sin línea antes de entrar en búsqueda
#define LINE_LOST_STEERING     12.0f   // steering suave para cuando recién se pierde la línea

#define OBJ_DETECT_THRESHOLD_VAL   2000.0f   // objeto detectado si ADC < este valor (sin objeto: ~4095, con objeto: <2000)
#define OBJ_DETECT_DEBOUNCE_CNT    10         // ciclos consecutivos (100 ms) para confirmar objeto
#define OBJ_PRE_REVERSE_HOLD_MS    1000U      // tiempo de balance estatico antes de rotar
// Giro 90° por encoders: medir distancia entre centros de rueda y ajustar aquí
#define OBJ_ROTATE_TRACK_WIDTH     0.220f    // metros entre centros de ruedas (medido: 220 mm)
#define OBJ_HOLD_DURATION_MS       2000U     // ms de balance estatico en OBJ_HOLD antes de wall-following
#define OBJ_WALL_ADC_IDX           6          // índice del sensor lateral (ADC7 = adcAvg[6])
#define OBJ_WALL_THRESHOLD         3750.0f   // ADC < umbral → objeto visible; > umbral → perdido
#define OBJ_WALL_TOO_CLOSE_THOLD   500.0f    // ADC7 < este valor → demasiado cerca, pivot derecha
#define OBJ_WALL_APPROACH_ANGLE    2.0f      // ángulo de avance lento buscando la pared (°)
#define OBJ_WALL_APPROACH_TIMEOUT  6000U     // ms máximos buscando la pared antes de rendirse
#define OBJ_WALL_FWD_ANGLE         4.5f      // ángulo máximo de avance en WALL_FWD (°)
#define OBJ_WALL_BRAKE_ANGLE       1.0f      // freno activo máximo por sobrevelocidad en WALL_FWD (°)
#define OBJ_WALL_OVERSPEED_VEL     0.90f     // velocidad medida para frenar; evita regular fino con encoders cuantizados
#define OBJ_WALL_PIVOT_POWER       8.0f      // potencia del pivot en WALL_TURN/WALL_FWD
#define OBJ_WALL_LINE_IGNORE_MS    3000U     // ms al inicio de WALL_FWD en que se ignora la línea

// Parámetros del arco evasivo (OBJ_ARC) — ajustables en runtime desde Qt
// OBJ_ARC_DIFF_TARGET: diferencial de velocidad deseado entre ruedas [rps] (derecha - izquierda)
// Positivo = gira a la izquierda, negativo = gira a la derecha (convención empírica). Típico: ±0.3–1.5
float OBJ_ARC_DIFF_TARGET =  0.5f;   // rps diferencial objetivo; positivo = izquierda (convención empírica); tunable
float OBJ_ARC_ANGLE       =  0.5f;   // ángulo de avance durante el arco (°); tunable
#define OBJ_ARC_KP          15.0f    // ganancia proporcional del PI de diferencial
#define OBJ_ARC_KI           3.0f    // ganancia integral del PI de diferencial
#define OBJ_ARC_STEER_MAX   30.0f    // límite de steering_adjustment durante el arco

#define DISPLAY_UPDATE_INTERVAL_IDLE_MS    100U
#define DISPLAY_UPDATE_INTERVAL_ACTIVE_MS   60U

// Bias hardcodeado del MPU — descomenta MPU_USE_FIXED_BIAS y reemplaza los valores
// con los que imprimió "BIAS ax=... ay=... az=... gx=... gy=... gz=..." por USB.
// Mientras esté comentado, el robot calibra en cada arranque (requiere estar en balance).
#define MPU_USE_FIXED_BIAS
#ifdef MPU_USE_FIXED_BIAS
#define FIXED_BIAS_AX    -46
#define FIXED_BIAS_AY    4950	// + = adelante, - = atras
#define FIXED_BIAS_AZ    1980
#define FIXED_BIAS_GX    -441
#define FIXED_BIAS_GY    -107
#define FIXED_BIAS_GZ    -54
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t is250us, tmo100ms, is10ms, is2ms;

uint8_t BufUSBTx[256], nBytesTx;
extern USBD_HandleTypeDef hUsbDeviceFS;
char usbBuffer[128];
uint8_t  espUSBBuf[ESP_USB_BUF_SIZE];
volatile uint16_t espUSBBufIw, espUSBBufIr;
static uint8_t usb_tx_buf[USB_TX_BUF_SIZE];
static volatile uint16_t tx_head = 0;
static volatile uint16_t tx_tail = 0;
static volatile uint8_t usb_tx_busy = 0;
static const char HEX_DIGITS[] = "0123456789ABCDEF";	// Tabla de dígitos hex para USB

//static uint8_t ema_initialized = 0;
static uint8_t aliveCounter, mpu6050Counter;
uint8_t mpuDataReady = 0;
uint8_t mpu_initialized = 0;
static volatile uint8_t mpu_req_pending = 0;
static float roll_deg = 0.0f;	// Ángulo de balanceo (eje Y, usado para el equilibrio)
static float pitch_deg = 0.0f;	// Ángulo de inclinación (eje X)

volatile uint16_t adcValues[8];
static uint32_t adcSum[BAR_COUNT] = {0};	// Sumas acumuladas de las últimas ADC_AVERAGE_SIZE muestras
static uint16_t adcBuf[BAR_COUNT][ADC_AVERAGE_SIZE] = {{0}};		// Buffers circulares: [canal][posición en la ventana]
static uint8_t maIndex = 0;		// Índice circular común para todos los canales
static uint16_t adcAvg[BAR_COUNT] = {0};
static volatile uint32_t adc_dma_last_ms = 0;
static volatile uint8_t  adc_recover_pending = 0;

uint8_t i2c1_tx_busy = 0;

uint8_t *sendAllSensors;

uint8_t dataTx, dataRx;
static uint8_t unerRxBuffer[RXBUFSIZE];
static uint8_t unerTxBuffer[TXBUFSIZE];
static _sRx    unerRx;
_sTx    unerTx;
static uint8_t esp01RxBuf[ESP01RXBUFAT];
static uint16_t esp01IwRx = 0;
static uint16_t esp01IrRx = 0;		/* Índice de lectura para el buffer UDP entrante */
uint8_t  espUSBBuf[ESP_USB_BUF_SIZE];
volatile uint16_t espUSBBufIw, espUSBBufIr;

const char *wifiSSID     = "FCAL";
const char *wifiPassword = "fcalconcordia.06-2019";
const char *wifiIp = "172.23.205.98";

//const char *wifiSSID     = "MEGACABLE FIBRA-2.4G-ckd0";
//const char *wifiPassword = "djg19dlk";
//const char *wifiIp 		 = "192.168.100.5";

//const char *wifiSSID     = "Delco_Mendelevich";
//const char *wifiPassword = "toyotakia";
//const char *wifiIp = "192.168.1.55";

//const char *wifiSSID     = "Wifi Habitaciones";
//const char *wifiPassword = "toyotakia";
//const char *wifiIp = "192.168.1.48";

int16_t motorRightVelocity = 0;
int16_t motorLeftVelocity  = 0;

static float integral = 0.0f;
static float steering_adjustment = 0.0f;
static float filtered_roll_deg = 0.0f;

uint8_t robot_state = ROBOT_STATE_IDLE; // Controls the main state machine of the robot

static uint32_t manual_cmd_last_ms = 0;
static float manual_setpoint_cmd = 0.0f;
static float manual_steering_cmd = 0.0f;

static float setpoint_trim = 0.0f;

static float line_error_prev = 0.0f;
static float line_integral = 0.0f;

uint8_t f_resetMassCenter = 0; // Resetea el centro de gravedad en el cual el auto hace balance

// LOGGING VARIABLES
static uint32_t log_counter = 0;
static uint8_t  log_header_sent = 0;
uint8_t f_send_csv_log = 0;
uint8_t f_send_wifi_log = 0;
uint8_t f_change_display = 0;

static uint8_t f_wifi_connected = 0;
static uint8_t f_fallen = 0;   // 1 = caído, motores apagados

static float dt_real        = 0.0f;		// Monitoreo del perÃ­odo real entre muestras

static float velocity_est     = 0.0f;  // velocidad estimada fusionada (gyro + accel)
static float velocity_est_f   = 0.0f;  // versión filtrada para el setpoint
static float vel_from_accel   = 0.0f;  // velocidad integrada del acelerómetro (m/s)
static float dynamic_setpoint = 0.0f;  // setpoint variable calculado cada ciclo
static float dynamic_setpoint_f = 0.0f;
static float base_setpoint_f  = 0.0f;
static float brake_setpoint_f = 0.0f;

float KP_value;
float KD_value;
float KI_value;
float KV_brake_value;

// Line Follower Variables
float KP_LINE = 10.0f;
float KD_LINE = 2.0f;
float KI_LINE = 0.5f;
float LINE_THRESHOLD = 3000.0f;  // entre piso blanco (~1800) y cinta negra (~3800)
float LINE_ANGLE = 2.0f;        // inclinación máxima (°) para avanzar en seguimiento de línea

// Trim del centroide de línea: corrige desplazamiento físico del array de sensores.
// Positivo = corrige deriva a la DERECHA (array montado a la izquierda del eje del robot).
// Negativo = corrige deriva a la IZQUIERDA.
// Ajustar empíricamente: empezar con 0.05, aumentar de a 0.05 hasta que el robot siga centrado.
static float line_error_trim_f = 0.1f;

// Baseline de cada sensor ADC (valor mínimo sobre superficie blanca, sin línea).
// Medirlos colocando el robot sobre el piso blanco y leyendo adcAvg[] en la pantalla 4.
// La sustracción de baseline elimina el offset entre sensores y mejora el centroide.
// COMPLETAR con los valores medidos:
static const float ADC_BASELINE[4] = { 2000.0f, 1550.0f, 1370.0f, 1730.0f };  // s0, s1, s2, s3 — calibrado sobre fondo blanco
float LINE_SPEED_TARGET = 2.5f;  // m/s objetivo en seguimiento de línea (ajustable en runtime)
float OBJ_DETECT_THRESHOLD_f = OBJ_DETECT_THRESHOLD_VAL;  // umbral objeto, ajustable en runtime

static eLineState line_state       = LINE_STATE_FOLLOWING;
static uint32_t   line_lost_ms     = 0;   // tick cuando se vio la línea por última vez
static uint32_t   line_lost_entered_ms = 0; // tick cuando se entró al estado LOST (no usado actualmente)
static uint8_t    line_seen_since_entry = 0; // evita búsqueda automática antes de adquirir línea
static float      line_search_dir  = 1.0f; // dirección de búsqueda (+1 o -1)
static float      last_line_dir    = 1.0f; // última dirección válida de la línea (+1 o -1)
static uint8_t    line_was_centered_on_lost = 1; // 1 = venía centrado (ADC del medio) al perder la línea, 0 = venía por los extremos
// 2026-07-01: reemplaza el enfoque de "pico reciente de line_error" (con
// cualquier decay terminaba clasificando todo como "por el extremo" por
// el bamboleo normal de curvas, o al revés). En cambio: qué sensor(es)
// tenían señal en el último ciclo con línea detectada. Si el ÚLTIMO
// sensor con señal fue un extremo (w[0] o w[3]) SIN soporte de los
// sensores del medio (w[1]/w[2] en cero), es una pérdida por el extremo.
static uint8_t    last_detected_edge_only = 0;
static float      line_error_f_d   = 0.0f;

static uint8_t upside_down_count = 0;
static uint8_t upright_count = 0;
static uint8_t fall_count = 0;

static uint8_t key_prev = 1;
static uint32_t key_last_ms = 0;
static uint32_t key_click_time = 0;
static uint8_t  key_click_count = 0;

static float manual_setpoint_ramped = 0.0f;  // setpoint de rampa aplicada

// Giro preciso en modo MANUAL (encoders + giroscopio)
#define MANUAL_ROT_PIVOT_POWER  15.0f   // PWM de pivot durante el giro
#define MANUAL_ROT_SLOWDOWN_DEG 55.0f   // grados antes del target para reducir potencia (debe ser > 90*(1-0.55)=40.5 para que realmente se active)
#define MANUAL_ROT_BRAKE_POWER   5.0f   // contra-pivot suave para frenar inercia sin revertir
// Counts promedio (|dr|+|dl|)/2 esperados para un giro de 90°.
// Ajustar con el valor que imprime "ROT_ENC" en USB al terminar cada giro.
#define MANUAL_ROT_ENC_TARGET  380.0f
static float    manual_rot_target_deg  = 0.0f;
static uint8_t  manual_rot_trigger     = 0;    // set por UNER para arrancar
static uint8_t  manual_rot_active      = 0;    // 1 = giro en curso
static uint8_t  manual_rot_phase       = 0;    // 0=avance, 1=contra-frenado activo
static float    manual_rot_heading     = 0.0f; // ángulo acumulado por giroscopio (°)
static float    manual_rot_enc_counts  = 0.0f; // counts promedio acumulados en el giro actual
static uint32_t manual_rot_start_ms   = 0;
static int32_t  manual_rot_enc_r0     = 0;
static int32_t  manual_rot_enc_l0     = 0;
static uint32_t manual_seq_next_ms     = 0;   // deadline: siguiente giro no antes de este tick
static uint8_t  manual_auto_rot_step   = 0;   // 0=90°der, 1=90°izq
static float line_angle_ramped      = 0.0f;  // rampa de avance en line follower
static float line_enc_angle_corr     = 0.0f;  // corrección P de angulo por deficit de velocidad encoder
static float line_reverse_boost     = 0.0f;  // extra de angulo si encoders muestran reversa
static float line_vel_integral      = 0.0f;  // integral del PI de velocidad (idea 1)
static float line_obj_rev_vel_integral = 0.0f;  // integral del PI de velocidad en reversa por obstaculo
static float line_steer_fb_int      = 0.0f;  // integral del inner steering PI (idea 2)
static float speed_right_rps_s      = 0.0f;  // velocidad rueda derecha, accesible fuera del bloque encoder
static float speed_left_rps_s       = 0.0f;  // velocidad rueda izquierda, accesible fuera del bloque encoder
static float line_error_disp        = 0.0f;  // último error de línea, para mostrar en pantalla
static float pwm_sat_prev = 0.0f;
static float prev_error = 0.0f;
static uint8_t balance_hold_active = 0;
static uint32_t obj_detect_ignore_until_ms = 0;  // ignora sensores de objeto hasta este tick
static uint8_t  prev_all_line_black        = 1;  // todos los sensores de línea en negro (robot en el aire)
static uint32_t all_black_start_ms         = 0;  // tick en que empezaron a verse todos negros
static uint8_t  f_in_air                   = 0;  // 1 = robot en el aire >2s → motores detenidos
static float    accel_motion_f             = 0.0f;  // variación reciente del acelerómetro (EMA), para distinguir "en la mano" de "quieto sobre blanco"
static uint32_t lrot_brake_start_ms        = 0;  // inicio del frenado previo al giro 180°
static uint32_t lrot_settle_start_ms       = 0;  // inicio de la pausa de estabilización post-180°
static uint32_t edge_wait_start_ms         = 0;  // inicio del frenado previo al giro de 90° (perdida por un extremo)
static uint32_t obj_pre_rev_start_ms = 0;
static uint8_t  obj_rot_initialized  = 0;
static int32_t  obj_rot_r0           = 0;
static int32_t  obj_rot_l0           = 0;
static uint32_t obj_rot_start_ms     = 0;
static uint8_t  obj_rot_phase        = 0;    // 0=spin, 1=freno activo
static float    obj_rot_heading      = 0.0f; // ángulo gz acumulado durante giro
static uint32_t obj_rot_phase1_ms    = 0;    // inicio de fase 1
static uint8_t  obj_rev_initialized  = 0;         // flag sesión actual de OBJ_REVERSE
static int32_t  obj_rev_r0           = 0;
static int32_t  obj_rev_l0           = 0;
static uint32_t obj_brake_start_ms   = 0;         // inicio de OBJ_BRAKE (file-scope para reset en caída)
static uint32_t obj_pre_rotate_ms    = 0;         // inicio del wait post-freno antes de OBJ_ROTATE
static uint32_t obj_hold_start_ms    = 0;         // inicio de OBJ_HOLD (timer 2s antes de OBJ_ARC)
static float    obj_arc_steer_int    = 0.0f;      // integral del PI de diferencial en OBJ_ARC
static float    obj_rev_straight_int = 0.0f;      // integral del PI de enderezamiento en OBJ_REVERSE
static uint32_t obj_wall_approach_start_ms = 0;    // timestamp de entrada a WALL_APPROACH
static uint32_t obj_wall_fwd_start_ms = 0;         // timestamp de entrada a WALL_FWD (para ignorar línea 3s)

volatile uint8_t tick2ms_count = 0;
static uint8_t encoder_sampler_initialized = 0;
static uint8_t encoder_right_prev_state = 0;
static uint8_t encoder_left_prev_state  = 0;

static uint8_t uart_tx_byte = 0;

static int16_t ax = 0, ay = 0, az = 0;
static int16_t gx = 0, gy = 0, gz = 0;

static float gyro_f = 0.0f;

static float accel_roll_f = 0.0f;

static uint32_t last_display_ms = 0;

static volatile uint32_t mpu_irq_timestamp_us = 0;
volatile uint8_t mpu_data_ready_for_ctrl = 0;

volatile int32_t encoder_right = 0;
volatile int32_t encoder_left  = 0;

uint8_t steer_pid_enabled    = 0;     // 0 = open-loop (gyro Z), 1 = lazo cerrado encoders
static float steer_pid_integral   = 0.0f;
static float steer_pid_prev_error = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void USBRxData(uint8_t *buf, int len);
void USB_BufferPush(uint8_t b);
void USB_DebugSend(const uint8_t *data, uint16_t len);
void USB_DebugStr(const char *s);	// Envía una cadena C terminada en '\0' por USB.
void USB_DebugHex(uint8_t b);	// Envía un byte representado en dos dígitos hexadecimales ASCII.
void USB_DebugUInt(unsigned int v);		// Envía un entero sin signo en formato decimal ASCII.
void USB_Debug(const char *fmt, ...);	// Mini-printf para debug: soporta %s, %c, %d/%u, %X y %%.
uint8_t usb_enqueue_tx(const uint8_t *data, uint16_t len);
void usb_service_tx(void);
static int  uart_send_byte(uint8_t byte);

void my_ssd1306_init(void *ctx);
int my_ssd1306_write_cmd(void *ctx, uint8_t cmd);
int my_ssd1306_write_data(void *ctx, const uint8_t *data, uint16_t len);
int my_ssd1306_write_data_async(void *ctx, const uint8_t *data, uint16_t len);
uint8_t my_ssd1306_is_busy(void *ctx);
void my_ssd1306_errorCb(void *ctx, int err);
void my_ssd1306_delay_ms(void *ctx, uint32_t ms);

int  mpu_writeReg(void   *ctx, uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t length);
int  mpu_readReg(void   *ctx, uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t length);
int  mpu_readRegDMA(void   *ctx, uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t length);
void mpu_delayMs(void    *ctx, uint32_t ms);
void mpu_errorCb(void *ctx, int err);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void MPU6050_RegisterPlatform(MPU6050_Platform_t *plat);
void MPU6050_ProcessDMA(void);

void UpdateADC_MovingAverage(void);

void calculate_tilt(int16_t ax, int16_t ay, int16_t az, float *out_roll_deg, float *out_pitch_deg);

void updateDisplay(void);

static void esp01_chpd(uint8_t val);
void ESP01_AttachChangeState(OnESP01ChangeState aOnESP01ChangeState);
void appOnESP01ChangeState(_eESP01STATUS state);

void ProcessEspRxLimited(void);

static void ControlStep10ms(void);
static float ComputeSteeringPID(float speed_r, float speed_l, float sp);
static void  SteeringPID_Reset(void);
static void  SampleEncoders250us(void);
static void  ADC1_Recover(void);
static void  I2C1_Recover(void);

static inline uint32_t micros(void) {
    return DWT->CYCCNT / (SystemCoreClock / 1000000);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void SampleEncoders250us(void)
{
    static const int8_t quad_delta[16] = {
         0, -1, +1,  0,
        +1,  0,  0, -1,
        -1,  0,  0, +1,
         0, +1, -1,  0
    };

    uint8_t ra = (uint8_t)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
    uint8_t rb = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
    uint8_t la = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
    uint8_t lb = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);

    uint8_t right_state = (uint8_t)((ra << 1) | rb);
    uint8_t left_state  = (uint8_t)((la << 1) | lb);

    if (!encoder_sampler_initialized) {
        encoder_right_prev_state = right_state;
        encoder_left_prev_state  = left_state;
        encoder_sampler_initialized = 1;
        return;
    }

    uint8_t right_idx = (uint8_t)((encoder_right_prev_state << 2) | right_state);
    uint8_t left_idx  = (uint8_t)((encoder_left_prev_state  << 2) | left_state);

    // Signo invertido a propósito: la tabla quad_delta da la convención opuesta
    // a la que tenía el conteo viejo por EXTI (verificado a mano transición por
    // transición). Sin este signo, velocity_est_f queda invertido y el freno
    // traslacional (ComputeBrakeSetpointTarget) empuja en el sentido del
    // movimiento en lugar de frenarlo -> el robot se acelera en vez de balancear.
    encoder_right -= quad_delta[right_idx];
    encoder_left  -= quad_delta[left_idx];

    encoder_right_prev_state = right_state;
    encoder_left_prev_state  = left_state;
}

SSD1306_Ctx_t ssd_ctx = {
  .hi2c      = &hi2c1,
  .busy_flag = &i2c1_tx_busy
};

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

MPU6050_Platform_t mpuPlat = {
  .ctx         = &hi2c1,
  .writeReg    = mpu_writeReg,
  .readReg     = mpu_readReg,
  .readRegDMA  = mpu_readRegDMA,
  .delayMs     = mpu_delayMs,
  .onError    = mpu_errorCb
};

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {        // 10 ms
        is10ms = 1;
    }

    if (htim->Instance == TIM2) {        // 250 µs
        is250us = 1;
        SampleEncoders250us();
    }

    if (htim->Instance == TIM5) {        // 2 ms
    	if (tick2ms_count < 10) tick2ms_count++;
    }
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        I2C_Manager_OnMemRxCplt(hi2c);
        i2c1_tx_busy = I2C_Manager_IsBusy();
    }
}
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        I2C_Manager_OnMasterTxCplt(hi2c);
        i2c1_tx_busy = I2C_Manager_IsBusy();
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        I2C_Manager_OnError(hi2c);
        i2c1_tx_busy = I2C_Manager_IsBusy();
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        adc_dma_last_ms = HAL_GetTick();
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        adc_recover_pending = 1;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        ESP01_WriteRX(dataRx);      // Sube byte al buffer AT
        //ESP01_Task();    // dentro hace ESP01ATDecode()
        HAL_UART_Receive_IT(&huart1, &dataRx, 1);
    }
}

void USB_BufferPush(uint8_t b){
    espUSBBuf[espUSBBufIw++] = b;
    espUSBBufIw &= (ESP_USB_BUF_SIZE - 1);
}

void USBRxData(uint8_t *buf, int len) {
    for (int i = 0; i < len; i++) {
        //UNER_PushByte(buf[i]);
    }
}

// Envío “atómico” por USB (fragmenta en trozos de ≤64 bytes)
void USB_DebugSend(const uint8_t *data, uint16_t len) {
    // Fragmenta en trozos ≤64B y encolarlos sin bloquear
    while (len) {
        uint16_t chunk = (len > 64 ? 64 : len);
        usb_enqueue_tx(data, chunk);
        data  += chunk;
        len   -= chunk;
    }
}

// Envía una cadena literal
void USB_DebugStr(const char *s) {
    USB_DebugSend((uint8_t*)s, (uint16_t)strlen(s));
}

// Envía un byte como dos dígitos hex ASCII
void USB_DebugHex(uint8_t b) {
    char h[2] = { HEX_DIGITS[b >> 4], HEX_DIGITS[b & 0xF] };
    USB_DebugSend((uint8_t*)h, 2);
}

// Envía un entero sin signo en decimal
void USB_DebugUInt(unsigned int v) {
    char buf[10];
    int  pos = 0;
    if (v == 0) {
        USB_DebugSend((uint8_t*)"0", 1);
        return;
    }
    while (v) {
        buf[pos++] = '0' + (v % 10);
        v /= 10;
    }
    // buf[] está al revés
    for (int i = pos - 1; i >= 0; i--) {
        USB_DebugSend((uint8_t*)&buf[i], 1);
    }
}

// La nueva USB_Debug sin vsnprintf:
void USB_Debug(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);

    while (*fmt) {
        if (*fmt == '%') {
            fmt++;
            switch (*fmt) {
                case 's': { // cadena
                    char *s = va_arg(ap, char*);
                    USB_DebugStr(s);
                    break;
                }
                case 'c': { // caracter
                    char c = (char)va_arg(ap, int);
                    USB_DebugSend((uint8_t*)&c, 1);
                    break;
                }
                case 'u': // entero sin signo
                case 'd': { // entero con signo (le damos igual)
                    unsigned int v = va_arg(ap, unsigned int);
                    USB_DebugUInt(v);
                    break;
                }
                case 'X': { // hex byte
                    uint8_t b = (uint8_t)va_arg(ap, unsigned int);
                    USB_DebugHex(b);
                    break;
                }
                case '%': { // literal ‘%’
                    USB_DebugSend((uint8_t*)"%", 1);
                    break;
                }
                default: // desconocido, lo imprimimos tal cual
                    USB_DebugSend((uint8_t*)fmt, 1);
            }
        } else {
            // carácter normal
            USB_DebugSend((uint8_t*)fmt, 1);
        }
        fmt++;
    }

    va_end(ap);
}


uint8_t usb_enqueue_tx(const uint8_t *data, uint16_t len) {
    uint16_t next;
    for (uint16_t i = 0; i < len; i++) {
        next = (tx_head + 1) & USB_TX_BUF_MASK;
        if (next == tx_tail) {
            // buffer lleno
            return 0;
        }
        usb_tx_buf[tx_head] = data[i];
        tx_head = next;
    }
    if (usb_tx_busy == 0) {
        usb_service_tx();
    }
    return 1;
}

void usb_service_tx(void) {
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

    // 1) Si antes estábamos “busy” y el driver ya terminó, liberamos la bandera
    if (usb_tx_busy && hcdc->TxState == 0) {
        usb_tx_busy = 0;
    }

    // 2) Si aún está ocupada la línea o no hay datos, no hacemos nada
    if (usb_tx_busy || tx_head == tx_tail) {
        return;
    }

    // 3) Preparamos el siguiente chunk y lo enviamos
    uint8_t chunk[64];
    uint16_t cnt = 0;
    while (cnt < 64 && tx_tail != tx_head) {
        chunk[cnt++] = usb_tx_buf[tx_tail];
        tx_tail = (tx_tail + 1) & USB_TX_BUF_MASK;
    }
    if (cnt) {
        usb_tx_busy = 1;
        CDC_Transmit_FS(chunk, cnt);
    }
}

static int uart_send_byte(uint8_t byte) {
    return (HAL_UART_Transmit(&huart1, &byte, 1, 2) == HAL_OK) ? 1 : 0;
}

void PWM_init(void)
{
    // TIM3 → PB4 (CH1), PB5 (CH2)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    // TIM4 → PB6 (CH1), PB7 (CH2)
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

    // Duty inicial 0%
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0); // PB4
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0); // PB5
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0); // PB6
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0); // PB7
}

// Motor derecho: TIM3 -> PB4 (CH1 = avance), PB5 (CH2 = reversa)
// Motor izquierdo: TIM4 -> PB6 (CH1 = reversa), PB7 (CH2 = avance)
void MotorControl(int16_t setMotorRight, int16_t setMotorLeft)
{
    // Si ambos son cero, apagar completamente los canales PWM
    if (setMotorRight == 0 && setMotorLeft == 0) {
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1); // PB4
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2); // PB5
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1); // PB6
        HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2); // PB7
        return;
    }

    // Asegurarse que los canales estén activos antes de escribir duty
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

    // Limitar a [-100, 100]
    if (setMotorRight > 100)  setMotorRight = 100;
    if (setMotorRight < -100) setMotorRight = -100;
    if (setMotorLeft > 100)   setMotorLeft  = 100;
    if (setMotorLeft < -100)  setMotorLeft  = -100;

    uint32_t arr3 = __HAL_TIM_GET_AUTORELOAD(&htim3);
    uint32_t arr4 = __HAL_TIM_GET_AUTORELOAD(&htim4);

    uint32_t dutyR = (setMotorRight >= 0) ? (uint32_t)setMotorRight : (uint32_t)(-setMotorRight);
    uint32_t dutyL = (setMotorLeft  >= 0) ? (uint32_t)setMotorLeft  : (uint32_t)(-setMotorLeft);

    uint32_t ccrR = ((arr3 + 1) * dutyR) / 100U;
    uint32_t ccrL = ((arr4 + 1) * dutyL) / 100U;

    if (ccrR > arr3) ccrR = arr3;
    if (ccrL > arr4) ccrL = arr4;

    // ----- Motor derecho (TIM3) -----
    if (setMotorRight >= 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccrR);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ccrR);
    }

    // ----- Motor izquierdo (TIM4) -----
    if (setMotorLeft >= 0) {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, ccrL);
    } else {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ccrL);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
    }
}


void my_ssd1306_init(void *ctx) {
    //MX_I2C1_Init();
}

int my_ssd1306_write_cmd(void *ctx, uint8_t cmd) {
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
int my_ssd1306_write_data(void *ctx, const uint8_t *data, uint16_t len) {
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
        i2c1_tx_busy = I2C_Manager_IsBusy();
        return;
    }

    i2c1_tx_busy = I2C_Manager_IsBusy();
}

int my_ssd1306_write_data_async(void *ctx, const uint8_t *data, uint16_t len)
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
    i2c1_tx_busy = I2C_Manager_IsBusy();
    return 0;
}


// Callback busy-check
uint8_t my_ssd1306_is_busy(void *ctx) {
    (void)ctx;
    return I2C_Manager_IsBusy() ? 1 : 0;
}

void my_ssd1306_errorCb(void *ctx, int err) {
	USB_Debug("ERROR SSD1306: 0x");
	USB_DebugHex(err);
	USB_Debug("\r\n");
	i2c1_tx_busy = I2C_Manager_IsBusy();
	SSD1306_ResetUpdateState();
}

// Callback delay
void my_ssd1306_delay_ms(void *ctx, uint32_t ms) {
    HAL_Delay(ms);
}

int mpu_writeReg(void *ctx, uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef st = HAL_I2C_Mem_Write((I2C_HandleTypeDef*)ctx, devAddr, regAddr,
    					I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
    if (st != HAL_OK) {
        mpuPlat.onError(mpuPlat.ctx, st);
        return -1;
    }
    return 0;
}

int mpu_readReg(void *ctx, uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read((I2C_HandleTypeDef*)ctx, devAddr, regAddr,
    					I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
    if (st != HAL_OK) {
        mpuPlat.onError(mpuPlat.ctx, st);
        return -1;
    }
    return 0;
}

static void mpu_dma_done_cb(void *context, HAL_StatusTypeDef status)
{
    mpu_req_pending = 0;
    if (status == HAL_OK) {
        MPU6050_ProcessDMA();
        mpu_data_ready_for_ctrl = 1;  // <-- flag para el main loop
    } else {
        mpu_errorCb(mpuPlat.ctx, status);
    }
    i2c_process_pending = 1;
}

int mpu_readRegDMA(void *ctx,
                   uint8_t devAddr,
                   uint8_t regAddr,
                   uint8_t *data,
                   uint16_t len)
{
    I2C_Request_t req = {
        .type       = I2C_REQ_MEM_READ_DMA,
        .hi2c       = (I2C_HandleTypeDef*)ctx,
        .devAddr    = devAddr,
        .memAddr    = regAddr,
        .memAddSize = I2C_MEMADD_SIZE_8BIT,
        .data       = data,
        .len        = len,
        .callback   = mpu_dma_done_cb,
        .context    = NULL
    };

    __disable_irq();
    uint8_t enqueued;
    if (I2C_Manager_IsBusy()) {
        enqueued = I2C_Manager_Enqueue(&req);
    } else {
        enqueued = I2C_Manager_EnqueuePriority(&req);
    }
    __enable_irq();

    if (!enqueued) {
        mpu_req_pending = 0;
        return -1;
    }

    // NO llamar Process() acá — solo marcar pending
    // El main loop lo va a llamar
    i2c_process_pending = 1;

    i2c1_tx_busy = I2C_Manager_IsBusy();
    return 0;
}


void mpu_delayMs(void *ctx, uint32_t ms) {
    HAL_Delay(ms);
}

void mpu_errorCb(void *ctx, int err) {
    (void)ctx;

    mpu_req_pending = 0;
    USB_Debug("MPU ERROR err=%d\r\n", err);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MPU_INT_Pin)
    {
        mpu_irq_timestamp_us = micros();  // timestamp exacto del dato

        if (!f_resetMassCenter && mpu_initialized && !mpu_req_pending)
        {
            // La lectura del MPU siempre se encola con prioridad. Si el I2C
            // estÃ¡ ocupado por OLED u otra tarea, la request del sensor queda
            // primera en cola y se ejecuta apenas se libera el bus.
            mpu_req_pending = 1;
            MPU6050_StartRead_DMA();
        }
    }

}

void calculate_tilt(int16_t ax, int16_t ay, int16_t az,
                    float *out_roll_deg, float *out_pitch_deg)
{
    // roll = atan2( ay, az )
    *out_roll_deg = atan2f(ay, az) * (180.0f / M_PI);

    // pitch = atan2( -ax, sqrt( ay² + az² ) )
    float ay_f = (float)ay;
    float az_f = (float)az;
    float denom = sqrtf(ay_f*ay_f + az_f*az_f);
    *out_pitch_deg = atan2f(-ax, denom) * (180.0f / M_PI);
}

void UpdateADC_MovingAverage(void) {
    for (uint8_t ch = 0; ch < BAR_COUNT; ++ch) {
        // Resta la muestra más antigua e incluye la nueva
        adcSum[ch] = adcSum[ch] - adcBuf[ch][maIndex] + adcValues[ch];

        adcBuf[ch][maIndex] = adcValues[ch];	        // Guarda la nueva en el buffer

        adcAvg[ch] = adcSum[ch] / ADC_AVERAGE_SIZE;	        // Calcula el promedio truncado
    }
    maIndex = (maIndex + 1) % ADC_AVERAGE_SIZE;	    // Avanza en el buffer circular
}

static void ADC1_Recover(void)
{
    HAL_ADC_Stop_DMA(&hadc1);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_OVR);
    __HAL_DMA_DISABLE(hadc1.DMA_Handle);
    __HAL_DMA_CLEAR_FLAG(hadc1.DMA_Handle,
                         DMA_FLAG_TCIF0_4 | DMA_FLAG_HTIF0_4 |
                         DMA_FLAG_TEIF0_4 | DMA_FLAG_DMEIF0_4 | DMA_FLAG_FEIF0_4);
    __HAL_DMA_ENABLE(hadc1.DMA_Handle);

    memset(adcSum, 0, sizeof(adcSum));
    memset(adcBuf, 0, sizeof(adcBuf));
    memset(adcAvg, 0, sizeof(adcAvg));
    maIndex = 0;

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcValues, 8);

    adc_dma_last_ms = HAL_GetTick();
    adc_recover_pending = 0;
    USB_Debug("ADC RECOVER\r\n");
}

static void esp01_chpd(uint8_t val) {
    // CH_PD_GPIO_Port y CH_PD_Pin vienen de MX_GPIO_Init()
    HAL_GPIO_WritePin(CH_PD_GPIO_Port, CH_PD_Pin,
                      val ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void appOnESP01ChangeState(_eESP01STATUS state) {
    if (state == ESP01_WIFI_NEW_IP) {
        ESP01_StartUDP(wifiIp, 30010, 30000);  // Ttenemos IP propia, arrancamos el socket UDP
        f_wifi_connected = 1;
    }

    // Capturamos TODOS los estados que implican pérdida de conexión
	if (state == ESP01_UDPTCP_DISCONNECTED  ||
		state == ESP01_WIFI_DISCONNECTED    ||
		state == ESP01_WIFI_RECONNECTING    ||
		state == ESP01_NOT_INIT             ||
		state == ESP01_WIFI_NOT_SETED) {
		f_wifi_connected = 0;
	}
}

void ProcessEspRxLimited(void) {
    uint8_t count = 0;

    while ((((esp01IwRx - esp01IrRx) & UDP_RX_MASK) != 0) && (count < UDP_BYTES_PER_CYCLE)) {
        uint8_t b = esp01RxBuf[esp01IrRx];
        esp01IrRx = (esp01IrRx + 1) & UDP_RX_MASK;

        UNER_PushByte(b);

        count++;
    }
}

static float clampf_local(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static float apply_deadbandf(float value, float deadband)
{
    if (value > deadband) return value - deadband;
    if (value < -deadband) return value + deadband;
    return 0.0f;
}


static float ComputeBrakeSetpointTarget(uint8_t state)
{
    if ((state == ROBOT_STATE_LINE_FOLLOWING) ||
        (state == ROBOT_STATE_BALANCE_AND_SPEED)) {
        return 0.0f;
    }

    float vel_for_brake = apply_deadbandf(velocity_est_f, BRAKE_VEL_DEADBAND);
    vel_for_brake = clampf_local(vel_for_brake, -BRAKE_VEL_MAX, BRAKE_VEL_MAX);
    float brake_tilt_max = (state == ROBOT_STATE_MANUAL_CONTROL)
                         ? BRAKE_TILT_MAX_MANUAL
                         : BRAKE_TILT_MAX;
    float abs_vel = fabsf(vel_for_brake);
    float brake_mag = KV_BRAKE * abs_vel;
    if (abs_vel > BRAKE_VEL_THRESHOLD)
        brake_mag += KV_brake_value * (abs_vel - BRAKE_VEL_THRESHOLD);

    if (vel_for_brake < 0.0f)
        brake_mag = -brake_mag;

    return clampf_local(brake_mag, -brake_tilt_max, brake_tilt_max);
}

static void SteeringPID_Reset(void)
{
    steer_pid_integral   = 0.0f;
    steer_pid_prev_error = 0.0f;
}

// Calcula corrección de steering por lazo cerrado de encoders.
// sp: diferencia de velocidad deseada en rps (0 = ir recto, >0 = girar a la derecha).
// Retorna el valor de corrección equivalente a steering_adjustment / yaw_correction.
// Convención de signo: positivo → rueda derecha más lenta → gira a la derecha.
static float ComputeSteeringPID(float speed_r, float speed_l, float sp)
{
    float error = sp - (speed_r - speed_l);

    float p = STEER_KP * error;

    steer_pid_integral += error * DT_CTRL_FIXED;
    if (steer_pid_integral >  STEER_I_MAX) steer_pid_integral =  STEER_I_MAX;
    if (steer_pid_integral < -STEER_I_MAX) steer_pid_integral = -STEER_I_MAX;
    float i = STEER_KI * steer_pid_integral;

    float d = STEER_KD * (error - steer_pid_prev_error) / DT_CTRL_FIXED;
    steer_pid_prev_error = error;

    float out = p + i + d;
    if (out >  STEER_OUT_MAX) out =  STEER_OUT_MAX;
    if (out < -STEER_OUT_MAX) out = -STEER_OUT_MAX;
    return out;
}

static void FormatSignedFixed(char *buf, size_t buf_size, float value, uint8_t decimals)
{
    int32_t scale = 1;
    for (uint8_t i = 0; i < decimals; i++) {
        scale *= 10;
    }

    int32_t scaled = (value >= 0.0f)
                   ? (int32_t)(value * (float)scale + 0.5f)
                   : (int32_t)(value * (float)scale - 0.5f);

    uint32_t abs_scaled = (scaled < 0) ? (uint32_t)(-scaled) : (uint32_t)scaled;
    uint32_t int_part   = abs_scaled / (uint32_t)scale;
    uint32_t frac_part  = abs_scaled % (uint32_t)scale;
    char sign           = (scaled < 0) ? '-' : '+';

    switch (decimals) {
        case 0:
            snprintf(buf, buf_size, "%c%lu", sign, (unsigned long)int_part);
            break;
        case 1:
            snprintf(buf, buf_size, "%c%lu.%01lu", sign,
                     (unsigned long)int_part, (unsigned long)frac_part);
            break;
        case 2:
            snprintf(buf, buf_size, "%c%lu.%02lu", sign,
                     (unsigned long)int_part, (unsigned long)frac_part);
            break;
        default:
            snprintf(buf, buf_size, "%c%lu.%03lu", sign,
                     (unsigned long)int_part, (unsigned long)frac_part);
            break;
    }
}

void updateDisplay(void) {
    SSD1306_Fill(SSD1306_COLOR_BLACK);

    if (f_change_display == 0) {
        // -------------------------------------------------------
        // PANTALLA 0
        // -------------------------------------------------------
        const uint16_t left_x  = 2;
        const uint16_t right_x = SCREEN_W / 2 + 2;
        char num_buf[16];
        char line_buf[20];
        const float balance_error = dynamic_setpoint_f - filtered_roll_deg;

        // Línea divisoria vertical
        SSD1306_DrawLine(SCREEN_W/2, 0, SCREEN_W/2, SCREEN_H - 1, SSD1306_COLOR_WHITE);

        // -------------------------------------------------------
        // Mitad izquierda: métricas útiles de balance
        // -------------------------------------------------------
        {
            const uint16_t rows_y[4] = { 2, 15, 28, 41 };

            FormatSignedFixed(num_buf, sizeof(num_buf), dynamic_setpoint_f, 1);
            snprintf(line_buf, sizeof(line_buf), "SP:%s", num_buf);
            SSD1306_GotoXY(left_x, rows_y[0]);
            SSD1306_Puts(line_buf, &Font_7x10, SSD1306_COLOR_WHITE);

            FormatSignedFixed(num_buf, sizeof(num_buf), balance_error, 1);
            snprintf(line_buf, sizeof(line_buf), "ER:%s", num_buf);
            SSD1306_GotoXY(left_x, rows_y[1]);
            SSD1306_Puts(line_buf, &Font_7x10, SSD1306_COLOR_WHITE);

            FormatSignedFixed(num_buf, sizeof(num_buf), velocity_est_f, 1);
            snprintf(line_buf, sizeof(line_buf), "VE:%s", num_buf);
            SSD1306_GotoXY(left_x, rows_y[2]);
            SSD1306_Puts(line_buf, &Font_7x10, SSD1306_COLOR_WHITE);

            FormatSignedFixed(num_buf, sizeof(num_buf), brake_setpoint_f, 1);
            snprintf(line_buf, sizeof(line_buf), "BR:%s", num_buf);
            SSD1306_GotoXY(left_x, rows_y[3]);
            SSD1306_Puts(line_buf, &Font_7x10, SSD1306_COLOR_WHITE);
        }

        // -------------------------------------------------------
		// Mitad derecha: P, D, I + estado actual del robot
		// -------------------------------------------------------
		{
			const uint16_t rx = right_x;

			// --- Filas 0-2: P, D, I con sus valores ---
			const char *param_labels[3] = { "P:", "D:", "I:" };
			float       param_vals[3]   = { KP_value, KD_value, KI_value };

			for (uint8_t i = 0; i < 3; i++) {
				uint16_t y = 1 + i * 10;

				SSD1306_GotoXY(rx, y);
				SSD1306_Puts(param_labels[i], &Font_7x10, SSD1306_COLOR_WHITE);

				char fbuf[10];
				float v = param_vals[i];
				uint8_t neg = (v < 0);
				if (neg) v = -v;
				uint32_t int_part  = (uint32_t)v;
				uint32_t frac_part = (uint32_t)((v - (float)int_part) * 1000.0f + 0.5f);
				if (neg)
					snprintf(fbuf, sizeof(fbuf), "-%lu.%03lu",
							 (unsigned long)int_part, (unsigned long)frac_part);
				else
					snprintf(fbuf, sizeof(fbuf), "%lu.%03lu",
							 (unsigned long)int_part, (unsigned long)frac_part);

				uint16_t label_w = 2 * Font_7x10.FontWidth;
				SSD1306_GotoXY(rx + label_w, y);
				SSD1306_Puts(fbuf, &Font_7x10, SSD1306_COLOR_WHITE);
			}

			// --- Separador horizontal ---
			SSD1306_DrawLine(SCREEN_W / 2 + 1, 32, SCREEN_W - 1, 32, SSD1306_COLOR_WHITE);

			// -------------------------------------------------------
			// Lado derecho inferior: modo + ángulo
			// -------------------------------------------------------
			{
			    const uint16_t x0 = SCREEN_W / 2 + 4;

			    if (robot_state == ROBOT_STATE_MANUAL_CONTROL) {
					const uint16_t ax = x0;
					const uint16_t ay = 38;
					uint8_t cmd = UNER_GetLastManualCmd();

					if (cmd == MOVE_FORWARD) {
						// Flecha ARRIBA ↑
						SSD1306_DrawPixel(ax + 3, ay + 0, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 2, ay + 1, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 3, ay + 1, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 4, ay + 1, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 1, ay + 2, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 3, ay + 2, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 5, ay + 2, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 3, ay + 3, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 3, ay + 4, SSD1306_COLOR_WHITE);
					} else if (cmd == MOVE_BACKWARD) {
						// Flecha ABAJO ↓
						SSD1306_DrawPixel(ax + 3, ay + 0, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 3, ay + 1, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 1, ay + 2, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 3, ay + 2, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 5, ay + 2, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 2, ay + 3, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 3, ay + 3, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 4, ay + 3, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 3, ay + 4, SSD1306_COLOR_WHITE);
					} else if (cmd == MOVE_RIGHT) {
						// Flecha DERECHA →
						SSD1306_DrawPixel(ax + 0, ay + 2, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 1, ay + 2, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 2, ay + 2, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 3, ay + 2, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 4, ay + 2, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 3, ay + 1, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 4, ay + 1, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 5, ay + 2, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 3, ay + 3, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 4, ay + 3, SSD1306_COLOR_WHITE);
					} else if (cmd == MOVE_LEFT) {
						// Flecha IZQUIERDA ←
						SSD1306_DrawPixel(ax + 5, ay + 2, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 4, ay + 2, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 3, ay + 2, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 2, ay + 2, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 1, ay + 2, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 0, ay + 2, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 1, ay + 1, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 2, ay + 1, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 1, ay + 3, SSD1306_COLOR_WHITE);
						SSD1306_DrawPixel(ax + 2, ay + 3, SSD1306_COLOR_WHITE);
					} else {
						// STOP: cuadradito
						SSD1306_DrawFilledRectangle(ax + 1, ay + 1, 5, 3, SSD1306_COLOR_WHITE);
					}
				}

			    // -------- MODO (abajo a la derecha) --------
			    const char *mode_str = "IDLE";
			    switch (robot_state) {
			        case ROBOT_STATE_IDLE:
			            mode_str = "IDLE";
			            break;
			        case ROBOT_STATE_BALANCE_ONLY:
			            mode_str = "BAL";
			            break;
			        case ROBOT_STATE_BALANCE_AND_SPEED:
			            mode_str = "SPEED";
			            break;
			        case ROBOT_STATE_LINE_FOLLOWING:
			            mode_str = "LINE";
			            break;
                    case ROBOT_STATE_MANUAL_CONTROL:
                        mode_str = "MANUAL";
                        break;
                    case ROBOT_STATE_MOTOR_TEST:
                        mode_str = "TEST";
                        break;
			        default:
			            mode_str = "UNK";
			            break;
			    }

			    {
			        uint16_t x = x0 + 15;
			        uint16_t y = 38;   // modo abajo
			        for (const char *p = mode_str; *p; p++) {
			            SSD1306_DrawChar5x7(*p, x, y);
			            x += Font_5x7.FontWidth + 1;
			        }
			    }

			    // línea separadora chiquita
			    SSD1306_DrawLine(x0, 51, SCREEN_W - 2, 51, SSD1306_COLOR_WHITE);

			    // -------- ANGULO (debajo del modo) --------
			    {
			        char angbuf[16];
			        uint16_t x = x0;
			        uint16_t y = 54;

			        int32_t ang10;
			        int32_t ent;
			        int32_t dec;

			        // redondeo a 1 decimal
			        if (filtered_roll_deg >= 0.0f) {
			            ang10 = (int32_t)(filtered_roll_deg * 10.0f + 0.5f);
			        } else {
			            ang10 = (int32_t)(filtered_roll_deg * 10.0f - 0.5f);
			        }

			        ent = ang10 / 10;
			        dec = ang10 % 10;
			        if (dec < 0) dec = -dec;

			        // texto corto y seguro
			        snprintf(angbuf, sizeof(angbuf), "A:%ld.%ld", ent, dec);

			        for (char *p = angbuf; *p; p++) {
			            if (*p == '-') {
			                SSD1306_DrawLine(x, y + 3, x + 3, y + 3, SSD1306_COLOR_WHITE);
			                x += 5;
			            } else if (*p == ':') {
			                SSD1306_DrawPixel(x + 1, y + 1, SSD1306_COLOR_WHITE);
			                SSD1306_DrawPixel(x + 1, y + 4, SSD1306_COLOR_WHITE);
			                x += 4;
			            } else if (*p == '.') {
			                SSD1306_DrawPixel(x + 1, y + 6, SSD1306_COLOR_WHITE);
			                x += 3;
			            } else {
			                SSD1306_DrawChar5x7(*p, x, y);
			                x += Font_5x7.FontWidth + 1;
			            }
			        }

			        // trim del setpoint a la derecha del angulo
			        {
			            x += 3;
			            float tv = setpoint_trim;
			            int32_t tv10 = (tv >= 0.0f)
			                ? (int32_t)(tv * 10.0f + 0.5f)
			                : (int32_t)(tv * 10.0f - 0.5f);
			            int32_t tv_ent = tv10 / 10;
			            int32_t tv_dec = tv10 % 10;
			            if (tv_dec < 0) tv_dec = -tv_dec;

			            if (tv_ent >  99) tv_ent =  99;
			            if (tv_ent < -99) tv_ent = -99;
			            char trimbuf[6];
			            trimbuf[0] = (tv < 0) ? '-' : '+';
			            trimbuf[1] = '0' + (char)(tv_ent < 0 ? -tv_ent : tv_ent);
			            trimbuf[2] = '.';
			            trimbuf[3] = '0' + (char)tv_dec;
			            trimbuf[4] = '\0';

			            for (char *q = trimbuf; *q && x < SCREEN_W - 4; q++) {
			                if (*q == '-') {
			                    SSD1306_DrawLine(x, y + 3, x + 3, y + 3, SSD1306_COLOR_WHITE);
			                    x += 5;
			                } else if (*q == '+') {
			                    SSD1306_DrawLine(x,     y + 3, x + 4, y + 3, SSD1306_COLOR_WHITE);
			                    SSD1306_DrawLine(x + 2, y + 1, x + 2, y + 5, SSD1306_COLOR_WHITE);
			                    x += 6;
			                } else if (*q == '.') {
			                    SSD1306_DrawPixel(x + 1, y + 6, SSD1306_COLOR_WHITE);
			                    x += 3;
			                } else {
			                    SSD1306_DrawChar5x7(*q, x, y);
			                    x += Font_5x7.FontWidth + 1;
			                }
			            }
			        }
			    }
			}
		}

        // -------------------------------------------------------
        // Spinner
        // -------------------------------------------------------
        {
            const uint16_t ix = 32;
            const uint16_t iy = 52;
            const uint16_t sx = 48;
            const uint16_t sy = 56;

            static uint8_t spinPhase = 0;

            if (f_wifi_connected) {
                // outer arc (9px wide)
                SSD1306_DrawPixel(ix+3,iy+0,SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix+4,iy+0,SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix+5,iy+0,SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix+2,iy+1,SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix+6,iy+1,SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix+1,iy+2,SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix+7,iy+2,SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix+0,iy+3,SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix+8,iy+3,SSD1306_COLOR_WHITE);
                // middle arc (5px wide)
                SSD1306_DrawPixel(ix+3,iy+3,SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix+4,iy+3,SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix+5,iy+3,SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix+2,iy+4,SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix+6,iy+4,SSD1306_COLOR_WHITE);
                // inner arc (3px)
                SSD1306_DrawPixel(ix+3,iy+5,SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix+4,iy+5,SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix+5,iy+5,SSD1306_COLOR_WHITE);
                // dot
                SSD1306_DrawPixel(ix+4,iy+6,SSD1306_COLOR_WHITE);
            } else {
                SSD1306_DrawLine(ix+1, iy+1, ix+7, iy+5, SSD1306_COLOR_WHITE);
                SSD1306_DrawLine(ix+7, iy+1, ix+1, iy+5, SSD1306_COLOR_WHITE);
            }

            static const int8_t spokes[8][4] = {
                { 0, -4,  0, -3},
                { 3, -3,  2, -2},
                { 4,  0,  3,  0},
                { 3,  3,  2,  2},
                { 0,  4,  0,  3},
                {-3,  3, -2,  2},
                {-4,  0, -3,  0},
                {-3, -3, -2, -2},
            };

            for (uint8_t s = 0; s < 3; s++) {
                uint8_t idx = (spinPhase + s) % 8;
                SSD1306_DrawPixel(sx + spokes[idx][0], sy + spokes[idx][1], SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(sx + spokes[idx][2], sy + spokes[idx][3], SSD1306_COLOR_WHITE);
            }

            uint8_t head = (spinPhase + 3) % 8;
            SSD1306_DrawPixel(sx + spokes[head][0],     sy + spokes[head][1],     SSD1306_COLOR_WHITE);
            SSD1306_DrawPixel(sx + spokes[head][2],     sy + spokes[head][3],     SSD1306_COLOR_WHITE);
            SSD1306_DrawPixel(sx + spokes[head][0] + 1, sy + spokes[head][1],     SSD1306_COLOR_WHITE);
            SSD1306_DrawPixel(sx + spokes[head][2] + 1, sy + spokes[head][3],     SSD1306_COLOR_WHITE);

            spinPhase = (spinPhase + 1) % 8;
        }

    }  else if (f_change_display == 1) {
        // -------------------------------------------------------
        // PANTALLA 1: Estado general + estado line follower
        // -------------------------------------------------------

        // ----- 8 barras ADC izquierda: 1-4 sensores línea, 5-8 sensores objeto -----
        {
            const uint16_t bar_top   = 2;
            const uint16_t digit_y   = 55;
            const uint16_t bar_max_h = (digit_y - bar_top - 1) * 3 / 4;   // 75% de la altura máxima
            const uint16_t bar_width = 7;   // 8×7 + 9×1 = 65 px en área de 70 px
            const uint16_t spacing   = 1;

            // Las barras 1-4 (línea) dejan 5px arriba para el indicador negro/blanco
            const uint16_t ind_h     = 4;  // altura reservada para indicador (px)
            const uint16_t bar_line_top = bar_top + ind_h;  // y=6: donde empieza la barra de línea

            for (uint8_t i = 0; i < 8; i++) {
                // ADC 1-4 en orden invertido (igual que antes), ADC 5-8 directo
                uint8_t adc_idx = (i < 4) ? (3 - i) : i;
                uint16_t v = adcAvg[adc_idx] > 4095 ? 4095 : adcAvg[adc_idx];
                uint16_t x0 = spacing + i * (bar_width + spacing);

                if (i < 4) {
                    // Barra reducida: empieza en bar_line_top para dejar espacio al indicador
                    uint16_t line_bar_max_h = (digit_y - bar_line_top - 1) * 3 / 4;  // 75%
                    uint16_t h = (uint32_t)v * line_bar_max_h / 4095;
                    uint16_t y0 = digit_y - 1 - h;
                    if (h > 0)
                        SSD1306_DrawFilledRectangle(x0, y0, bar_width, h, SSD1306_COLOR_WHITE);

                    // Indicador negro/blanco: relleno = negro, borde = blanco
                    if (adcAvg[adc_idx] > (uint16_t)LINE_THRESHOLD)
                        SSD1306_DrawFilledRectangle(x0, bar_top, bar_width, ind_h - 1, SSD1306_COLOR_WHITE);
                    else
                        SSD1306_DrawRectangle(x0, bar_top, bar_width, ind_h - 1, SSD1306_COLOR_WHITE);
                } else {
                    uint16_t h = (uint32_t)v * bar_max_h / 4095;
                    uint16_t y0 = digit_y - 1 - h;
                    if (h > 0)
                        SSD1306_DrawFilledRectangle(x0, y0, bar_width, h, SSD1306_COLOR_WHITE);
                }

                SSD1306_DrawChar5x7('1' + adc_idx, x0 + 1, digit_y);
            }

            // Separador punteado vertical entre grupo línea (1-4) y objeto (5-8)
            uint16_t sep_x = spacing + 4 * (bar_width + spacing) - 1;  // x=32
            for (uint16_t py = bar_top; py < digit_y - 1; py += 3) {
                SSD1306_DrawPixel(sep_x, py, SSD1306_COLOR_WHITE);
            }

            SSD1306_DrawLine(0, digit_y - 1, 70, digit_y - 1, SSD1306_COLOR_WHITE);
        }

        // ----- línea divisoria vertical -----
        SSD1306_DrawLine(71, 0, 71, SCREEN_H - 1, SSD1306_COLOR_WHITE);

        // ----- lado derecho -----
        {
            const char *line_mode_str = "OFF";

            if (robot_state != ROBOT_STATE_LINE_FOLLOWING) {
                line_mode_str = "OFF";
            } else {
                switch (line_state) {
                    case LINE_STATE_FOLLOWING:            line_mode_str = "SIGUE";  break;
                    case LINE_STATE_LOST:                 line_mode_str = "PERDI";  break;
                    case LINE_STATE_SEARCHING:            line_mode_str = "BUSCA";  break;
                    case LINE_STATE_LOST_BRAKE:           line_mode_str = "FRENA";  break;
                    case LINE_STATE_LOST_ROTATE:          line_mode_str = "GIRO";   break;
                    case LINE_STATE_LOST_SETTLE:          line_mode_str = "ESTAB";  break;
                    case LINE_STATE_LOST_FWD:             line_mode_str = "AVNZA";  break;
                    case LINE_STATE_EDGE_WAIT:             line_mode_str = "EFRENA"; break;
                    case LINE_STATE_EDGE_ROTATE:           line_mode_str = "EGIRO";  break;
                    case LINE_STATE_EDGE_SETTLE:           line_mode_str = "EESTAB"; break;
                    case LINE_STATE_EDGE_FWD:              line_mode_str = "EAVNZA"; break;
                    case LINE_STATE_GIVEN_UP:              line_mode_str = "PARADO"; break;
                    case LINE_STATE_PERP_ROTATE:           line_mode_str = "PGIRO";  break;
                    case LINE_STATE_OBJ_PRE_REVERSE_HOLD: line_mode_str = "ESPER";  break;
                    case LINE_STATE_OBJ_REVERSE:          line_mode_str = "RETRO";  break;
                    case LINE_STATE_OBJ_BRAKE:            line_mode_str = "STOP";   break;
                    case LINE_STATE_OBJ_ROTATE:           line_mode_str = "ESQUIV"; break;
                    case LINE_STATE_OBJ_HOLD:             line_mode_str = "PAUSA";  break;
                    case LINE_STATE_OBJ_WALL_APPROACH:    line_mode_str = "APRCH";  break;
                    case LINE_STATE_OBJ_WALL_FWD:         line_mode_str = "PARED";  break;
                    case LINE_STATE_OBJ_WALL_TURN:        line_mode_str = "GIRAP";  break;
                    default:                              line_mode_str = "UNK";    break;
                }
            }

            // --- SP: setpoint activo del balance PID (y=1, arriba del todo) ---
            {
                char buf[14];
                float v = dynamic_setpoint_f;
                uint8_t neg = (v < 0.0f);
                if (neg) v = -v;
                uint32_t vi = (uint32_t)v;
                uint32_t vd = (uint32_t)((v - vi) * 100.0f + 0.5f);
                if (vd >= 100) { vd = 0; vi++; }
                snprintf(buf, sizeof(buf), "SP:%c%lu.%02lu", neg ? '-' : '+',
                         (unsigned long)vi, (unsigned long)vd);
                uint16_t x = 73;
                for (char *p = buf; *p; p++) {
                    SSD1306_DrawChar5x7(*p, x, 1);
                    x += Font_5x7.FontWidth + 1;
                }
            }

            // --- KP (y=10) ---
            {
                char buf[14];
                float v = KP_LINE;
                uint32_t vi = (uint32_t)v;
                uint32_t vd = (uint32_t)((v - vi) * 1000.0f + 0.5f);
                if (vd >= 1000) { vd = 0; vi++; }
                snprintf(buf, sizeof(buf), "KP:%lu.%03lu",
                         (unsigned long)vi, (unsigned long)vd);
                uint16_t x = 73;
                for (char *p = buf; *p; p++) {
                    SSD1306_DrawChar5x7(*p, x, 10);
                    x += Font_5x7.FontWidth + 1;
                }
            }

            // --- ADC7 — sensor lateral pared (y=19) ---
            {
                char buf[14];
                snprintf(buf, sizeof(buf), "A7:%lu", (unsigned long)adcAvg[6]);
                uint16_t x = 73;
                for (char *p = buf; *p; p++) {
                    SSD1306_DrawChar5x7(*p, x, 19);
                    x += Font_5x7.FontWidth + 1;
                }
            }

            // --- separador ---
            SSD1306_DrawLine(72, 28, 127, 28, SSD1306_COLOR_WHITE);

            // --- sub-estado línea en Font_7x10 centrado (y=32) ---
            {
                uint16_t len = 0;
                for (const char *p = line_mode_str; *p; p++) len++;
                uint16_t sw = (len > 0) ? (len * 8 - 1) : 0;
                const uint16_t panel_w = 55;  // x=73..127
                uint16_t sx = 73 + ((panel_w > sw) ? (panel_w - sw) / 2 : 0);
                SSD1306_GotoXY(sx, 32);
                SSD1306_Puts(line_mode_str, &Font_7x10, SSD1306_COLOR_WHITE);
            }

            // --- WiFi icon abajo izquierda + spinner abajo derecha ---
            {
                // WiFi icon (9x7) en x=73, y=55
                const uint16_t ix = 73;
                const uint16_t iy = 55;
                if (f_wifi_connected) {
                    SSD1306_DrawPixel(ix+3,iy+0,SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix+4,iy+0,SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix+5,iy+0,SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix+2,iy+1,SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix+6,iy+1,SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix+1,iy+2,SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix+7,iy+2,SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix+0,iy+3,SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix+8,iy+3,SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix+3,iy+3,SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix+4,iy+3,SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix+5,iy+3,SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix+2,iy+4,SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix+6,iy+4,SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix+3,iy+5,SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix+4,iy+5,SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix+5,iy+5,SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix+4,iy+6,SSD1306_COLOR_WHITE);
                } else {
                    SSD1306_DrawLine(ix+1, iy+1, ix+7, iy+5, SSD1306_COLOR_WHITE);
                    SSD1306_DrawLine(ix+7, iy+1, ix+1, iy+5, SSD1306_COLOR_WHITE);
                }

                // Spinner en x=118, y=58
                static uint8_t spinPhaseLF = 0;
                static const int8_t spokes[8][4] = {
                    { 0, -3,  0, -2},
                    { 2, -2,  1, -1},
                    { 3,  0,  2,  0},
                    { 2,  2,  1,  1},
                    { 0,  3,  0,  2},
                    {-2,  2, -1,  1},
                    {-3,  0, -2,  0},
                    {-2, -2, -1, -1},
                };
                const uint16_t sx = 118;
                const uint16_t sy = 58;
                for (uint8_t s = 0; s < 3; s++) {
                    uint8_t idx = (spinPhaseLF + s) % 8;
                    SSD1306_DrawPixel(sx + spokes[idx][0], sy + spokes[idx][1], SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(sx + spokes[idx][2], sy + spokes[idx][3], SSD1306_COLOR_WHITE);
                }
                uint8_t head = (spinPhaseLF + 3) % 8;
                SSD1306_DrawPixel(sx + spokes[head][0],     sy + spokes[head][1],     SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(sx + spokes[head][2],     sy + spokes[head][3],     SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(sx + spokes[head][0] + 1, sy + spokes[head][1],     SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(sx + spokes[head][2] + 1, sy + spokes[head][3],     SSD1306_COLOR_WHITE);
                spinPhaseLF = (spinPhaseLF + 1) % 8;
            }
        }
	} else if (f_change_display == 2) {
        // -------------------------------------------------------
        // PANTALLA 1: Barras ADC pantalla completa (8 canales)
        // -------------------------------------------------------

        SSD1306_GotoXY(50, 0);
        SSD1306_Puts("ADC", &Font_7x10, SSD1306_COLOR_WHITE);

        const uint16_t bar_top   = 11;
        const uint16_t digit_y   = 55;
        const uint16_t bar_max_h = digit_y - bar_top - 1;

        const uint16_t bar_spacing = 2;
        const uint16_t bar_width   = (SCREEN_W - (BAR_COUNT + 1) * bar_spacing) / BAR_COUNT;

        for (uint8_t i = 0; i < BAR_COUNT; i++) {
            uint16_t v  = adcAvg[i] > 4095 ? 4095 : adcAvg[i];
            uint16_t h  = (uint32_t)v * bar_max_h / 4095;
            uint16_t x0 = bar_spacing + i * (bar_width + bar_spacing);
            uint16_t y0 = digit_y - 1 - h;

            if (h > 0)
                SSD1306_DrawFilledRectangle(x0, y0, bar_width, h, SSD1306_COLOR_WHITE);

            uint16_t tx = x0 + (bar_width - Font_5x7.FontWidth) / 2;
            SSD1306_DrawChar5x7('1' + i, tx, digit_y);
        }

        SSD1306_DrawLine(0, digit_y - 1, SCREEN_W - 1, digit_y - 1, SSD1306_COLOR_WHITE);

    } else if (f_change_display == 3) {
        // -------------------------------------------------------
        // PANTALLA 2: ADC 1..4 barras izquierda | parámetros línea derecha
        // -------------------------------------------------------

        const uint16_t split_x = 55;
        SSD1306_DrawLine(split_x, 0, split_x, SCREEN_H - 1, SSD1306_COLOR_WHITE);

        // ----- lado izquierdo: 4 barras ADC en orden invertido -----
		// izquierda = ADC4, derecha = ADC1
		{
			const uint8_t  adc_count  = 4;
			const uint16_t left_w     = split_x - 1;
			const uint16_t bar_top    = 2;
			const uint16_t digit_y    = 55;
			const uint16_t bar_max_h  = digit_y - bar_top - 1;
			const uint16_t spacing    = 2;
			const uint16_t bar_width  = (left_w - (adc_count + 1) * spacing) / adc_count;

			for (uint8_t i = 0; i < adc_count; i++) {
				uint8_t adc_idx = (adc_count - 1) - i;   // 3,2,1,0
				uint16_t v = adcAvg[adc_idx] > 4095 ? 4095 : adcAvg[adc_idx];
				uint16_t h = (uint32_t)v * bar_max_h / 4095;
				uint16_t x0 = spacing + i * (bar_width + spacing);
				uint16_t y0 = digit_y - 1 - h;

				if (h > 0)
					SSD1306_DrawFilledRectangle(x0, y0, bar_width, h, SSD1306_COLOR_WHITE);

				uint16_t tx = x0 + (bar_width - Font_5x7.FontWidth) / 2;
				SSD1306_DrawChar5x7('1' + adc_idx, tx, digit_y);   // muestra 4,3,2,1
			}

			SSD1306_DrawLine(0, digit_y - 1, left_w - 1, digit_y - 1, SSD1306_COLOR_WHITE);
		}

        // ----- lado derecho: parámetros de línea -----
        // Formato: "KP=0.000" en Font_5x7, 5 filas × 11px = 55px → caben justo en 64px
        {
            const uint16_t rx = split_x + 3;

            const char *labels[5] = { "KP=", "KD=", "KI=", "TH=", "SP=" };
            float values[5] = { KP_LINE, KD_LINE, KI_LINE, LINE_THRESHOLD, LINE_ANGLE };

            for (uint8_t i = 0; i < 5; i++) {
                uint16_t y = 1 + i * 11;
                uint16_t x = rx;

                // Imprimir label (ej: "KP=")
                for (const char *p = labels[i]; *p; p++) {
                    SSD1306_DrawChar5x7(*p, x, y);
                    x += Font_5x7.FontWidth + 1;
                }

                // Valor numérico
                float v = values[i];
                uint8_t neg = (v < 0.0f);
                if (neg) v = -v;

                char fbuf[10];
                if (i == 3) {
                    // TH: entero sin decimales (valor grande tipo 3000)
                    snprintf(fbuf, sizeof(fbuf), "%lu", (unsigned long)(uint32_t)v);
                } else {
                    // Resto: formato 0.000
                    uint32_t int_part  = (uint32_t)v;
                    uint32_t frac_part = (uint32_t)((v - (float)int_part) * 1000.0f + 0.5f);
                    if (neg)
                        snprintf(fbuf, sizeof(fbuf), "-%lu.%03lu",
                                 (unsigned long)int_part, (unsigned long)frac_part);
                    else
                        snprintf(fbuf, sizeof(fbuf), "%lu.%03lu",
                                 (unsigned long)int_part, (unsigned long)frac_part);
                }

                for (char *p = fbuf; *p; p++) {
                    SSD1306_DrawChar5x7(*p, x, y);
                    x += Font_5x7.FontWidth + 1;
                }
            }
        }

        // ----- spinner: esquina inferior derecha -----
        {
            // Zona derecha: desde split_x+1 hasta SCREEN_W-1 = 72px de ancho
            // Centrado horizontalmente en esa zona, pegado al fondo
            const uint16_t sx = split_x + (SCREEN_W - split_x) / 2;  // centro horizontal zona derecha
            const uint16_t sy = SCREEN_H - 6;                          // lo más abajo posible (radio=4px)

            static uint8_t spinPhase2 = 0;

            static const int8_t spokes[8][4] = {
                { 0, -4,  0, -3},
                { 3, -3,  2, -2},
                { 4,  0,  3,  0},
                { 3,  3,  2,  2},
                { 0,  4,  0,  3},
                {-3,  3, -2,  2},
                {-4,  0, -3,  0},
                {-3, -3, -2, -2},
            };

            for (uint8_t s = 0; s < 3; s++) {
                uint8_t idx = (spinPhase2 + s) % 8;
                SSD1306_DrawPixel(sx + spokes[idx][0], sy + spokes[idx][1], SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(sx + spokes[idx][2], sy + spokes[idx][3], SSD1306_COLOR_WHITE);
            }
            uint8_t head = (spinPhase2 + 3) % 8;
            SSD1306_DrawPixel(sx + spokes[head][0],     sy + spokes[head][1],     SSD1306_COLOR_WHITE);
            SSD1306_DrawPixel(sx + spokes[head][2],     sy + spokes[head][3],     SSD1306_COLOR_WHITE);
            SSD1306_DrawPixel(sx + spokes[head][0] + 1, sy + spokes[head][1],     SSD1306_COLOR_WHITE);
            SSD1306_DrawPixel(sx + spokes[head][2] + 1, sy + spokes[head][3],     SSD1306_COLOR_WHITE);

            spinPhase2 = (spinPhase2 + 1) % 8;
        }
    } else if (f_change_display == 4) {
        // -------------------------------------------------------
        // PANTALLA 3: ADC1..ADC4 numéricos + spinner
        // -------------------------------------------------------

        {
            // Grid 4×2: ADC1-4 columna izquierda, ADC5-8 columna derecha
            // Font_5x7: "A1=4095" = 7 chars × 6px = 42px por columna
            const uint16_t col_l = 1;
            const uint16_t col_r = 67;
            const uint16_t rows[4] = {1, 14, 27, 40};
            char buf[12];

            for (uint8_t i = 0; i < 4; i++) {
                uint16_t x;

                snprintf(buf, sizeof(buf), "A%u=%4u", i + 1, adcAvg[i]);
                x = col_l;
                for (char *p = buf; *p; p++) {
                    SSD1306_DrawChar5x7(*p, x, rows[i]);
                    x += Font_5x7.FontWidth + 1;
                }

                snprintf(buf, sizeof(buf), "A%u=%4u", i + 5, adcAvg[i + 4]);
                x = col_r;
                for (char *p = buf; *p; p++) {
                    SSD1306_DrawChar5x7(*p, x, rows[i]);
                    x += Font_5x7.FontWidth + 1;
                }
            }

            SSD1306_DrawLine(63, 0, 63, 52, SSD1306_COLOR_WHITE);
        }

        // ----- spinner abajo a la derecha -----
        {
            const uint16_t sx = SCREEN_W - 10;
            const uint16_t sy = SCREEN_H - 10;

            static uint8_t spinPhase3 = 0;

            static const int8_t spokes[8][4] = {
                { 0, -4,  0, -3},
                { 3, -3,  2, -2},
                { 4,  0,  3,  0},
                { 3,  3,  2,  2},
                { 0,  4,  0,  3},
                {-3,  3, -2,  2},
                {-4,  0, -3,  0},
                {-3, -3, -2, -2},
            };

            for (uint8_t s = 0; s < 3; s++) {
                uint8_t idx = (spinPhase3 + s) % 8;
                SSD1306_DrawPixel(sx + spokes[idx][0], sy + spokes[idx][1], SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(sx + spokes[idx][2], sy + spokes[idx][3], SSD1306_COLOR_WHITE);
            }

            uint8_t head = (spinPhase3 + 3) % 8;
            SSD1306_DrawPixel(sx + spokes[head][0],     sy + spokes[head][1],     SSD1306_COLOR_WHITE);
            SSD1306_DrawPixel(sx + spokes[head][2],     sy + spokes[head][3],     SSD1306_COLOR_WHITE);
            SSD1306_DrawPixel(sx + spokes[head][0] + 1, sy + spokes[head][1],     SSD1306_COLOR_WHITE);
            SSD1306_DrawPixel(sx + spokes[head][2] + 1, sy + spokes[head][3],     SSD1306_COLOR_WHITE);

            spinPhase3 = (spinPhase3 + 1) % 8;
        }

    } else if (f_change_display == 5) {
        // -------------------------------------------------------
        // PANTALLA 5: Valores MPU grandes + estado WiFi
        // Izquierda: giroscopio (gx, gy, gz)
        // Derecha:   acelerometro (ax, ay, az)
        // Abajo:     gyro filtrado | WiFi
        // -------------------------------------------------------
        char buf[12];

        // Divisor vertical entre gyro y accel
        SSD1306_DrawLine(63, 0, 63, 36, SSD1306_COLOR_WHITE);
        // Divisor horizontal para fila de WiFi
        SSD1306_DrawLine(0, 37, SCREEN_W - 1, 37, SSD1306_COLOR_WHITE);

        // Etiquetas columna izquierda (Font_5x7 para ahorrar espacio)
        SSD1306_DrawChar5x7('G', 0, 1);
        SSD1306_DrawChar5x7('Y', 6, 1);
        SSD1306_DrawChar5x7('G', 0, 13);
        SSD1306_DrawChar5x7('X', 6, 13);
        SSD1306_DrawChar5x7('G', 0, 25);
        SSD1306_DrawChar5x7('Z', 6, 25);

        // Valores giroscopio (Font_7x10)
        snprintf(buf, sizeof(buf), "%+6d", (int)gy);
        SSD1306_GotoXY(13, 1);
        SSD1306_Puts(buf, &Font_7x10, SSD1306_COLOR_WHITE);

        snprintf(buf, sizeof(buf), "%+6d", (int)gx);
        SSD1306_GotoXY(13, 13);
        SSD1306_Puts(buf, &Font_7x10, SSD1306_COLOR_WHITE);

        snprintf(buf, sizeof(buf), "%+6d", (int)gz);
        SSD1306_GotoXY(13, 25);
        SSD1306_Puts(buf, &Font_7x10, SSD1306_COLOR_WHITE);

        // Etiquetas columna derecha
        SSD1306_DrawChar5x7('A', 65, 1);
        SSD1306_DrawChar5x7('Y', 71, 1);
        SSD1306_DrawChar5x7('A', 65, 13);
        SSD1306_DrawChar5x7('X', 71, 13);
        SSD1306_DrawChar5x7('A', 65, 25);
        SSD1306_DrawChar5x7('Z', 71, 25);

        // Valores acelerometro (Font_7x10)
        snprintf(buf, sizeof(buf), "%+6d", (int)ay);
        SSD1306_GotoXY(78, 1);
        SSD1306_Puts(buf, &Font_7x10, SSD1306_COLOR_WHITE);

        snprintf(buf, sizeof(buf), "%+6d", (int)ax);
        SSD1306_GotoXY(78, 13);
        SSD1306_Puts(buf, &Font_7x10, SSD1306_COLOR_WHITE);

        snprintf(buf, sizeof(buf), "%+6d", (int)az);
        SSD1306_GotoXY(78, 25);
        SSD1306_Puts(buf, &Font_7x10, SSD1306_COLOR_WHITE);

        // Fila inferior izquierda: gyro filtrado en deg/s
        snprintf(buf, sizeof(buf), "w%+6.1f", (double)gyro_f);
        SSD1306_GotoXY(0, 41);
        SSD1306_Puts(buf, &Font_7x10, SSD1306_COLOR_WHITE);

        // Fila inferior derecha: estado WiFi con icono
        {
            const uint16_t ix = 119;
            const uint16_t iy = 41;
            if (f_wifi_connected) {
                SSD1306_DrawPixel(ix+3,iy+0,SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix+0,iy+2,SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix+6,iy+2,SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix+1,iy+4,SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix+5,iy+4,SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix+2,iy+6,SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix+4,iy+6,SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix+3,iy+8,SSD1306_COLOR_WHITE);
            } else {
                SSD1306_DrawPixel(ix+0,iy+0,SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix+6,iy+0,SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix+1,iy+1,SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix+5,iy+1,SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix+2,iy+2,SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix+4,iy+2,SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix+3,iy+3,SSD1306_COLOR_WHITE);
            }
            SSD1306_GotoXY(65, 53);
            SSD1306_Puts(f_wifi_connected ? "CONECTADO" : "SIN WIFI ", &Font_5x7, SSD1306_COLOR_WHITE);
        }

    } else if (f_change_display == 6) {
        // -------------------------------------------------------
        // PANTALLA 6: Evasión de objeto — estado grande + barras ADC5 y ADC7
        // -------------------------------------------------------

        // Estado del esquive centrado con Font_7x10 (máxima legibilidad)
        {
            const char *obj_str = "OBJ";
            switch (line_state) {
                case LINE_STATE_OBJ_PRE_REVERSE_HOLD: obj_str = "ESPER";  break;
                case LINE_STATE_OBJ_REVERSE:          obj_str = "RETRO";  break;
                case LINE_STATE_OBJ_BRAKE:            obj_str = "STOP";   break;
                case LINE_STATE_OBJ_ROTATE:           obj_str = "ESQUIV"; break;
                case LINE_STATE_OBJ_HOLD:             obj_str = "PAUSA";  break;
                case LINE_STATE_OBJ_WALL_APPROACH:    obj_str = "APRCH";  break;
                case LINE_STATE_OBJ_WALL_FWD:         obj_str = "PARED";  break;
                case LINE_STATE_OBJ_WALL_TURN:        obj_str = "GIRAP";  break;
                default: break;
            }
            uint16_t len = 0;
            for (const char *p = obj_str; *p; p++) len++;
            uint16_t sw = (len > 0) ? (len * 8 - 1) : 0;  // Font_7x10: 7px + 1px spacing
            uint16_t sx = (SCREEN_W > sw) ? (SCREEN_W - sw) / 2 : 0;
            SSD1306_GotoXY(sx, 1);
            SSD1306_Puts(obj_str, &Font_7x10, SSD1306_COLOR_WHITE);
        }

        // Divisor horizontal bajo el texto de estado
        SSD1306_DrawLine(0, 13, SCREEN_W - 1, 13, SSD1306_COLOR_WHITE);
        // Divisor vertical entre barra ADC5 (izq) y ADC7 (der)
        SSD1306_DrawLine(63, 13, 63, SCREEN_H - 1, SSD1306_COLOR_WHITE);

        const uint16_t bar_top6   = 22;
        const uint16_t bar_bot6   = 54;
        const uint16_t bar_max_h6 = bar_bot6 - bar_top6;  // 32px
        const uint16_t bar_w6     = 40;

        // --- Barra izquierda: ADC5 (adcAvg[4]) — sensor de objeto frontal ---
        {
            const uint16_t bx = (64 - bar_w6) / 2;         // 12
            uint16_t v = adcAvg[4] > 4095 ? 4095 : adcAvg[4];
            uint16_t h = (uint32_t)v * bar_max_h6 / 4095;

            // Label "A5" centrado sobre la barra
            uint16_t lx = bx + (bar_w6 - 2 * (Font_5x7.FontWidth + 1)) / 2;
            SSD1306_DrawChar5x7('A', lx, 15);
            SSD1306_DrawChar5x7('5', lx + Font_5x7.FontWidth + 1, 15);

            // Borde de la barra
            SSD1306_DrawLine(bx,              bar_top6, bx + bar_w6 - 1, bar_top6, SSD1306_COLOR_WHITE);
            SSD1306_DrawLine(bx,              bar_bot6, bx + bar_w6 - 1, bar_bot6, SSD1306_COLOR_WHITE);
            SSD1306_DrawLine(bx,              bar_top6, bx,              bar_bot6, SSD1306_COLOR_WHITE);
            SSD1306_DrawLine(bx + bar_w6 - 1, bar_top6, bx + bar_w6 - 1, bar_bot6, SSD1306_COLOR_WHITE);

            // Relleno proporcional al valor
            if (h > 0) SSD1306_DrawFilledRectangle(bx + 1, bar_bot6 - h, bar_w6 - 2, h, SSD1306_COLOR_WHITE);

            // Valor numérico debajo
            char buf[6];
            snprintf(buf, sizeof(buf), "%4u", (unsigned)v);
            uint16_t vx = bx;
            for (char *p = buf; *p; p++) {
                SSD1306_DrawChar5x7(*p, vx, 57);
                vx += (uint16_t)(Font_5x7.FontWidth + 1);
            }
        }

        // --- Barra derecha: ADC7 (adcAvg[6]) — sensor lateral de pared ---
        {
            const uint16_t bx = 64 + (64 - bar_w6) / 2;   // 76
            uint16_t v = adcAvg[6] > 4095 ? 4095 : adcAvg[6];
            uint16_t h = (uint32_t)v * bar_max_h6 / 4095;

            // Label "A7" centrado sobre la barra
            uint16_t lx = bx + (bar_w6 - 2 * (Font_5x7.FontWidth + 1)) / 2;
            SSD1306_DrawChar5x7('A', lx, 15);
            SSD1306_DrawChar5x7('7', lx + Font_5x7.FontWidth + 1, 15);

            // Borde de la barra
            SSD1306_DrawLine(bx,              bar_top6, bx + bar_w6 - 1, bar_top6, SSD1306_COLOR_WHITE);
            SSD1306_DrawLine(bx,              bar_bot6, bx + bar_w6 - 1, bar_bot6, SSD1306_COLOR_WHITE);
            SSD1306_DrawLine(bx,              bar_top6, bx,              bar_bot6, SSD1306_COLOR_WHITE);
            SSD1306_DrawLine(bx + bar_w6 - 1, bar_top6, bx + bar_w6 - 1, bar_bot6, SSD1306_COLOR_WHITE);

            // Relleno proporcional al valor
            if (h > 0) SSD1306_DrawFilledRectangle(bx + 1, bar_bot6 - h, bar_w6 - 2, h, SSD1306_COLOR_WHITE);

            // Valor numérico debajo
            char buf[6];
            snprintf(buf, sizeof(buf), "%4u", (unsigned)v);
            uint16_t vx = bx;
            for (char *p = buf; *p; p++) {
                SSD1306_DrawChar5x7(*p, vx, 57);
                vx += (uint16_t)(Font_5x7.FontWidth + 1);
            }
        }

    } else {
        SSD1306_GotoXY(30, 25);
        SSD1306_Puts("DISPLAY?", &Font_7x10, SSD1306_COLOR_WHITE);
    }
    SSD1306_RequestUpdate();
}

// Recupera el bus I2C cuando el DMA queda colgado sin completar la transferencia.
// Misma secuencia de 9 pulsos SCL que se usa en el boot; tarda ~20 ms.
static void I2C1_Recover(void)
{
    IWDG->KR = 0xAAAAU;  // patear antes de los HAL_Delay

    HAL_DMA_Abort(&hdma_i2c1_rx);
    HAL_DMA_Abort(&hdma_i2c1_tx);

    GPIO_InitTypeDef gi = {0};
    HAL_I2C_DeInit(&hi2c1);

    gi.Pin   = GPIO_PIN_8 | GPIO_PIN_9;
    gi.Mode  = GPIO_MODE_OUTPUT_OD;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gi);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(2);

    for (uint8_t i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   HAL_Delay(1);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET) break;
    }
    // Condición STOP
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);   HAL_Delay(2);

    gi.Mode      = GPIO_MODE_AF_OD;
    gi.Alternate = GPIO_AF4_I2C1;
    gi.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &gi);
    HAL_I2C_Init(&hi2c1);

    I2C_Manager_Init();
    i2c1_tx_busy    = 0;
    mpu_req_pending = 0;

    USB_DebugStr("I2C RECOVER\r\n");
}

static void ControlStep10ms(void)
{
		if (!MPU6050_IsDataReady()) return;
		MPU6050_ClearDataReady();

		MPU6050_GetAccel(&ax, &ay, &az);
		MPU6050_GetGyro(&gx, &gy, &gz);

		// Métrica de "se está moviendo/manipulando con la mano": suma de |delta|
		// del acelerómetro crudo entre ciclos, suavizada. Quieto sobre una
		// superficie (aunque no vea la línea) da un valor bajo y estable; en la
		// mano, aunque se intente sostener firme, siempre hay variación sostenida.
		// Usado para distinguir "lo levantaron" de "está parado sobre blanco".
		static int16_t accel_prev_x = 0, accel_prev_y = 0, accel_prev_z = 0;
		static uint8_t accel_prev_init = 0;
		if (!accel_prev_init) {
			accel_prev_x = ax; accel_prev_y = ay; accel_prev_z = az;
			accel_prev_init = 1;
		}
		float accel_delta = fabsf((float)(ax - accel_prev_x)) +
		                     fabsf((float)(ay - accel_prev_y)) +
		                     fabsf((float)(az - accel_prev_z));
		accel_motion_f += 0.30f * (accel_delta - accel_motion_f);
		accel_prev_x = ax; accel_prev_y = ay; accel_prev_z = az;

		// Encoder speed — leer conteos acumulados desde IRQ y calcular velocidad
		static int32_t enc_right_prev = 0;
		static int32_t enc_left_prev  = 0;
		__disable_irq();
		int32_t enc_r = encoder_right;
		int32_t enc_l = encoder_left;
		__enable_irq();
		int32_t delta_right = enc_r - enc_right_prev;
		int32_t delta_left  = enc_l - enc_left_prev;
		enc_right_prev = enc_r;
		enc_left_prev  = enc_l;
		#define ENC_CPR        28  // 4x quadrature: ambos canales A y B, RISING+FALLING
		#define ENC_VEL_SCALE  0.17750f  // 2π × r_rueda (radio 2.825 cm) → m/s
		float speed_right_rps = (float)delta_right / (ENC_CPR * DT_CTRL_FIXED);
		float speed_left_rps  = (float)delta_left  / (ENC_CPR * DT_CTRL_FIXED);
		float vel_enc = -((speed_right_rps + speed_left_rps) * 0.5f) * ENC_VEL_SCALE;
		vel_enc = clampf_local(vel_enc, -20.0f, 20.0f);
		velocity_est    = vel_enc;
		velocity_est_f += VEL_LPF_BETA * (velocity_est - velocity_est_f);
		speed_right_rps_s = speed_right_rps;
		speed_left_rps_s  = speed_left_rps;

		// Steering PID de lazo cerrado: disponible mientras speed_r/l están en scope.
		// steer_correction se usa en la sección de motores BALANCE más adelante.
		static float steer_correction = 0.0f;
		if (steer_pid_enabled) {
		    steer_correction = ComputeSteeringPID(speed_right_rps, speed_left_rps, 0.0f);
		} else {
		    steer_correction = 0.0f;
		    SteeringPID_Reset();
		}

		// -------------------------------------------------------
		// TIMING — usar timestamp real del IRQ del MPU
		// -------------------------------------------------------
		static uint32_t last_mpu_us = 0;

		uint32_t sample_us = mpu_irq_timestamp_us;
		const float dt_ctrl = DT_CTRL_FIXED;

		if (last_mpu_us == 0) {
		    last_mpu_us = sample_us;
		    dt_real = DT_CTRL_FIXED;
		    return;   // primer ciclo: todavía no hay dt válido
		}

		uint32_t diff_us = sample_us - last_mpu_us;
		last_mpu_us = sample_us;

		dt_real = (float)diff_us * 1e-6f;

        // El control usa un dt fijo para desacoplar el PID del jitter del sensor y de las latencias.
        const uint8_t late_cycle = 0;  // ya no tiene sentido detectar ciclos tardíos

        // -------------------------------------------------------
        // FILTRO IMU
        // -------------------------------------------------------
        const float ANG_SIGN = +1.0f;

        float gyro_rate_dps = ANG_SIGN * ((float)gx / 100.0f);
        if (gyro_rate_dps >  250.0f) gyro_rate_dps =  250.0f;
        if (gyro_rate_dps < -250.0f) gyro_rate_dps = -250.0f;

        gyro_f = gyro_rate_dps;
        if (gyro_f >  180.0f) gyro_f =  180.0f;
        if (gyro_f < -180.0f) gyro_f = -180.0f;

        float accel_ang_deg = ANG_SIGN * (atan2f((float)ay, (float)az) * (180.0f / M_PI));

        // Inicialización del filtro en la primera muestra — evita que arranque en 0°
        static uint8_t filter_init = 0;
        if (!filter_init) {
            filtered_roll_deg = accel_ang_deg;
            filter_init = 1;
        }

        // Filtro complementario (fusión gyro + accel)
        filtered_roll_deg = ALPHA * (filtered_roll_deg + gyro_f * dt_ctrl)
                          + (1.0f - ALPHA) * accel_ang_deg;

        if (filtered_roll_deg >  180.0f) filtered_roll_deg =  180.0f;
        if (filtered_roll_deg < -180.0f) filtered_roll_deg = -180.0f;

        // -------------------------------------------------------
        // LINE FOLLOWER INPUTS
        // -------------------------------------------------------
        float line_error = 0.0f;
        float line_angle_cmd = LINE_ANGLE;
        float line_desired_forward_vel = 0.0f;
        float line_forward_vel = 0.0f;
        float line_obj_reverse_angle_cmd = 0.0f;
        uint8_t line_detected = 0;
        float w_sum = 0.0f;
        if (robot_state == ROBOT_STATE_LINE_FOLLOWING) {
            // Detección de objetos: sensores de largo alcance ADC 5-8 (índices 4-7).
            // Si cualquier sensor supera el umbral durante OBJ_DETECT_DEBOUNCE_CNT ciclos
            // consecutivos (30 ms), se sale a IDLE y los motores se apagan.
            {
                static uint8_t obj_cnt = 0;
                uint8_t obj_now = ((float)adcAvg[4] < OBJ_DETECT_THRESHOLD_f ||
                                   (float)adcAvg[5] < OBJ_DETECT_THRESHOLD_f ||
                                   (float)adcAvg[6] < OBJ_DETECT_THRESHOLD_f ||
                                   (float)adcAvg[7] < OBJ_DETECT_THRESHOLD_f);
                if (obj_now) { if (obj_cnt < OBJ_DETECT_DEBOUNCE_CNT) obj_cnt++; }
                else         { obj_cnt = 0; }
                if (obj_cnt >= OBJ_DETECT_DEBOUNCE_CNT &&
                    line_state == LINE_STATE_FOLLOWING &&
                    HAL_GetTick() >= obj_detect_ignore_until_ms) {
                    line_state           = LINE_STATE_OBJ_PRE_REVERSE_HOLD;
                    obj_pre_rev_start_ms = HAL_GetTick();
                    obj_cnt              = 0;
                    steering_adjustment = 0.0f;
                    line_integral       = 0.0f;
                    line_obj_rev_vel_integral = 0.0f;
                    line_error_prev     = 0.0f;
                    line_error_f_d      = 0.0f;
                    obj_rev_initialized = 0;
                    obj_rot_initialized = 0;
                    obj_rot_phase       = 0;
                    obj_rot_heading     = 0.0f;
                    obj_rot_phase1_ms   = 0;
                }
            }

            // Usa adcAvg[] (promedio de 15 muestras a 4 kHz = ventana 3.75 ms) en lugar
            // del snapshot DMA crudo. Evita el retraso del filtro EMA que impedía
            // alcanzar LINE_THRESHOLD en el primer ciclo sobre la línea.
            // Centroide cuadrático con sustracción de baseline.
            // Sensores a posiciones relativas {-3,-1,+1,+3} × 5.75 mm desde el centro.
            // Peso cuadrático: pos² × sign(pos) → coeficientes {-9,-1,+1,+9}.
            // Normalización: 9 × w_sum → error en [-1, +1].
            // Convención: izquierda = positivo, derecha = negativo.
            float s[4] = {
                (float)adcAvg[0], (float)adcAvg[1],
                (float)adcAvg[2], (float)adcAvg[3]
            };

            // Sustracción de baseline: peso = señal por encima del fondo de cada sensor.
            // Si el ADC no supera el umbral, contribuye 0 (ruido/luz ambiente).
            float w[4];
            for (int i = 0; i < 4; i++) {
                float signal = s[i] - ADC_BASELINE[i];
                w[i] = (s[i] > LINE_THRESHOLD) ? fmaxf(signal, 0.0f) : 0.0f;
            }
            w_sum = w[0] + w[1] + w[2] + w[3];

            line_detected = (w_sum > 0.0f);

            // Detección de "en el aire": todos los 4 sensores en negro Y el
            // acelerómetro variando constantemente (= lo tienen en la mano).
            // Si los 4 sensores están en negro pero el acelerómetro está quieto,
            // significa que el robot está parado sobre el piso — NO se detienen
            // los motores. Dos sub-casos según esté siguiendo línea o buscando:
            // (a) en LINE_STATE_FOLLOWING, los 4 sensores en negro a la vez con
            //     el robot quieto/sin manipular significa que está cruzando una
            //     línea PERPENDICULAR a su dirección de avance (ej. una "+"): la
            //     franja negra cubre los 4 sensores de punta a punta. Dispara un
            //     giro instantáneo de 90° (LINE_STATE_PERP_ROTATE) para alinearse
            //     y retomar el seguimiento normal. Debounce corto (50ms) para
            //     filtrar ruido de un solo ciclo, no los 2s de "en el aire".
            // (b) en cualquier otro sub-estado (buscando la línea), no hace nada
            //     especial — sigue el flujo normal de búsqueda (LOST/EDGE).
            // Umbral de movimiento (ACCEL_MOTION_THRESHOLD) es un punto de
            // partida, ajustar según pruebas físicas (unidades: LSB crudos del
            // MPU6050 por ciclo, EMA). "En el aire" sigue necesitando >2s.
            // Cuando baja al piso (all_black→0) → ignorar obstáculos 3s (mano al soltar).
            {
                const float ACCEL_MOTION_THRESHOLD = 300.0f;
                const uint32_t PERP_DEBOUNCE_MS = 50U;
                uint8_t all_black = (adcAvg[0] > (uint16_t)LINE_THRESHOLD &&
                                     adcAvg[1] > (uint16_t)LINE_THRESHOLD &&
                                     adcAvg[2] > (uint16_t)LINE_THRESHOLD &&
                                     adcAvg[3] > (uint16_t)LINE_THRESHOLD);
                uint8_t accel_moving = (accel_motion_f > ACCEL_MOTION_THRESHOLD);
                if (all_black) {
                    if (all_black_start_ms == 0) all_black_start_ms = HAL_GetTick();
                    uint32_t all_black_elapsed = HAL_GetTick() - all_black_start_ms;
                    if (accel_moving) {
                        if (all_black_elapsed > 2000U) f_in_air = 1;
                    } else {
                        f_in_air = 0;
                        if (line_state == LINE_STATE_FOLLOWING &&
                            all_black_elapsed > PERP_DEBOUNCE_MS) {
                            line_state          = LINE_STATE_PERP_ROTATE;
                            obj_rot_initialized = 0;
                            obj_rot_phase       = 0;
                            obj_rot_heading     = 0.0f;
                            obj_rot_phase1_ms   = 0;
                            obj_rot_start_ms    = 0;
                            steering_adjustment = 0.0f;
                        }
                    }
                } else {
                    all_black_start_ms = 0;
                    f_in_air = 0;
                }
                if (prev_all_line_black && !all_black)
                    obj_detect_ignore_until_ms = HAL_GetTick() + 3000U;
                prev_all_line_black = all_black;
            }

            if (line_detected) {
                // Numerador cuadrático: coeficientes {+9,+1,-1,-9} (izq=positivo)
                float num = 9.0f*w[0] + 1.0f*w[1] - 1.0f*w[2] - 9.0f*w[3];
                line_error = num / (9.0f * w_sum) + line_error_trim_f;

                // Umbral más bajo porque sensores internos solo generan ±0.11
                if (line_error > 0.02f)       last_line_dir =  1.0f;
                else if (line_error < -0.02f) last_line_dir = -1.0f;

                // Snapshot literal: ¿el único sensor con señal fue un extremo, sin
                // ningún soporte de los sensores del medio? Se congela en su último
                // valor al perder la línea (este bloque solo corre con line_detected).
                last_detected_edge_only = ((w[0] > 0.0f || w[3] > 0.0f) &&
                                            w[1] <= 0.0f && w[2] <= 0.0f);
            }
            line_error_disp = line_error;

            // Velocidad deseada cae cuadráticamente con el error de línea.
            // Floor 10%: en curva cerrada el robot frena casi al mínimo (era 25%).
            // Sin floor: speed_factor→0 → line_angle_cmd=0 → pwm_sat→0 → spin puro.
            float speed_factor = fmaxf(0.0f, 1.0f - fabsf(line_error) / 0.45f);
            speed_factor *= speed_factor;
            line_desired_forward_vel = line_detected
                ? fmaxf(LINE_SPEED_TARGET * 0.20f, LINE_SPEED_TARGET * speed_factor)
                : 0.0f;
            // 2026-07-02: deadband aplicado (mismo fix que LOST_FWD/EDGE_FWD) — sin esto,
            // un solo pulso de encoder en un ciclo (~0.32-0.63 m/s, ver BRAKE_VEL_DEADBAND)
            // se leía como sobrevelocidad real y disparaba el freno fuerte (LINE_VEL_KP_BRAKE)
            // constantemente, aunque la velocidad real estuviera cerca del target. Esto
            // pasaba pese a bajar sp_step_max, porque el disparador se repetía todo el
            // tiempo por ruido, no por una rampa demasiado rápida.
            line_forward_vel = apply_deadbandf(fmaxf(0.0f, -velocity_est_f), BRAKE_VEL_DEADBAND);

            // PI de velocidad → inclinación de avance
            if (line_state == LINE_STATE_FOLLOWING) {
                float vel_error = line_desired_forward_vel - line_forward_vel;
                float line_vel_kp_eff;
                if (vel_error > 0.0f) {
                    // Acelerando: acumula integral solo en positivo
                    line_vel_integral += vel_error * DT_CTRL_FIXED;
                    line_vel_integral = clampf_local(line_vel_integral, 0.0f, LINE_VEL_I_MAX);
                    line_vel_kp_eff = LINE_VEL_KP;
                } else {
                    // Sobre-velocidad: decae la integral (freno no depende del windup)
                    // y usa ganancia P más alta para frenar de verdad, no solo soltar el gas.
                    line_vel_integral *= 0.80f;
                    line_vel_kp_eff = LINE_VEL_KP_BRAKE;
                }
                line_angle_cmd = clampf_local(
                    line_vel_kp_eff * vel_error + LINE_VEL_KI * line_vel_integral,
                    -3.0f, LINE_ANGLE
                );
            } else {
                line_vel_integral *= 0.80f;
                line_angle_cmd = 0.0f;
            }

            if (line_state == LINE_STATE_OBJ_REVERSE) {
                float reverse_speed_target = clampf_local(
                    LINE_SPEED_TARGET * 0.25f,
                    LINE_OBJ_REV_SPEED_MIN,
                    LINE_OBJ_REV_SPEED_MAX
                );
                if (line_detected) {
                    float reverse_track_scale = fmaxf(0.35f, speed_factor);
                    reverse_speed_target = fmaxf(LINE_OBJ_REV_SPEED_MIN,
                                                 reverse_speed_target * reverse_track_scale);
                }

                float reverse_vel = fmaxf(0.0f, velocity_est_f);
                float reverse_vel_error = reverse_speed_target - reverse_vel;
                if (reverse_vel_error > 0.0f) {
                    line_obj_rev_vel_integral += reverse_vel_error * DT_CTRL_FIXED;
                    line_obj_rev_vel_integral = clampf_local(
                        line_obj_rev_vel_integral, 0.0f, LINE_VEL_I_MAX);
                } else {
                    line_obj_rev_vel_integral *= 0.80f;
                }

                float reverse_angle_mag = clampf_local(
                    LINE_VEL_KP * reverse_vel_error +
                    LINE_VEL_KI * line_obj_rev_vel_integral,
                    0.0f,
                    LINE_OBJ_REV_TILT_MAX
                );
                line_obj_reverse_angle_cmd = -reverse_angle_mag;
            } else {
                line_obj_rev_vel_integral *= 0.80f;
            }

        } else if ((robot_state == ROBOT_STATE_BALANCE_AND_SPEED) ||
                   (robot_state == ROBOT_STATE_BALANCE_ONLY)) {
            // velocity_est ya actualizado desde encoders al inicio del ciclo
        }


        // -------------------------------------------------------
        // CAMBIOS DE ESTADO
        // -------------------------------------------------------
        static eRobotState prev_robot_state   = ROBOT_STATE_IDLE;
        static uint8_t     display_before_line = 0;  // guarda el display previo al modo línea

        if (robot_state == ROBOT_STATE_LINE_FOLLOWING && prev_robot_state != ROBOT_STATE_LINE_FOLLOWING) {
            display_before_line = f_change_display;  // guardar display actual
            f_change_display    = 1;                 // cambiar al display de línea
            integral            = 0.0f;
            line_integral       = 0.0f;
            line_error_prev     = 0.0f;
            line_error_f_d      = 0.0f;
            steering_adjustment = 0.0f;
            velocity_est        = 0.0f;
            velocity_est_f      = 0.0f;
            vel_from_accel      = 0.0f;
            line_vel_integral   = 0.0f;
            line_obj_rev_vel_integral = 0.0f;
            line_enc_angle_corr = 0.0f;
            line_reverse_boost  = 0.0f;
            balance_hold_active = 0;
            line_steer_fb_int   = 0.0f;
            speed_right_rps_s   = 0.0f;
            speed_left_rps_s    = 0.0f;
            line_state          = LINE_STATE_FOLLOWING;
            line_seen_since_entry = 0;
            line_was_centered_on_lost = 1;
            last_detected_edge_only   = 0;
            dynamic_setpoint    = SETPOINT_ANGLE + setpoint_trim;
            dynamic_setpoint_f  = SETPOINT_ANGLE + setpoint_trim;
            base_setpoint_f     = SETPOINT_ANGLE + setpoint_trim;
            brake_setpoint_f    = 0.0f;
            line_lost_ms        = HAL_GetTick();
            line_angle_ramped   = 0.0f;
            obj_pre_rev_start_ms = 0;
        }

        if (robot_state != ROBOT_STATE_LINE_FOLLOWING && prev_robot_state == ROBOT_STATE_LINE_FOLLOWING) {
            f_change_display = display_before_line;  // restaurar display anterior
        }

        // Sub-estado línea: entrar/salir de modo evasión cambia display 1 ↔ 6
        {
            static eLineState prev_line_state_disp = LINE_STATE_FOLLOWING;
            if (robot_state == ROBOT_STATE_LINE_FOLLOWING) {
                int prev_obj = (prev_line_state_disp >= LINE_STATE_OBJ_PRE_REVERSE_HOLD);
                int curr_obj = (line_state           >= LINE_STATE_OBJ_PRE_REVERSE_HOLD);
                if (!prev_obj && curr_obj) {
                    f_change_display = 6;  // entrar en evasión → pantalla OBJ
                } else if (prev_obj && !curr_obj) {
                    f_change_display = 1;  // volver al seguidor de línea
                }
            }
            prev_line_state_disp = line_state;
        }

        if (robot_state == ROBOT_STATE_IDLE && prev_robot_state != ROBOT_STATE_IDLE) {
            integral            = 0.0f;
            pwm_sat_prev        = 0.0f;
            dynamic_setpoint    = SETPOINT_ANGLE + setpoint_trim;
            dynamic_setpoint_f  = SETPOINT_ANGLE + setpoint_trim;
            base_setpoint_f     = SETPOINT_ANGLE + setpoint_trim;
            brake_setpoint_f    = 0.0f;
        }

        if ((robot_state == ROBOT_STATE_BALANCE_ONLY ||
             robot_state == ROBOT_STATE_BALANCE_AND_SPEED ||
             robot_state == ROBOT_STATE_MANUAL_CONTROL) &&
            (prev_robot_state != robot_state)) {

            integral            = 0.0f;
            steering_adjustment = 0.0f;
            velocity_est        = 0.0f;
            velocity_est_f      = 0.0f;
            vel_from_accel      = 0.0f;
            dynamic_setpoint    = SETPOINT_ANGLE + setpoint_trim;
            dynamic_setpoint_f  = SETPOINT_ANGLE + setpoint_trim;
            base_setpoint_f     = SETPOINT_ANGLE + setpoint_trim;
            brake_setpoint_f    = 0.0f;
            pwm_sat_prev        = 0.0f;

            if (robot_state == ROBOT_STATE_MANUAL_CONTROL) {
                manual_setpoint_cmd    = 0.0f;
                manual_steering_cmd    = 0.0f;
                manual_cmd_last_ms     = HAL_GetTick();
                manual_setpoint_ramped = 0.0f;
                steering_adjustment    = 0.0f;
                manual_seq_next_ms     = HAL_GetTick() + 2500U; // primera pausa al entrar al modo
                manual_auto_rot_step   = 0;
            }
        }

        prev_robot_state = robot_state;

        // -------------------------------------------------------
        // SETPOINT DINÁMICO
        // -------------------------------------------------------
        float base_setpoint_target  = SETPOINT_ANGLE + setpoint_trim;
        float brake_setpoint_target = 0.0f;

        if (robot_state == ROBOT_STATE_LINE_FOLLOWING) {
            if (line_state == LINE_STATE_FOLLOWING && line_detected) {
                // Regulación de velocidad por encoders:
                // - vel < 0 (avance): reducir ángulo si va muy rápido → frena
                // - vel > 0 (reversa): scale=1 siempre → ángulo completo → empuja hacia adelante
                float vel_fwd = fminf(0.0f, velocity_est_f);
                float vel_scale = fmaxf(0.0f, 1.0f - fabsf(vel_fwd) / (LINE_SPEED_TARGET * 0.9f));
                float stability_scale = 1.0f;
                // Referencia: base_setpoint_f (setpoint actual de línea), no el upright (0°).
                // Sin esto, stability_scale penaliza la inclinación correcta de avance.
                float balance_err_abs = fabsf(filtered_roll_deg - base_setpoint_f);
                float gyro_abs = fabsf(gyro_f);

                if (balance_err_abs > 1.5f)
                    stability_scale *= fmaxf(0.60f, 1.0f - (balance_err_abs - 1.5f) / 3.0f);
                if (gyro_abs > 20.0f)
                    stability_scale *= fmaxf(0.60f, 1.0f - (gyro_abs - 20.0f) / 45.0f);

                // Corrección P directa por déficit de velocidad de encoder:
                // frac=1 cuando el robot está parado, frac=0 cuando alcanza la vel deseada.
                float enc_deficit = fmaxf(0.0f, line_desired_forward_vel - line_forward_vel);
                float enc_deficit_frac = (line_desired_forward_vel > 0.01f)
                    ? clampf_local(enc_deficit / line_desired_forward_vel, 0.0f, 1.0f)
                    : 0.0f;
                line_enc_angle_corr = clampf_local(
                    LINE_ENC_CORR_KP * enc_deficit_frac, 0.0f, LINE_ENC_CORR_MAX);

                float reverse_vel = fmaxf(0.0f, velocity_est_f);
                float reverse_target = 0.0f;
                if (reverse_vel > LINE_REV_VEL_START) {
                    reverse_target = LINE_REV_BOOST_MAX *
                        clampf_local(
                            (reverse_vel - LINE_REV_VEL_START) /
                            (LINE_REV_VEL_FULL - LINE_REV_VEL_START),
                            0.0f, 1.0f
                        );
                    if (stability_scale < 0.75f) stability_scale = 0.75f;
                }

                float reverse_delta = reverse_target - line_reverse_boost;
                float reverse_step = (reverse_delta > 0.0f) ? LINE_REV_BOOST_UP : LINE_REV_BOOST_DOWN;
                if (reverse_delta >  reverse_step) reverse_delta =  reverse_step;
                if (reverse_delta < -reverse_step) reverse_delta = -reverse_step;
                line_reverse_boost += reverse_delta;

                if (line_angle_cmd < 0.0f) {
                    // Freno activo: no aplicar boosts positivos ni vel_scale (que sería 0 y mataría el freno)
                    base_setpoint_target = line_angle_cmd * stability_scale;
                } else {
                    float line_angle_with_boost = clampf_local(
                        line_angle_cmd + line_enc_angle_corr + line_reverse_boost,
                        0.0f, LINE_ANGLE + LINE_ENC_CORR_MAX + LINE_REV_BOOST_MAX
                    );
                    base_setpoint_target = line_angle_with_boost * vel_scale * stability_scale;
                }
            } else if (line_state == LINE_STATE_OBJ_PRE_REVERSE_HOLD) {
                // Hold pre-reversa: balance estatico con freno de encoders para suavizar transicion.
                base_setpoint_target  = SETPOINT_ANGLE + setpoint_trim;
                brake_setpoint_target = ComputeBrakeSetpointTarget(ROBOT_STATE_BALANCE_ONLY);
                line_enc_angle_corr   = 0.0f;
                line_reverse_boost    = 0.0f;
            } else if (line_state == LINE_STATE_OBJ_REVERSE) {
                // Reversa siguiendo linea: PI de velocidad pide inclinacion hacia atras.
                base_setpoint_target  = SETPOINT_ANGLE + setpoint_trim + line_obj_reverse_angle_cmd;
                brake_setpoint_target = 0.0f;
                line_enc_angle_corr   = 0.0f;
                line_reverse_boost    = 0.0f;
            } else if (line_state == LINE_STATE_OBJ_BRAKE) {
                // Frenado post-reversa: upright con freno de encoders hasta velocidad ≈ 0
                base_setpoint_target  = SETPOINT_ANGLE + setpoint_trim;
                brake_setpoint_target = ComputeBrakeSetpointTarget(ROBOT_STATE_BALANCE_ONLY);
                line_enc_angle_corr   = 0.0f;
                line_reverse_boost    = 0.0f;
            } else if (line_state == LINE_STATE_LOST_FWD) {
                // Avance post-180°: control P continuo de velocidad (ver LOST_FWD_* arriba).
                // 2026-07-01: deadband aplicado a lfwd_vel — un solo pulso simultáneo de
                // ambas ruedas ya lee ~0.63 m/s (por encima del target 0.45) aunque el
                // robot esté prácticamente quieto, disparando el freno fuerte y haciendo
                // que retroceda. Mismo BRAKE_VEL_DEADBAND que usa ComputeBrakeSetpointTarget.
                {
                    float lfwd_vel_raw = fmaxf(0.0f, -velocity_est_f);
                    float lfwd_vel     = apply_deadbandf(lfwd_vel_raw, BRAKE_VEL_DEADBAND);
                    float vel_error    = LOST_FWD_SPEED_TARGET - lfwd_vel;
                    float kp           = (vel_error < 0.0f) ? LOST_FWD_KP_BRAKE : LOST_FWD_KP;
                    base_setpoint_target = clampf_local(kp * vel_error,
                                                        -LOST_FWD_BRAKE_MAX, LOST_FWD_ANGLE_MAX);
                }
                brake_setpoint_target = 0.0f;
                line_enc_angle_corr   = 0.0f;
                line_reverse_boost    = 0.0f;
            } else if (line_state == LINE_STATE_LOST_BRAKE) {
                // Frenado previo al giro 180°: upright con freno de encoders MÁS INTENSO
                // que el freno estándar de balance (LOST_BRAKE_BOOST), para no alejarse
                // tanto de donde se perdió la línea antes de empezar a buscar. No se
                // sube el freno estándar de ComputeBrakeSetpointTarget en general porque
                // eso reintroduciría la oscilación en balance común (ver fix anterior).
                base_setpoint_target  = SETPOINT_ANGLE + setpoint_trim;
                brake_setpoint_target = clampf_local(
                    ComputeBrakeSetpointTarget(ROBOT_STATE_BALANCE_ONLY) * LOST_BRAKE_BOOST,
                    -BRAKE_TILT_MAX, BRAKE_TILT_MAX);
                line_enc_angle_corr   = 0.0f;
                line_reverse_boost    = 0.0f;
            } else if (line_state == LINE_STATE_LOST_SETTLE) {
                // Estabilización post-180°: upright con freno de encoders, sin avance
                base_setpoint_target  = SETPOINT_ANGLE + setpoint_trim;
                brake_setpoint_target = ComputeBrakeSetpointTarget(ROBOT_STATE_BALANCE_ONLY);
                line_enc_angle_corr   = 0.0f;
                line_reverse_boost    = 0.0f;
            } else if (line_state == LINE_STATE_EDGE_WAIT) {
                // Frenado previo al giro (perdida por un extremo), mismo boost que LOST_BRAKE.
                base_setpoint_target  = SETPOINT_ANGLE + setpoint_trim;
                brake_setpoint_target = clampf_local(
                    ComputeBrakeSetpointTarget(ROBOT_STATE_BALANCE_ONLY) * LOST_BRAKE_BOOST,
                    -BRAKE_TILT_MAX, BRAKE_TILT_MAX);
                line_enc_angle_corr   = 0.0f;
                line_reverse_boost    = 0.0f;
            } else if (line_state == LINE_STATE_EDGE_SETTLE) {
                // Estabilización post-90°: upright con freno de encoders, sin avance
                base_setpoint_target  = SETPOINT_ANGLE + setpoint_trim;
                brake_setpoint_target = ComputeBrakeSetpointTarget(ROBOT_STATE_BALANCE_ONLY);
                line_enc_angle_corr   = 0.0f;
                line_reverse_boost    = 0.0f;
            } else if (line_state == LINE_STATE_EDGE_FWD) {
                // Avance post-giro: mismo control P continuo que LOST_FWD, mismo deadband.
                {
                    float efwd_vel_raw = fmaxf(0.0f, -velocity_est_f);
                    float efwd_vel     = apply_deadbandf(efwd_vel_raw, BRAKE_VEL_DEADBAND);
                    float vel_error    = LOST_FWD_SPEED_TARGET - efwd_vel;
                    float kp           = (vel_error < 0.0f) ? LOST_FWD_KP_BRAKE : LOST_FWD_KP;
                    base_setpoint_target = clampf_local(kp * vel_error,
                                                        -LOST_FWD_BRAKE_MAX, LOST_FWD_ANGLE_MAX);
                }
                brake_setpoint_target = 0.0f;
                line_enc_angle_corr   = 0.0f;
                line_reverse_boost    = 0.0f;
            } else if (line_state == LINE_STATE_GIVEN_UP) {
                // Reposo total: ni el giro de 180 ni el de 90 encontraron la línea.
                // Upright con freno de encoders, sin ningún intento de búsqueda.
                base_setpoint_target  = SETPOINT_ANGLE + setpoint_trim;
                brake_setpoint_target = ComputeBrakeSetpointTarget(ROBOT_STATE_BALANCE_ONLY);
                line_enc_angle_corr   = 0.0f;
                line_reverse_boost    = 0.0f;
            } else if (line_state == LINE_STATE_OBJ_ROTATE ||
                       line_state == LINE_STATE_LOST_ROTATE ||
                       line_state == LINE_STATE_EDGE_ROTATE ||
                       line_state == LINE_STATE_PERP_ROTATE) {
                // Rotación: upright sin avance, sin freno de encoders
                base_setpoint_target  = SETPOINT_ANGLE + setpoint_trim;
                brake_setpoint_target = 0.0f;
                line_enc_angle_corr   = 0.0f;
                line_reverse_boost    = 0.0f;
            } else if (line_state == LINE_STATE_OBJ_HOLD) {
                // Hold post-rotación: upright con freno traslacional, yaw-lock en switch
                base_setpoint_target  = SETPOINT_ANGLE + setpoint_trim;
                brake_setpoint_target = ComputeBrakeSetpointTarget(ROBOT_STATE_BALANCE_ONLY);
                line_enc_angle_corr   = 0.0f;
                line_reverse_boost    = 0.0f;
            } else if (line_state == LINE_STATE_OBJ_WALL_APPROACH) {
                // Avanza despacio buscando la pared; ángulo fijo hasta que ADC7 la detecte.
                base_setpoint_target  = OBJ_WALL_APPROACH_ANGLE;
                brake_setpoint_target = 0.0f;
                line_enc_angle_corr   = 0.0f;
                line_reverse_boost    = 0.0f;
            } else if (line_state == LINE_STATE_OBJ_WALL_FWD) {
                // Avance por pared: angulo fijo para no bambolear por cuantizacion de encoders.
                // Solo frena si la velocidad filtrada supera claramente el rango seguro.
                {
                    // Camino activo: angulo fijo para avanzar recto; freno solo por sobrevelocidad clara.
                    float wf_fwd_vel = fmaxf(0.0f, -velocity_est_f);
                    float wf_cmd = (wf_fwd_vel > OBJ_WALL_OVERSPEED_VEL)
                                 ? -OBJ_WALL_BRAKE_ANGLE
                                 :  OBJ_WALL_FWD_ANGLE;
                    base_setpoint_target = wf_cmd;
                }
                brake_setpoint_target = 0.0f;
                line_enc_angle_corr   = 0.0f;
                line_reverse_boost    = 0.0f;
            } else if (line_state == LINE_STATE_OBJ_WALL_TURN) {
                // Wall-following girando: upright puro, motores controlados por line_pivot_active
                base_setpoint_target  = SETPOINT_ANGLE + setpoint_trim;
                brake_setpoint_target = 0.0f;
                line_enc_angle_corr   = 0.0f;
                line_reverse_boost    = 0.0f;
            } else {
                // Línea perdida (LOST/SEARCHING): upright puro.
                // El robot pasa por LOST un solo ciclo antes de ir a LOST_ROTATE.
                base_setpoint_target  = SETPOINT_ANGLE + setpoint_trim;
                brake_setpoint_target = 0.0f;
                line_enc_angle_corr   = 0.0f;
                line_reverse_boost    = 0.0f;
            }

            // Limitador global de velocidad: frena en CUALQUIER sub-estado si se supera el target.
            // Solo actúa en estados de avance (brake_setpoint_target==0 y base>0) que no tienen
            // su propio control de frenado, y excluye rotaciones donde el pivot no debe frenarse.
            // 2026-07-02: excluido también LINE_STATE_FOLLOWING — ese estado YA regula su
            // propia velocidad con el PI de línea (LINE_VEL_KP/KP_BRAKE/KI), que calcula un
            // ángulo de frenado gradual. Este limitador, al no estar excluido, competía con
            // esa regulación suave: apenas la velocidad cruzaba el 90% del target, TAPABA
            // el ángulo suave del PI con un frenazo a fondo (ComputeBrakeSetpointTarget,
            // pensado para frenar del todo en balance estático) en vez de dejar que el PI
            // termine de regular — eso causaba el "acelera y de golpe frena hasta quedar
            // totalmente quieto" reportado por el usuario en vez de una regulación fluida.
            if (brake_setpoint_target == 0.0f &&
                base_setpoint_target  >  0.0f &&
                line_state != LINE_STATE_FOLLOWING &&
                line_state != LINE_STATE_LOST_ROTATE &&
                line_state != LINE_STATE_OBJ_ROTATE  &&
                line_state != LINE_STATE_EDGE_ROTATE &&
                line_state != LINE_STATE_PERP_ROTATE &&
                line_state != LINE_STATE_OBJ_WALL_TURN) {
                float global_fwd_vel = fmaxf(0.0f, -velocity_est_f);
                if (global_fwd_vel > LINE_SPEED_TARGET * 0.90f) {
                    float overspeed_brake = ComputeBrakeSetpointTarget(ROBOT_STATE_BALANCE_ONLY);
                    if (overspeed_brake < base_setpoint_target)
                        base_setpoint_target = overspeed_brake;
                }
            }

            line_angle_ramped = base_setpoint_target; // mantener variable para telemetría
        } else if (robot_state == ROBOT_STATE_MANUAL_CONTROL) {

            // ── Ciclo automático: 90°der → pausa 2.5 s → 90°izq → pausa 2.5 s → rep ──
            // manual_seq_next_ms == 0  →  desarmado (giro en curso o no iniciado)
            // manual_seq_next_ms != 0  →  armado; dispara cuando HAL_GetTick() >= deadline
            if (!manual_rot_active && !f_fallen &&
                manual_seq_next_ms != 0 &&
                HAL_GetTick() >= manual_seq_next_ms) {
                switch (manual_auto_rot_step) {
                    case 0: manual_rot_target_deg =  90.0f; break;
                    case 1: manual_rot_target_deg = -90.0f; break;
                    case 2: manual_rot_target_deg = 180.0f; break;
                    default: manual_auto_rot_step = 0; manual_rot_target_deg = 90.0f; break;
                }
                manual_auto_rot_step = (manual_auto_rot_step + 1) % 3;
                manual_rot_trigger   = 1;
                manual_seq_next_ms   = 0; // desarmar; se re-arma al terminar el giro
            }

            // ── Giro preciso: arranque por trigger UNER ──────────────────────
            if (manual_rot_trigger && !manual_rot_active && !f_fallen) {
                manual_rot_active   = 1;
                manual_rot_trigger  = 0;
                // seq_next queda en 0 durante toda la rotación
                manual_rot_phase    = 0;
                manual_rot_heading  = 0.0f;
                manual_rot_start_ms = HAL_GetTick();
                __disable_irq();
                manual_rot_enc_r0 = encoder_right;
                manual_rot_enc_l0 = encoder_left;
                __enable_irq();
                manual_setpoint_cmd = 0.0f;
                manual_steering_cmd = 0.0f;
            }

            if (manual_rot_active) {
                float gz_dps = (float)gz / 131.0f;
                manual_rot_heading += gz_dps * DT_CTRL_FIXED;

                // Counts encoder acumulados desde inicio del giro (ambos sentidos)
                __disable_irq();
                int32_t enc_dr = encoder_right - manual_rot_enc_r0;
                int32_t enc_dl = encoder_left  - manual_rot_enc_l0;
                __enable_irq();
                manual_rot_enc_counts = (fabsf((float)enc_dr) + fabsf((float)enc_dl)) * 0.5f;

                float abs_target  = fabsf(manual_rot_target_deg);
                // Heading compuesto: gz (funciona bien CW) + encoder (funciona ambos sentidos)
                float enc_heading_deg = manual_rot_enc_counts * (90.0f / MANUAL_ROT_ENC_TARGET);
                float abs_heading = fmaxf(fabsf(manual_rot_heading), enc_heading_deg);

                uint32_t elapsed_ms = HAL_GetTick() - manual_rot_start_ms;

                // Timeout total de seguridad
                uint32_t timeout_ms = (uint32_t)(abs_target / 90.0f * 3000.0f) + 2000U;

                // Tiempo máximo de fase 0; encoder sale antes según fracción ajustada por ángulo:
                // 90°→0.85 (funciona bien), 180°→0.70 (menos momentum al entrar al freno).
                uint32_t phase0_max_ms = (uint32_t)(abs_target / 90.0f * 1200.0f);
                float enc_exit_frac = 0.85f - fmaxf(0.0f, (abs_target - 90.0f) / 90.0f) * 0.15f;
                float enc_phase0_thr = (abs_target / 90.0f) * MANUAL_ROT_ENC_TARGET * enc_exit_frac;

                // Tiempo máximo de freno fase 1
                uint32_t phase1_max_ms = 300U;

                if (elapsed_ms > timeout_ms) {
                    // Timeout total: abortar
                    manual_rot_active   = 0;
                    manual_rot_phase    = 0;
                    manual_seq_next_ms  = HAL_GetTick() + 2500U;
                    manual_setpoint_cmd = 0.0f;
                    manual_steering_cmd = 0.0f;
                    manual_setpoint_ramped  = SETPOINT_ANGLE + setpoint_trim;
                    steering_adjustment     = 0.0f;
                    manual_cmd_last_ms      = HAL_GetTick();
                    integral                = 0.0f;
                    brake_setpoint_f        = ComputeBrakeSetpointTarget(robot_state);
                } else if (manual_rot_phase == 0 &&
                           (fabsf(manual_rot_heading) >= abs_target * 0.80f ||
                            manual_rot_enc_counts >= enc_phase0_thr ||
                            elapsed_ms >= phase0_max_ms)) {
                    // Al 80% por gz (derecha), al 85% de MANUAL_ROT_ENC_TARGET (ambos), o por tiempo
                    manual_rot_phase    = 1;
                    manual_rot_start_ms = HAL_GetTick();
                } else if (manual_rot_phase == 1) {
                    uint32_t phase1_elapsed = HAL_GetTick() - manual_rot_start_ms;
                    int overshoot = (abs_heading > abs_target * 1.3f);
                    // Salida por velocidad de rueda: cuando las ruedas se detienen → giro completo
                    float rot_vel = (fabsf(speed_right_rps_s) + fabsf(speed_left_rps_s)) * 0.5f;
                    int vel_settled = (rot_vel < 2.0f && phase1_elapsed >= 80U);
                    if (vel_settled || phase1_elapsed >= phase1_max_ms || overshoot) {
                        // Imprimir counts reales por USB para calibrar MANUAL_ROT_ENC_TARGET
                        char _dbg[48];
                        snprintf(_dbg, sizeof(_dbg), "ROT_ENC %.0f dr=%ld dl=%ld\r\n",
                                 manual_rot_enc_counts, (long)enc_dr, (long)enc_dl);
                        USB_DebugStr(_dbg);
                        manual_rot_active   = 0;
                        manual_rot_phase    = 0;
                        manual_seq_next_ms  = HAL_GetTick() + 2500U; // pausa 2.5 s desde el fin
                        manual_setpoint_cmd = 0.0f;
                        manual_steering_cmd     = 0.0f;
                        manual_setpoint_ramped  = SETPOINT_ANGLE + setpoint_trim;
                        steering_adjustment     = 0.0f;
                        manual_cmd_last_ms      = HAL_GetTick();
                        integral                = 0.0f;
                        brake_setpoint_f        = ComputeBrakeSetpointTarget(robot_state);
                    }
                }

                // Durante el giro: upright puro, sin avance ni freno traslacional
                base_setpoint_target  = SETPOINT_ANGLE + setpoint_trim;
                brake_setpoint_target = 0.0f;
                manual_setpoint_ramped = base_setpoint_target;
            } else {
                // Control manual normal
                if (HAL_GetTick() - manual_cmd_last_ms > 60) {
                    manual_setpoint_cmd *= 0.96f;
                    manual_steering_cmd *= 0.90f;
                    if (fabsf(manual_setpoint_cmd) < 0.01f) manual_setpoint_cmd = 0.0f;
                    if (fabsf(manual_steering_cmd) < 0.01f) manual_steering_cmd = 0.0f;
                }

                const float manual_safe_angle = 15.0f;
                const float manual_max_angle  = 35.0f;
                float abs_roll = fabsf(filtered_roll_deg);
                float safety_factor;
                if (abs_roll <= manual_safe_angle) {
                    safety_factor = 1.0f;
                } else if (abs_roll >= manual_max_angle) {
                    safety_factor = 0.0f;
                } else {
                    safety_factor = 1.0f - ((abs_roll - manual_safe_angle) / (manual_max_angle - manual_safe_angle));
                }

                float scaled_cmd = manual_setpoint_cmd * safety_factor;
                const float conflict_threshold = 10.0f;
                if (filtered_roll_deg >  conflict_threshold && scaled_cmd > 0.0f) scaled_cmd = 0.0f;
                if (filtered_roll_deg < -conflict_threshold && scaled_cmd < 0.0f) scaled_cmd = 0.0f;

                const float RAMP_RATE_UP   = 0.01f;
                const float RAMP_RATE_DOWN = 0.008f;
                float ramp_target = SETPOINT_ANGLE + setpoint_trim + scaled_cmd;
                float ramp_delta  = ramp_target - manual_setpoint_ramped;
                float ramp_rate   = (ramp_delta > 0.0f) ? RAMP_RATE_UP : RAMP_RATE_DOWN;
                if (fabsf(ramp_delta) <= ramp_rate) {
                    manual_setpoint_ramped = ramp_target;
                } else {
                    manual_setpoint_ramped += (ramp_delta > 0.0f) ? ramp_rate : -ramp_rate;
                }

                base_setpoint_target  = manual_setpoint_ramped;
                brake_setpoint_target = ComputeBrakeSetpointTarget(robot_state);
            }
        } else if (robot_state == ROBOT_STATE_BALANCE_AND_SPEED) {
            base_setpoint_target = SETPOINT_ANGLE + setpoint_trim;
        } else if (robot_state == ROBOT_STATE_BALANCE_ONLY) {
            // BALANCE_ONLY: tilt setpoint against velocity to brake post-push drift
            base_setpoint_target  = SETPOINT_ANGLE + setpoint_trim;
            brake_setpoint_target = ComputeBrakeSetpointTarget(robot_state);
        } else {
            base_setpoint_target = SETPOINT_ANGLE + setpoint_trim;
        }

        float sp_limit = (robot_state == ROBOT_STATE_BALANCE_AND_SPEED) ? 2.0f : 5.0f;
        base_setpoint_target = clampf_local(base_setpoint_target,
                                            -sp_limit - brake_setpoint_target,
                                             sp_limit - brake_setpoint_target);

        dynamic_setpoint = base_setpoint_target + brake_setpoint_target;
        if (dynamic_setpoint >  sp_limit) dynamic_setpoint =  sp_limit;
        if (dynamic_setpoint < -sp_limit) dynamic_setpoint = -sp_limit;

        {
            static float prev_setpoint_trim = 0.0f;
            float trim_delta = setpoint_trim - prev_setpoint_trim;
            const float TRIM_STEP = 0.008f;
            if (trim_delta >  TRIM_STEP) trim_delta =  TRIM_STEP;
            if (trim_delta < -TRIM_STEP) trim_delta = -TRIM_STEP;
            if (trim_delta != 0.0f) {
                prev_setpoint_trim += trim_delta;
                base_setpoint_f += trim_delta;
            }
        }

        {
            float sp_step_max;

            if (robot_state == ROBOT_STATE_MANUAL_CONTROL) {
                sp_step_max = 0.1f;
            } else if (robot_state == ROBOT_STATE_LINE_FOLLOWING) {
                sp_step_max = 0.15f;
            } else {
                sp_step_max = 0.1f;
            }

            float sp_delta = base_setpoint_target - base_setpoint_f;

            if (sp_delta >  sp_step_max) sp_delta =  sp_step_max;
            if (sp_delta < -sp_step_max) sp_delta = -sp_step_max;

            base_setpoint_f += sp_delta;
        }

        {
            float brake_step_max = (robot_state == ROBOT_STATE_MANUAL_CONTROL)
                                 ? BRAKE_TILT_STEP_MAN
                                 : BRAKE_TILT_STEP_BAL;

            float brake_delta = brake_setpoint_target - brake_setpoint_f;
            if (brake_delta >  brake_step_max) brake_delta =  brake_step_max;
            if (brake_delta < -brake_step_max) brake_delta = -brake_step_max;

            brake_setpoint_f += brake_delta;
        }

        // Término de posición: integra velocidad para corregir deriva lenta
        dynamic_setpoint_f = base_setpoint_f + brake_setpoint_f;
        dynamic_setpoint_f = clampf_local(dynamic_setpoint_f, -sp_limit, sp_limit);
        // En modo linea: FOLLOWING frena hasta -3.0° (antes -1.5, freno más agresivo
        // 2026-07-01); OBJ_REVERSE permite inclinar mas hacia atras.
        if (robot_state == ROBOT_STATE_LINE_FOLLOWING) {
            float line_negative_limit = (line_state == LINE_STATE_OBJ_REVERSE)
                                      ? LINE_OBJ_REV_TILT_MAX
                                      : 3.0f;
            if (dynamic_setpoint_f < -line_negative_limit)
                dynamic_setpoint_f = -line_negative_limit;
        }
        brake_setpoint_f   = dynamic_setpoint_f - base_setpoint_f;

        // -------------------------------------------------------
        // FALL DETECTION
        // -------------------------------------------------------
        float abs_roll_filt = fabsf(filtered_roll_deg);
        float abs_roll_raw  = fabsf(accel_ang_deg);

        uint8_t upright_now     = (abs_roll_raw < RECOVER_ANGLE);
        uint8_t upside_down_now = (abs_roll_raw > UPSIDE_DOWN_ANGLE);

        static uint8_t dead_zone_count = 0;

        if (abs_roll_raw >= FALL_ANGLE) {
            if (dead_zone_count < 10) dead_zone_count++;
        } else {
            dead_zone_count = 0;
        }

        uint8_t in_dead_zone = (dead_zone_count >= 10);

        if (abs_roll_filt > FALL_ANGLE) {
            if (fall_count < 5) fall_count++;
        } else {
            fall_count = 0;
        }
        uint8_t fall_by_angle = (fall_count >= 3);

        if (upright_now) {
            if (upright_count < 20) upright_count++;
        } else {
            upright_count = 0;
        }
        uint8_t recover_by_angle = (upright_count >= 5);

        if (upside_down_now) {
            if (upside_down_count < 20) upside_down_count++;
        } else {
            upside_down_count = 0;
        }
        uint8_t fall_upside_down = (upside_down_count >= 5);

        if (!f_fallen) {
            if (fall_by_angle || fall_upside_down || in_dead_zone) {
                f_fallen = 1;
                integral            = 0.0f;
                balance_hold_active = 0;
                velocity_est        = 0.0f;
                velocity_est_f      = 0.0f;
                vel_from_accel      = 0.0f;
                line_integral       = 0.0f;
                line_obj_rev_vel_integral = 0.0f;
                line_error_prev     = 0.0f;
                line_error_f_d      = 0.0f;
                steering_adjustment = 0.0f;
                gyro_f              = 0.0f;
                motorRightVelocity  = 0;
                motorLeftVelocity   = 0;
                dynamic_setpoint    = SETPOINT_ANGLE + setpoint_trim;
                dynamic_setpoint_f  = SETPOINT_ANGLE + setpoint_trim;
                base_setpoint_f     = SETPOINT_ANGLE + setpoint_trim;
                brake_setpoint_f    = 0.0f;
                pwm_sat_prev        = 0.0f;
                obj_rev_initialized = 0;
                obj_rot_initialized = 0;
                obj_rot_start_ms    = 0;
                obj_rot_phase       = 0;
                obj_rot_heading     = 0.0f;
                obj_rot_phase1_ms   = 0;
                obj_brake_start_ms   = 0;
                obj_pre_rotate_ms    = 0;
                obj_hold_start_ms    = 0;
                obj_rev_straight_int = 0.0f;
                obj_arc_steer_int    = 0.0f;
                obj_wall_approach_start_ms = 0;
                obj_wall_fwd_start_ms      = 0;
                lrot_brake_start_ms        = 0;
                lrot_settle_start_ms       = 0;
                edge_wait_start_ms         = 0;
                // 2026-07-02: si se cayó mientras esquivaba un obstáculo (cualquier
                // LINE_STATE_OBJ_*, que son los últimos del enum, de ahí el ">="),
                // abandona la evasión y vuelve directo a seguir línea normal en vez
                // de retomar la secuencia de esquive donde había quedado. Ignora
                // detección de objeto 5s (mismo patrón ya usado en otras transiciones
                // de salida de evasión) por si el objeto sigue ahí.
                if (line_state >= LINE_STATE_OBJ_PRE_REVERSE_HOLD) {
                    line_state                 = LINE_STATE_FOLLOWING;
                    line_seen_since_entry      = 0;
                    line_was_centered_on_lost  = 1;
                    last_detected_edge_only    = 0;
                    obj_detect_ignore_until_ms = HAL_GetTick() + 5000U;
                }
            }
        } else {
            if (recover_by_angle && !fall_upside_down && !in_dead_zone) {
                f_fallen = 0;
                accel_roll_f      = accel_ang_deg;
                filtered_roll_deg = accel_ang_deg;
                integral                  = 0.0f;
                balance_hold_active       = 0;
                velocity_est              = 0.0f;
                velocity_est_f            = 0.0f;
                vel_from_accel            = 0.0f;
                line_integral             = 0.0f;
                line_obj_rev_vel_integral = 0.0f;
                line_error_prev           = 0.0f;
                line_error_f_d            = 0.0f;
                steering_adjustment       = 0.0f;
                dynamic_setpoint    = SETPOINT_ANGLE + setpoint_trim;
                dynamic_setpoint_f  = SETPOINT_ANGLE + setpoint_trim;
                base_setpoint_f     = SETPOINT_ANGLE + setpoint_trim;
                brake_setpoint_f    = 0.0f;
                upright_count       = 0;
                upside_down_count   = 0;
                fall_count          = 0;
                dead_zone_count     = 0;
                pwm_sat_prev        = 0.0f;
                prev_error          = 0.0f;
            } else {
                motorRightVelocity = 0;
                motorLeftVelocity  = 0;
                gyro_f = 0.0f;
                filtered_roll_deg = accel_ang_deg;
                integral           = 0.0f;
				velocity_est       = 0.0f;
				velocity_est_f     = 0.0f;
				vel_from_accel     = 0.0f;
                dynamic_setpoint   = SETPOINT_ANGLE + setpoint_trim;
                dynamic_setpoint_f = SETPOINT_ANGLE + setpoint_trim;
                base_setpoint_f    = SETPOINT_ANGLE + setpoint_trim;
                brake_setpoint_f   = 0.0f;
				steering_adjustment = 0.0f;
            }
        }

        // -------------------------------------------------------
        // PID
        // -------------------------------------------------------
        float error   = dynamic_setpoint_f - filtered_roll_deg;
        float control_error = error;
        float p_term  = 0.0f;
        float i_term  = 0.0f;
        float d_term  = 0.0f;
        float output  = 0.0f;
        float pwm_cmd = 0.0f;
        float pwm_sat = 0.0f;
        uint8_t sat_flag = 0;

        float log_p_line = 0.0f;
        float log_i_line = 0.0f;
        float log_d_line = 0.0f;

        if (!f_fallen) {
            p_term = KP_value * control_error;
            i_term = KI_value * integral;

            float d_error = (control_error - prev_error) / dt_ctrl;
            if (d_error >  500.0f) d_error =  500.0f;
            if (d_error < -500.0f) d_error = -500.0f;
            d_term = KD_value * d_error;

            if (d_term >  15.0f) d_term =  15.0f;
            if (d_term < -15.0f) d_term = -15.0f;

            {
                float abs_error = fabsf(error);
                float abs_gyro  = fabsf(gyro_f);
                float balance_pi_scale = 1.0f;
                float balance_d_scale  = 1.0f;

                // 2026-07-01: también se excluye en LOST_FWD/EDGE_FWD (avance a ciegas
                // post-giro, sin línea detectada) — sin esto, el hold se activaba apenas
                // el ángulo objetivo rampeaba desde 0° (error chico) y silenciaba el PID
                // justo cuando debía empujar al robot hacia adelante: se quedaba quieto.
                if (robot_state == ROBOT_STATE_LINE_FOLLOWING &&
                    (line_detected ||
                     line_state == LINE_STATE_LOST_FWD ||
                     line_state == LINE_STATE_EDGE_FWD))
                    balance_hold_active = 0;
                else if (!balance_hold_active) {
                    if (abs_error <= BALANCE_HOLD_ENTER_ANGLE_DEG &&
                        abs_gyro  <= BALANCE_HOLD_ENTER_GYRO_DPS)
                        balance_hold_active = 1;
                } else {
                    if (abs_error >= BALANCE_HOLD_EXIT_ANGLE_DEG ||
                        abs_gyro  >= BALANCE_HOLD_EXIT_GYRO_DPS)
                        balance_hold_active = 0;
                }

                if (balance_hold_active) {
                    balance_pi_scale = 0.0f;
                    balance_d_scale  = 0.0f;
                    integral *= 0.98f;
                } else {
                    float x = abs_error / SOFT_ZONE_ANGLE_DEG;
                    if (x > 1.0f) x = 1.0f;
                    balance_pi_scale = SOFT_ZONE_MIN_SCALE + (1.0f - SOFT_ZONE_MIN_SCALE) * x;
                    balance_d_scale  = balance_pi_scale;
                }

                p_term *= balance_pi_scale;
                d_term *= balance_d_scale;
            }

            output = p_term + i_term + d_term;

            prev_error = control_error;
            pwm_cmd = output;
            pwm_sat = pwm_cmd;

            if (pwm_sat >  50.0f) { pwm_sat =  50.0f; sat_flag = 1; }
            if (pwm_sat < -50.0f) { pwm_sat = -50.0f; sat_flag = 1; }

            if (robot_state == ROBOT_STATE_LINE_FOLLOWING &&
                line_state == LINE_STATE_FOLLOWING) {
                if (pwm_sat >  40.0f) pwm_sat =  40.0f;
                if (pwm_sat < -40.0f) pwm_sat = -40.0f;
            }

            if (robot_state == ROBOT_STATE_MANUAL_CONTROL) {
                if (pwm_sat >  55.0f) pwm_sat =  55.0f;
                if (pwm_sat < -55.0f) pwm_sat = -55.0f;
            }

            float pwm_limit;
            if (robot_state == ROBOT_STATE_LINE_FOLLOWING) {
                pwm_limit = 40.0f;
            } else {
                pwm_limit = 100.0f;
            }

            {
                float pwm_step_max;

                if (robot_state == ROBOT_STATE_BALANCE_ONLY) {
                    pwm_step_max = 8.0f;
                } else {
                    pwm_step_max = 8.0f;
                }

                float pwm_delta = pwm_sat - pwm_sat_prev;
                if (pwm_delta >  pwm_step_max) pwm_delta =  pwm_step_max;
                if (pwm_delta < -pwm_step_max) pwm_delta = -pwm_step_max;

                pwm_sat = pwm_sat_prev + pwm_delta;
                pwm_sat_prev = pwm_sat;
            }

            if (!late_cycle) {
                if (fabsf(pwm_sat) <= pwm_limit) {
                    integral += control_error * dt_ctrl;
                } else {
                    if (pwm_cmd >  pwm_limit && control_error < 0) integral += control_error * dt_ctrl;
                    else if (pwm_cmd < -pwm_limit && control_error > 0) integral += control_error * dt_ctrl;
                }
            }

            sat_flag = (fabsf(pwm_sat) >= pwm_limit) ? 1 : 0;

            if (integral >  I_MAX) integral =  I_MAX;
            if (integral < -I_MAX) integral = -I_MAX;

            if (robot_state == ROBOT_STATE_BALANCE_ONLY) {
                if (integral >  8.0f) integral =  8.0f;
                if (integral < -8.0f) integral = -8.0f;
            }

            if (robot_state == ROBOT_STATE_LINE_FOLLOWING) {
                if (integral >  8.0f) integral =  8.0f;
                if (integral < -8.0f) integral = -8.0f;
            }

            if (robot_state == ROBOT_STATE_BALANCE_AND_SPEED) {
                if (integral >  15.0f) integral =  15.0f;
                if (integral < -15.0f) integral = -15.0f;
            }

            if (robot_state == ROBOT_STATE_MANUAL_CONTROL) {
                if (integral >  8.0f) integral =  8.0f;
                if (integral < -8.0f) integral = -8.0f;
            }

            if (robot_state == ROBOT_STATE_LINE_FOLLOWING) {
                uint8_t line_pivot_active = 0;

                switch (line_state) {
                    case LINE_STATE_FOLLOWING:
                    {
                        if (line_detected) {
                            line_seen_since_entry = 1;
                            line_lost_ms = HAL_GetTick();

                            float p_line = KP_LINE * line_error;

                            if (!late_cycle) {
                                line_integral += line_error * dt_ctrl;
                            }

                            if (line_integral >  5.0f) line_integral =  5.0f;
                            if (line_integral < -5.0f) line_integral = -5.0f;

                            float i_line = KI_LINE * line_integral;

                            line_error_f_d += 0.4f * (line_error - line_error_f_d);

                            float d_line = 0.0f;
                            if (!late_cycle) {
                                float line_delta = line_error_f_d - line_error_prev;
                                if (line_delta >  0.3f) line_delta =  0.3f;
                                if (line_delta < -0.3f) line_delta = -0.3f;
                                d_line = KD_LINE * (line_delta / dt_ctrl);
                            }
                            line_error_prev = line_error_f_d;

                            float steering_cmd = p_line + i_line + d_line;

                            // Steering directo: proporcional al error de línea sin inner PI.
                            // El inner PI de encoders generaba problemas si el signo de
                            // diff_actual no coincidía con la convención esperada.
                            steering_adjustment = clampf_local(steering_cmd, -20.0f, 20.0f);

                            log_p_line = p_line;
                            log_i_line = i_line;
                            log_d_line = d_line;

                        } else {
                            if (!line_seen_since_entry) {
                                // Todavía no adquirió la línea desde que se activó el modo:
                                // mejor quedarse estable que disparar una búsqueda ciega.
                                line_lost_ms = HAL_GetTick();
                                line_integral = 0.0f;
                                steering_adjustment = 0.0f;
                                break;
                            }
                            uint32_t ms_sin_linea = HAL_GetTick() - line_lost_ms;

                            // Sin línea: conservar steering_adjustment (último valor conocido)
                            // para que el robot siga corrigiendo hacia la línea.

                            if (ms_sin_linea > 1000) {
                                line_lost_entered_ms = HAL_GetTick();
                                line_search_dir      = last_line_dir;
                                line_integral        = 0.0f;
                                steering_adjustment  = 0.0f;
                                // Snapshot: ¿el último sensor con señal fue un extremo sin
                                // soporte del medio, o vino acompañado/era del medio?
                                line_was_centered_on_lost = !last_detected_edge_only;
                                last_detected_edge_only   = 0;
                                // Centrado → secuencia de giro 180° (LOST/LOST_BRAKE/LOST_ROTATE).
                                // Por un extremo (curva) → secuencia de giro 90° hacia ese lado
                                // (EDGE_WAIT/EDGE_ROTATE/EDGE_FWD).
                                line_state = line_was_centered_on_lost
                                           ? LINE_STATE_LOST
                                           : LINE_STATE_EDGE_WAIT;
                                lrot_brake_start_ms = 0;
                            }
                        }

                        if (steering_adjustment >  20.0f) steering_adjustment =  20.0f;
                        if (steering_adjustment < -20.0f) steering_adjustment = -20.0f;
                        break;
                    }

                    case LINE_STATE_LOST:
                    case LINE_STATE_SEARCHING:
                        // Sin línea: frenar primero, luego girar 180°.
                        if (line_detected) {
                            line_seen_since_entry = 1;
                            line_integral       = 0.0f;
                            line_error_prev     = 0.0f;
                            line_lost_ms        = HAL_GetTick();
                            line_steer_fb_int   = 0.0f;
                            line_state          = LINE_STATE_FOLLOWING;
                        } else {
                            line_state          = LINE_STATE_LOST_BRAKE;
                            lrot_brake_start_ms = 0;
                        }
                        steering_adjustment = 0.0f;
                        break;

                    case LINE_STATE_LOST_BRAKE:
                    {
                        // Frena hasta velocidad baja (o timeout) antes del giro 180°.
                        const uint32_t LBRAKE_TIMEOUT  = 1500U;
                        const float    LBRAKE_VEL_THR  = 0.15f;  // m/s

                        if (lrot_brake_start_ms == 0) lrot_brake_start_ms = HAL_GetTick();

                        if (line_detected) {
                            line_integral        = 0.0f;
                            line_error_prev      = 0.0f;
                            line_lost_ms         = HAL_GetTick();
                            line_steer_fb_int    = 0.0f;
                            line_state           = LINE_STATE_FOLLOWING;
                            lrot_brake_start_ms  = 0;
                            break;
                        }

                        uint32_t elapsed = HAL_GetTick() - lrot_brake_start_ms;
                        if (fabsf(velocity_est_f) < LBRAKE_VEL_THR || elapsed >= LBRAKE_TIMEOUT) {
                            // Solo se llega acá cuando se perdió centrado (ver la rama de
                            // salida de LINE_STATE_FOLLOWING); el caso "por los extremos" va
                            // directo a LINE_STATE_EDGE_WAIT y nunca pasa por este estado.
                            line_state          = LINE_STATE_LOST_ROTATE;
                            obj_rot_initialized = 0;
                            obj_rot_phase       = 0;
                            obj_rot_heading     = 0.0f;
                            obj_rot_phase1_ms   = 0;
                            obj_rot_start_ms    = 0;
                            lrot_brake_start_ms = 0;
                        }
                        steering_adjustment = 0.0f;
                        break;
                    }

                    case LINE_STATE_LOST_ROTATE:
                    {
                        // Giro 180° derecha para buscar la línea perdida.
                        // Misma lógica gz+encoder que OBJ_ROTATE y MANUAL, escalada a 180°.
                        // 2026-07-01: 760->829->880. Seguía quedando corto en 829; nueva
                        // pasada de ajuste fino (+~6% más). Si sigue corto, seguir subiendo
                        // de a ~5-8% por vez.
                        const float  LROT_ENC_TARGET   = 880.0f;
                        const float  LROT_PIVOT        = 14.0f;  // subido de 12: giro un poco más agresivo (2026-07-01)
                        // Subido de 9 a 16 (2026-07-01): freno mucho más agresivo y fijo por
                        // un tiempo corto y acotado (ver LROT_BRAKE_DURATION más abajo), en
                        // vez de intentar frenar con precisión — no hace falta esa precisión,
                        // solo cortar la rotación rápido y seguir.
                        const float  LROT_BRAKE        = 16.0f;
                        const float  LROT_SLOWDOWN_DEG = 55.0f;
                        const float  LROT_ABS_TARGET   = 180.0f;
                        // 2026-07-01: P0_MAX y P1_MAX acortados de 4000/6000 a 1500/800 —
                        // con esos valores, si algo impedía completar los counts a tiempo
                        // (fricción, superficie, etc.) el giro podía tardar hasta ~10s en
                        // salir por timeout, sintiéndose "pegado en GIRO". Ahora el peor
                        // caso total (fase 0 + freno fijo de 400ms + colchón fase 1) es
                        // de ~2.3s. Son colchones de seguridad, no cortes normales — la
                        // salida normal sigue siendo completar LROT_ENC_TARGET counts.
                        const uint32_t LROT_P0_MAX     = 1500U;
                        const uint32_t LROT_P1_MAX     = 800U;
                        // enc_exit_frac 0.60: frena al 60% del recorrido → 44% restante para frenar (era 0.70)
                        const float  LROT_ENC_FRAC     = 0.60f;

                        if (!obj_rot_initialized) {
                            __disable_irq();
                            obj_rot_r0 = encoder_right;
                            obj_rot_l0 = encoder_left;
                            __enable_irq();
                            obj_rot_initialized = 1;
                            obj_rot_start_ms    = HAL_GetTick();
                            obj_rot_phase       = 0;
                            obj_rot_heading     = 0.0f;
                            obj_rot_phase1_ms   = 0;
                        }

                        // 2026-07-01: ya NO sale anticipadamente si ve la línea a mitad de
                        // giro (a pedido del usuario) — una detección parcial/fugaz mientras
                        // el robot está girando rápido puede ser espuria y cortar el giro a
                        // mitad de camino deja al robot en mal ángulo. El giro completo
                        // siempre termina por encoder; recién en LOST_SETTLE/LOST_FWD se
                        // vuelve a evaluar `line_detected`.
                        float lrot_gz = (float)gz / 131.0f;
                        obj_rot_heading += lrot_gz * DT_CTRL_FIXED;

                        __disable_irq();
                        int32_t lrot_dr = encoder_right - obj_rot_r0;
                        int32_t lrot_dl = encoder_left  - obj_rot_l0;
                        __enable_irq();
                        float lrot_counts    = (fabsf((float)lrot_dr) + fabsf((float)lrot_dl)) * 0.5f;
                        float lrot_enc_deg   = lrot_counts * (LROT_ABS_TARGET / LROT_ENC_TARGET);
                        float lrot_abs_hdg   = fmaxf(fabsf(obj_rot_heading), lrot_enc_deg);
                        float lrot_enc_thr   = LROT_ENC_TARGET * LROT_ENC_FRAC;

                        if (obj_rot_phase == 0) {
                            uint32_t elapsed = HAL_GetTick() - obj_rot_start_ms;
                            if (lrot_abs_hdg >= LROT_ABS_TARGET * 0.70f ||  // antes 0.80 → entraba tarde al freno
                                lrot_counts  >= lrot_enc_thr ||
                                elapsed      >= LROT_P0_MAX) {
                                obj_rot_phase     = 1;
                                obj_rot_phase1_ms = HAL_GetTick();
                            } else {
                                float remaining = LROT_ABS_TARGET - lrot_abs_hdg;
                                float slowdown  = (remaining < LROT_SLOWDOWN_DEG)
                                                ? (remaining / LROT_SLOWDOWN_DEG) : 1.0f;
                                float pivot = LROT_PIVOT * fmaxf(slowdown, 0.0f);
                                line_pivot_active  = 1;
                                motorRightVelocity = (int16_t)clampf_local(-(pwm_sat + pivot), -60.0f, 60.0f);
                                motorLeftVelocity  = (int16_t)clampf_local(-(pwm_sat - pivot), -60.0f, 60.0f);
                            }
                        }

                        if (obj_rot_phase == 1) {
                            uint32_t p1e      = HAL_GetTick() - obj_rot_phase1_ms;
                            int enc_done      = (lrot_counts >= LROT_ENC_TARGET);
                            int overshoot     = (lrot_abs_hdg > LROT_ABS_TARGET * 1.2f);
                            // 2026-07-01: sin buscar una velocidad "exacta" — freno fuerte a
                            // torque fijo (sin escalar) durante un tiempo acotado y corto
                            // (LROT_BRAKE_DURATION), y sale sí o sí. No tiene sentido quedarse
                            // pegado en GIRO tratando de afinar la velocidad de frenado.
                            const uint32_t LROT_BRAKE_DURATION = 400U;
                            if ((enc_done && p1e >= LROT_BRAKE_DURATION) || p1e >= LROT_P1_MAX || overshoot) {
                                // 2026-07-01: bypass de LOST_SETTLE a pedido del usuario — apenas
                                // termina el giro por encoder, arranca a avanzar sin esperar nada.
                                // (LOST_SETTLE queda en el código sin usar por si se necesita
                                // reactivar la pausa de estabilización más adelante.)
                                // Si ya quedó justo sobre la línea al terminar el giro, no hace
                                // falta avanzar nada: directo a FOLLOWING.
                                if (line_detected) {
                                    line_seen_since_entry = 1;
                                    line_integral         = 0.0f;
                                    line_error_prev       = 0.0f;
                                    line_lost_ms          = HAL_GetTick();
                                    line_state            = LINE_STATE_FOLLOWING;
                                } else {
                                    line_state          = LINE_STATE_LOST_FWD;
                                    line_lost_ms        = HAL_GetTick();
                                }
                                obj_rot_initialized = 0;
                                obj_rot_phase       = 0;
                                obj_rot_heading     = 0.0f;
                                steering_adjustment = 0.0f;
                            } else if (!enc_done) {
                                // Encoder aún no llegó al target: seguir pivoteando suave
                                float remaining = LROT_ENC_TARGET - lrot_counts;
                                float ramp = fminf(remaining / (LROT_ENC_TARGET * 0.15f), 1.0f);
                                float pivot = LROT_PIVOT * 0.4f * fmaxf(ramp, 0.2f);
                                line_pivot_active  = 1;
                                motorRightVelocity = (int16_t)clampf_local(-(pwm_sat + pivot), -60.0f, 60.0f);
                                motorLeftVelocity  = (int16_t)clampf_local(-(pwm_sat - pivot), -60.0f, 60.0f);
                            } else {
                                // Encoder OK: freno fuerte a torque fijo (sin escalar), 400ms
                                // como mucho (ver LROT_BRAKE_DURATION arriba).
                                line_pivot_active  = 1;
                                motorRightVelocity = (int16_t)clampf_local(-(pwm_sat - LROT_BRAKE), -60.0f, 60.0f);
                                motorLeftVelocity  = (int16_t)clampf_local(-(pwm_sat + LROT_BRAKE), -60.0f, 60.0f);
                            }
                        }
                        break;
                    }

                    case LINE_STATE_LOST_SETTLE:
                    {
                        // Post-180°: pausa de estabilización fuerte (upright + freno de
                        // encoders) antes de arrancar a avanzar. Sale por tiempo mínimo
                        // cumplido Y velocidad/inclinación estabilizadas. El timeout es
                        // solo un colchón de seguridad (3s, no un corte normal) — antes
                        // (1200ms) cortaba antes de frenar de verdad y el robot arrancaba
                        // a avanzar con velocidad residual del giro.
                        const uint32_t LSETTLE_MIN_MS   = 400U;
                        const uint32_t LSETTLE_TIMEOUT  = 3000U;
                        const float    LSETTLE_VEL_THR  = 0.10f;   // m/s
                        const float    LSETTLE_TILT_THR = 3.0f;    // grados respecto al setpoint

                        if (lrot_settle_start_ms == 0) lrot_settle_start_ms = HAL_GetTick();
                        steering_adjustment = 0.0f;

                        if (line_detected) {
                            line_seen_since_entry = 1;
                            line_integral         = 0.0f;
                            line_error_prev       = 0.0f;
                            line_lost_ms          = HAL_GetTick();
                            line_state            = LINE_STATE_FOLLOWING;
                            lrot_settle_start_ms  = 0;
                            break;
                        }

                        uint32_t elapsed  = HAL_GetTick() - lrot_settle_start_ms;
                        float tilt_err    = fabsf(filtered_roll_deg - dynamic_setpoint_f);
                        int settled       = (elapsed >= LSETTLE_MIN_MS) &&
                                            (fabsf(velocity_est_f) < LSETTLE_VEL_THR) &&
                                            (tilt_err < LSETTLE_TILT_THR);

                        if (settled || elapsed >= LSETTLE_TIMEOUT) {
                            line_state           = LINE_STATE_LOST_FWD;
                            line_lost_ms         = HAL_GetTick();
                            lrot_settle_start_ms = 0;
                        }
                        break;
                    }

                    case LINE_STATE_LOST_FWD:
                    {
                        // Post-180°: avanza con velocidad controlada hasta encontrar la línea.
                        // 5s sin encontrarla (antes 3s) → reposo total (GIVEN_UP), no reintenta
                        // la búsqueda. Más tiempo para darle chance de reencontrar la línea.
                        const uint32_t LOST_FWD_TIMEOUT = 5000U;
                        steering_adjustment = 0.0f;
                        if (line_detected) {
                            line_seen_since_entry = 1;
                            line_integral     = 0.0f;
                            line_error_prev   = 0.0f;
                            line_lost_ms      = HAL_GetTick();
                            line_state        = LINE_STATE_FOLLOWING;
                        } else if (f_fallen ||
                                   (HAL_GetTick() - line_lost_ms) > LOST_FWD_TIMEOUT) {
                            line_state = LINE_STATE_GIVEN_UP;
                        }
                        break;
                    }

                    case LINE_STATE_EDGE_WAIT:
                    {
                        // Perdida por un extremo (curva): frena hasta velocidad baja (o
                        // timeout) antes del giro de 90°. La entrada a este estado ya
                        // ocurrió >=1s después de perder la línea (ver salida de FOLLOWING),
                        // así que solo falta esperar a que el robot esté quieto.
                        const uint32_t EWAIT_TIMEOUT  = 1500U;
                        const float    EWAIT_VEL_THR  = 0.15f;  // m/s

                        if (edge_wait_start_ms == 0) edge_wait_start_ms = HAL_GetTick();
                        steering_adjustment = 0.0f;

                        uint32_t elapsed = HAL_GetTick() - edge_wait_start_ms;
                        if (fabsf(velocity_est_f) < EWAIT_VEL_THR || elapsed >= EWAIT_TIMEOUT) {
                            line_state          = LINE_STATE_EDGE_ROTATE;
                            obj_rot_initialized = 0;
                            obj_rot_phase       = 0;
                            obj_rot_heading     = 0.0f;
                            obj_rot_phase1_ms   = 0;
                            obj_rot_start_ms    = 0;
                            edge_wait_start_ms  = 0;
                        }
                        break;
                    }

                    case LINE_STATE_EDGE_ROTATE:
                    {
                        // Gira 60° (antes 90°, a pedido del usuario) hacia el lado donde se
                        // vio la línea por última vez (line_search_dir: +1=izquierda,
                        // -1=derecha, convención "izq=positivo"). Misma lógica gz+encoder
                        // que LOST_ROTATE. No sale anticipadamente si ve la línea (igual
                        // que LOST_ROTATE) — siempre completa el giro por encoder.
                        // Bajado de 60° a 45° (2026-07-01). EROT_ENC_TARGET y
                        // EROT_SLOWDOWN_DEG escalados en proporción 45/90 respecto a los
                        // valores base de 90° (440 y 55).
                        const float  EROT_ENC_TARGET   = 220.0f;
                        const float  EROT_PIVOT        = 14.0f;
                        const float  EROT_BRAKE        = 16.0f;  // ver comentario en LOST_ROTATE (2026-07-01)
                        const float  EROT_SLOWDOWN_DEG = 28.0f;
                        const float  EROT_ABS_TARGET   = 45.0f;
                        const uint32_t EROT_P0_MAX     = 1000U;  // ver comentario en LOST_ROTATE (2026-07-01)
                        const uint32_t EROT_P1_MAX     = 800U;
                        const float  EROT_ENC_FRAC     = 0.60f;
                        // dir: +1 = pivotea derecha (convención de OBJ_ROTATE/LOST_ROTATE),
                        // -1 = pivotea izquierda (como OBJ_WALL_TURN). 2026-07-01: signo
                        // invertido tras comprobar en el robot que giraba al lado contrario
                        // del esperado — con esto, line_search_dir>0 (izq) -> dir=+1 (derecha).
                        const float  EROT_DIR          = line_search_dir;

                        if (!obj_rot_initialized) {
                            __disable_irq();
                            obj_rot_r0 = encoder_right;
                            obj_rot_l0 = encoder_left;
                            __enable_irq();
                            obj_rot_initialized = 1;
                            obj_rot_start_ms    = HAL_GetTick();
                            obj_rot_phase       = 0;
                            obj_rot_heading     = 0.0f;
                            obj_rot_phase1_ms   = 0;
                        }

                        float erot_gz = (float)gz / 131.0f;
                        obj_rot_heading += erot_gz * DT_CTRL_FIXED;

                        __disable_irq();
                        int32_t erot_dr = encoder_right - obj_rot_r0;
                        int32_t erot_dl = encoder_left  - obj_rot_l0;
                        __enable_irq();
                        float erot_counts  = (fabsf((float)erot_dr) + fabsf((float)erot_dl)) * 0.5f;
                        float erot_enc_deg = erot_counts * (EROT_ABS_TARGET / EROT_ENC_TARGET);
                        float erot_abs_hdg = fmaxf(fabsf(obj_rot_heading), erot_enc_deg);
                        float erot_enc_thr = EROT_ENC_TARGET * EROT_ENC_FRAC;

                        if (obj_rot_phase == 0) {
                            uint32_t elapsed = HAL_GetTick() - obj_rot_start_ms;
                            if (erot_abs_hdg >= EROT_ABS_TARGET * 0.70f ||
                                erot_counts  >= erot_enc_thr ||
                                elapsed      >= EROT_P0_MAX) {
                                obj_rot_phase     = 1;
                                obj_rot_phase1_ms = HAL_GetTick();
                            } else {
                                float remaining = EROT_ABS_TARGET - erot_abs_hdg;
                                float slowdown  = (remaining < EROT_SLOWDOWN_DEG)
                                                ? (remaining / EROT_SLOWDOWN_DEG) : 1.0f;
                                float pivot = EROT_DIR * EROT_PIVOT * fmaxf(slowdown, 0.0f);
                                line_pivot_active  = 1;
                                motorRightVelocity = (int16_t)clampf_local(-(pwm_sat + pivot), -60.0f, 60.0f);
                                motorLeftVelocity  = (int16_t)clampf_local(-(pwm_sat - pivot), -60.0f, 60.0f);
                            }
                        }

                        if (obj_rot_phase == 1) {
                            uint32_t p1e  = HAL_GetTick() - obj_rot_phase1_ms;
                            int enc_done  = (erot_counts >= EROT_ENC_TARGET);
                            int overshoot = (erot_abs_hdg > EROT_ABS_TARGET * 1.2f);
                            // Ver comentario equivalente en LOST_ROTATE (2026-07-01): freno
                            // fuerte a torque fijo por un tiempo corto y acotado, sin buscar
                            // una velocidad "exacta".
                            const uint32_t EROT_BRAKE_DURATION = 400U;
                            if ((enc_done && p1e >= EROT_BRAKE_DURATION) || p1e >= EROT_P1_MAX || overshoot) {
                                // Si ya quedó justo sobre la línea al terminar el giro, no hace
                                // falta avanzar nada: directo a FOLLOWING (2026-07-01).
                                if (line_detected) {
                                    line_seen_since_entry = 1;
                                    line_integral         = 0.0f;
                                    line_error_prev       = 0.0f;
                                    line_lost_ms          = HAL_GetTick();
                                    line_state            = LINE_STATE_FOLLOWING;
                                    lrot_settle_start_ms  = 0;
                                } else {
                                    // Giro terminado: pausa de estabilización antes de avanzar
                                    line_state          = LINE_STATE_EDGE_SETTLE;
                                    lrot_settle_start_ms = 0;
                                }
                                obj_rot_initialized = 0;
                                obj_rot_phase       = 0;
                                obj_rot_heading     = 0.0f;
                                steering_adjustment = 0.0f;
                            } else if (!enc_done) {
                                float remaining = EROT_ENC_TARGET - erot_counts;
                                float ramp = fminf(remaining / (EROT_ENC_TARGET * 0.15f), 1.0f);
                                float pivot = EROT_DIR * EROT_PIVOT * 0.4f * fmaxf(ramp, 0.2f);
                                line_pivot_active  = 1;
                                motorRightVelocity = (int16_t)clampf_local(-(pwm_sat + pivot), -60.0f, 60.0f);
                                motorLeftVelocity  = (int16_t)clampf_local(-(pwm_sat - pivot), -60.0f, 60.0f);
                            } else {
                                // Freno fuerte a torque fijo (sin escalar), acotado por
                                // EROT_BRAKE_DURATION arriba.
                                float pivot = EROT_DIR * EROT_BRAKE;
                                line_pivot_active  = 1;
                                motorRightVelocity = (int16_t)clampf_local(-(pwm_sat - pivot), -60.0f, 60.0f);
                                motorLeftVelocity  = (int16_t)clampf_local(-(pwm_sat + pivot), -60.0f, 60.0f);
                            }
                        }
                        break;
                    }

                    case LINE_STATE_PERP_ROTATE:
                    {
                        // Cruce perpendicular detectado (los 4 ADC en negro sin manipular
                        // el robot): gira 90° fijo y vuelve directo a FOLLOWING, sin pasar
                        // por ningún estado de avance — se asume que quedó sobre la línea.
                        // Sin información de hacia qué lado está la línea (los 4 sensores
                        // vieron lo mismo), dirección fija: siempre a la derecha, igual que
                        // la convención por defecto de OBJ_ROTATE/LOST_ROTATE. Mismo esquema
                        // de freno fijo corto que LOST_ROTATE/EDGE_ROTATE (2026-07-01):
                        // no sale anticipadamente ante ninguna condición, siempre completa
                        // el giro por encoder y luego un freno fijo breve.
                        const float  PROT_ENC_TARGET  = 440.0f; // calibración base de 90°
                        const float  PROT_PIVOT       = 14.0f;
                        const float  PROT_BRAKE       = 16.0f;
                        const float  PROT_SLOWDOWN_DEG = 55.0f;
                        const float  PROT_ABS_TARGET  = 90.0f;
                        const uint32_t PROT_P0_MAX    = 1500U;
                        const uint32_t PROT_P1_MAX    = 800U;
                        const uint32_t PROT_BRAKE_DURATION = 400U;
                        const float  PROT_ENC_FRAC    = 0.60f;
                        const float  PROT_DIR         = 1.0f;   // siempre a la derecha

                        if (!obj_rot_initialized) {
                            __disable_irq();
                            obj_rot_r0 = encoder_right;
                            obj_rot_l0 = encoder_left;
                            __enable_irq();
                            obj_rot_initialized = 1;
                            obj_rot_start_ms    = HAL_GetTick();
                            obj_rot_phase       = 0;
                            obj_rot_heading     = 0.0f;
                            obj_rot_phase1_ms   = 0;
                        }

                        float prot_gz = (float)gz / 131.0f;
                        obj_rot_heading += prot_gz * DT_CTRL_FIXED;

                        __disable_irq();
                        int32_t prot_dr = encoder_right - obj_rot_r0;
                        int32_t prot_dl = encoder_left  - obj_rot_l0;
                        __enable_irq();
                        float prot_counts  = (fabsf((float)prot_dr) + fabsf((float)prot_dl)) * 0.5f;
                        float prot_enc_deg = prot_counts * (PROT_ABS_TARGET / PROT_ENC_TARGET);
                        float prot_abs_hdg = fmaxf(fabsf(obj_rot_heading), prot_enc_deg);
                        float prot_enc_thr = PROT_ENC_TARGET * PROT_ENC_FRAC;

                        if (obj_rot_phase == 0) {
                            uint32_t elapsed = HAL_GetTick() - obj_rot_start_ms;
                            if (prot_abs_hdg >= PROT_ABS_TARGET * 0.70f ||
                                prot_counts  >= prot_enc_thr ||
                                elapsed      >= PROT_P0_MAX) {
                                obj_rot_phase     = 1;
                                obj_rot_phase1_ms = HAL_GetTick();
                            } else {
                                float remaining = PROT_ABS_TARGET - prot_abs_hdg;
                                float slowdown  = (remaining < PROT_SLOWDOWN_DEG)
                                                ? (remaining / PROT_SLOWDOWN_DEG) : 1.0f;
                                float pivot = PROT_DIR * PROT_PIVOT * fmaxf(slowdown, 0.0f);
                                line_pivot_active  = 1;
                                motorRightVelocity = (int16_t)clampf_local(-(pwm_sat + pivot), -60.0f, 60.0f);
                                motorLeftVelocity  = (int16_t)clampf_local(-(pwm_sat - pivot), -60.0f, 60.0f);
                            }
                        }

                        if (obj_rot_phase == 1) {
                            uint32_t p1e      = HAL_GetTick() - obj_rot_phase1_ms;
                            int enc_done      = (prot_counts >= PROT_ENC_TARGET);
                            int overshoot     = (prot_abs_hdg > PROT_ABS_TARGET * 1.2f);
                            if ((enc_done && p1e >= PROT_BRAKE_DURATION) || p1e >= PROT_P1_MAX || overshoot) {
                                line_state          = LINE_STATE_FOLLOWING;
                                line_seen_since_entry = 1;
                                line_integral       = 0.0f;
                                line_error_prev     = 0.0f;
                                line_lost_ms        = HAL_GetTick();
                                obj_rot_initialized = 0;
                                obj_rot_phase       = 0;
                                obj_rot_heading     = 0.0f;
                                steering_adjustment = 0.0f;
                            } else if (!enc_done) {
                                float remaining = PROT_ENC_TARGET - prot_counts;
                                float ramp = fminf(remaining / (PROT_ENC_TARGET * 0.15f), 1.0f);
                                float pivot = PROT_DIR * PROT_PIVOT * 0.4f * fmaxf(ramp, 0.2f);
                                line_pivot_active  = 1;
                                motorRightVelocity = (int16_t)clampf_local(-(pwm_sat + pivot), -60.0f, 60.0f);
                                motorLeftVelocity  = (int16_t)clampf_local(-(pwm_sat - pivot), -60.0f, 60.0f);
                            } else {
                                float pivot = PROT_DIR * PROT_BRAKE;
                                line_pivot_active  = 1;
                                motorRightVelocity = (int16_t)clampf_local(-(pwm_sat - pivot), -60.0f, 60.0f);
                                motorLeftVelocity  = (int16_t)clampf_local(-(pwm_sat + pivot), -60.0f, 60.0f);
                            }
                        }
                        break;
                    }

                    case LINE_STATE_EDGE_SETTLE:
                    {
                        // Post-90°: pausa de estabilización idéntica a LOST_SETTLE (mismo
                        // fix 2026-07-01: timeout es colchón de seguridad, no corte normal).
                        const uint32_t ESETTLE_MIN_MS   = 400U;
                        const uint32_t ESETTLE_TIMEOUT  = 3000U;
                        const float    ESETTLE_VEL_THR  = 0.10f;
                        const float    ESETTLE_TILT_THR = 3.0f;

                        if (lrot_settle_start_ms == 0) lrot_settle_start_ms = HAL_GetTick();
                        steering_adjustment = 0.0f;

                        if (line_detected) {
                            line_seen_since_entry = 1;
                            line_integral         = 0.0f;
                            line_error_prev       = 0.0f;
                            line_lost_ms          = HAL_GetTick();
                            line_state            = LINE_STATE_FOLLOWING;
                            lrot_settle_start_ms  = 0;
                            break;
                        }

                        uint32_t elapsed  = HAL_GetTick() - lrot_settle_start_ms;
                        float tilt_err    = fabsf(filtered_roll_deg - dynamic_setpoint_f);
                        int settled       = (elapsed >= ESETTLE_MIN_MS) &&
                                            (fabsf(velocity_est_f) < ESETTLE_VEL_THR) &&
                                            (tilt_err < ESETTLE_TILT_THR);

                        if (settled || elapsed >= ESETTLE_TIMEOUT) {
                            line_state           = LINE_STATE_EDGE_FWD;
                            line_lost_ms         = HAL_GetTick();
                            lrot_settle_start_ms = 0;
                        }
                        break;
                    }

                    case LINE_STATE_EDGE_FWD:
                    {
                        // Post-90°: avanza con velocidad controlada hasta encontrar la línea.
                        // 5s sin encontrarla (antes 3s) → reposo total (GIVEN_UP), no reintenta
                        // la búsqueda. Más tiempo para darle chance de reencontrar la línea.
                        const uint32_t EDGE_FWD_TIMEOUT = 5000U;
                        steering_adjustment = 0.0f;
                        if (line_detected) {
                            line_seen_since_entry = 1;
                            line_integral     = 0.0f;
                            line_error_prev   = 0.0f;
                            line_lost_ms      = HAL_GetTick();
                            line_state        = LINE_STATE_FOLLOWING;
                        } else if (f_fallen ||
                                   (HAL_GetTick() - line_lost_ms) > EDGE_FWD_TIMEOUT) {
                            line_state = LINE_STATE_GIVEN_UP;
                        }
                        break;
                    }

                    case LINE_STATE_GIVEN_UP:
                        // Reposo total: ni el giro de 180° ni el de 90° encontraron la línea.
                        // No hay timeout ni reintento — se queda balanceado en el lugar hasta
                        // que se detecte la línea de nuevo (el usuario la repone a mano).
                        steering_adjustment = 0.0f;
                        if (line_detected) {
                            line_seen_since_entry = 1;
                            line_integral       = 0.0f;
                            line_error_prev     = 0.0f;
                            line_lost_ms        = HAL_GetTick();
                            line_state          = LINE_STATE_FOLLOWING;
                        }
                        break;

                    case LINE_STATE_OBJ_PRE_REVERSE_HOLD:
                    {
                        steering_adjustment = 0.0f;
                        line_integral       = 0.0f;
                        line_error_prev     = 0.0f;
                        line_error_f_d      = 0.0f;

                        if ((HAL_GetTick() - obj_pre_rev_start_ms) >= OBJ_PRE_REVERSE_HOLD_MS) {
                            line_state          = LINE_STATE_OBJ_REVERSE;
                            obj_pre_rev_start_ms = 0;
                            obj_rev_initialized = 0;
                            obj_rot_initialized = 0;
                            obj_rot_phase       = 0;
                            obj_rot_heading     = 0.0f;
                            obj_rot_phase1_ms   = 0;
                        }
                        break;
                    }

                    case LINE_STATE_OBJ_REVERSE:
                    {
                        const int32_t rev_target_counts = 200; // ~10 cm

                        if (!obj_rev_initialized) {
                            __disable_irq();
                            obj_rev_r0 = encoder_right;
                            obj_rev_l0 = encoder_left;
                            __enable_irq();
                            obj_rev_initialized  = 1;
                            obj_rev_straight_int = 0.0f;
                        }

                        __disable_irq();
                        int32_t rev_dr = encoder_right - obj_rev_r0;
                        int32_t rev_dl = encoder_left  - obj_rev_l0;
                        __enable_irq();

                        int32_t rev_counts = (abs(rev_dr) + abs(rev_dl)) / 2;

                        if (rev_counts >= rev_target_counts) {
                            line_state          = LINE_STATE_OBJ_BRAKE;
                            obj_rev_initialized = 0;
                            obj_rot_initialized = 0;
                            obj_rot_phase       = 0;
                            obj_rot_heading     = 0.0f;
                            obj_rot_phase1_ms   = 0;
                            obj_brake_start_ms  = 0;
                            steering_adjustment = 0.0f;
                            line_obj_rev_vel_integral = 0.0f;
                            line_integral       = 0.0f;
                            line_error_prev     = 0.0f;
                            line_error_f_d      = 0.0f;
                        } else {
                            // Steering en reversa: si ve línea, corrige al revés.
                            // Si no ve línea: encoder feedback para ir recto
                            // (compensa asimetría mecánica entre ruedas).
                            if (line_detected) {
                                float reverse_line_error = -line_error;
                                float p_line = KP_LINE * reverse_line_error * LINE_OBJ_REV_STEER_GAIN;

                                if (!late_cycle) {
                                    line_integral += reverse_line_error * dt_ctrl;
                                }
                                if (line_integral >  5.0f) line_integral =  5.0f;
                                if (line_integral < -5.0f) line_integral = -5.0f;

                                float i_line = KI_LINE * line_integral * LINE_OBJ_REV_STEER_GAIN;

                                line_error_f_d += 0.4f * (reverse_line_error - line_error_f_d);

                                float d_line = 0.0f;
                                if (!late_cycle) {
                                    float line_delta = line_error_f_d - line_error_prev;
                                    if (line_delta >  0.3f) line_delta =  0.3f;
                                    if (line_delta < -0.3f) line_delta = -0.3f;
                                    d_line = KD_LINE * (line_delta / dt_ctrl) * LINE_OBJ_REV_STEER_GAIN;
                                }
                                line_error_prev = line_error_f_d;

                                float rev_steer = p_line + i_line + d_line;
                                steering_adjustment = clampf_local(
                                    rev_steer,
                                    -LINE_OBJ_REV_STEER_MAX,
                                    LINE_OBJ_REV_STEER_MAX
                                );

                                log_p_line = p_line;
                                log_i_line = i_line;
                                log_d_line = d_line;
                            } else {
                                // PI de enderezamiento: target = rev_dr == rev_dl.
                                // Si el signo curva al reves, negar diff_err.
                                float diff_err = (float)(rev_dr - rev_dl);
                                obj_rev_straight_int += diff_err * DT_CTRL_FIXED;
                                if (obj_rev_straight_int >  30.0f) obj_rev_straight_int =  30.0f;
                                if (obj_rev_straight_int < -30.0f) obj_rev_straight_int = -30.0f;
                                float enc_corr = 3.0f * diff_err + 0.5f * obj_rev_straight_int;
                                if (enc_corr >  15.0f) enc_corr =  15.0f;
                                if (enc_corr < -15.0f) enc_corr = -15.0f;
                                steering_adjustment = enc_corr;
                            }
                        }
                        // Setpoint de reversa calculado por PI en la seccion de setpoints.
                        break;
                    }

                    case LINE_STATE_OBJ_BRAKE:
                    {
                        // Fase 1: espera a que la velocidad caiga. Fase 2: wait 2s antes de rotar.
                        const float    BRAKE_VEL_THR  = 0.05f;
                        const uint32_t BRAKE_TIMEOUT  = 1500U;
                        const uint32_t PRE_ROTATE_MS  = 2000U;
                        if (obj_brake_start_ms == 0) obj_brake_start_ms = HAL_GetTick();

                        // Fase 1: frenar hasta quieto o timeout
                        if (obj_pre_rotate_ms == 0) {
                            if (fabsf(velocity_est_f) < BRAKE_VEL_THR ||
                                (HAL_GetTick() - obj_brake_start_ms) > BRAKE_TIMEOUT) {
                                obj_pre_rotate_ms = HAL_GetTick();  // arranca fase 2
                            }
                        }
                        // Fase 2: wait 1s quieto antes de girar
                        if (obj_pre_rotate_ms != 0 &&
                            (HAL_GetTick() - obj_pre_rotate_ms) >= PRE_ROTATE_MS) {
                            line_state          = LINE_STATE_OBJ_ROTATE;
                            obj_rot_initialized = 0;
                            obj_rot_phase       = 0;
                            obj_rot_heading     = 0.0f;
                            obj_rot_phase1_ms   = 0;
                            steering_adjustment = 0.0f;
                            obj_brake_start_ms  = 0;
                            obj_pre_rotate_ms   = 0;
                        }
                        break;
                    }

                    case LINE_STATE_OBJ_ROTATE:
                    {
                        // Giro 90° derecha: fase 0 (spin+slowdown) + fase 1 (freno).
                        // Misma lógica que MANUAL: heading compuesto gz+encoder, slowdown ramp,
                        // detección de overshoot en fase 1. Dirección fija: derecha (dir=+1).
                        // Parámetros idénticos al giro MANUAL para comportamiento equivalente
                        const float  LINE_ROT_ENC_TARGET   = 380.0f; // igual que MANUAL_ROT_ENC_TARGET
                        const float  LINE_ROT_PIVOT        = 15.0f;  // igual que MANUAL_ROT_PIVOT_POWER
                        const float  LINE_ROT_BRAKE        = 5.0f;   // igual que MANUAL_ROT_BRAKE_POWER
                        const float  LINE_ROT_SLOWDOWN_DEG = 55.0f;  // igual que MANUAL_ROT_SLOWDOWN_DEG
                        const uint32_t LINE_ROT_P0_MAX     = 1200U;  // igual que MANUAL para 90°
                        const uint32_t LINE_ROT_P1_MAX     = 300U;   // igual que MANUAL

                        if (!obj_rot_initialized) {
                            __disable_irq();
                            obj_rot_r0 = encoder_right;
                            obj_rot_l0 = encoder_left;
                            __enable_irq();
                            obj_rot_initialized = 1;
                            obj_rot_start_ms    = HAL_GetTick();
                            obj_rot_phase       = 0;
                            obj_rot_heading     = 0.0f;
                            obj_rot_phase1_ms   = 0;
                        }

                        // Heading gz acumulado
                        float obj_gz_dps = (float)gz / 131.0f;
                        obj_rot_heading += obj_gz_dps * DT_CTRL_FIXED;

                        __disable_irq();
                        int32_t dr = encoder_right - obj_rot_r0;
                        int32_t dl = encoder_left  - obj_rot_l0;
                        __enable_irq();
                        float rot_counts = (fabsf((float)dr) + fabsf((float)dl)) * 0.5f;

                        // Heading compuesto: máximo entre gz y encoder (igual que MANUAL).
                        float enc_heading_deg = rot_counts * (90.0f / LINE_ROT_ENC_TARGET);
                        float abs_heading     = fmaxf(fabsf(obj_rot_heading), enc_heading_deg);
                        float enc_phase0_thr  = LINE_ROT_ENC_TARGET * 0.85f;

                        if (obj_rot_phase == 0) {
                            uint32_t elapsed = HAL_GetTick() - obj_rot_start_ms;
                            if (abs_heading >= 90.0f * 0.80f ||
                                rot_counts  >= enc_phase0_thr ||
                                elapsed     >= LINE_ROT_P0_MAX) {
                                obj_rot_phase     = 1;
                                obj_rot_phase1_ms = HAL_GetTick();
                            } else {
                                // Slowdown ramp en los últimos LINE_ROT_SLOWDOWN_DEG
                                float remaining = 90.0f - abs_heading;
                                float slowdown  = (remaining < LINE_ROT_SLOWDOWN_DEG)
                                                ? (remaining / LINE_ROT_SLOWDOWN_DEG)
                                                : 1.0f;
                                float pivot = LINE_ROT_PIVOT * fmaxf(slowdown, 0.0f);
                                line_pivot_active = 1;
                                motorRightVelocity = (int16_t)clampf_local(
                                    -(pwm_sat + pivot), -60.0f, 60.0f);
                                motorLeftVelocity  = (int16_t)clampf_local(
                                    -(pwm_sat - pivot), -60.0f, 60.0f);
                            }
                        }

                        if (obj_rot_phase == 1) {
                            uint32_t p1_elapsed = HAL_GetTick() - obj_rot_phase1_ms;
                            float rot_vel  = (fabsf(speed_right_rps_s) + fabsf(speed_left_rps_s)) * 0.5f;
                            int vel_ok     = (rot_vel < 2.0f && p1_elapsed >= 80U);
                            int overshoot  = (abs_heading > 90.0f * 1.3f);
                            if (vel_ok || p1_elapsed >= LINE_ROT_P1_MAX || overshoot) {
                                line_state          = LINE_STATE_OBJ_HOLD;
                                steering_adjustment = 0.0f;
                                obj_rot_initialized = 0;
                                obj_rot_phase       = 0;
                                obj_rot_heading     = 0.0f;
                                obj_rot_start_ms    = 0;
                            } else {
                                // Freno suave con pwm_sat para mantener balance (igual que MANUAL)
                                line_pivot_active = 1;
                                motorRightVelocity = (int16_t)clampf_local(
                                    -(pwm_sat - LINE_ROT_BRAKE), -60.0f, 60.0f);
                                motorLeftVelocity  = (int16_t)clampf_local(
                                    -(pwm_sat + LINE_ROT_BRAKE), -60.0f, 60.0f);
                            }
                        }
                        break;
                    }

                    case LINE_STATE_OBJ_HOLD:
                    {
                        // Balance estático 2s, luego pasa a OBJ_ARC para hacer el arco de vuelta a la línea.
                        steering_adjustment = 0.0f;
                        if (obj_hold_start_ms == 0) obj_hold_start_ms = HAL_GetTick();
                        if (f_fallen) {
                            line_state        = LINE_STATE_FOLLOWING;
                            line_integral     = 0.0f;
                            line_error_prev   = 0.0f;
                            line_error_f_d    = 0.0f;
                            line_lost_ms      = HAL_GetTick();
                            obj_hold_start_ms = 0;
                            obj_detect_ignore_until_ms = HAL_GetTick() + 5000U;
                        } else if ((HAL_GetTick() - obj_hold_start_ms) >= OBJ_HOLD_DURATION_MS) {
                            line_state                 = LINE_STATE_OBJ_WALL_APPROACH;
                            obj_hold_start_ms          = 0;
                            obj_wall_approach_start_ms = HAL_GetTick();
                        }
                        break;
                    }

                    case LINE_STATE_OBJ_ARC:
                        // (no usado — reservado)
                        line_state = LINE_STATE_OBJ_WALL_APPROACH;
                        break;

                    case LINE_STATE_OBJ_WALL_APPROACH:
                    {
                        // Avanza despacio con OBJ_WALL_APPROACH_ANGLE hasta que ADC7 detecta la pared.
                        // Timeout OBJ_WALL_APPROACH_TIMEOUT → vuelve a FOLLOWING si no encuentra pared.
                        uint8_t wall_found = ((float)adcAvg[OBJ_WALL_ADC_IDX] < OBJ_WALL_THRESHOLD);
                        if (f_fallen) {
                            line_state                 = LINE_STATE_FOLLOWING;
                            obj_wall_approach_start_ms = 0;
                            obj_detect_ignore_until_ms = HAL_GetTick() + 5000U;
                        } else if (wall_found) {
                            line_state                 = LINE_STATE_OBJ_WALL_FWD;
                            obj_wall_approach_start_ms = 0;
                            obj_wall_fwd_start_ms      = HAL_GetTick();
                        } else if ((HAL_GetTick() - obj_wall_approach_start_ms) >= OBJ_WALL_APPROACH_TIMEOUT) {
                            line_state                 = LINE_STATE_FOLLOWING;
                            obj_wall_approach_start_ms = 0;
                            obj_detect_ignore_until_ms = HAL_GetTick() + 5000U;
                        }
                        // Mientras avanza: steering neutro, setpoint manejado por bloque de setpoints.
                        steering_adjustment = 0.0f;
                        break;
                    }

                    case LINE_STATE_OBJ_WALL_FWD:
                    {
                        // Wall-following: avanza mientras ADC7 ve el objeto.
                        // Si pierde el objeto → pivot izquierda. Si ve línea (tras 3s) → FOLLOWING.
                        // Si demasiado cerca (ADC7 < TOO_CLOSE) → pivot derecha igual que OBJ_ROTATE.
                        if (obj_wall_fwd_start_ms == 0)
                            obj_wall_fwd_start_ms = HAL_GetTick();
                        uint8_t line_ignore = ((HAL_GetTick() - obj_wall_fwd_start_ms) < OBJ_WALL_LINE_IGNORE_MS);
                        float wall_adc       = (float)adcAvg[OBJ_WALL_ADC_IDX];
                        uint8_t wall_visible = (wall_adc < OBJ_WALL_THRESHOLD);
                        uint8_t too_close    = (wall_adc < OBJ_WALL_TOO_CLOSE_THOLD);
                        if (line_detected && !line_ignore) {
                            line_seen_since_entry = 1;
                            line_state          = LINE_STATE_FOLLOWING;
                            line_integral       = 0.0f;
                            line_obj_rev_vel_integral = 0.0f;
                            line_error_prev     = 0.0f;
                            line_error_f_d      = 0.0f;
                            line_lost_ms        = HAL_GetTick();
                            steering_adjustment = 0.0f;
                            obj_wall_fwd_start_ms = 0;
                            obj_detect_ignore_until_ms = HAL_GetTick() + 5000U;
                        } else if (too_close) {
                            // Demasiado cerca: pivot derecha (alejar del obstáculo)
                            line_pivot_active  = 1;
                            motorRightVelocity = (int16_t)clampf_local(
                                -(pwm_sat + OBJ_WALL_PIVOT_POWER), -30.0f, 30.0f);
                            motorLeftVelocity  = (int16_t)clampf_local(
                                -(pwm_sat - OBJ_WALL_PIVOT_POWER), -30.0f, 30.0f);
                        } else if (!wall_visible) {
                            line_state            = LINE_STATE_OBJ_WALL_TURN;
                            steering_adjustment   = 0.0f;
                            obj_wall_fwd_start_ms = 0;
                        } else {
                            // Avanza hacia adelante con ángulo fijo; yaw-lock se aplica al calcular motores.
                            steering_adjustment = 0.0f;
                        }
                        break;
                    }

                    case LINE_STATE_OBJ_WALL_TURN:
                    {
                        // Pivot izquierda hasta re-ver el objeto en ADC7 o detectar línea.
                        // Si demasiado cerca (ADC7 < TOO_CLOSE) → pivot derecha en cambio.
                        float wall_adc       = (float)adcAvg[OBJ_WALL_ADC_IDX];
                        uint8_t wall_visible = (wall_adc < OBJ_WALL_THRESHOLD);
                        uint8_t too_close    = (wall_adc < OBJ_WALL_TOO_CLOSE_THOLD);
                        if (line_detected) {
                            line_seen_since_entry = 1;
                            line_state          = LINE_STATE_FOLLOWING;
                            line_integral       = 0.0f;
                            line_obj_rev_vel_integral = 0.0f;
                            line_error_prev     = 0.0f;
                            line_error_f_d      = 0.0f;
                            line_lost_ms        = HAL_GetTick();
                            steering_adjustment = 0.0f;
                            line_pivot_active   = 0;
                            obj_detect_ignore_until_ms = HAL_GetTick() + 5000U;
                        } else if (too_close) {
                            // Demasiado cerca: pivot derecha (alejar del obstáculo)
                            line_pivot_active  = 1;
                            motorRightVelocity = (int16_t)clampf_local(
                                -(pwm_sat + OBJ_WALL_PIVOT_POWER), -30.0f, 30.0f);
                            motorLeftVelocity  = (int16_t)clampf_local(
                                -(pwm_sat - OBJ_WALL_PIVOT_POWER), -30.0f, 30.0f);
                        } else if (wall_visible) {
                            line_state            = LINE_STATE_OBJ_WALL_FWD;
                            line_pivot_active     = 0;
                            obj_wall_fwd_start_ms = HAL_GetTick();
                        } else {
                            // Pivot izquierda: signo opuesto a OBJ_ROTATE (que pivotea derecha).
                            line_pivot_active  = 1;
                            motorRightVelocity = (int16_t)clampf_local(
                                -(pwm_sat - OBJ_WALL_PIVOT_POWER), -30.0f, 30.0f);
                            motorLeftVelocity  = (int16_t)clampf_local(
                                -(pwm_sat + OBJ_WALL_PIVOT_POWER), -30.0f, 30.0f);
                        }
                        break;
                    }
                }

                // Cuando sigue línea: limitar integral para evitar deriva.
                // Cuando pierde línea (LOST/SEARCHING): permitir hasta ±8 como
                // BALANCE_ONLY para que el motor izquierdo tenga suficiente salida.
                if (line_state == LINE_STATE_FOLLOWING) {
                    if (integral >  2.0f) integral =  2.0f;
                    if (integral < -2.0f) integral = -2.0f;
                }

                if (!line_pivot_active) {
                    float half_steer = steering_adjustment * 0.5f;

                    // En LOST/SEARCHING/OBJ_HOLD: corrección yaw igual que BALANCE_ONLY.
                    // OBJ_ROTATE usa steering_adjustment directo (calculado en el case).
                    // 2026-07-01: agregados LOST_FWD/EDGE_FWD (avance ciego post-giro) —
                    // faltaban en esta lista pese a ser análogos a OBJ_WALL_FWD (avanzar
                    // recto buscando algo); sin esto no había ninguna corrección de rumbo
                    // y el robot podía irse curvando durante la búsqueda.
                    if (line_state == LINE_STATE_LOST     ||
                        line_state == LINE_STATE_SEARCHING ||
                        line_state == LINE_STATE_OBJ_PRE_REVERSE_HOLD ||
                        line_state == LINE_STATE_OBJ_HOLD ||
                        line_state == LINE_STATE_OBJ_WALL_FWD ||
                        line_state == LINE_STATE_LOST_FWD ||
                        line_state == LINE_STATE_EDGE_FWD) {
                        float gz_dps_line = (float)gz / 131.0f;
                        half_steer = gz_dps_line * 0.3f;
                    }

                    float mR = pwm_sat - half_steer;
                    float mL = pwm_sat + half_steer;

                    if (mR >  40.0f) mR =  40.0f;
                    if (mR < -40.0f) mR = -40.0f;
                    if (mL >  40.0f) mL =  40.0f;
                    if (mL < -40.0f) mL = -40.0f;

                    motorRightVelocity = -(int16_t)mL;
                    motorLeftVelocity  = -(int16_t)mR;
                }

            } else if (robot_state == ROBOT_STATE_MANUAL_CONTROL) {
                line_integral       = 0.0f;
                line_obj_rev_vel_integral = 0.0f;
                line_error_prev     = 0.0f;
                line_state          = LINE_STATE_FOLLOWING;

                if (manual_rot_active) {
                    // dir > 0 = derecha: right retrocede, left avanza.
                    float dir = (manual_rot_target_deg >= 0.0f) ? 1.0f : -1.0f;

                    steering_adjustment = 0.0f;
                    if (manual_rot_phase == 0) {
                        // Fase 0: spin con componente de balance + slowdown
                        // manual_rot_enc_counts ya fue actualizado en el bloque de estado
                        float _enc_hdg = manual_rot_enc_counts * (90.0f / MANUAL_ROT_ENC_TARGET);
                        float _abs_hdg = fmaxf(fabsf(manual_rot_heading), _enc_hdg);
                        float abs_remaining = fabsf(manual_rot_target_deg) - _abs_hdg;
                        float slowdown = (abs_remaining < MANUAL_ROT_SLOWDOWN_DEG)
                                       ? (abs_remaining / MANUAL_ROT_SLOWDOWN_DEG)
                                       : 1.0f;
                        float pivot = MANUAL_ROT_PIVOT_POWER * fmaxf(slowdown, 0.0f);
                        motorRightVelocity = (int16_t)clampf_local(-(pwm_sat + dir * pivot), -60.0f, 60.0f);
                        motorLeftVelocity  = (int16_t)clampf_local(-(pwm_sat - dir * pivot), -60.0f, 60.0f);
                    } else {
                        // Fase 1: contra-rotación CON pwm_sat para mantener balance.
                        // La diferencia entre ambos motores siempre es 2×BRAKE_POWER (frena la inercia),
                        // mientras que pwm_sat actúa como modo común y mantiene el equilibrio.
                        motorRightVelocity = (int16_t)clampf_local(-(pwm_sat - dir * MANUAL_ROT_BRAKE_POWER), -60.0f, 60.0f);
                        motorLeftVelocity  = (int16_t)clampf_local(-(pwm_sat + dir * MANUAL_ROT_BRAKE_POWER), -60.0f, 60.0f);
                    }
                } else {
                    {
                        const float STEER_RATE = 1.5f;
                        float steer_delta = manual_steering_cmd - steering_adjustment;
                        if (steer_delta >  STEER_RATE) steer_delta =  STEER_RATE;
                        if (steer_delta < -STEER_RATE) steer_delta = -STEER_RATE;
                        steering_adjustment += steer_delta;
                    }

                    static float prev_steering_cmd = 0.0f;
                    if ((manual_steering_cmd > 0.5f && prev_steering_cmd < -0.5f) ||
                        (manual_steering_cmd < -0.5f && prev_steering_cmd > 0.5f)) {
                        integral = 0.0f;
                    }
                    prev_steering_cmd = manual_steering_cmd;

                    float mR = pwm_sat - steering_adjustment;
                    float mL = pwm_sat + steering_adjustment;
                    if (mR >  100.0f) mR =  100.0f;
                    if (mR < -100.0f) mR = -100.0f;
                    if (mL >  100.0f) mL =  100.0f;
                    if (mL < -100.0f) mL = -100.0f;

                    motorRightVelocity = -(int16_t)mL;
                    motorLeftVelocity  = -(int16_t)mR;
                } // end else (no rotation active)

            } else if (robot_state != ROBOT_STATE_MOTOR_TEST) {
                line_integral       = 0.0f;
                line_obj_rev_vel_integral = 0.0f;
                line_error_prev     = 0.0f;
                steering_adjustment = 0.0f;
                line_state          = LINE_STATE_FOLLOWING;

                float gz_dps = (float)gz / 131.0f;
                // steer_pid_enabled=0: corrección open-loop por giroscopio Z (comportamiento original)
                // steer_pid_enabled=1: corrección de lazo cerrado por encoders
                float correction = steer_pid_enabled
                                 ? steer_correction
                                 : (-gz_dps * 0.3f);

                float mR = pwm_sat + correction;
                float mL = pwm_sat - correction;

                if (mR >  100.0f) mR =  100.0f;
                if (mR < -100.0f) mR = -100.0f;
                if (mL >  100.0f) mL =  100.0f;
                if (mL < -100.0f) mL = -100.0f;

                motorRightVelocity = -(int16_t)mL;
                motorLeftVelocity  = -(int16_t)mR;
            }
            // ROBOT_STATE_MOTOR_TEST: SETMOTORSPEED controla directamente
        }

        roll_deg = filtered_roll_deg;

        // -------------------------------------------------------
        // LOG MINIMO EN 2ms
        // -------------------------------------------------------
        log_counter++;

        if (f_send_wifi_log && (log_counter % LOG_WIFI_DECIM == 0)) {
            WifiLogData_t wlog;
            wlog.t_ms       = HAL_GetTick();
            wlog.roll_filt  = filtered_roll_deg;
            wlog.output     = output;
            wlog.p_term     = p_term;
            wlog.i_term     = i_term;
            wlog.d_term     = d_term;
            wlog.mR         = motorRightVelocity;
            wlog.mL         = motorLeftVelocity;
            wlog.dyn_sp     = dynamic_setpoint_f;
            wlog.dt_ctrl_us = (uint32_t)(dt_ctrl * 1000000.0f);

            wlog.line_error          = line_error;
            wlog.p_line              = log_p_line;
            wlog.i_line              = log_i_line;
            wlog.d_line              = log_d_line;
            wlog.steering_adjustment = steering_adjustment;
            wlog.adc1                = adcValues[0];
            wlog.adc2                = adcValues[1];
            wlog.adc3                = adcValues[2];
            wlog.adc4                = adcValues[3];

            UNER_SendWifiLogData(&wlog);
        }

        if (f_send_csv_log && !log_header_sent) {
            USB_DebugStr("t_ms,dt_us,dt_ctrl_us,accel_roll,accel_roll_f,gyro_y,gyro_f,roll_filt,dyn_sp,error,p,i,d,output,pwm_cmd,pwm_sat,sat,mR,mL,pitch,ax,ay,az,gx,gy,gz\r\n");
            log_header_sent = 1;
        }

        if (f_send_csv_log && (log_counter % LOG_DECIM == 0)) {
            char buf[128];

            int roll_i   = (int)(filtered_roll_deg * 1000.0f);
            int p_i      = (int)(p_term * 1000.0f);
            int i_i      = (int)(i_term * 1000.0f);
            int d_i      = (int)(d_term * 1000.0f);
            int sp_i     = (int)(dynamic_setpoint_f * 1000.0f);
            int error_i  = (int)(error * 1000.0f);
            int gyrof_i  = (int)(gyro_f * 1000.0f);
            int accel_i  = (int)(accel_ang_deg * 1000.0f);
            int accelf_i = (int)(accel_roll_f * 1000.0f);
            int output_i = (int)(output * 1000.0f);
            int pwmcmd_i = (int)(pwm_cmd * 100.0f);
            int pwmsat_i = (int)(pwm_sat * 100.0f);
            int pitch_i  = (int)(pitch_deg * 1000.0f);

            int ax_i = (int)ax;
            int ay_i = (int)ay;
            int az_i = (int)az;
            int gx_i = (int)gx;
            int gy_i = (int)gy;
            int gz_i = (int)gz;

            int len = snprintf(buf, sizeof(buf),
                "%lu,%lu,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                HAL_GetTick(),                 // t_ms
                (uint32_t)(dt_real * 1000000.0f), // dt_us
                (uint32_t)(dt_ctrl * 1000000.0f), // dt_ctrl_us
                accel_i,                       // accel_roll x1000
                accelf_i,                      // accel_roll_f x1000
                (int)(gyro_rate_dps * 1000.0f),// gyro_y x1000
                gyrof_i,                       // gyro_f x1000
                roll_i,                        // roll_filt x1000
                sp_i,                          // dyn_sp x1000
                error_i,                       // error x1000
                p_i,                           // p x1000
                i_i,                           // i x1000
                d_i,                           // d x1000
                output_i,                      // output x1000
                pwmcmd_i,                      // pwm_cmd x100
                pwmsat_i,                      // pwm_sat x100
                sat_flag,                      // sat
                motorRightVelocity,            // mR
                motorLeftVelocity,             // mL
                pitch_i,                       // pitch x1000
                ax_i, ay_i, az_i,              // accel raw
                gx_i, gy_i, gz_i               // gyro raw
            );

            if (len > 0) {
                USB_DebugSend((uint8_t*)buf, (uint16_t)len);
            }
        }



    // En el aire >2s: detener motores hasta que los sensores de línea vean superficie.
    if (robot_state == ROBOT_STATE_LINE_FOLLOWING && f_in_air) {
        motorRightVelocity = 0;
        motorLeftVelocity  = 0;
    }

    if (robot_state == ROBOT_STATE_MOTOR_TEST) {
        // Modo test: SETMOTORSPEED controla directamente, sin PID ni compensación
        MotorControl(motorRightVelocity, -motorLeftVelocity);
    } else if ((robot_state != ROBOT_STATE_IDLE) && !f_fallen) {
        int16_t mR_comp = motorRightVelocity;
        if      (mR_comp > 0)  mR_comp = (int16_t)( mR_comp + MOTOR_RIGHT_DEADBAND);
        else if (mR_comp < 0)  mR_comp = (int16_t)( mR_comp - MOTOR_RIGHT_DEADBAND);
        if (mR_comp >  100) mR_comp =  100;
        if (mR_comp < -100) mR_comp = -100;

        int16_t mL_comp = -motorLeftVelocity;
        if      (mL_comp > 0)  mL_comp = (int16_t)( mL_comp + MOTOR_LEFT_DEADBAND);
        else if (mL_comp < 0)  mL_comp = (int16_t)( mL_comp - MOTOR_LEFT_DEADBAND);
        if (mL_comp >  100) mL_comp =  100;
        if (mL_comp < -100) mL_comp = -100;

        MotorControl(mR_comp, mL_comp);
    } else {
        MotorControl(0, 0);
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  // ── I2C Bus Recovery ─────────────────────────────────────────────────────
  // Si el MCU se resetó a mitad de una transacción I2C (IWDG, HardFault),
  // el esclavo (MPU-6050 o SSD1306) puede quedar con SDA en LOW, bloqueando
  // todas las transacciones siguientes y causando freeze en el boot.
  // Se generan 9 pulsos manuales en SCL para forzar al esclavo a soltar SDA,
  // luego condición STOP, antes de cualquier uso real del periférico I2C.
  {
    GPIO_InitTypeDef gi = {0};
    HAL_I2C_DeInit(&hi2c1);                       // libera el periférico y los pines AF

    gi.Pin   = GPIO_PIN_8 | GPIO_PIN_9;            // PB8=SCL, PB9=SDA
    gi.Mode  = GPIO_MODE_OUTPUT_OD;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gi);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(5);

    for (int _i = 0; _i < 9; _i++) {              // 9 pulsos SCL
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
      HAL_Delay(1);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
      HAL_Delay(1);
      if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET)
        break;                                     // SDA libre, listo
    }

    // Condición STOP: SDA low → high con SCL high
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);   HAL_Delay(5);

    // Restaurar pines a función alternativa I2C1 y reinicializar el periférico
    gi.Mode      = GPIO_MODE_AF_OD;
    gi.Alternate = GPIO_AF4_I2C1;
    gi.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &gi);
    HAL_I2C_Init(&hi2c1);
  }

  // ── IWDG (Independent Watchdog) ──────────────────────────────────────────
  // Resetea el MCU si el loop principal deja de ejecutarse por más de ~2s.
  // LSI (~32 kHz) es independiente del clock principal; no se puede detener.
  IWDG->KR  = 0x5555U;
  IWDG->PR  = 0x06U;            // /256 → tick ≈ 8 ms
  IWDG->RLR = 250U;             // timeout = 251 × 8 ms ≈ 2 s
  { uint32_t _t = HAL_GetTick();
    while (IWDG->SR && (HAL_GetTick() - _t) < 50U) {} }
  IWDG->KR  = 0xAAAAU;
  IWDG->KR  = 0xCCCCU;

  // Prioridades de interrupts — CubeMX las resetea a 0 al regenerar; las fijamos acá.
  // Fix 2026-07-01: DMA de I2C subido a prio 0 (antes empataba en 1 con EXTI15_10),
  // para que el fin de transacción I2C/MPU siempre pueda preemptar a los encoders.
  // Con el giro de 180° generando ráfagas de EXTI15_10, el empate dejaba el DMA I2C
  // pendiente en cola -> freeze de lectura MPU -> PWM congelado al último valor
  // (alto, en pleno pivot) -> robot "aceleradísimo" y trabado hasta que el IWDG resetea.
  // Los encoders ya no usan EXTI: se decodifican por muestreo periódico con TIM2 (250 us),
  // dejando EXTI exclusivamente para MPU_INT y evitando storms/re-entradas compartidas.
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);  // I2C1 RX DMA
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);  // I2C1 TX DMA
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 2, 0);  // ADC1 DMA
  HAL_NVIC_SetPriority(EXTI15_10_IRQn,    2, 0);  // MPU (PB12)
  HAL_NVIC_SetPriority(TIM2_IRQn,         3, 0);  // muestreo de encoders, debajo de DMA/I2C y MPU
  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);              // PA8 ya no interrumpe
  HAL_NVIC_SetPriority(TIM5_IRQn,         6, 0);  // libre / no usado por encoders

  CDC_Attach_Rx(USBRxData);
  nBytesTx = 0;
  HAL_UART_Receive_IT(&huart1, &dataRx, 1);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcValues, 8);
  adc_dma_last_ms = HAL_GetTick();
  HAL_TIM_Base_Start_IT(&htim1);   // 10 ms
  HAL_TIM_Base_Start_IT(&htim2);   // 250 us (si lo vas a usar)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

  I2C_Manager_Init();

  PWM_init();

  static _sESP01Handle esp01Handle = {
      .aDoCHPD         = esp01_chpd,                  // Controla CH_PD
      .aWriteUSARTByte = uart_send_byte,          // Envía un byte por UART
      .bufRX           = esp01RxBuf,              // Buffer de recepción
      .iwRX            = &esp01IwRx,              // Índice de escritura
      .sizeBufferRX    = sizeof(esp01RxBuf)       // Tamaño del buffer
  };

  ESP01_Init(&esp01Handle);                        // Copia el handle interno :contentReference[oaicite:1]{index=1}
  ESP01_AttachChangeState(appOnESP01ChangeState);
  esp01_chpd(1);  // Pone CH_PD a nivel alto para sacar al módulo de reset
  HAL_Delay(100);
  ESP01_AttachDebugStr(ESP01_USB_DbgStr);
  ESP01_SetWIFI(wifiSSID, wifiPassword);

  unerRx.buff = unerRxBuffer;
  unerRx.mask = RXBUFSIZE - 1;
  unerTx.buff = unerTxBuffer;
  unerTx.mask = TXBUFSIZE - 1;
  UNER_Init(&unerRx, &unerTx, &ax, &ay, &az, &gx, &gy, &gz);
  UNER_RegisterADCBuffer(adcAvg, 8);  // array adcValues[8]
  UNER_RegisterMotorSpeed(&motorRightVelocity, &motorLeftVelocity);
  UNER_RegisterAngle(&roll_deg, &pitch_deg);
  UNER_RegisterProportionalControl(&KP_value, &KD_value, &KI_value, &KV_brake_value);
  UNER_RegisterSteering(&steering_adjustment);
  UNER_RegisterFlags(NULL, &f_resetMassCenter, &f_send_csv_log, &f_send_wifi_log, &f_change_display);
  UNER_RegisterLineControl(&KP_LINE, &KD_LINE, &KI_LINE, &LINE_THRESHOLD, &LINE_SPEED_TARGET, NULL);
  UNER_RegisterManualControl(&manual_setpoint_cmd, &manual_steering_cmd, &manual_cmd_last_ms);
  UNER_RegisterRotationCmd(&manual_rot_target_deg, &manual_rot_trigger);
  UNER_RegisterSetpointTrim(&setpoint_trim);
  UNER_RegisterRobotState(&robot_state);

  SSD1306_RegisterPlatform(&SSD1306_plat);
  SSD1306_Init();

  SSD1306_DrawBitmap(0, 0, unerLogo, 128, 64, SSD1306_COLOR_WHITE);
  SSD1306_UpdateScreen_Blocking();

  MPU6050_RegisterPlatform(&mpuPlat);
  int status = MPU6050_Init();

  if (status != MPU6050_OK) {
      mpu_initialized = 0;
      USB_Debug("MPU INIT FAIL\r\n");
  } else {
      mpu_initialized = 1;
      USB_Debug("MPU INIT OK\r\n");

#ifdef MPU_USE_FIXED_BIAS
      MPU6050_SetBias(FIXED_BIAS_AX, FIXED_BIAS_AY, FIXED_BIAS_AZ,
                      FIXED_BIAS_GX, FIXED_BIAS_GY, FIXED_BIAS_GZ);
      USB_Debug("MPU BIAS FIJO OK\r\n");
#else
      HAL_Delay(500); // Espera que se estabilice el movimiento del switch de encendido
      MPU6050_Calibrate();
      USB_Debug("MPU CAL OK\r\n");
      {
          int32_t b_ax, b_ay, b_az, b_gx, b_gy, b_gz;
          MPU6050_GetBias(&b_ax, &b_ay, &b_az, &b_gx, &b_gy, &b_gz);
          char bias_buf[80];
          int blen = snprintf(bias_buf, sizeof(bias_buf),
              "BIAS ax=%ld ay=%ld az=%ld gx=%ld gy=%ld gz=%ld\r\n",
              b_ax, b_ay, b_az, b_gx, b_gy, b_gz);
          if (blen > 0) USB_DebugSend((uint8_t*)bias_buf, (uint16_t)blen);
      }
#endif

      mpu_req_pending = 1;
      USB_Debug("MPU first read request\r\n");
      MPU6050_StartRead_DMA();
  }



  tmo100ms = 10;
  is10ms   = 0;
  is250us  = 0;
  mpu6050Counter = 0;
  aliveCounter = 0;

  motorRightVelocity = 0;
  motorLeftVelocity  = 0;

  esp01IwRx = 0;
  esp01IrRx = 0;

  KP_value = KP;
  KD_value = KD;
  KI_value = KI;
  KV_brake_value = KV_BRAKE_STRONG;
  dynamic_setpoint = SETPOINT_ANGLE + setpoint_trim;
  dynamic_setpoint_f = SETPOINT_ANGLE + setpoint_trim;
  base_setpoint_f = SETPOINT_ANGLE + setpoint_trim;
  brake_setpoint_f = 0.0f;

  // Initialize DWT for micros()
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  // Use raw bit 0 if DWT_CTRL_CYCCNT_Msk is not defined (standard for Cortex-M4)
  DWT->CTRL |= 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  IWDG->KR = 0xAAAAU;  // patada al IWDG: resetea el contador (debe llegar cada <2s)

	  // Control sincronizado con MPU — corre exactamente cuando hay dato, 100 Hz estable.
	  // Watchdog integrado: si pasan >150 ms sin dato nuevo (DMA I2C colgado),
	  // se ejecuta un bus recovery de I2C y se relanza la lectura del MPU sin resetear el MCU.
	  {
	      static uint32_t last_ctrl_ms = 0;

	      if (mpu_data_ready_for_ctrl) {
	          mpu_data_ready_for_ctrl = 0;
	          last_ctrl_ms = HAL_GetTick();
	          ControlStep10ms();
	      }

	      if (mpu_initialized && last_ctrl_ms != 0 &&
	          (HAL_GetTick() - last_ctrl_ms) > 150U) {
	          I2C1_Recover();
	          mpu_req_pending = 1;
	          MPU6050_StartRead_DMA();
	          last_ctrl_ms = HAL_GetTick();
	      }
	  }

	  if (i2c_process_pending) {
	      i2c_process_pending = 0;
	      I2C_Manager_Process();
	  }

	  if(is10ms) {
	      is10ms = 0;

	      static uint8_t subtick = 0;
	      subtick = (subtick + 1) % 10;

          if (adc_recover_pending ||
              (adc_dma_last_ms != 0 && (HAL_GetTick() - adc_dma_last_ms) > 50U)) {
              ADC1_Recover();
          }

	      // Estas dos van siempre — son rápidas y críticas
	      ESP01_Timeout10ms();
	      UpdateADC_MovingAverage();

	      // El resto se reparte en subticks
	      switch (subtick) {
	          case 0:
	              ESP01_Task();
	              break;
	          case 1:
	              break;
	          case 2:
	              if (UNER_ShouldSendAllSensors()) UNER_SendAllSensors();
	              break;
	          case 3:
	        	  if (f_resetMassCenter && !I2C_Manager_IsBusy()) {
	        	      MPU6050_Calibrate();
	        	      f_resetMassCenter = 0;
	        	      if (mpu_initialized && !mpu_req_pending) {
	        	          mpu_req_pending = 1;
	        	          MPU6050_StartRead_DMA();
	        	      }
	        	  }
	              break;
	          case 4:
	              tmo100ms--;
	              if (tmo100ms == 0) {
	                  tmo100ms = 4;
	                  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	                  HAL_GPIO_TogglePin(INTEGRATED_LED_GPIO_Port, INTEGRATED_LED_Pin);
	              }
	              break;
	          default:
	              break;
	      }

	      // ── Botón KEY: simple / doble / triple / largo ──────────────────────────
	      {
	                  uint8_t key_now = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
	                  uint32_t now    = HAL_GetTick();

	                  // Flanco de bajada: empieza pulsación
	                  if (key_prev == 1 && key_now == 0) {
	                      key_last_ms = now;
	                  }

	                  // Click largo: detectar mientras está presionado, sin esperar soltar
	                  if (key_now == 0 && key_last_ms != 0 && (now - key_last_ms) > 800) {
	                      f_change_display = (f_change_display + 1) % 6;
	                      key_last_ms = 0;   // ← evita que se dispare repetidamente
	                  }

	                  // Flanco de subida: soltó el botón
	                  if (key_prev == 0 && key_now == 1) {
	                      uint32_t held = now - key_last_ms;

	                      if (key_last_ms != 0 && held > 20) {
	                          uint32_t since_last = now - key_click_time;
	                          key_click_time = now;  // ← siempre actualizar al soltar

	                          if (since_last < 400 && key_click_count > 0) {
	                              // Continúa la secuencia
	                              key_click_count++;
	                          } else {
	                              // Nueva secuencia
	                              key_click_count = 1;
	                          }
	                      }
	                      key_last_ms = 0;
	                  }

	                  // Resolución por timeout: 400ms desde el último click sin otro click
	                  if (key_click_count > 0 && (now - key_click_time) > 400) {

	                      if (key_click_count == 1) {
	                          if (robot_state == ROBOT_STATE_IDLE) {
	                              robot_state = ROBOT_STATE_BALANCE_ONLY;
	                          } else {
	                              robot_state = ROBOT_STATE_IDLE;
	                          }

	                      } else if (key_click_count == 2) {
	                          if (robot_state == ROBOT_STATE_IDLE ||
	                              robot_state == ROBOT_STATE_BALANCE_ONLY) {
	                              robot_state = ROBOT_STATE_LINE_FOLLOWING;
	                              obj_detect_ignore_until_ms = HAL_GetTick() + 4000U;
	                          } else {
	                              robot_state = ROBOT_STATE_IDLE;
	                          }

	                      } else if (key_click_count == 3) {
	                          if (robot_state == ROBOT_STATE_BALANCE_AND_SPEED) {
	                              robot_state = ROBOT_STATE_IDLE;
	                          } else {
	                              robot_state = ROBOT_STATE_BALANCE_AND_SPEED;
	                          }
	                      } else if (key_click_count == 4) {
                              if (robot_state == ROBOT_STATE_MANUAL_CONTROL) {
                                  robot_state = ROBOT_STATE_IDLE;
                              } else {
                                  robot_state = ROBOT_STATE_MANUAL_CONTROL;
                              }
                          } else if (key_click_count >= 5) {
                              if (robot_state == ROBOT_STATE_MOTOR_TEST) {
                                  robot_state = ROBOT_STATE_IDLE;
                              } else {
                                  robot_state = ROBOT_STATE_MOTOR_TEST;
                              }
                          }

	                      key_click_count = 0;
	                  }

	                  key_prev = key_now;
	              }
	  }

	  ProcessEspRxLimited();
	  UNER_Task(); 		// Procesa tramas UNER recibidas
	  usb_service_tx();

	  if (i2c_process_pending) {
	      i2c_process_pending = 0;  // limpiar ANTES de llamar Process
	      I2C_Manager_Process();
	  }
	  i2c1_tx_busy = I2C_Manager_IsBusy();
	  SSD1306_UpdateScreen();

	  if (SSD1306_IsUpdateDone() && !i2c1_tx_busy && !mpu_req_pending && !mpu_data_ready_for_ctrl) {
	      uint32_t now = HAL_GetTick();
	      uint32_t display_interval_ms =
	              ((robot_state == ROBOT_STATE_IDLE) || f_fallen)
	              ? DISPLAY_UPDATE_INTERVAL_IDLE_MS
	              : DISPLAY_UPDATE_INTERVAL_ACTIVE_MS;
	      if ((now - last_display_ms) >= display_interval_ms) {
	          last_display_ms = now;
	          updateDisplay();
	      }
	  }

	  if (dataTx && huart1.gState == HAL_UART_STATE_READY) {
	      uart_tx_byte = dataTx;
	      dataTx = 0;
	      HAL_UART_Transmit_IT(&huart1, &uart_tx_byte, 1);
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 5;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 9599;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 95;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 249;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 959;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 959;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 95;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|LED_BLINKER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 LED_BLINKER_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|LED_BLINKER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MPU_INT_Pin */
  GPIO_InitStruct.Pin = MPU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MPU_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* Encoder RIGHT — PA8 canal A, PB13 canal B (cuadratura por muestreo con TIM2) */
  GPIO_InitStruct.Pin   = GPIO_PIN_8;
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Encoder RIGHT canal B + Encoder LEFT canal A y B — entradas simples, sin EXTI */
  GPIO_InitStruct.Pin   = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      for (volatile uint32_t i = 0; i < 300000; i++);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
