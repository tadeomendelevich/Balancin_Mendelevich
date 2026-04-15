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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
    ROBOT_STATE_IDLE = 0,
    ROBOT_STATE_BALANCE_ONLY,
    ROBOT_STATE_BALANCE_AND_SPEED,
    ROBOT_STATE_LINE_FOLLOWING,
    ROBOT_STATE_MANUAL_CONTROL
} eRobotState;

// --- Line Search & Loss Control ---
typedef enum {
    LINE_STATE_FOLLOWING = 0,  // Siguiendo línea normalmente
    LINE_STATE_LOST,           // Línea perdida, frenando y buscando
    LINE_STATE_SEARCHING,      // Girando suavemente para buscar
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
#define ADC_AVERAGE_SIZE 	40

#define BAR_COUNT    		8
#define BAR_SPACING  		2
#define SCREEN_W     		SSD1306_WIDTH
#define SCREEN_H    		SSD1306_HEIGHT

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define ESP_USB_BUF_SIZE	512

// PID
#define KP     		3.100f
#define KD     		0.180f
#define KI    		0.010f

#define MOTOR_GAIN 		2.5f
#define SETPOINT_ANGLE 	0.0f

// Complementary Filter
#define ALPHA 0.98f
#define DT 0.010f

// LOGGING MACROS
#define LOG_ENABLE 1
#define LOG_DECIM  5		// Frecuencia de envio de log csv mediante USB
#define LOG_WIFI_DECIM 10	// Frecuencia de envio de log binario mediante WIFI

// Filter Control Parameters
#define I_MAX  100.0f       // Max Integral Term
#define DT_MIN 0.005f       // Min valid DT (5ms)
#define DT_MAX 0.05f        // Max valid DT (50ms)

// Fall detection (hysteresis)
#define FALL_ANGLE           60.0f
#define RECOVER_ANGLE        2.0f
#define UPSIDE_DOWN_ANGLE    120.0f  // más agresivo para detectar boca abajo antes
#define DEAD_ZONE_ANGLE      15.0f   // entre 35° y 120° → zona muerta, motores off

// --- DT fijo calibrado ---
#define DT_WARMUP_SAMPLES	200
#define BETA_JITTER    		0.01f

//#define KV_BRAKE         0.20f  // cuánto inclina el setpoint por velocidad estimada
//#define VEL_DECAY        0.970f // decaimiento del estimado (1.0=sin decay, 0.99=decay rápido)
#define KV_BRAKE         0.08f
#define VEL_DECAY        0.970f
#define VEL_LPF_BETA     0.06f
#define INTEGRAL_DECAY   0.990f
#define KV_LINE_BRAKE 	 0.10f

#define LINE_LOST_TIMEOUT_MS   2000// ms sin línea antes de entrar en búsqueda
#define LINE_LOST_STEERING     12.0f // steering suave para cuando recién se pierde la línea
#define LINE_ANGLE_MIN  	   0.00f

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
static float roll_deg = 0.0f;	// Ángulo de balanceo (eje Y, usado para el equilibrio)
static float pitch_deg = 0.0f;	// Ángulo de inclinación (eje X)


uint16_t adcValues[8];
static uint32_t adcSum[BAR_COUNT] = {0};	// Sumas acumuladas de las últimas ADC_AVERAGE_SIZE muestras
static uint16_t adcBuf[BAR_COUNT][ADC_AVERAGE_SIZE] = {{0}};		// Buffers circulares: [canal][posición en la ventana]
static uint8_t maIndex = 0;		// Índice circular común para todos los canales
static uint16_t adcAvg[BAR_COUNT] = {0};

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

//const char *wifiSSID     = "FCAL";
//const char *wifiPassword = "fcalconcordia.06-2019";
//const char *wifiIp = "172.23.205.98";

const char *wifiSSID     = "MEGACABLE FIBRA-2.4G-ckd0";
const char *wifiPassword = "djg19dlk";
const char *wifiIp 		 = "192.168.100.5";

//const char *wifiSSID     = "Delco_Mendelevich";
//const char *wifiPassword = "toyotakia";
//const char *wifiIp = "192.168.1.36";

//const char *wifiSSID     = "Wifi Habitaciones";
//const char *wifiPassword = "toyotakia";
//const char *wifiIp = "192.168.1.48";

int16_t motorRightVelocity = 0;
int16_t motorLeftVelocity  = 0;

static float integral = 0.0f;
static float steering_adjustment = 0.0f;
static float filtered_roll_deg = 0.0f;

static uint32_t last_ctrl_us = 0;

uint8_t robot_state = ROBOT_STATE_IDLE; // Controls the main state machine of the robot

static uint32_t manual_cmd_last_ms = 0;
static float manual_setpoint_cmd = 0.0f;
static float manual_steering_cmd = 0.0f;

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

static float    dt_fixed        = DT;          // arranca con el define como fallback
static uint8_t  dt_calibrated   = 0;
static uint32_t dt_warmup_count = 0;
static double   dt_warmup_sum   = 0.0;

static float dt_real        = 0.0f;		// Monitoreo de jitter
static float dt_jitter_max  = 0.0f;
static float dt_jitter_ema  = 0.0f;

static float velocity_est     = 0.0f;  // velocidad lineal estimada (en "unidades gyro integradas")
static float velocity_est_f   = 0.0f;  // versión filtrada para el setpoint
static float dynamic_setpoint = 0.0f;  // setpoint variable calculado cada ciclo
static float dynamic_setpoint_f = 0.0f;

float KP_value;
float KD_value;
float KI_value;
float KV_brake_value;

// Line Follower Variables
float KP_LINE = 10.0f;
float KD_LINE = 0.0f;
float KI_LINE = 0.0f;
float LINE_THRESHOLD = 3000.0f;
float LINE_ANGLE = 1;  // Base inclination (degrees) for forward movement

static eLineState line_state       = LINE_STATE_FOLLOWING;
static uint32_t   line_lost_ms     = 0;   // tick cuando se perdió la línea
static float      line_search_dir  = 1.0f; // dirección de búsqueda (+1 o -1)
static float      last_line_dir    = 1.0f; // última dirección válida de la línea (+1 o -1)
static float      line_error_f_d   = 0.0f;

static uint8_t upside_down_count = 0;
static uint8_t upright_count = 0;
static uint8_t fall_count = 0;

static uint8_t key_prev = 1;
static uint32_t key_last_ms = 0;
static uint32_t key_click_time = 0;
static uint8_t  key_click_count = 0;

static float manual_setpoint_ramped = 0.0f;  // setpoint de rampa aplicada
static float line_angle_ramped      = 0.0f;  // rampa de avance en line follower
static float pwm_sat_prev = 0.0f;

volatile uint8_t tick2ms_count = 0;

static uint8_t uart_tx_byte = 0;

static int16_t ax = 0, ay = 0, az = 0;
static int16_t gx = 0, gy = 0, gz = 0;

static float gyro_f = 0.0f;

static float accel_roll_f = 0.0f;
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

static inline uint32_t micros(void) {
    return DWT->CYCCNT / (SystemCoreClock / 1000000);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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
    }

    if (htim->Instance == TIM5) {        // 2 ms
    	if (tick2ms_count < 10) tick2ms_count++;
    }
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c->Instance == I2C1) {
		MPU6050_ProcessDMA();  // actualiza variables globales
		i2c1_tx_busy = 0;
	}
}

// I2C1 TX DMA completo (lo usa SSD1306 cuando transmite por DMA)
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        i2c1_tx_busy = 0;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        i2c1_tx_busy = 0;
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
    return (HAL_UART_Transmit(&huart1, &byte, 1, 100) == HAL_OK) ? 1 : 0;
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

// Callback comando
int my_ssd1306_write_cmd(void *ctx, uint8_t cmd) {
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR, (uint8_t[]){0x00, cmd}, 2, HAL_MAX_DELAY);

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
int my_ssd1306_write_data_async(void *ctx, const uint8_t *data, uint16_t len) {
    SSD1306_Ctx_t *c = (SSD1306_Ctx_t*)ctx;
    // 1) Si el bus está ocupado, lanza error
    if (*c->busy_flag) {
        SSD1306_plat.onError(ctx, -1);     // -1 = BUSY_ERROR
        return -1;
    }

    // 2) Preparo buffer (control byte + datos)
    static uint8_t dmaBuf[1 + SSD1306_WIDTH];
    dmaBuf[0] = 0x40;
    memcpy(&dmaBuf[1], data, len);

    // 3) Marco el bus como ocupado
    *c->busy_flag = 1;

    // 4) Lanzo la DMA
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit_DMA(c->hi2c, SSD1306_I2C_ADDR, dmaBuf,len + 1);
    if (st != HAL_OK) {
        // si falla al arrancar, limpio flag y notifico
        *c->busy_flag = 0;
        SSD1306_plat.onError(ctx, (int)st);
        return -1;
    }
    return 0;
}


// Callback busy-check
uint8_t my_ssd1306_is_busy(void *ctx) {
    return i2c1_tx_busy ? 1 : 0;
}

void my_ssd1306_errorCb(void *ctx, int err) {
	USB_Debug("ERROR SSD1306: 0x");
	USB_DebugHex(err);
	USB_Debug("\r\n");
    i2c1_tx_busy = 0;
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

int mpu_readRegDMA(void *ctx,
                   uint8_t devAddr,
                   uint8_t regAddr,
                   uint8_t *data,
                   uint16_t len)
{
    // Si el bus está ocupado, salimos
    if (i2c1_tx_busy) {
        return -1;
    }
    // Tomamos el bus
    i2c1_tx_busy = 1;

    HAL_StatusTypeDef st = HAL_I2C_Mem_Read_DMA(
        (I2C_HandleTypeDef*)ctx,
        devAddr,
        regAddr,
        I2C_MEMADD_SIZE_8BIT,
        data,
        len
    );
    if (st != HAL_OK) {
        // Si no arranca, liberamos y notificamos
        i2c1_tx_busy = 0;
        mpuPlat.onError(mpuPlat.ctx, st);
        return -1;
    }
    return 0;
}


void mpu_delayMs(void *ctx, uint32_t ms) {
    HAL_Delay(ms);
}

void mpu_errorCb(void *ctx, int err) {
    static uint8_t error_printed = 0;
    if (!error_printed) {
        error_printed = 1;
        mpu_initialized = 0;  // impide futuras lecturas DMA
        USB_Debug("ERROR MPU6050: 0x%02X\r\n", err);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == MPU_INT_Pin)
    {
        if (!i2c1_tx_busy)
        {
            i2c1_tx_busy = 1;
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

void updateDisplay(void) {
    SSD1306_Fill(SSD1306_COLOR_BLACK);

    if (f_change_display == 0) {
        // -------------------------------------------------------
        // PANTALLA 0
        // -------------------------------------------------------

        // Línea divisoria vertical
        SSD1306_DrawLine(SCREEN_W/2, 0, SCREEN_W/2, SCREEN_H - 1, SSD1306_COLOR_WHITE);

        // -------------------------------------------------------
        // MPU6050: 6 filas pegadas al fondo, izquierda
        // -------------------------------------------------------
        {
            const char* labels[6] = { "AX:", "AY:", "AZ:", "GX:", "GY:", "GZ:" };
            int16_t     values[6];
            MPU6050_GetAccel(&values[0], &values[1], &values[2]);
            MPU6050_GetGyro (&values[3], &values[4], &values[5]);

            char buf[8];
            const uint16_t y_start = SCREEN_H - 6 * 10 + 2;

            for (int i = 0; i < 6; i++) {
                uint16_t y = y_start + i * 10;
                uint16_t x = 2;

                for (const char *p = labels[i]; *p; p++) {
                    if (*p == ':') {
                        SSD1306_DrawPixel(x + 1, y + 1, SSD1306_COLOR_WHITE);
                        SSD1306_DrawPixel(x + 1, y + 4, SSD1306_COLOR_WHITE);
                        x += 4;
                    } else {
                        SSD1306_DrawChar5x7(*p, x, y);
                        x += Font_5x7.FontWidth + 1;
                    }
                }

                itoa(values[i], buf, 10);
                for (char *p = buf; *p; p++) {
                    if (*p == '-') {
                        SSD1306_DrawLine(x, y + 3, x + 3, y + 3, SSD1306_COLOR_WHITE);
                        x += 5;
                    } else {
                        SSD1306_DrawChar5x7(*p, x, y);
                        x += Font_5x7.FontWidth + 1;
                    }
                }
            }
        }

        // -------------------------------------------------------
        // Zona superior izquierda: ícono WiFi grande
        // -------------------------------------------------------
        {
            const uint16_t ix = 40;
            const uint16_t iy = 2;

            if (f_wifi_connected) {
                for (int dy = 0; dy < 3; dy++)
                    for (int dx = 0; dx < 3; dx++)
                        SSD1306_DrawPixel(ix + 9 + dx, iy + 13 + dy, SSD1306_COLOR_WHITE);

                SSD1306_DrawPixel(ix + 6,  iy + 10, SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 7,  iy + 9,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 8,  iy + 9,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 9,  iy + 9,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 10, iy + 9,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 11, iy + 9,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 12, iy + 10, SSD1306_COLOR_WHITE);

                SSD1306_DrawPixel(ix + 3,  iy + 7,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 4,  iy + 6,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 5,  iy + 5,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 7,  iy + 4,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 8,  iy + 4,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 9,  iy + 4,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 10, iy + 4,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 11, iy + 4,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 13, iy + 5,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 14, iy + 6,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 15, iy + 7,  SSD1306_COLOR_WHITE);

                SSD1306_DrawPixel(ix + 0,  iy + 6,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 1,  iy + 4,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 2,  iy + 3,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 3,  iy + 2,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 5,  iy + 1,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 6,  iy + 0,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 7,  iy + 0,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 8,  iy + 0,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 9,  iy + 0,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 10, iy + 0,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 11, iy + 0,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 12, iy + 0,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 13, iy + 1,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 15, iy + 2,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 16, iy + 3,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 17, iy + 4,  SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(ix + 18, iy + 6,  SSD1306_COLOR_WHITE);

            } else {
                for (int k = 0; k < 13; k++) {
                    SSD1306_DrawPixel(ix + k,      iy + k,     SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix + k + 1,  iy + k,     SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix + 12 - k, iy + k,     SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix + 13 - k, iy + k,     SSD1306_COLOR_WHITE);
                }
            }
        }

        // -------------------------------------------------------
		// Mitad derecha: P, D, I + estado actual del robot
		// -------------------------------------------------------
		{
			const uint16_t rx = SCREEN_W / 2 + 2;

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
			    }
			}
		}

        // -------------------------------------------------------
        // Spinner
        // -------------------------------------------------------
        {
            const uint16_t sx = 48;
            const uint16_t sy = 56;

            static uint8_t spinPhase = 0;

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

        // ----- 4 barras ADC izquierda -----
        {
            const uint8_t  adc_count  = 4;
            const uint16_t bar_top    = 2;
            const uint16_t digit_y    = 55;
            const uint16_t bar_max_h  = digit_y - bar_top - 1;
            const uint16_t spacing    = 2;
            const uint16_t left_w     = 55;
            const uint16_t bar_width  = (left_w - (adc_count + 1) * spacing) / adc_count;

            for (uint8_t i = 0; i < adc_count; i++) {
                uint8_t adc_idx = (adc_count - 1) - i;
                uint16_t v = adcAvg[adc_idx] > 4095 ? 4095 : adcAvg[adc_idx];
                uint16_t h = (uint32_t)v * bar_max_h / 4095;
                uint16_t x0 = spacing + i * (bar_width + spacing);
                uint16_t y0 = digit_y - 1 - h;

                if (h > 0) {
                    SSD1306_DrawFilledRectangle(x0, y0, bar_width, h, SSD1306_COLOR_WHITE);
                }

                uint16_t tx = x0 + (bar_width - Font_5x7.FontWidth) / 2;
                SSD1306_DrawChar5x7('1' + adc_idx, tx, digit_y);
            }

            SSD1306_DrawLine(0, digit_y - 1, left_w - 1, digit_y - 1, SSD1306_COLOR_WHITE);
        }

        // ----- línea divisoria vertical -----
        SSD1306_DrawLine(57, 0, 57, SCREEN_H - 1, SSD1306_COLOR_WHITE);

        // ----- lado derecho -----
        {
            const char *robot_mode_str = "IDLE";
            const char *line_mode_str  = "OFF";

            switch (robot_state) {
                case ROBOT_STATE_IDLE:
                    robot_mode_str = "IDLE";
                    break;
                case ROBOT_STATE_BALANCE_ONLY:
                    robot_mode_str = "BAL";
                    break;
                case ROBOT_STATE_BALANCE_AND_SPEED:
                    robot_mode_str = "SPEED";
                    break;
                case ROBOT_STATE_LINE_FOLLOWING:
                    robot_mode_str = "LINE";
                    break;
                case ROBOT_STATE_MANUAL_CONTROL:
                    robot_mode_str = "MAN";
                    break;
                default:
                    robot_mode_str = "UNK";
                    break;
            }

            if (robot_state != ROBOT_STATE_LINE_FOLLOWING) {
                line_mode_str = "OFF";
            } else {
                switch (line_state) {
                    case LINE_STATE_FOLLOWING:
                        line_mode_str = "FOLLOW";
                        break;
                    case LINE_STATE_LOST:
                        line_mode_str = "LOST";
                        break;
                    case LINE_STATE_SEARCHING:
                        line_mode_str = "SEARCH";
                        break;
                    default:
                        line_mode_str = "UNK";
                        break;
                }
            }

            // WiFi chico arriba a la derecha
            {
                const uint16_t ix = 112;
                const uint16_t iy = 3;

                if (f_wifi_connected) {
                    SSD1306_DrawPixel(ix + 3, iy + 6, SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix + 4, iy + 6, SSD1306_COLOR_WHITE);

                    SSD1306_DrawPixel(ix + 2, iy + 5, SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix + 3, iy + 4, SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix + 4, iy + 4, SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix + 5, iy + 5, SSD1306_COLOR_WHITE);

                    SSD1306_DrawPixel(ix + 1, iy + 4, SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix + 2, iy + 3, SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix + 3, iy + 2, SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix + 4, iy + 2, SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix + 5, iy + 3, SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix + 6, iy + 4, SSD1306_COLOR_WHITE);

                    SSD1306_DrawPixel(ix + 0, iy + 3, SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix + 1, iy + 2, SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix + 2, iy + 1, SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix + 3, iy + 0, SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix + 4, iy + 0, SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix + 5, iy + 1, SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix + 6, iy + 2, SSD1306_COLOR_WHITE);
                    SSD1306_DrawPixel(ix + 7, iy + 3, SSD1306_COLOR_WHITE);
                } else {
                    SSD1306_DrawLine(ix + 1, iy + 1, ix + 6, iy + 6, SSD1306_COLOR_WHITE);
                    SSD1306_DrawLine(ix + 6, iy + 1, ix + 1, iy + 6, SSD1306_COLOR_WHITE);
                }
            }

            // línea separadora horizontal
            SSD1306_DrawLine(62, 15, 123, 15, SSD1306_COLOR_WHITE);

            // modo del auto grande en el centro
            {
                uint16_t title_w = strlen(robot_mode_str) * Font_7x10.FontWidth;
                uint16_t title_x = 58 + (70 - title_w) / 2;

                SSD1306_GotoXY(title_x, 20);
                SSD1306_Puts(robot_mode_str, &Font_7x10, SSD1306_COLOR_WHITE);
            }

            // línea chica inferior
            SSD1306_DrawLine(62, 47, 123, 47, SSD1306_COLOR_WHITE);

            // modo del seguidor abajo, a la izquierda del spinner
            {
                uint16_t x = 62;
                uint16_t y = 50;

                for (const char *p = line_mode_str; *p; p++) {
                    SSD1306_DrawChar5x7(*p, x, y);
                    x += Font_5x7.FontWidth + 1;
                }
            }

            // spinner abajo a la derecha
            {
                const uint16_t sx = 118;
                const uint16_t sy = 57;

                static uint8_t spinPhaseLF = 0;

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
            char buf[20];

            snprintf(buf, sizeof(buf), "ADC1=%4u", adcAvg[0]);
            SSD1306_GotoXY(2, 2);
            SSD1306_Puts(buf, &Font_7x10, SSD1306_COLOR_WHITE);

            snprintf(buf, sizeof(buf), "ADC2=%4u", adcAvg[1]);
            SSD1306_GotoXY(2, 16);
            SSD1306_Puts(buf, &Font_7x10, SSD1306_COLOR_WHITE);

            snprintf(buf, sizeof(buf), "ADC3=%4u", adcAvg[2]);
            SSD1306_GotoXY(2, 30);
            SSD1306_Puts(buf, &Font_7x10, SSD1306_COLOR_WHITE);

            snprintf(buf, sizeof(buf), "ADC4=%4u", adcAvg[3]);
            SSD1306_GotoXY(2, 44);
            SSD1306_Puts(buf, &Font_7x10, SSD1306_COLOR_WHITE);
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

    } else {
        SSD1306_GotoXY(30, 25);
        SSD1306_Puts("DISPLAY?", &Font_7x10, SSD1306_COLOR_WHITE);
    }
    SSD1306_RequestUpdate();
}

static void ControlStep10ms(void)
{

    if (MPU6050_IsDataReady()) {
        MPU6050_ClearDataReady();

        MPU6050_GetAccel(&ax, &ay, &az);
        MPU6050_GetGyro(&gx, &gy, &gz);

        // -------------------------------------------------------
        // TIMING
        // -------------------------------------------------------
        uint32_t now_us = micros();
        if (last_ctrl_us == 0) last_ctrl_us = now_us - 10000;
        float dt = (float)(now_us - last_ctrl_us) * 1e-6f;
        last_ctrl_us = now_us;

        if (dt < DT_MIN) dt = DT_MIN;
        if (dt > DT_MAX) dt = DT_MAX;

        dt_real = dt;

        if (!dt_calibrated) {
            dt_warmup_sum += dt_real;
            dt_warmup_count++;
            if (dt_warmup_count >= DT_WARMUP_SAMPLES) {
                dt_fixed = (float)(dt_warmup_sum / dt_warmup_count);
                dt_calibrated = 1;
            }
        }

        float jitter = fabsf(dt_real - dt_fixed);
        dt_jitter_ema = dt_jitter_ema + BETA_JITTER * (jitter - dt_jitter_ema);
        if (jitter > dt_jitter_max) dt_jitter_max = jitter;

        // dt fijo para control
        const float dt_ctrl = dt_fixed;

        // ciclo tarde
        uint8_t late_cycle = (dt_real > (dt_fixed * 1.8f)) ? 1 : 0;

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

        // Filtro complementario (fusión gyro + accel)
        filtered_roll_deg = ALPHA * (filtered_roll_deg + gyro_f * dt_fixed)
                          + (1.0f - ALPHA) * accel_ang_deg;

        // -------------------------------------------------------
        // LINE FOLLOWER INPUTS
        // -------------------------------------------------------
        float line_error = 0.0f;
        float line_angle_cmd = LINE_ANGLE;
        uint8_t line_detected = 0;
        float w_sum = 0.0f;
        static float adc_f[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        const float ADC_BETA = 0.3f;

        if (robot_state == ROBOT_STATE_LINE_FOLLOWING) {
            velocity_est = VEL_DECAY * (velocity_est + gyro_f * dt_ctrl);
            if (velocity_est >  20.0f) velocity_est =  20.0f;
            if (velocity_est < -20.0f) velocity_est = -20.0f;
            velocity_est_f += VEL_LPF_BETA * (velocity_est - velocity_est_f);

            adc_f[0] += ADC_BETA * ((float)adcValues[0] - adc_f[0]);
            adc_f[1] += ADC_BETA * ((float)adcValues[1] - adc_f[1]);
            adc_f[2] += ADC_BETA * ((float)adcValues[2] - adc_f[2]);
            adc_f[3] += ADC_BETA * ((float)adcValues[3] - adc_f[3]);

            float s0 = adc_f[0];
            float s1 = adc_f[1];
            float s2 = adc_f[2];
            float s3 = adc_f[3];

            float w0 = (s0 > LINE_THRESHOLD) ? s0 : 0.0f;
            float w1 = (s1 > LINE_THRESHOLD) ? s1 : 0.0f;
            float w2 = (s2 > LINE_THRESHOLD) ? s2 : 0.0f;
            float w3 = (s3 > LINE_THRESHOLD) ? s3 : 0.0f;
            w_sum = w0 + w1 + w2 + w3;

            line_detected = (w_sum > 0.0f);

            if (line_detected) {
                line_error = ((w0 * 1.0f + w1 * 0.33f) - (w3 * 1.0f + w2 * 0.33f)) / w_sum;

                if (line_error > 0.05f) {
                    last_line_dir = 1.0f;
                } else if (line_error < -0.05f) {
                    last_line_dir = -1.0f;
                }
            }

            float abs_line_error = fabsf(line_error);
            float forward_factor = 1.0f - (abs_line_error / 0.5f);

            if (forward_factor > 1.0f) forward_factor = 1.0f;
            if (forward_factor < 0.0f) forward_factor = 0.0f;

            line_angle_cmd = LINE_ANGLE_MIN + (LINE_ANGLE - LINE_ANGLE_MIN) * forward_factor;

            if (line_angle_cmd > LINE_ANGLE) line_angle_cmd = LINE_ANGLE;
            if (line_angle_cmd < LINE_ANGLE_MIN) line_angle_cmd = LINE_ANGLE_MIN;

        } else {
            if (robot_state == ROBOT_STATE_BALANCE_AND_SPEED) {
                velocity_est = VEL_DECAY * (velocity_est + gyro_f * dt_ctrl);
                if (velocity_est >  20.0f) velocity_est =  20.0f;
                if (velocity_est < -20.0f) velocity_est = -20.0f;
                velocity_est_f += VEL_LPF_BETA * (velocity_est - velocity_est_f);
            } else {
                velocity_est = 0.0f;
                velocity_est_f = 0.0f;
            }
        }

        // -------------------------------------------------------
        // CAMBIOS DE ESTADO
        // -------------------------------------------------------
        static eRobotState prev_robot_state = ROBOT_STATE_IDLE;

        if (robot_state == ROBOT_STATE_LINE_FOLLOWING && prev_robot_state != ROBOT_STATE_LINE_FOLLOWING) {
            integral            = 0.0f;
            line_integral       = 0.0f;
            line_error_prev     = 0.0f;
            line_error_f_d      = 0.0f;
            steering_adjustment = 0.0f;
            velocity_est        = 0.0f;
            velocity_est_f      = 0.0f;
            line_state          = LINE_STATE_FOLLOWING;
            dynamic_setpoint    = SETPOINT_ANGLE;
            dynamic_setpoint_f  = SETPOINT_ANGLE;
            line_lost_ms        = HAL_GetTick();
            line_angle_ramped     = 0.0f;
        }

        if ((robot_state == ROBOT_STATE_BALANCE_ONLY ||
             robot_state == ROBOT_STATE_BALANCE_AND_SPEED ||
             robot_state == ROBOT_STATE_MANUAL_CONTROL) &&
            (prev_robot_state != robot_state)) {

            integral            = 0.0f;
            steering_adjustment = 0.0f;
            velocity_est        = 0.0f;
            velocity_est_f      = 0.0f;
            dynamic_setpoint    = SETPOINT_ANGLE;
            dynamic_setpoint_f  = SETPOINT_ANGLE;
            pwm_sat_prev        = 0.0f;

            if (robot_state == ROBOT_STATE_MANUAL_CONTROL) {
                manual_setpoint_cmd    = 0.0f;
                manual_steering_cmd    = 0.0f;
                manual_cmd_last_ms     = HAL_GetTick();
                manual_setpoint_ramped = 0.0f;
                steering_adjustment    = 0.0f;
            }
        }

        prev_robot_state = robot_state;

        // -------------------------------------------------------
        // SETPOINT DINÁMICO
        // -------------------------------------------------------
        if (robot_state == ROBOT_STATE_LINE_FOLLOWING) {
            float line_target_angle = 0.0f;

            if (line_state == LINE_STATE_FOLLOWING && line_detected) {
                line_target_angle = line_angle_cmd;
            } else {
                line_target_angle = 0.0f;
            }

            {
                const float LINE_RAMP_UP   = 0.001f;
                const float LINE_RAMP_DOWN = 0.0008f;

                float ramp_delta = line_target_angle - line_angle_ramped;
                float ramp_rate  = (ramp_delta > 0.0f) ? LINE_RAMP_UP : LINE_RAMP_DOWN;

                if (fabsf(ramp_delta) <= ramp_rate) {
                    line_angle_ramped = line_target_angle;
                } else {
                    line_angle_ramped += (ramp_delta > 0.0f) ? ramp_rate : -ramp_rate;
                }
            }

            dynamic_setpoint = SETPOINT_ANGLE + line_angle_ramped;

            if (dynamic_setpoint >  LINE_ANGLE) dynamic_setpoint =  LINE_ANGLE;
            if (dynamic_setpoint < -LINE_ANGLE) dynamic_setpoint = -LINE_ANGLE;
        } else if (robot_state == ROBOT_STATE_MANUAL_CONTROL) {
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

            float ramp_target = SETPOINT_ANGLE + scaled_cmd;
            float ramp_delta  = ramp_target - manual_setpoint_ramped;
            float ramp_rate   = (ramp_delta > 0.0f) ? RAMP_RATE_UP : RAMP_RATE_DOWN;

            if (fabsf(ramp_delta) <= ramp_rate) {
                manual_setpoint_ramped = ramp_target;
            } else {
                manual_setpoint_ramped += (ramp_delta > 0.0f) ? ramp_rate : -ramp_rate;
            }

            velocity_est = VEL_DECAY * (velocity_est + gyro_f * dt_ctrl);
            if (velocity_est >  20.0f) velocity_est =  20.0f;
            if (velocity_est < -20.0f) velocity_est = -20.0f;
            velocity_est_f += VEL_LPF_BETA * (velocity_est - velocity_est_f);

            dynamic_setpoint = manual_setpoint_ramped - KV_brake_value * velocity_est_f;

        } else if (robot_state == ROBOT_STATE_BALANCE_AND_SPEED) {
            dynamic_setpoint = SETPOINT_ANGLE;
        } else {
            dynamic_setpoint = SETPOINT_ANGLE;
        }

        float sp_limit = (robot_state == ROBOT_STATE_BALANCE_AND_SPEED) ? 2.0f : 5.0f;
        if (dynamic_setpoint >  sp_limit) dynamic_setpoint =  sp_limit;
        if (dynamic_setpoint < -sp_limit) dynamic_setpoint = -sp_limit;

        {
            float sp_step_max;

            if (robot_state == ROBOT_STATE_MANUAL_CONTROL) {
                sp_step_max = 0.004f;
            } else if (robot_state == ROBOT_STATE_LINE_FOLLOWING) {
                sp_step_max = 0.0015f;
            } else {
                sp_step_max = 0.0005f;
            }

            float sp_delta = dynamic_setpoint - dynamic_setpoint_f;

            if (sp_delta >  sp_step_max) sp_delta =  sp_step_max;
            if (sp_delta < -sp_step_max) sp_delta = -sp_step_max;

            dynamic_setpoint_f += sp_delta;
        }

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
                velocity_est        = 0.0f;
                velocity_est_f      = 0.0f;
                line_integral       = 0.0f;
                line_error_prev     = 0.0f;
                steering_adjustment = 0.0f;
                gyro_f              = 0.0f;
                motorRightVelocity  = 0;
                motorLeftVelocity   = 0;
                pwm_sat_prev        = 0.0f;
            }
        } else {
            if (recover_by_angle && !fall_upside_down && !in_dead_zone) {
                f_fallen = 0;
                accel_roll_f      = accel_ang_deg;
                filtered_roll_deg = accel_ang_deg;
                integral            = 0.0f;
                velocity_est        = 0.0f;
                velocity_est_f      = 0.0f;
                line_integral       = 0.0f;
                line_error_prev     = 0.0f;
                steering_adjustment = 0.0f;
                dynamic_setpoint    = SETPOINT_ANGLE;
                dynamic_setpoint_f  = SETPOINT_ANGLE;
                upright_count       = 0;
                upside_down_count   = 0;
                fall_count          = 0;
                dead_zone_count     = 0;
                pwm_sat_prev        = 0.0f;
            } else {
                motorRightVelocity = 0;
                motorLeftVelocity  = 0;
                gyro_f = 0.0f;
                filtered_roll_deg = accel_ang_deg;
            }
        }

        // -------------------------------------------------------
        // PID
        // -------------------------------------------------------
        float error   = dynamic_setpoint_f - filtered_roll_deg;
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
            if (robot_state == ROBOT_STATE_MANUAL_CONTROL) {
                integral *= 0.970f;
            } else if (robot_state == ROBOT_STATE_BALANCE_AND_SPEED) {
                // no decay, acumula libremente dentro de límites
            } else if (robot_state == ROBOT_STATE_LINE_FOLLOWING) {
                // no decay aquí tampoco
            }

            p_term = KP_value * error;
            i_term = KI_value * integral;

            float gyro_for_d = gyro_f;
            if (gyro_for_d >  150.0f) gyro_for_d =  150.0f;
            if (gyro_for_d < -150.0f) gyro_for_d = -150.0f;
            d_term = -KD_value * gyro_for_d;

            if (d_term >  15.0f) d_term =  15.0f;
            if (d_term < -15.0f) d_term = -15.0f;

            output = p_term + i_term + d_term;

            pwm_cmd = output * MOTOR_GAIN;
            pwm_sat = pwm_cmd;

            if (pwm_sat >  100.0f) { pwm_sat =  100.0f; sat_flag = 1; }
            if (pwm_sat < -100.0f) { pwm_sat = -100.0f; sat_flag = 1; }

            if (robot_state == ROBOT_STATE_LINE_FOLLOWING) {
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
            } else if (robot_state == ROBOT_STATE_MANUAL_CONTROL) {
                pwm_limit = 55.0f;
            } else {
                pwm_limit = 100.0f;
            }

            {
                float pwm_step_max;

                if (robot_state == ROBOT_STATE_MANUAL_CONTROL) {
                    pwm_step_max = 2.0f;
                } else if (robot_state == ROBOT_STATE_LINE_FOLLOWING) {
                    pwm_step_max = 2.0f;
                } else if (robot_state == ROBOT_STATE_BALANCE_ONLY) {
                    pwm_step_max = 8.0f;   // ← más rápido, el péndulo no espera
                } else {
                    pwm_step_max = 5.0f;
                }

                float pwm_delta = pwm_sat - pwm_sat_prev;
                if (pwm_delta >  pwm_step_max) pwm_delta =  pwm_step_max;
                if (pwm_delta < -pwm_step_max) pwm_delta = -pwm_step_max;

                pwm_sat = pwm_sat_prev + pwm_delta;
                pwm_sat_prev = pwm_sat;
            }

            if (robot_state != ROBOT_STATE_MANUAL_CONTROL) {
                if (!late_cycle) {
                    if (fabsf(error) > 0.2f) {
                    	if (fabsf(pwm_sat) <= pwm_limit) {
                            integral += error * dt_ctrl;
                        } else {
                            if (pwm_cmd >  pwm_limit && error < 0) integral += error * dt_ctrl;
                            else if (pwm_cmd < -pwm_limit && error > 0) integral += error * dt_ctrl;
                        }
                    } else {
                        integral *= 0.95f;
                    }
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
                if (integral >  2.0f) integral =  2.0f;
                if (integral < -2.0f) integral = -2.0f;
            }

            if (robot_state == ROBOT_STATE_BALANCE_AND_SPEED) {
                if (integral >  15.0f) integral =  15.0f;
                if (integral < -15.0f) integral = -15.0f;
            }

            if (robot_state == ROBOT_STATE_MANUAL_CONTROL) {
                if (integral >  1.5f) integral =  1.5f;
                if (integral < -1.5f) integral = -1.5f;
            }

            if (robot_state == ROBOT_STATE_LINE_FOLLOWING) {
                uint8_t line_pivot_active = 0;

                switch (line_state) {
                    case LINE_STATE_FOLLOWING:
                    {
                        uint8_t line_detected_robust = line_detected && (w_sum > LINE_THRESHOLD * 0.5f);

                        if (line_detected_robust) {
                            line_lost_ms = HAL_GetTick();

                            float p_line = KP_LINE * line_error * fabsf(line_error);

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

                            float steering_target = p_line + i_line + d_line;
                            float steering_delta = steering_target - steering_adjustment;
                            float steering_rate_limit = 2.0f;

                            if (steering_delta >  steering_rate_limit) steering_delta =  steering_rate_limit;
                            if (steering_delta < -steering_rate_limit) steering_delta = -steering_rate_limit;

                            steering_adjustment += steering_delta;

                            log_p_line = p_line;
                            log_i_line = i_line;
                            log_d_line = d_line;

                        } else {
                            uint32_t ms_sin_linea = HAL_GetTick() - line_lost_ms;

                            if (ms_sin_linea > 1000) {
                                line_state      = LINE_STATE_LOST;
                                line_search_dir = last_line_dir;
                                line_integral   = 0.0f;
                            }
                        }

                        if (steering_adjustment >  20.0f) steering_adjustment =  20.0f;
                        if (steering_adjustment < -20.0f) steering_adjustment = -20.0f;
                        break;
                    }

                    case LINE_STATE_LOST:
                        if (line_detected) {
                            line_integral   = 0.0f;
                            line_error_prev = 0.0f;
                            line_lost_ms    = HAL_GetTick();
                            line_state      = LINE_STATE_FOLLOWING;
                        } else if ((HAL_GetTick() - line_lost_ms) > LINE_LOST_TIMEOUT_MS) {
                            line_state = LINE_STATE_SEARCHING;
                        } else {
                            float target = last_line_dir * 4.0f;
                            steering_adjustment += 0.02f * (target - steering_adjustment);
                        }
                        break;

                    case LINE_STATE_SEARCHING:
                        if (line_detected) {
                            line_integral   = 0.0f;
                            line_error_prev = 0.0f;
                            line_lost_ms    = HAL_GetTick();
                            line_state      = LINE_STATE_FOLLOWING;
                        } else {
                            float target = last_line_dir * 6.0f;
                            steering_adjustment += 0.02f * (target - steering_adjustment);
                        }
                        break;
                }

                if (integral >  2.0f) integral =  2.0f;
                if (integral < -2.0f) integral = -2.0f;
                if (pwm_sat >  40.0f) pwm_sat =  40.0f;
                if (pwm_sat < -40.0f) pwm_sat = -40.0f;

                if (!line_pivot_active) {
                    float half_steer = steering_adjustment * 0.5f;

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
                line_error_prev     = 0.0f;
                line_state          = LINE_STATE_FOLLOWING;

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

            } else {
                line_integral       = 0.0f;
                line_error_prev     = 0.0f;
                steering_adjustment = 0.0f;
                line_state          = LINE_STATE_FOLLOWING;

                float gz_dps = (float)gz / 131.0f;
                float yaw_correction = -gz_dps * 0.3f;

                float mR = pwm_sat + yaw_correction;
                float mL = pwm_sat - yaw_correction;

                if (mR >  100.0f) mR =  100.0f;
                if (mR < -100.0f) mR = -100.0f;
                if (mL >  100.0f) mL =  100.0f;
                if (mL < -100.0f) mL = -100.0f;

                motorRightVelocity = -(int16_t)mL;
                motorLeftVelocity  = -(int16_t)mR;
            }
        } else {
            motorRightVelocity = 0;
            motorLeftVelocity  = 0;
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
            wlog.dt_ctrl_us = (uint32_t)(dt_real * 1000000.0f);

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

        if (mpu_initialized && !i2c1_tx_busy && !f_resetMassCenter) {
            MPU6050_StartRead_DMA();
        }

    } else {
        if (mpu_initialized && !i2c1_tx_busy && !f_resetMassCenter) {
            MPU6050_StartRead_DMA();
        }
    }

    if ((robot_state != ROBOT_STATE_IDLE) && !f_fallen) {
        MotorControl(motorRightVelocity, motorLeftVelocity);
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
  CDC_Attach_Rx(USBRxData);
  nBytesTx = 0;
  HAL_UART_Receive_IT(&huart1, &dataRx, 1);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcValues, 8);
  HAL_TIM_Base_Start_IT(&htim1);   // 10 ms
  HAL_TIM_Base_Start_IT(&htim2);   // 250 us (si lo vas a usar)
  HAL_TIM_Base_Start_IT(&htim5);   // 2 ms (500 Hz)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

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
  UNER_RegisterLineControl(&KP_LINE, &KD_LINE, &KI_LINE, &LINE_THRESHOLD, &LINE_ANGLE, NULL);
  UNER_RegisterManualControl(&manual_setpoint_cmd, &manual_steering_cmd, &manual_cmd_last_ms);
  UNER_RegisterRobotState(&robot_state);

  SSD1306_RegisterPlatform(&SSD1306_plat);
  SSD1306_Init();

  SSD1306_DrawBitmap(0, 0, unerLogo, 128, 64, SSD1306_COLOR_WHITE);
  SSD1306_UpdateScreen_Blocking();

  MPU6050_RegisterPlatform(&mpuPlat);
  int status = MPU6050_Init();
  if (status != MPU6050_OK) {
   mpu_initialized = 0;
      USB_Debug("ERROR MPU6050: NO SE HA PODIDO INICIALIZAR EL MODULO MPU6050\r\n");
  } else {
    mpu_initialized = 1;
    MPU6050_Calibrate();		// Calibración
    MPU6050_StartRead_DMA();	// Lanzo priemra lectura
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
  KV_brake_value = KV_BRAKE;

  // Initialize DWT for micros()
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  // Use raw bit 0 if DWT_CTRL_CYCCNT_Msk is not defined (standard for Cortex-M4)
  DWT->CTRL |= 1;

  HAL_Delay(500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  while (tick2ms_count) {
		  tick2ms_count--;
	  }

	  if(is10ms) {
	      is10ms = 0;
		      ControlStep10ms();
	      static uint8_t subtick = 0;
	      subtick = (subtick + 1) % 10;

	      // Estas dos van siempre — son rápidas y críticas
	      ESP01_Timeout10ms();
	      UpdateADC_MovingAverage();

	      // El resto se reparte en subticks
	      switch (subtick) {
	          case 0:
	              ESP01_Task();
	              break;
	          case 1:
	              if (SSD1306_IsUpdateDone() && !i2c1_tx_busy) updateDisplay();
	              break;
	          case 2:
	              if (UNER_ShouldSendAllSensors()) UNER_SendAllSensors();
	              break;
	          case 3:
	              if (f_resetMassCenter && !i2c1_tx_busy) {
	                  MPU6050_Calibrate();
	                  f_resetMassCenter = 0;
	                  if (mpu_initialized) MPU6050_StartRead_DMA();
	              }
	              break;
	          case 4:
	              tmo100ms--;
	              if (tmo100ms == 0) {
	                  tmo100ms = 10;
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
	                      f_change_display = (f_change_display + 1) % 5;
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
	                          } else {
	                              robot_state = ROBOT_STATE_IDLE;
	                          }

	                      } else if (key_click_count == 3) {
	                          if (robot_state == ROBOT_STATE_BALANCE_AND_SPEED) {
	                              robot_state = ROBOT_STATE_IDLE;
	                          } else {
	                              robot_state = ROBOT_STATE_BALANCE_AND_SPEED;
	                          }
	                      } else if (key_click_count >= 4) {
                              if (robot_state == ROBOT_STATE_MANUAL_CONTROL) {
                                  robot_state = ROBOT_STATE_IDLE;
                              } else {
                                  robot_state = ROBOT_STATE_MANUAL_CONTROL;
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
	  if (!i2c1_tx_busy) {
		  SSD1306_UpdateScreen();
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

  /* USER CODE BEGIN MX_GPIO_Init_2 */

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
