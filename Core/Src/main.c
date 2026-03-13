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

// --- Line Search & Loss Control ---
typedef enum {
    LINE_STATE_FOLLOWING = 0,  // Siguiendo línea normalmente
    LINE_STATE_LOST,           // Línea perdida, frenando y buscando
    LINE_STATE_SEARCHING,      // Girando suavemente para buscar
} eLineState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_Pin 			GPIO_PIN_13
#define LED_GPIO_Port 		GPIOC

#define CH_PD_GPIO_Port  	GPIOB
#define CH_PD_Pin        	GPIO_PIN_2

#define UDP_RX_SIZE  	 	512
#define UDP_RX_MASK   		(UDP_RX_SIZE - 1)
#define USB_TX_BUF_SIZE 	512
#define USB_TX_BUF_MASK 	(USB_TX_BUF_SIZE-1)

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
#define KP     		2.900f	// 2.745f antes
#define KD     		0.180f	// 0.17f antes
#define KI    		0.030f
#define BETA_G 		0.060f		 // LPF for Gyro
#define BETA_A 		0.020f        // LPF for Accel

#define MOTOR_GAIN 		2.5f
#define SETPOINT_ANGLE 	0.0f

// Complementary Filter
#define ALPHA 0.98f
#define DT 0.002f

// LOGGING MACROS
#define LOG_ENABLE 1
#define LOG_DECIM  5		// Frecuencia de envio de log csv mediante USB
#define LOG_WIFI_DECIM 10	// Frecuencia de envio de log binario mediante WIFI

// Filter Control Parameters
#define I_MAX  100.0f       // Max Integral Term
#define DT_MIN 0.0005f      // Min valid DT (0.5ms)
#define DT_MAX 0.01f        // Max valid DT (10ms)

// Fall detection (hysteresis)
#define FALL_ANGLE      38.0f   // grados: detecta caída
#define RECOVER_ANGLE   4.0f   // grados: condición para volver a balancear
#define UPSIDE_DOWN_ANGLE    140.0f   // desde acá ya lo consideramos boca abajo

// --- DT fijo calibrado ---
#define DT_WARMUP_SAMPLES	200
#define BETA_JITTER    		0.01f

#define KV_BRAKE         0.20f  // cuánto inclina el setpoint por velocidad estimada
#define VEL_DECAY        0.970f // decaimiento del estimado (1.0=sin decay, 0.99=decay rápido)
#define VEL_LPF_BETA     0.20f  // suavizado de la velocidad estimada
#define KV_LINE_BRAKE 	 0.10f

#define LINE_LOST_TIMEOUT_MS   2000// ms sin línea antes de entrar en búsqueda
#define LINE_LOST_STEERING     12.0f // steering suave para cuando recién se pierde la línea
#define LINE_SEARCH_STEERING   20.0f // steering suave para buscar (era 30 de máx)
#define LINE_SEARCH_ANGLE      0.2f  // ángulo mínimo de avance durante búsqueda
#define LINE_ANGLE_MIN  	   0.05f

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
static uint8_t sendModulesCounter, aliveCounter, mpu6050Counter;
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
//const char *wifiIp = "192.168.1.39";

//const char *wifiSSID     = "Wifi Habitaciones";
//const char *wifiPassword = "toyotakia";
//const char *wifiIp = "192.168.1.52";

int16_t motorRightVelocity = 0;
int16_t motorLeftVelocity  = 0;

static float integral = 0.0f;
static float steering_adjustment = 0.0f;
static float filtered_roll_deg = 0.0f;

// New Control Variables
static uint32_t last_ctrl_us = 0;
static float gyro_f = 0.0f;
static float accel_roll_f = 0.0f;

uint8_t f_line_following = 0; // Enables line following mode
static float line_error_prev = 0.0f;
static float line_integral = 0.0f;

uint8_t f_balancing = 0;	// En 0 (cero) desactiva los motores del PID y en 1 ativa los motores con el PID
uint8_t f_resetMassCenter = 0; // Resetea el centro de gravedad en el cual el auto hace balance

// LOGGING VARIABLES
static uint32_t log_counter = 0;
static uint32_t last_log_us = 0;
static uint8_t  log_header_sent = 0;
uint8_t f_send_csv_log = 0;
uint8_t f_send_wifi_log = 0;
uint8_t f_change_display = 2;

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
float BETA_G_value;
float BETA_A_value;
float KV_brake_value;

// Line Follower Variables
float KP_LINE = 23.0f;
float KD_LINE = 4.0f;
float KI_LINE = 0.0f;
float LINE_THRESHOLD = 3000.0f;
float LINE_ANGLE = 0.08f;  // Base inclination (degrees) for forward movement

static eLineState line_state       = LINE_STATE_FOLLOWING;
static uint32_t   line_lost_ms     = 0;   // tick cuando se perdió la línea
static float      line_search_dir  = 1.0f; // dirección de búsqueda (+1 o -1)
static float      last_line_dir    = 1.0f; // última dirección válida de la línea (+1 o -1)

static uint8_t upside_down_count = 0;
static uint8_t upright_count = 0;




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
void MPU6050_RegisterPlatform(MPU6050_Platform_t *plat);
void MPU6050_ProcessDMA(void);

void UpdateADC_MovingAverage(void);

void calculate_tilt(int16_t ax, int16_t ay, int16_t az, float *out_roll_deg, float *out_pitch_deg);

void updateDisplay(void);

static void esp01_chpd(uint8_t val);
void ESP01_AttachChangeState(OnESP01ChangeState aOnESP01ChangeState);
void appOnESP01ChangeState(_eESP01STATUS state);

static inline uint32_t micros(void) {
    return DWT->CYCCNT / (SystemCoreClock / 1000000);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static char* fast_cat_int(char* buf, int32_t val) {
    if (val < 0) {
        *buf++ = '-';
        val = -val;
    }
    char tmp[12];
    int i = 0;
    if (val == 0) {
        tmp[i++] = '0';
    } else {
        while (val > 0) {
            tmp[i++] = (val % 10) + '0';
            val /= 10;
        }
    }
    while (i > 0) {
        *buf++ = tmp[--i];
    }
    return buf;
}

static char* fast_cat_uint(char* buf, uint32_t val) {
    char tmp[12];
    int i = 0;
    if (val == 0) {
        tmp[i++] = '0';
    } else {
        while (val > 0) {
            tmp[i++] = (val % 10) + '0';
            val /= 10;
        }
    }
    while (i > 0) {
        *buf++ = tmp[--i];
    }
    return buf;
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
    }

    if (htim->Instance == TIM5) {        // 2 ms
            is2ms = 1;
    }
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance != I2C1) return;
    i2c1_tx_busy = 0;
    MPU6050_ProcessDMA();
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
        // Mitad derecha: P, D, I, BG, BA, KV
        // -------------------------------------------------------
        {
            const uint16_t rx = SCREEN_W / 2 + 2;

            const char *param_labels[6] = { "P:", "D:", "I:", "BG:", "BA:", "KV:" };
            float       param_vals[6]   = { KP_value, KD_value, KI_value,
                                            BETA_G_value, BETA_A_value, KV_BRAKE };

            for (uint8_t i = 0; i < 6; i++) {
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

                uint16_t label_w = (i < 3) ? 2 * Font_7x10.FontWidth
                                           : 3 * Font_7x10.FontWidth;
                SSD1306_GotoXY(rx + label_w, y);
                SSD1306_Puts(fbuf, &Font_7x10, SSD1306_COLOR_WHITE);
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

    } else if (f_change_display == 1) {
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

    } else if (f_change_display == 2) {
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
    } else if (f_change_display == 3) {
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

  int16_t ax, ay, az;	// Inicializo variables de aceleracion y giroscopio
  int16_t gx, gy, gz;

  unerRx.buff = unerRxBuffer;
  unerRx.mask = RXBUFSIZE - 1;
  unerTx.buff = unerTxBuffer;
  unerTx.mask = TXBUFSIZE - 1;
  UNER_Init(&unerRx, &unerTx, &ax, &ay, &az, &gx, &gy, &gz);
  UNER_RegisterADCBuffer(adcAvg, 8);  // array adcValues[8]
  UNER_RegisterMotorSpeed(&motorRightVelocity, &motorLeftVelocity);
  UNER_RegisterAngle(&roll_deg, &pitch_deg);
  UNER_RegisterProportionalControl(&KP_value, &KD_value, &KI_value, &BETA_G_value, &BETA_A_value, &KV_brake_value);
  UNER_RegisterSteering(&steering_adjustment);
  UNER_RegisterFlags(&f_balancing, &f_resetMassCenter, &f_send_csv_log, &f_send_wifi_log, &f_change_display);
  UNER_RegisterLineControl(&KP_LINE, &KD_LINE, &KI_LINE, &LINE_THRESHOLD, &LINE_ANGLE, &f_line_following);

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
  BETA_G_value = BETA_G;
  BETA_A_value = BETA_A;
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
	  if(is2ms) {
	      is2ms = 0;

	      if (MPU6050_IsDataReady()) {
	          MPU6050_ClearDataReady();

	          MPU6050_GetAccel(&ax, &ay, &az);
	          MPU6050_GetGyro(&gx, &gy, &gz);

	          // 1. Medir DT real siempre
	          uint32_t now_us = micros();
	          if (last_ctrl_us == 0) last_ctrl_us = now_us - 2000;
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
	                  USB_Debug("DT_FIXED calibrado: %d us\r\n", (int)(dt_fixed * 1e6f));
	              }
	          }

	          float jitter = fabsf(dt_real - dt_fixed);
	          dt_jitter_ema = dt_jitter_ema + BETA_JITTER * (jitter - dt_jitter_ema);
	          if (jitter > dt_jitter_max) dt_jitter_max = jitter;

	          const float ANG_SIGN = +1.0f;

	          // 2) Gyro LPF
	          float gyro_rate_dps = ANG_SIGN * ((float)gx / 131.0f);
	          gyro_f += BETA_G_value * (gyro_rate_dps - gyro_f);

	          // 3) Accel angle (roll) LPF
	          float accel_ang_deg = ANG_SIGN * (atan2f(ay, az) * (180.0f / M_PI));
	          accel_roll_f += BETA_A_value * (accel_ang_deg - accel_roll_f);

	          // 4) Complementary Filter
	          filtered_roll_deg = ALPHA * (filtered_roll_deg + gyro_f * dt_fixed)
	                            + (1.0f - ALPHA) * accel_roll_f;

	          // Variables de línea
	          float line_error = 0.0f;
	          float line_angle_cmd = LINE_ANGLE;
	          uint8_t line_detected = 0;

	          if (f_line_following) {
	              velocity_est   = 0.0f;
	              velocity_est_f = 0.0f;

	              float s0 = adcValues[0];
	              float s1 = adcValues[1];
	              float s2 = adcValues[2];
	              float s3 = adcValues[3];

	              float w0 = (s0 > LINE_THRESHOLD) ? s0 : 0.0f;
	              float w1 = (s1 > LINE_THRESHOLD) ? s1 : 0.0f;
	              float w2 = (s2 > LINE_THRESHOLD) ? s2 : 0.0f;
	              float w3 = (s3 > LINE_THRESHOLD) ? s3 : 0.0f;
	              float w_sum = w0 + w1 + w2 + w3;

	              line_detected = (w_sum > 0.0f);

	              if (line_detected) {
	                  line_error = ((w0 * 1.0f + w1 * 0.33f) - (w3 * 1.0f + w2 * 0.33f)) / w_sum;

	                  // Actualizar memoria de la última dirección válida
	                  if (line_error > 0.05f) {
	                      last_line_dir = 1.0f;
	                  } else if (line_error < -0.05f) {
	                      last_line_dir = -1.0f;
	                  }
	              }

	              float abs_line_error = fabsf(line_error);
	              float forward_factor = abs_line_error / 0.5f;

	              if (forward_factor > 1.0f) forward_factor = 1.0f;
	              if (forward_factor < 0.0f) forward_factor = 0.0f;

	              line_angle_cmd = LINE_ANGLE_MIN + (LINE_ANGLE - LINE_ANGLE_MIN) * forward_factor;

	              if (line_angle_cmd > LINE_ANGLE) line_angle_cmd = LINE_ANGLE;
	              if (line_angle_cmd < LINE_ANGLE_MIN) line_angle_cmd = LINE_ANGLE_MIN;

	          } else {
	              velocity_est    = VEL_DECAY * (velocity_est + gyro_f * dt_fixed);
	              velocity_est_f += VEL_LPF_BETA * (velocity_est - velocity_est_f);
	          }

	          static uint8_t prev_line_following = 0;
	          if (f_line_following && !prev_line_following) {
	              integral            = 0.0f;
	              line_integral       = 0.0f;
	              line_error_prev     = 0.0f;
	              steering_adjustment = 0.0f;
	              velocity_est        = 0.0f;
	              velocity_est_f      = 0.0f;
	              line_state          = LINE_STATE_FOLLOWING;
	              dynamic_setpoint    = SETPOINT_ANGLE;
	              dynamic_setpoint_f  = SETPOINT_ANGLE;
	          }
	          prev_line_following = f_line_following;

	          // 6. Setpoint dinámico
	          if (f_line_following) {
	              dynamic_setpoint = SETPOINT_ANGLE + line_angle_cmd;

	              if (dynamic_setpoint >  LINE_ANGLE) dynamic_setpoint =  LINE_ANGLE;
	              if (dynamic_setpoint < -LINE_ANGLE) dynamic_setpoint = -LINE_ANGLE;
	          } else {
	              dynamic_setpoint = SETPOINT_ANGLE + (velocity_est_f * KV_brake_value);
	          }

	          if (dynamic_setpoint >  5.0f) dynamic_setpoint =  5.0f;
	          if (dynamic_setpoint < -5.0f) dynamic_setpoint = -5.0f;

	          float sp_step_max = 0.0005f;  // Variacion maxima de angulo de avance para seguir linea
	          float sp_delta = dynamic_setpoint - dynamic_setpoint_f;

	          if (sp_delta >  sp_step_max) sp_delta =  sp_step_max;
	          if (sp_delta < -sp_step_max) sp_delta = -sp_step_max;

	          dynamic_setpoint_f += sp_delta;

	          // -------------------------------------------------------
	          // FALL DETECTION
	          // -------------------------------------------------------
	          float abs_roll_filt = fabsf(filtered_roll_deg);
	          float abs_roll_raw  = fabsf(accel_ang_deg);

	          uint8_t upright_now     = (abs_roll_raw < RECOVER_ANGLE);
	          uint8_t upside_down_now = (abs_roll_raw > UPSIDE_DOWN_ANGLE);
	          uint8_t fall_by_angle   = (abs_roll_filt > FALL_ANGLE);

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
	              if (fall_by_angle || fall_upside_down) {
	                  f_fallen = 1;

	                  integral            = 0.0f;
	                  velocity_est        = 0.0f;
	                  velocity_est_f      = 0.0f;
	                  line_integral       = 0.0f;
	                  line_error_prev     = 0.0f;
	                  steering_adjustment = 0.0f;

	                  motorRightVelocity  = 0;
	                  motorLeftVelocity   = 0;
	              }
	          } else {
	              if (recover_by_angle && !fall_upside_down) {
	                  f_fallen = 0;

	                  accel_roll_f      = accel_ang_deg;
	                  filtered_roll_deg = accel_ang_deg;
	                  gyro_f            = 0.0f;

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
	              } else {
	                  motorRightVelocity = 0;
	                  motorLeftVelocity  = 0;
	              }
	          }

	          float error   = dynamic_setpoint_f - filtered_roll_deg;
	          float p_term  = 0.0f;
	          float i_term  = 0.0f;
	          float d_term  = 0.0f;
	          float output  = 0.0f;
	          float pwm_cmd = 0.0f;
	          float pwm_sat = 0.0f;
	          uint8_t sat_flag = 0;

	          if (!f_fallen) {
	              p_term = KP_value * error;
	              i_term = KI_value * integral;
	              d_term = -KD_value * gyro_f;
	              output = p_term + i_term + d_term;

	              pwm_cmd = output * MOTOR_GAIN;
	              pwm_sat = pwm_cmd;

	              if (pwm_sat >  100.0f) { pwm_sat =  100.0f; sat_flag = 1; }
	              if (pwm_sat < -100.0f) { pwm_sat = -100.0f; sat_flag = 1; }

	              if (f_line_following) {
	                  if (pwm_sat >  40.0f) pwm_sat =  40.0f;
	                  if (pwm_sat < -40.0f) pwm_sat = -40.0f;
	              }

	              float pwm_limit = f_line_following ? 40.0f : 100.0f;
	              if (fabsf(pwm_cmd) <= pwm_limit) {
	                  integral += error * dt_fixed;
	              } else {
	                  if (pwm_cmd >  pwm_limit && error < 0) integral += error * dt_fixed;
	                  else if (pwm_cmd < -pwm_limit && error > 0) integral += error * dt_fixed;
	              }

	              sat_flag = (fabsf(pwm_cmd) > pwm_limit) ? 1 : 0;

	              if (integral >  I_MAX) integral =  I_MAX;
	              if (integral < -I_MAX) integral = -I_MAX;

	              if (f_line_following) {
	                  if (integral >  2.0f) integral =  2.0f;
	                  if (integral < -2.0f) integral = -2.0f;
	              }

	              if (f_line_following) {
	                  uint8_t line_pivot_active = 0;

	                  switch (line_state) {
	                      case LINE_STATE_FOLLOWING:
	                          if (line_detected) {
	                              float p_line = KP_LINE * line_error;

	                              line_integral += line_error * dt_fixed;
	                              if (line_integral >  5.0f) line_integral =  5.0f;
	                              if (line_integral < -5.0f) line_integral = -5.0f;

	                              float i_line = KI_LINE * line_integral;
	                              float d_line = KD_LINE * ((line_error - line_error_prev) / dt_fixed);

	                              line_error_prev = line_error;
	                              steering_adjustment = p_line + i_line + d_line;
	                          } else {
	                              line_lost_ms = HAL_GetTick();
	                              line_state = LINE_STATE_LOST;
	                              line_search_dir = last_line_dir;
	                              line_integral = 0.0f; // Resetear integral para no afectar el control perdido
	                          }

	                          if (steering_adjustment >  30.0f) steering_adjustment =  30.0f;
	                          if (steering_adjustment < -30.0f) steering_adjustment = -30.0f;
	                          break;

	                      case LINE_STATE_LOST:
	                          if (line_detected) {
	                              line_integral   = 0.0f;
	                              line_error_prev = 0.0f;
	                              line_state      = LINE_STATE_FOLLOWING;
	                              // Dejamos que el controlador retome desde donde está para que sea suave
	                          } else if ((HAL_GetTick() - line_lost_ms) > LINE_LOST_TIMEOUT_MS) {
	                              line_state = LINE_STATE_SEARCHING;
	                          } else {
	                              // En vez de decaer, buscar suavemente hacia la dirección recordada
	                              float target_steering = line_search_dir * LINE_LOST_STEERING;
	                              // Filtro suave para llegar al valor objetivo
	                              steering_adjustment += 0.05f * (target_steering - steering_adjustment);
	                          }
	                          break;

	                      case LINE_STATE_SEARCHING:
	                          if (line_detected) {
	                              line_integral   = 0.0f;
	                              line_error_prev = 0.0f;
	                              line_state      = LINE_STATE_FOLLOWING;
	                          } else {
	                              // Búsqueda más agresiva en la misma dirección
	                              float target_steering = line_search_dir * LINE_SEARCH_STEERING;
	                              steering_adjustment += 0.05f * (target_steering - steering_adjustment);
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

	              } else {
	                  line_integral       = 0.0f;
	                  line_error_prev     = 0.0f;
	                  steering_adjustment = 0.0f;
	                  line_state          = LINE_STATE_FOLLOWING;

	                  float steering_scale = 1.0f - (fabsf(steering_adjustment) / 100.0f);
	                  float mR = pwm_sat * steering_scale + steering_adjustment;
	                  float mL = pwm_sat * steering_scale - steering_adjustment;

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

	          // --- LOGGING ---
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
	              wlog.dt_ctrl_us = (uint32_t)(dt * 1000000.0f);
	              UNER_SendWifiLogData(&wlog);
	          }

	          if (LOG_ENABLE && f_send_csv_log && (log_counter % LOG_DECIM == 0)) {
	              uint32_t dt_ctrl_us = (uint32_t)(dt * 1000000.0f);
	              uint32_t t_now_log  = micros();
	              uint32_t dt_log_us  = t_now_log - last_log_us;
	              last_log_us = t_now_log;
	              uint32_t t_ms = HAL_GetTick();

	              if (!log_header_sent) {
	                  char *header = "t_ms,dt_us,dt_ctrl_us,accel_roll,accel_roll_f,gyro_y,gyro_f,roll_filt,dyn_sp,error,p,i,d,output,pwm_cmd,pwm_sat,sat,mR,mL,pitch,ax,ay,az,gx,gy,gz\r\n";
	                  usb_enqueue_tx((uint8_t*)header, strlen(header));
	                  log_header_sent = 1;
	              }

	              float ay_f_log = (float)ay;
	              float az_f_log = (float)az;
	              float denom = sqrtf(ay_f_log*ay_f_log + az_f_log*az_f_log);
	              float accel_pitch_deg = atan2f(-ax, denom) * (180.0f / M_PI);

	              char line[256];
	              int32_t accel_mdeg   = (int32_t)(accel_ang_deg * 1000.0f);
	              int32_t accel_f_mdeg = (int32_t)(accel_roll_f * 1000.0f);
	              int32_t gyro_mdps    = (int32_t)(gyro_rate_dps * 1000.0f);
	              int32_t gyro_f_mdps  = (int32_t)(gyro_f * 1000.0f);
	              int32_t roll_mdeg    = (int32_t)(filtered_roll_deg * 1000.0f);
	              int32_t error_mdeg   = (int32_t)(error * 1000.0f);
	              int32_t p_m          = (int32_t)(p_term * 1000.0f);
	              int32_t i_m          = (int32_t)(i_term * 1000.0f);
	              int32_t d_m          = (int32_t)(d_term * 1000.0f);
	              int32_t out_m        = (int32_t)(output * 1000.0f);
	              int32_t pwm_cmd_c    = (int32_t)(pwm_cmd * 100.0f);
	              int32_t pwm_sat_c    = (int32_t)(pwm_sat * 100.0f);
	              int32_t pitch_mdeg   = (int32_t)(accel_pitch_deg * 1000.0f);
	              int32_t dyn_sp_m     = (int32_t)(dynamic_setpoint_f * 1000.0f);

	              char *ptr = line;
	              ptr = fast_cat_uint(ptr, t_ms); *ptr++ = ',';
	              ptr = fast_cat_uint(ptr, dt_log_us); *ptr++ = ',';
	              ptr = fast_cat_uint(ptr, dt_ctrl_us); *ptr++ = ',';
	              ptr = fast_cat_int(ptr, accel_mdeg); *ptr++ = ',';
	              ptr = fast_cat_int(ptr, accel_f_mdeg); *ptr++ = ',';
	              ptr = fast_cat_int(ptr, gyro_mdps); *ptr++ = ',';
	              ptr = fast_cat_int(ptr, gyro_f_mdps); *ptr++ = ',';
	              ptr = fast_cat_int(ptr, roll_mdeg); *ptr++ = ',';
	              ptr = fast_cat_int(ptr, dyn_sp_m); *ptr++ = ',';
	              ptr = fast_cat_int(ptr, error_mdeg); *ptr++ = ',';
	              ptr = fast_cat_int(ptr, p_m); *ptr++ = ',';
	              ptr = fast_cat_int(ptr, i_m); *ptr++ = ',';
	              ptr = fast_cat_int(ptr, d_m); *ptr++ = ',';
	              ptr = fast_cat_int(ptr, out_m); *ptr++ = ',';
	              ptr = fast_cat_int(ptr, pwm_cmd_c); *ptr++ = ',';
	              ptr = fast_cat_int(ptr, pwm_sat_c); *ptr++ = ',';
	              ptr = fast_cat_int(ptr, sat_flag); *ptr++ = ',';
	              ptr = fast_cat_int(ptr, motorRightVelocity); *ptr++ = ',';
	              ptr = fast_cat_int(ptr, motorLeftVelocity); *ptr++ = ',';
	              ptr = fast_cat_int(ptr, pitch_mdeg); *ptr++ = ',';
	              ptr = fast_cat_int(ptr, ax); *ptr++ = ',';
	              ptr = fast_cat_int(ptr, ay); *ptr++ = ',';
	              ptr = fast_cat_int(ptr, az); *ptr++ = ',';
	              ptr = fast_cat_int(ptr, gx); *ptr++ = ',';
	              ptr = fast_cat_int(ptr, gy); *ptr++ = ',';
	              ptr = fast_cat_int(ptr, gz);
	              *ptr++ = '\r';
	              *ptr++ = '\n';

	              usb_enqueue_tx((uint8_t*)line, (uint16_t)(ptr - line));
	          }

	          if (mpu_initialized && !i2c1_tx_busy && !f_resetMassCenter) {
	              MPU6050_StartRead_DMA();
	          }

	      } else {
	          if (mpu_initialized && !i2c1_tx_busy && !f_resetMassCenter) {
	              MPU6050_StartRead_DMA();
	          }
	      }

	      if (f_balancing && !f_fallen) {
	          MotorControl(motorRightVelocity, motorLeftVelocity);
	      } else {
	          MotorControl(0, 0);
	      }
	  }

	  if(is10ms) {
	      is10ms = 0;

	      // -------------------------------------------------------
	      // BLOQUE 10ms: Comunicaciones, display, ADC, tareas lentas
	      // -------------------------------------------------------

	      ESP01_Timeout10ms();
	      ESP01_Task();

	      tmo100ms--;
	      if (tmo100ms == 0) {
	          tmo100ms = 10;
	          HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	          HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);
	      }

	      sendModulesCounter++;
	      if (sendModulesCounter >= 20) {
	          sendModulesCounter = 0;
	          if (UNER_ShouldSendAllSensors()) {
	              UNER_SendAllSensors();
	          }
	      }

	      UpdateADC_MovingAverage();

	      static uint8_t display_timer = 0;
	      display_timer++;
	      if (display_timer >= 5) { // 50ms (20Hz) refresh rate
		  display_timer = 0;
		      if (SSD1306_IsUpdateDone()) {
		          updateDisplay();
		      }
	      }

	      if (f_resetMassCenter) {
	          if (!i2c1_tx_busy) {
	              MPU6050_Calibrate();
	              f_resetMassCenter = 0;
	          }
	      }
	  }

	  while ((esp01IwRx - esp01IrRx) & UDP_RX_MASK) {
		  uint8_t b = esp01RxBuf[esp01IrRx];
		  esp01IrRx = (esp01IrRx + 1) & UDP_RX_MASK;

		  // DEBUG: mostrar en USB cada byte recibido
		  char dbg[6];
		  snprintf(dbg, sizeof(dbg), "%02X ", b);
		  USB_DebugStr(dbg);

		  UNER_PushByte(b);
	  }

	  UNER_Task(); 		// Procesa tramas UNER recibidas
	  usb_service_tx();
	  SSD1306_UpdateScreen();

	  if (dataTx) {
		  HAL_UART_Transmit(&huart1, &dataTx, 1, 100);
		  dataTx = 0;
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

  /*Configure GPIO pins : PB2 LED_BLINKER_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|LED_BLINKER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
