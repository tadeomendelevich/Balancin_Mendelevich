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
#define KP 1.0f
#define KD 0.12f
#define KI 0.0f

#define MOTOR_GAIN 3.0f

#define SETPOINT_ANGLE 0.0

// Complementary Filter
#define ALPHA 0.98f
#define DT 0.002f

// New Filter Parameters
#define BETA_G 0.15f
#define BETA_A 0.10f
#define I_MAX 100.0f
#define MIN_DT 0.0005f
#define MAX_DT 0.01f

// LOGGING MACROS
#define LOG_ENABLE 1
#define LOG_DECIM  5
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

int x = 1;
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
//const char *wifiIp = "192.168.1.39";

//const char *wifiSSID     = "Wifi Habitaciones";
//const char *wifiPassword = "toyotakia";
//const char *wifiIp = "192.168.1.52";

int16_t motorRightVelocity = 0;
int16_t motorLeftVelocity  = 0;

static float previous_error = 0.0f;
static float integral = 0.0f;
static float steering_adjustment = 0.0f;
static float filtered_roll_deg = 0.0f;

// New Filter Variables
static float gyro_f = 0.0f;
static float accel_roll_f = 0.0f;
static uint32_t last_ctrl_us = 0;
static uint32_t dt_ctrl_us = 0; // For logging

float KP_value;
float KD_value;
float KI_value;

uint8_t f_balancing = 0;	// En 0 (cero) desactiva los motores del PID y en 1 ativa los motores con el PID
uint8_t f_resetMassCenter = 0; // Resetea el centro de gravedad en el cual el auto hace balance

// LOGGING VARIABLES
static uint32_t log_counter = 0;
static uint32_t last_log_us = 0;
static uint8_t  log_header_sent = 0;
uint8_t f_send_csv_log = 0;
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
    // Bases (no hace falta IT si solo usás PWM)
    HAL_TIM_Base_Start(&htim3);
    HAL_TIM_Base_Start(&htim4);

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
    // Limitar a [-100, 100]
    if (setMotorRight > 100)  setMotorRight = 100;
    if (setMotorRight < -100) setMotorRight = -100;
    if (setMotorLeft > 100)   setMotorLeft  = 100;
    if (setMotorLeft < -100)  setMotorLeft  = -100;

    // ARR actuales de cada timer (evita hardcodear 999, etc.)
    uint32_t arr3 = __HAL_TIM_GET_AUTORELOAD(&htim3); // derecho
    uint32_t arr4 = __HAL_TIM_GET_AUTORELOAD(&htim4); // izquierdo

    // |duty| en %
    uint32_t dutyR = (setMotorRight >= 0) ? (uint32_t)setMotorRight : (uint32_t)(-setMotorRight);
    uint32_t dutyL = (setMotorLeft  >= 0) ? (uint32_t)setMotorLeft  : (uint32_t)(-setMotorLeft);

    // % -> CCR (en 32 bits para evitar overflow)
    uint32_t ccrR = (arr3 * dutyR) / 100U;
    uint32_t ccrL = (arr4 * dutyL) / 100U;

    // ----- Motor derecho (TIM3) -----
    if (setMotorRight >= 0) {
        // Avance: CH1 activo, CH2 apagado
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccrR);   // PB4
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);      // PB5
    } else {
        // Reversa: CH2 activo, CH1 apagado
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);      // PB4
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ccrR);   // PB5
    }

    // ----- Motor izquierdo (TIM4) -----
    if (setMotorLeft >= 0) {
        // Avance: CH2 activo, CH1 apagado
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);      // PB6
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, ccrL);   // PB7
    } else {
        // Reversa: CH1 activo, CH2 apagado
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, ccrL);   // PB6
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);      // PB7
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
        // Ahora que ya tenemos IP propia, arrancamos el socket UDP
        ESP01_StartUDP(wifiIp, 30010, 30000);
    }
    // Si más adelante quieres TCP, aquí gestionas ESP01_UDPTCP_CONNECTED, etc.
}

void updateDisplay(void) {
    // 1) Limpia todo el buffer
    SSD1306_Fill(SSD1306_COLOR_BLACK);

    // 2) Línea divisoria vertical en medio
    SSD1306_DrawLine(
        SCREEN_W/2, 0,
        SCREEN_W/2, SCREEN_H - 1,
        SSD1306_COLOR_WHITE
    );

    // 3) MPU6050 – valores en la mitad izquierda (sin cambios)
    {
        const char* labels[6] = { "AX:", "AY:", "AZ:", "GX:", "GY:", "GZ:" };
        int16_t    values[6];
        MPU6050_GetAccel(&values[0], &values[1], &values[2]);
        MPU6050_GetGyro (&values[3], &values[4], &values[5]);

        char buf[7];

        for (int i = 0; i < 6; i++) {
            uint16_t y = 2 + i * 10;
            SSD1306_GotoXY(2, y);
            SSD1306_Puts(labels[i], &Font_7x10, SSD1306_COLOR_WHITE);
            itoa(values[i], buf, 10);
            uint16_t x = 2 + strlen(labels[i]) * Font_7x10.FontWidth;
            for (char *p = buf; *p; p++) {
                if (*p == '-') {
                    SSD1306_DrawLine(x, y + 3, x + 4, y + 3, SSD1306_COLOR_WHITE);
                    x += 6;
                } else {
                    SSD1306_DrawDigit5x7(*p - '0', x, y);
                    x += Font_5x7.FontWidth + 1;
                }
            }
        }
    }

    // 4) Mitad derecha: KP/KD/KI arriba + barras ADC (mitad de altura) abajo
    {
        // Font_7x10: 7px de ancho por caracter.
        // Zona derecha: 64px de ancho total.
        // Etiqueta "P:" = 2 chars = 14px. Valor "0.20" = 4 chars = 28px. Total = 42px. OK.
        // Si el valor es negativo "-0.20" = 5 chars = 35px + 14px = 49px. OK.

        const uint16_t rx = SCREEN_W / 2 + 2;   // margen izquierdo de 2px

        const char *pid_labels[3] = { "P:", "D:", "I:" };
        float       pid_vals[3]   = { KP_value, KD_value, KI_value };

        for (uint8_t i = 0; i < 3; i++) {
            uint16_t y = 1 + i * 10;   // y = 1, 11, 21

            // Etiqueta
            SSD1306_GotoXY(rx, y);
            SSD1306_Puts(pid_labels[i], &Font_7x10, SSD1306_COLOR_WHITE);

            // Valor float con 2 decimales, formateado manualmente
            char fbuf[8];
            float v = pid_vals[i];
            uint8_t neg = (v < 0);
            if (neg) v = -v;
            uint32_t int_part  = (uint32_t)v;
            uint32_t frac_part = (uint32_t)((v - (float)int_part) * 100.0f + 0.5f);
            if (neg)
                snprintf(fbuf, sizeof(fbuf), "-%lu.%02lu",
                         (unsigned long)int_part, (unsigned long)frac_part);
            else
                snprintf(fbuf, sizeof(fbuf), "%lu.%02lu",
                         (unsigned long)int_part, (unsigned long)frac_part);

            // Posiciona tras los 2 chars de la etiqueta
            SSD1306_GotoXY(rx + 2 * Font_7x10.FontWidth, y);
            SSD1306_Puts(fbuf, &Font_7x10, SSD1306_COLOR_WHITE);
        }

        // Línea separadora tras los 3 PID (y = 3*10 + 2 = 32)
        uint16_t sep_y = 32;
        SSD1306_DrawLine(SCREEN_W/2, sep_y, SCREEN_W - 1, sep_y, SSD1306_COLOR_WHITE);

        // --- Barras ADC ---
        const uint16_t bar_region_y = sep_y + 2;                            // 34
        const uint16_t digit_h      = Font_5x7.FontHeight + 2;              // espacio para dígito
        const uint16_t bar_region_h = SCREEN_H - bar_region_y - digit_h;   // altura disponible

        // Barras a la MITAD de la altura disponible
        const uint16_t bar_max_h  = (bar_region_h / 2) + 10;
        const uint16_t bar_base_y = bar_region_y + bar_region_h - 1;       // base común

        const uint16_t rw         = SCREEN_W / 2 - 1;
        const uint16_t bar_spacing = BAR_SPACING;
        const uint16_t bar_width   = (rw - (BAR_COUNT + 1) * bar_spacing) / BAR_COUNT;

        for (uint8_t i = 0; i < BAR_COUNT; i++) {
            uint16_t v  = adcAvg[i] > 4000 ? 4000 : adcAvg[i];
            uint16_t h  = (uint32_t)v * bar_max_h / 4000;
            uint16_t x0 = SCREEN_W/2 + bar_spacing + i * (bar_width + bar_spacing);
            uint16_t y0 = bar_base_y - h + 1;
            if (h > 0)
                SSD1306_DrawFilledRectangle(x0, y0, bar_width, h, SSD1306_COLOR_WHITE);

            // Dígito 1–8 centrado bajo cada barra
            uint16_t tx = x0 + (bar_width > Font_5x7.FontWidth
                                 ? (bar_width - Font_5x7.FontWidth) / 2 : 0);
            uint16_t ty = bar_base_y + 2;
            SSD1306_DrawDigit5x7(i + 1, tx, ty);
        }
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
  UNER_RegisterProportionalControl(&KP_value, &KD_value, &KI_value);
  UNER_RegisterSteering(&steering_adjustment);
  UNER_RegisterFlags(&f_balancing, &f_resetMassCenter, &f_send_csv_log);

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

		  // Iniciar lectura del sensor MPU6050
		  if (mpu_initialized && !i2c1_tx_busy && !f_resetMassCenter) {
			  MPU6050_StartRead_DMA();
		  }

		  // Comprobar si hay datos listos
		  if (MPU6050_IsDataReady()) {
			  MPU6050_ClearDataReady();
			  mpuDataReady = 1;
		  }

		  if (mpuDataReady) {
			  mpuDataReady = 0;
			  MPU6050_GetAccel(&ax, &ay, &az);
			  MPU6050_GetGyro(&gx, &gy, &gz);

			  // EMA, filtro complementario, control PD...
			  // ... (toda la lógica de control se ejecuta aquí)

			  // --- Complementary Filter & PID Control ---
			  // 1. Calculate Real dt
			  uint32_t now_us = micros();
			  float dt = (now_us - last_ctrl_us) * 1e-6f;
			  dt_ctrl_us = now_us - last_ctrl_us; // Store for logging
			  last_ctrl_us = now_us;

			  // Clamp dt
			  if (dt < MIN_DT) dt = MIN_DT;
			  if (dt > MAX_DT) dt = MAX_DT;

			  float accel_roll_deg = atan2f(ay, az) * (180.0f / M_PI);
			  float gyro_y_dps = (float)gy / 131.0f; // Sensibilidad del giróscopo a ±250dps

			  // 2. Filters
			  // Gyro Low Pass Filter
			  gyro_f += BETA_G * (gyro_y_dps - gyro_f);

			  // Accel Low Pass Filter
			  accel_roll_f += BETA_A * (accel_roll_deg - accel_roll_f);

			  // 3. Complementary Filter (using real dt and filtered inputs)
			  filtered_roll_deg = ALPHA * (filtered_roll_deg + gyro_f * dt) + (1.0f - ALPHA) * accel_roll_f;
			  float error = SETPOINT_ANGLE - filtered_roll_deg;

			  // PID Calculation
			  // P Term
			  float p_term = KP_value * error;

			  // D Term (Derivative on Measurement)
			  float d_term = -KD_value * gyro_f;

			  // I Term (use accumulated integral)
			  float i_term = KI_value * integral;

			  // Output Calculation
			  float output = p_term + i_term + d_term;
			  float pwm_cmd = output * MOTOR_GAIN;

			  // Saturation
			  float pwm_sat = pwm_cmd;
			  uint8_t sat_flag = 0;

			  if (pwm_sat > 100.0f) {
				  pwm_sat = 100.0f;
				  sat_flag = 1;
			  } else if (pwm_sat < -100.0f) {
				  pwm_sat = -100.0f;
				  sat_flag = 1;
			  }

			  // Anti-windup (Conditional Integration)
			  uint8_t allow_integration = 0;
			  if (sat_flag == 0) {
				  allow_integration = 1;
			  } else {
				  // If saturated positive, only allow if error is negative (trying to reduce output)
				  if (pwm_cmd > 100.0f && error < 0.0f) allow_integration = 1;
				  // If saturated negative, only allow if error is positive (trying to increase output)
				  else if (pwm_cmd < -100.0f && error > 0.0f) allow_integration = 1;
			  }

			  if (allow_integration) {
				  integral += error * dt;
			  }

			  // Hard Clamp on Integral
			  if (integral > I_MAX) integral = I_MAX;
			  else if (integral < -I_MAX) integral = -I_MAX;

			  previous_error = error;

			  // Final Motor Output Saturation
			  float mR = pwm_sat + steering_adjustment;
			  float mL = pwm_sat - steering_adjustment;

			  if (mR > 100.0f) mR = 100.0f;
			  if (mR < -100.0f) mR = -100.0f;

			  if (mL > 100.0f) mL = 100.0f;
			  if (mL < -100.0f) mL = -100.0f;

			  motorRightVelocity = (int16_t)mR;
			  motorLeftVelocity  = (int16_t)mL;

			  // Actualizar variables para reporte
			  roll_deg = filtered_roll_deg;

			  // --- LOGGING ---
              log_counter++;
              if (LOG_ENABLE && f_send_csv_log && (log_counter % LOG_DECIM == 0)) {
                  uint32_t now_us = micros();
                  uint32_t dt_log_us = now_us - last_log_us;
                  last_log_us = now_us;
                  uint32_t t_ms = HAL_GetTick();

                  // 1. USB CSV Logging
                  if (!log_header_sent) {
                      char *header = "t_ms,dt_us,accel_roll,gyro_y,roll_filt,error,p,i,d,output,pwm_cmd,pwm_sat,sat,mR,mL,pitch,ax,ay,az,gx,gy,gz,dt_ctrl_us,gyro_f,accel_roll_f\r\n";
                      usb_enqueue_tx((uint8_t*)header, strlen(header));
                      log_header_sent = 1;
                  }

                  // Calculate pitch for logging (using same math as calculate_tilt)
                  float ay_f = (float)ay;
                  float az_f = (float)az;
                  float denom = sqrtf(ay_f*ay_f + az_f*az_f);
                  float accel_pitch_deg = atan2f(-ax, denom) * (180.0f / M_PI);

                  char line[256];
                  // Scale floats to integers for lightweight formatting
                  // Angles/Error: x1000 (milli-degrees)
                  // PID/Output: x1000
                  // PWM: x100
                  int32_t accel_mdeg = (int32_t)(accel_roll_deg * 1000.0f);
                  int32_t gyro_mdps  = (int32_t)(gyro_y_dps * 1000.0f);
                  int32_t roll_mdeg  = (int32_t)(filtered_roll_deg * 1000.0f);
                  int32_t error_mdeg = (int32_t)(error * 1000.0f);
                  int32_t p_m        = (int32_t)(p_term * 1000.0f);
                  int32_t i_m        = (int32_t)(i_term * 1000.0f);
                  int32_t d_m        = (int32_t)(d_term * 1000.0f);
                  int32_t out_m      = (int32_t)(output * 1000.0f);
                  int32_t pwm_cmd_c  = (int32_t)(pwm_cmd * 100.0f);
                  int32_t pwm_sat_c  = (int32_t)(pwm_sat * 100.0f);
                  int32_t pitch_mdeg = (int32_t)(accel_pitch_deg * 1000.0f);
                  int32_t gyro_f_mdps = (int32_t)(gyro_f * 1000.0f);
                  int32_t accel_f_mdeg = (int32_t)(accel_roll_f * 1000.0f);

                  int len = snprintf(line, sizeof(line),
                      "%lu,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%d,%d,%d,%ld,%d,%d,%d,%d,%d,%d,%lu,%ld,%ld\r\n",
                      t_ms, dt_log_us,
                      accel_mdeg, gyro_mdps, roll_mdeg, error_mdeg,
                      p_m, i_m, d_m, out_m,
                      pwm_cmd_c, pwm_sat_c, sat_flag,
                      motorRightVelocity, motorLeftVelocity,
					  pitch_mdeg, ax, ay, az, gx, gy, gz,
                      dt_ctrl_us, gyro_f_mdps, accel_f_mdeg
                  );

                  if (len > 0) {
                      usb_enqueue_tx((uint8_t*)line, len);
                  }

                  /* UDP LOGGING (DISABLED FOR USB-ONLY TESTING)
                  // 2. UDP UNER Binary Logging
                  LogData_t logData;
                  logData.t_ms       = t_ms;
                  logData.dt_us      = dt_log_us;
                  logData.accel_roll = accel_roll_deg;
                  logData.gyro_y     = gyro_y_dps;
                  logData.roll_filt  = filtered_roll_deg;
                  logData.error      = error;
                  logData.p_term     = p_term;
                  logData.i_term     = i_term;
                  logData.d_term     = d_term;
                  logData.output     = output;
                  logData.pwm_cmd    = pwm_cmd;
                  logData.pwm_sat    = pwm_sat;
                  logData.sat_flag   = sat_flag;
                  logData.mR         = motorRightVelocity;
                  logData.mL         = motorLeftVelocity;

                  UNER_SendLogData(&logData);
                  */
              }
		  }

		  if(f_balancing) { // Si estoy en modo balanceo
			  MotorControl(motorRightVelocity, motorLeftVelocity);	// Actualizo motores con valores de PID
		  } else {
			  MotorControl(0, 0);	// Apago motores
		  }
	  }

	  if(is10ms) {
		  is10ms = 0;

		  ESP01_Timeout10ms();  	// Requerido por la librería ESP01
		  ESP01_Task(); 	// Procesa tramas ESP01 recibidas

		  tmo100ms--;
		  if (tmo100ms == 0) {
			  tmo100ms = 10;
			  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin); // Blink LED
			  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);     // Blink LED PB10
		  }

		  sendModulesCounter++;
		  if (sendModulesCounter >= 20) {
			  sendModulesCounter = 0;
			  if (UNER_ShouldSendAllSensors()) {
				  UNER_SendAllSensors();
			  }
		  }

		  // La lógica de ADC y Display se queda aquí para no sobrecargar el bucle rápido
		  UpdateADC_MovingAverage();
		  if (SSD1306_IsUpdateDone()) {
			  updateDisplay();
		  }

		  if (f_resetMassCenter) {
			  if (!i2c1_tx_busy) {
				  MPU6050_Calibrate();		// Calibración del mpu para restablecer el centro de gravedad y balance del auto
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 95;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim4.Init.Prescaler = 95;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
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
