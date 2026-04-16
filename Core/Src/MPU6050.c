/*
 * MPU6050.c
 * Librería para el sensor MPU6050 (acelerómetro + giroscopio)
 * Usa comunicación I2C implementado con el bus DMA
 * Escala las lecturas a valores físicos en m/s² y °/s usando enteros (fixed-point)
 *
 * Autor: Mendelevich Tadeo
 * Fecha: Mayo 2025
 */

#include "MPU6050.h"
#include <stdlib.h>

#define ACCEL_SENS        16384   // LSB por g
#define GYRO_SENS         131     // LSB por °/s
#define ACCEL_SCALE_C     981     // 9.81 m/s² × 100 (centésimas)
#define GYRO_SCALE_C      100     // 1.00 °/s × 100
#define CALIB_SAMPLES  	  500
// Puntero a la implementación de plataforma
static const MPU6050_Platform_t *_platform = NULL;

void MPU6050_RegisterPlatform(MPU6050_Platform_t *plat) {
    _platform = plat;
}

// Variables convertidas a unidades físicas con escala ×100 (2 decimales fijos)
int16_t ax_real; // Aceleración en X [centésimas de m/s²]
int16_t ay_real;
int16_t az_real;

int16_t gx_real; // Velocidad angular en X [centésimas de grados/segundo]
int16_t gy_real;
int16_t gz_real;

// Buffer de recepción único (Accel + Temp + Gyro = 14 bytes)
uint8_t mpu_dma_buffer[MPU6050_READ_SIZE];

// Flags de estado
uint8_t mpu_data_ready  = 0;

// Variables RAW leídas directamente del sensor (int16_t = complemento a dos)
int32_t ax, ay, az, gx, gy, gz;

static int32_t bias_ax, bias_ay, bias_az;
static int32_t bias_gx, bias_gy, bias_gz;

int MPU6050_Init(void)
{
    uint8_t who = 0;
    _platform->readReg(_platform->ctx, MPU6050_ADDR, WHO_AM_I_REG, &who, 1);
    if (who != 0x68) {
        return MPU6050_ERROR;
    }

    uint8_t data;

    // 1️⃣ Wake up
    data = 0x00;
    _platform->writeReg(_platform->ctx, MPU6050_ADDR, PWR_MGMT_1_REG, &data, 1);

    // 2️⃣ DLPF
    data = 0x03; // ~44 Hz
    _platform->writeReg(_platform->ctx, MPU6050_ADDR, CONFIG_REG, &data, 1);

    // Sample Rate = 1 kHz / (1 + SMPLRT_DIV)
    // 1 kHz / (1 + 4) = 200 Hz → 5 ms
    uint8_t div = 9;
    _platform->writeReg(_platform->ctx, MPU6050_ADDR, 0x19, &div, 1);

    // 4️⃣ Gyro ±250 dps
    data = 0x00;
    _platform->writeReg(_platform->ctx, MPU6050_ADDR, GYRO_CONFIG_REG, &data, 1);

    // 5️⃣ Accel ±2g
    data = 0x00;
    _platform->writeReg(_platform->ctx, MPU6050_ADDR, ACCEL_CONFIG_REG, &data, 1);

    // 6️⃣ Interrupción DATA READY
    data = 0x01;
    _platform->writeReg(_platform->ctx, MPU6050_ADDR, 0x38, &data, 1);

    return MPU6050_OK;
}


void MPU6050_StartRead_DMA(void) {
    // Lectura en ráfaga de 14 bytes empezando en ACCEL_XOUT_H (0x3B)
    _platform->readRegDMA(_platform->ctx, MPU6050_ADDR, ACCEL_XOUT_H_REG, mpu_dma_buffer, MPU6050_READ_SIZE);
}

uint8_t MPU6050_IsDataReady(void) {
    return mpu_data_ready;
}

void MPU6050_ClearDataReady(void) {
    mpu_data_ready = 0;
}

void MPU6050_GetAccel(int16_t *ax, int16_t *ay, int16_t *az) {
    if (ax) *ax = ax_real;
    if (ay) *ay = ay_real;
    if (az) *az = az_real;
}

void MPU6050_GetGyro(int16_t *gx, int16_t *gy, int16_t *gz) {
    if (gx) *gx = gx_real;
    if (gy) *gy = gy_real;
    if (gz) *gz = gz_real;
}

void MPU6050_ProcessDMA(void) {
    // --- Procesar Acelerómetro (bytes 0-5) ---
    int16_t raw_ax = (int16_t)(mpu_dma_buffer[0] << 8 | mpu_dma_buffer[1]);
    int16_t raw_ay = (int16_t)(mpu_dma_buffer[2] << 8 | mpu_dma_buffer[3]);
    int16_t raw_az = (int16_t)(mpu_dma_buffer[4] << 8 | mpu_dma_buffer[5]);

    // Temp (bytes 6-7) ignorada por ahora

    // --- Procesar Giroscopio (bytes 8-13) ---
    int16_t raw_gx = (int16_t)(mpu_dma_buffer[8] << 8 | mpu_dma_buffer[9]);
    int16_t raw_gy = (int16_t)(mpu_dma_buffer[10] << 8 | mpu_dma_buffer[11]);
    int16_t raw_gz = (int16_t)(mpu_dma_buffer[12] << 8 | mpu_dma_buffer[13]);

    // ----------------------------
    // 1) Compensa bias Acelerómetro
    // ----------------------------
    raw_ax -= (int16_t)bias_ax;
    raw_ay -= (int16_t)bias_ay;
    raw_az -= (int16_t)bias_az;

    // X
    ax_real = (raw_ax * ACCEL_SCALE_C) >> 14;
    // Y
    ay_real = (raw_ay * ACCEL_SCALE_C) >> 14;
    // Z
    az_real = (raw_az * ACCEL_SCALE_C) >> 14;

    // ----------------------------
    // 2) Compensa bias Giroscopio
    // ----------------------------
    raw_gx -= (int16_t)bias_gx;
    raw_gy -= (int16_t)bias_gy;
    raw_gz -= (int16_t)bias_gz;

    // X
    gx_real = (raw_gx * GYRO_SCALE_C) / GYRO_SENS;
    // Y
    gy_real = (raw_gy * GYRO_SCALE_C) / GYRO_SENS;
    // Z
    gz_real = (raw_gz * GYRO_SCALE_C) / GYRO_SENS;

    mpu_data_ready = 1; // Todos los datos listos
}

void MPU6050_Calibrate(void) {
    int32_t sum_ax=0, sum_ay=0, sum_az=0;
    int32_t sum_gx=0, sum_gy=0, sum_gz=0;
    int16_t raw_ax, raw_ay, raw_az;
    int16_t raw_gx, raw_gy, raw_gz;
    uint8_t buf[14]; // 14 bytes: 6 accel + 2 temp + 6 gyro

    for (int i = 0; i < CALIB_SAMPLES; ++i) {
        // 1) Lee acelerómetro, temp y giroscopio RAW en una sola transacción I2C
        _platform->readReg(_platform->ctx, MPU6050_ADDR, ACCEL_XOUT_H_REG, buf, 14);

        raw_ax = (int16_t)(buf[0]<<8 | buf[1]);
        raw_ay = (int16_t)(buf[2]<<8 | buf[3]);
        raw_az = (int16_t)(buf[4]<<8 | buf[5]);
        sum_ax += raw_ax;
        sum_ay += raw_ay;
        sum_az += raw_az;

        // bytes 6 y 7 son temperatura (se ignoran)

        raw_gx = (int16_t)(buf[8]<<8 | buf[9]);
        raw_gy = (int16_t)(buf[10]<<8 | buf[11]);
        raw_gz = (int16_t)(buf[12]<<8 | buf[13]);
        sum_gx += raw_gx;
        sum_gy += raw_gy;
        sum_gz += raw_gz;

        _platform->delayMs(_platform->ctx, 1);
    }
    // 2) Guarda sesgos promedio (en LSB)
    bias_ax = sum_ax / CALIB_SAMPLES;
    bias_ay = sum_ay / CALIB_SAMPLES;
    // Para Z restamos 1g en LSB:
    bias_az = sum_az / CALIB_SAMPLES - ACCEL_SENS;
    bias_gx = sum_gx / CALIB_SAMPLES;
    bias_gy = sum_gy / CALIB_SAMPLES;
    bias_gz = sum_gz / CALIB_SAMPLES;
}
