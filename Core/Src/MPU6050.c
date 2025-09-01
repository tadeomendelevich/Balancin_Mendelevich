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

uint8_t next_is_accel = 1;

// Variables convertidas a unidades físicas con escala ×100 (2 decimales fijos)
int16_t ax_real; // Aceleración en X [centésimas de m/s²]
int16_t ay_real;
int16_t az_real;

int16_t gx_real; // Velocidad angular en X [centésimas de grados/segundo]
int16_t gy_real;
int16_t gz_real;

// Buffers de recepción
uint8_t accel_buf[6];
uint8_t gyro_buf[6];

// Flags de estado
uint8_t mpu_accel_ready = 0;
uint8_t mpu_gyro_ready  = 0;

// Variables RAW leídas directamente del sensor (int16_t = complemento a dos)
int32_t ax, ay, az, gx, gy, gz;

static int32_t bias_ax, bias_ay, bias_az;
static int32_t bias_gx, bias_gy, bias_gz;

int MPU6050_Init(void)
{
	uint8_t who = 0;
	_platform->readReg(_platform->ctx, MPU6050_ADDR, WHO_AM_I_REG, &who, 1);
	if (who != 0x68) {
		return MPU6050_ERROR;    // devuelve -1 si el sensor no responde con 0x68
	}
    uint8_t data;

    // Salir del modo de bajo consumo (modo sleep)
    // Escritura en el registro PWR_MGMT_1 (0x6B)
    data = 0x00;
    _platform->writeReg(_platform->ctx, MPU6050_ADDR, PWR_MGMT_1_REG, &data, 1);

    // Configurar acelerómetro con rango ±2g (registro ACCEL_CONFIG = 0x1C, valor 0x00)
    data = 0x00;
    _platform->writeReg(_platform->ctx, MPU6050_ADDR, ACCEL_CONFIG_REG, &data, 1);

    // Configurar giroscopio con rango ±250°/s (registro GYRO_CONFIG = 0x1B, valor 0x00)
    data = 0x00;
    _platform->writeReg(_platform->ctx, MPU6050_ADDR, GYRO_CONFIG_REG, &data, 1);

    return MPU6050_OK;
}


void MPU6050_StartRead_Accel_DMA(void) {
	_platform->readRegDMA(_platform->ctx, MPU6050_ADDR, ACCEL_XOUT_H_REG, accel_buf, 6);
}

/**
 * @brief  Inicia lectura de giroscopio por DMA (no bloqueante)
 */
void MPU6050_StartRead_Gyro_DMA(void) {
	_platform->readRegDMA(_platform->ctx, MPU6050_ADDR, GYRO_XOUT_H_REG, gyro_buf, 6);
}

// ------------------------------------------------------------
// ➤ Funciones de estado público
// ------------------------------------------------------------
uint8_t MPU6050_IsAccelReady(void) {
    return mpu_accel_ready;
}
uint8_t MPU6050_IsGyroReady(void) {
    return mpu_gyro_ready;
}

// (Opcional) Limpiar flags si quieres
void MPU6050_ClearAccelReady(void) {
    mpu_accel_ready = 0;
}
void MPU6050_ClearGyroReady(void) {
    mpu_gyro_ready = 0;
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
	if (next_is_accel) {
		// --- Procesar acelerómetro ---
		int16_t raw_ax = (int16_t)(accel_buf[0] << 8 | accel_buf[1]);
		int16_t raw_ay = (int16_t)(accel_buf[2] << 8 | accel_buf[3]);
		int16_t raw_az = (int16_t)(accel_buf[4] << 8 | accel_buf[5]);

		// 1) Compensa bias:
		raw_ax -= (int16_t)bias_ax;
		raw_ay -= (int16_t)bias_ay;
		raw_az -= (int16_t)bias_az;

		// X
		if (abs(raw_ax) <= OFFSET_AX) {
			ax_real = 0;
		} else {
			// Multiplico primero (evito overflow: raw_ax ±32767 × 981 < 2^31)
			ax_real = (raw_ax * ACCEL_SCALE_C) >> 14;
		}
		// Y
		if (abs(raw_ay) <= OFFSET_AY) {
			ay_real = 0;
		} else {
			ay_real = (raw_ay * ACCEL_SCALE_C) >> 14;
		}
		// Z (fallback a 1 g si está cerca de cero)
		if (abs(raw_az) <= OFFSET_AZ) {
			az_real = ACCEL_SCALE_C;  // 1 g = 9.81 m/s² → 981 centésimas
		} else {
			az_real = (raw_az * ACCEL_SCALE_C) >> 14;
		}

		mpu_accel_ready = 1;      // aviso a main
		next_is_accel = 0;
		//USB_Debug("Raw A: X=%d Y=%d Z=%d\n", raw_ax, raw_ay, raw_az);
	} else {
		// --- Procesar giroscopio ---
		int16_t raw_gx = (int16_t)(gyro_buf[0] << 8 | gyro_buf[1]);
		int16_t raw_gy = (int16_t)(gyro_buf[2] << 8 | gyro_buf[3]);
		int16_t raw_gz = (int16_t)(gyro_buf[4] << 8 | gyro_buf[5]);


		// Compensa bias de giro:
		raw_gx -= (int16_t)bias_gx;
		raw_gy -= (int16_t)bias_gy;
		raw_gz -= (int16_t)bias_gz;

		// X
		if (abs(raw_gx) <= OFFSET_GX) {
			gx_real = 0;
		} else {
			gx_real = (raw_gx * GYRO_SCALE_C) / GYRO_SENS;
		}
		// Y
		if (abs(raw_gy) <= OFFSET_GY) {
			gy_real = 0;
		} else {
			gy_real = (raw_gy * GYRO_SCALE_C) / GYRO_SENS;
		}
		// Z
		if (abs(raw_gz) <= OFFSET_GZ) {
			gz_real = 0;
		} else {
			gz_real = (raw_gz * GYRO_SCALE_C) / GYRO_SENS;
		}

		mpu_gyro_ready = 1;       // aviso a main
		MPU6050_StartRead_Gyro_DMA();
		//USB_Debug("Raw G: X=%d Y=%d Z=%d\n", raw_gx, raw_gy, raw_gz);
		next_is_accel  = 1;       // la próxima será accel
	}
}

void MPU6050_Calibrate(void) {
    int32_t sum_ax=0, sum_ay=0, sum_az=0;
    int32_t sum_gx=0, sum_gy=0, sum_gz=0;
    int16_t raw_ax, raw_ay, raw_az;
    int16_t raw_gx, raw_gy, raw_gz;
    uint8_t buf[6];

    for (int i = 0; i < CALIB_SAMPLES; ++i) {
        // 1) Lee acelerómetro RAW
        _platform->readReg(_platform->ctx, MPU6050_ADDR, ACCEL_XOUT_H_REG, buf, 6);
        raw_ax = (int16_t)(buf[0]<<8 | buf[1]);
        raw_ay = (int16_t)(buf[2]<<8 | buf[3]);
        raw_az = (int16_t)(buf[4]<<8 | buf[5]);
        sum_ax += raw_ax;
        sum_ay += raw_ay;
        sum_az += raw_az;

        // 2) Lee giroscopio RAW
        _platform->readReg(_platform->ctx, MPU6050_ADDR, GYRO_XOUT_H_REG, buf, 6);
        raw_gx = (int16_t)(buf[0]<<8 | buf[1]);
        raw_gy = (int16_t)(buf[2]<<8 | buf[3]);
        raw_gz = (int16_t)(buf[4]<<8 | buf[5]);
        sum_gx += raw_gx;
        sum_gy += raw_gy;
        sum_gz += raw_gz;

        _platform->delayMs(_platform->ctx, 5);
    }
    // 3) Guarda sesgos promedio (en LSB)
    bias_ax = sum_ax / CALIB_SAMPLES;
    bias_ay = sum_ay / CALIB_SAMPLES;
    // Para Z restamos 1g en LSB:
    bias_az = sum_az / CALIB_SAMPLES - ACCEL_SENS;
    bias_gx = sum_gx / CALIB_SAMPLES;
    bias_gy = sum_gy / CALIB_SAMPLES;
    bias_gz = sum_gz / CALIB_SAMPLES;
}



