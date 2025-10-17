/*
 * MPU6050.h
 * -----------------------------------------------
 * Cabecera de la librería para el sensor MPU6050.
 * Define las direcciones de registro, offsets, escala fija,
 * y prototipos de funciones para lectura e inicialización.
 *
 * MPU6050, sensor MEMS de 6 ejes (3 acelerómetro + 3 giroscopio),
 * que se comunica mediante el protocolo I2C.
 *
 * Autor: Tadeo Mendelevich
 * Fecha: Mayo 2025
 */

#include <stdint.h>  // Para usar tipos estándar como uint8_t, int16_t

#ifndef MPU6050_H_  // Evita inclusión múltiple
#define MPU6050_H_

typedef struct {
    void *ctx;  // puntero de usuario
    // Escritura de registro I2C (bloqueante)
    int  (*writeReg)(void *ctx,
                     uint8_t devAddr,
                     uint8_t regAddr,
                     uint8_t *data,
                     uint16_t length);
    // Lectura de registro I2C (bloqueante)
    int  (*readReg)(void *ctx,
                    uint8_t devAddr,
                    uint8_t regAddr,
                    uint8_t *data,
                    uint16_t length);
    // Lectura de registro I2C por DMA (no bloqueante)
    int  (*readRegDMA)(void *ctx,
                       uint8_t devAddr,
                       uint8_t regAddr,
                       uint8_t *data,
                       uint16_t length);
    // Retardo en milisegundos, para la incializacion bloqueante
    void (*delayMs)(void *ctx, uint32_t ms);
    // Callback de error. error puede ser HAL_ERROR, HAL_TIMEOUT, código I2C, etc.
	void (*onError)    (void *ctx, int error);
} MPU6050_Platform_t;

// ------------------------------------------------------
// ➤ Direcciones de registros internos del MPU6050
// ------------------------------------------------------

// Dirección del dispositivo (con AD0 = 0 → 0x68 << 1 = 0xD0 para escritura)
#define MPU6050_ADDR         0xD0

#define WHO_AM_I_REG         0x75  // Registro de identidad (debe devolver 0x68)
#define PWR_MGMT_1_REG       0x6B  // Registro para salir del modo de bajo consumo
#define GYRO_CONFIG_REG      0x1B  // Registro de configuración del giroscopio
#define ACCEL_CONFIG_REG     0x1C  // Registro de configuración del acelerómetro
#define ACCEL_XOUT_H_REG     0x3B  // Dirección base de lectura del acelerómetro
#define GYRO_XOUT_H_REG      0x43  // Dirección base de lectura del giroscopio

// ------------------------------------------------------
// ➤ Offsets digitales para calibración (medidos en reposo)
// ------------------------------------------------------
// Se aplican para compensar el ruido o desvío de fábrica

#define OFFSET_AX  450
#define OFFSET_AY  450
#define OFFSET_AZ  20000  // en reposo el Z mide 1g → ≈ 16384 + margen

#define OFFSET_GX  450
#define OFFSET_GY  350
#define OFFSET_GZ  350

// ------------------------------------------------------
// ➤ Parámetros de escala
// ------------------------------------------------------

// Gravedad terrestre en m/s²
#define GRAVEDAD            9.81

// Factor de multiplicación para convertir a escala fija (2 decimales)
#define MULTIPLICADORFLOAT  100  // Ejemplo: 9.81 × 100 = 981

// Códigos de retorno
#define MPU6050_OK           0
#define MPU6050_ERROR		-1

// ------------------------------------------------------
// ➤ Prototipos de funciones públicas
// ------------------------------------------------------

uint8_t MPU6050_WhoAmI(void);

/**
 * @brief Inicializa el MPU6050 (wake-up, config escala, comprueba WHO_AM_I).
 * @return MPU6050_OK en OK, MPU6050_ERR si WHO_AM_I no es 0x68.
 */
int MPU6050_Init(void);

// Lanza lectura no bloqueante por DMA
void MPU6050_StartRead_DMA(void);

// --------------------------------------------
// ➤ Estado y acceso a datos
// --------------------------------------------
/**
 * @brief  Indica si hay un nuevo par de lecturas (Accel + Gyro) listo.
 * @retval 1 si hay datos, 0 si no.
 */
uint8_t MPU6050_IsDataReady(void);

/**
 * @brief Limpia el flag que indica que hay datos listos.
*/
void MPU6050_ClearDataReady(void);

/**
 * @brief  Obtiene la última lectura de aceleración escalada.
 * @param  ax Pointer al entero donde se copiará ax_real.
 * @param  ay Pointer al entero donde se copiará ay_real.
 * @param  az Pointer al entero donde se copiará az_real.
 */
void MPU6050_GetAccel(int16_t *ax, int16_t *ay, int16_t *az);

/**
 * @brief  Obtiene la última lectura de velocidad angular escalada.
 * @param  gx Pointer al entero donde se copiará gx_real.
 * @param  gy Pointer al entero donde se copiará gy_real.
 * @param  gz Pointer al entero donde se copiará gz_real.
 */
void MPU6050_GetGyro(int16_t *gx, int16_t *gy, int16_t *gz);

// Registrar la plataforma antes de llamar a Init o StartRead
void MPU6050_RegisterPlatform(MPU6050_Platform_t *plat);

void MPU6050_Calibrate(void);

#endif /* MPU6050_H_ */
