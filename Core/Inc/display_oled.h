#ifndef DISPLAY_OLED_H
#define DISPLAY_OLED_H

#include "stm32f4xx_hal.h"
#include "robot_estados.h"

#define PANTALLA_ADC_CANTIDAD 8

/* Foto de los datos del robot al refrescar. El display no modifica el control. */
typedef struct {
    uint8_t linea_canal_cuarentena[4];
    uint16_t adc_mediana[PANTALLA_ADC_CANTIDAD];
    int16_t motor_derecho_velocidad;
    int16_t motor_izquierdo_velocidad;
    float roll_filtrado_grados;
    uint8_t estado_robot;
    float setpoint_trim;
    uint8_t f_cambiar_pantalla;
    uint8_t f_wifi_conectado;
    uint8_t f_caido;
    uint8_t vel_limite_falla;
    float vel_limite_disparo;
    uint32_t caida_alerta_hasta_ms;
    eFallReason caida_motivo;
    float caida_angulo;
    float caida_giro;
    float caida_velocidad;
    float caida_pwm;
    float velocidad_est_ema;
    float setpoint_dinamico_final;
    float KP_value;
    float KD_value;
    float KI_value;
    float KV_brake_value;
    float KP_LINE;
    float KD_LINE;
    float KI_LINE;
    float LINEA_UMBRAL_ADC;
    float LINE_SPEED_TARGET;
    eLineState linea_estado;
    float linea_error_display;
    float accel_movimiento_ema;
    int8_t obj_esquive_dir;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t giro_x;
    int16_t giro_y;
    int16_t giro_z;
    float giro_dps_clampeado;
    float odom_x_m;
    float odom_y_m;
    float odom_theta_grados;
    float linea_perdida_x_m;
    float linea_perdida_y_m;
    uint8_t linea_perdida_pose_valida;
    float obj_deteccion_umbral_adc;
    float obj_distancia_banda_piso_adc;
    uint8_t obj_pared_adc_indice;
    float obj_pared_umbral;
    float obj_pared_reversa_umbral;
    float obj_pared_muy_cerca_umbral;
} PantallaDatos;

/* Inicializa el OLED y muestra el logo, antes del arranque de la IMU. */
void Pantalla_Init(I2C_HandleTypeDef *hi2c, uint8_t *busy_flag);
/* Avanza el DMA. Devuelve 1 si puede dibujarse un cuadro sin competir con la IMU. */
/* Lee los flags volatile DESPUES de avanzar el DMA, igual que el loop original. */
uint8_t Pantalla_Procesar(uint8_t reposo_o_caida,
                         const volatile uint8_t *lectura_imu_pendiente,
                         const volatile uint8_t *dato_imu_listo);
/* datos se usa solo durante esta llamada; la transferencia usa el framebuffer. */
void Pantalla_Dibujar(const PantallaDatos *datos);
void Pantalla_MostrarWifi(uint32_t duracion_ms);
void Pantalla_ForzarActualizacion(void);

#endif
