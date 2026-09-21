#ifndef TELEMETRIA_H
#define TELEMETRIA_H

#include <stdint.h>

/* Datos instantaneos del controlador. Escalas fisicas originales:
 * angulos en grados, tiempos dt en segundos; conversion CSV al transmitir.
 * Estados conservan los valores de robot_estados.h usados por Qt.
 */
typedef struct {
    uint8_t f_enviar_log_wifi;
    uint8_t f_enviar_log_csv;
    uint8_t f_wifi_conectado;
    uint8_t flag_saturacion;
    uint8_t linea_detectada_display;
    uint8_t estado_robot;
    uint8_t linea_estado;
    int16_t motor_derecho_velocidad;
    int16_t motor_izquierdo_velocidad;
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t giro_x;
    int16_t giro_y;
    int16_t giro_z;
    float roll_filtrado_grados;
    float salida_pid;
    float termino_p;
    float termino_i;
    float termino_d;
    float setpoint_dinamico_final;
    float dt_ctrl;
    float linea_error;
    float log_p_line;
    float log_i_line;
    float log_d_line;
    float ajuste_direccion;
    float odom_x_m;
    float odom_y_m;
    float odom_theta_grados;
    float linea_error_display;
    float inclinacion_lateral_ema;
    float error;
    float giro_dps_clampeado;
    float accel_angulo_grados;
    float roll_accel_filtrado;
    float pwm_comando;
    float pwm_saturado;
    float pitch_grados;
    float dt_real;
    float giro_velocidad_dps;
    uint16_t adc_mediana[8];
} TelemetriaDatos;

/* Conserva la cadencia por ciclos (CSV cada 5, WiFi cada 10), el encabezado
 * CSV una vez por arranque y la odometria cada 500 ms mientras haya WiFi.
 * No retiene el puntero ni modifica datos. Llamar solo desde el loop principal.
 */
void Telemetria_Actualizar(const TelemetriaDatos *datos);

#endif
