/* Telemetria del robot: CSV USB, log WiFi y odometria periodica.
 * Invocar una vez por ciclo de control valido, desde el loop principal.
 */
#include "telemetria.h"
#include "comunicacion_usb.h"
#include "UNER.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

#define LOG_DECIMACION        5    // Frecuencia de envio de log csv mediante USB
#define LOG_WIFI_DECIMACION   10   // Frecuencia de envio de log binario mediante WIFI
#define WIFI_ODOM_PERIODO_MS  500  // Período del push de odometría/línea por WiFi (para graficar en Qt); no ligado a ACTIVATE_WIFI_LOG, arranca solo con la conexión

static uint32_t log_contador = 0;
static uint8_t  log_encabezado_enviado = 0;
static uint32_t ultimo_wifi_odom_ms = 0;   // timestamp del último push de WifiOdomData_t
static uint16_t wifi_odom_secuencia     = 0;   // contador incremental para detectar pérdida de paquetes en Qt

void Telemetria_Actualizar(const TelemetriaDatos *datos)
{
    log_contador++;

    if (datos->f_enviar_log_wifi && (log_contador % LOG_WIFI_DECIMACION == 0)) {
        WifiLogData_t wlog;
        wlog.t_ms       = HAL_GetTick();
        wlog.roll_filt  = datos->roll_filtrado_grados;
        wlog.output     = datos->salida_pid;
        wlog.p_term     = datos->termino_p;
        wlog.i_term     = datos->termino_i;
        wlog.d_term     = datos->termino_d;
        wlog.mR         = datos->motor_derecho_velocidad;
        wlog.mL         = datos->motor_izquierdo_velocidad;
        wlog.dyn_sp     = datos->setpoint_dinamico_final;
        wlog.dt_ctrl_us = (uint32_t)(datos->dt_ctrl * 1000000.0f);

        wlog.line_error          = datos->linea_error;
        wlog.p_line              = datos->log_p_line;
        wlog.i_line              = datos->log_i_line;
        wlog.d_line              = datos->log_d_line;
        wlog.steering_adjustment = datos->ajuste_direccion;
        wlog.adc1                = datos->adc_mediana[0];
        wlog.adc2                = datos->adc_mediana[1];
        wlog.adc3                = datos->adc_mediana[2];
        wlog.adc4                = datos->adc_mediana[3];

        UNER_SendWifiLogData(&wlog);
    }

    // Push de odometría/línea por WiFi para graficar (mapa XY, franja de línea) en
    // Qt: independiente de ACTIVATE_WIFI_LOG, arranca solo con f_wifi_conectado y a
    // un ritmo bajo (WIFI_ODOM_PERIODO_MS) para no competir por ancho de banda/CPU
    // con la telemetría de control ya existente.
    if (datos->f_wifi_conectado && (uint32_t)(HAL_GetTick() - ultimo_wifi_odom_ms) >= WIFI_ODOM_PERIODO_MS) {
        ultimo_wifi_odom_ms = HAL_GetTick();

        WifiOdomData_t odata;
        odata.seq           = wifi_odom_secuencia++;
        odata.t_ms          = HAL_GetTick();
        odata.x_m           = datos->odom_x_m;
        odata.y_m           = datos->odom_y_m;
        odata.theta_deg     = datos->odom_theta_grados;
        odata.line_error    = datos->linea_error_display;
        odata.line_detected = datos->linea_detectada_display;
        odata.robot_state   = datos->estado_robot;
        odata.line_state    = (uint8_t)datos->linea_estado;
        odata.adc5          = datos->adc_mediana[4];  // sensores de objeto: menos = más cerca,
        odata.adc6          = datos->adc_mediana[5];  // ~4095 = nada adelante — para graficar
        odata.adc7          = datos->adc_mediana[6];  // la barrera/cuerpo frente al robot en Qt
        odata.adc8          = datos->adc_mediana[7];
        odata.roll_deg      = datos->roll_filtrado_grados;  // balanceo → Vista 3D de Qt
        odata.lat_deg       = datos->inclinacion_lateral_ema;          // banking lateral → Vista 3D de Qt

        UNER_SendWifiOdomData(&odata);
    }

    if (datos->f_enviar_log_csv && !log_encabezado_enviado) {
        USB_DebugStr("t_ms,dt_us,dt_ctrl_us,accel_roll,roll_accel_filtrado,gyro_y,giro_dps_clampeado,roll_filt,dyn_sp,error,p,i,d,salida_pid,pwm_comando,pwm_saturado,sat,motor_der,motor_izq,pitch,accel_x,accel_y,accel_z,giro_x,giro_y,giro_z\r\n");
        log_encabezado_enviado = 1;
    }

    if (datos->f_enviar_log_csv && (log_contador % LOG_DECIMACION == 0)) {
        /* 26 columnas: 320 bytes cubren incluso enteros de 32 bits completos.
         * Persistente para no ampliar la pila del ciclo de control.
         */
        static char buf[320];

        int roll_i   = (int)(datos->roll_filtrado_grados * 1000.0f);
        int p_i      = (int)(datos->termino_p * 1000.0f);
        int i_i      = (int)(datos->termino_i * 1000.0f);
        int d_i      = (int)(datos->termino_d * 1000.0f);
        int sp_i     = (int)(datos->setpoint_dinamico_final * 1000.0f);
        int error_i  = (int)(datos->error * 1000.0f);
        int gyrof_i  = (int)(datos->giro_dps_clampeado * 1000.0f);
        int accel_i  = (int)(datos->accel_angulo_grados * 1000.0f);
        int accelf_i = (int)(datos->roll_accel_filtrado * 1000.0f);
        int output_i = (int)(datos->salida_pid * 1000.0f);
        int pwmcmd_i = (int)(datos->pwm_comando * 100.0f);
        int pwmsat_i = (int)(datos->pwm_saturado * 100.0f);
        int pitch_i  = (int)(datos->pitch_grados * 1000.0f);

        int ax_i = (int)datos->accel_x;
        int ay_i = (int)datos->accel_y;
        int az_i = (int)datos->accel_z;
        int gx_i = (int)datos->giro_x;
        int gy_i = (int)datos->giro_y;
        int gz_i = (int)datos->giro_z;

        int len = snprintf(buf, sizeof(buf),
            "%lu,%lu,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
            (unsigned long)HAL_GetTick(),                 // t_ms
            (unsigned long)(uint32_t)(datos->dt_real * 1000000.0f), // dt_us
            (unsigned long)(uint32_t)(datos->dt_ctrl * 1000000.0f), // dt_ctrl_us
            accel_i,                       // accel_roll x1000
            accelf_i,                      // roll_accel_filtrado x1000
            (int)(datos->giro_velocidad_dps * 1000.0f),// gyro_y x1000
            gyrof_i,                       // giro_dps_clampeado x1000
            roll_i,                        // roll_filt x1000
            sp_i,                          // dyn_sp x1000
            error_i,                       // error x1000
            p_i,                           // p x1000
            i_i,                           // i x1000
            d_i,                           // d x1000
            output_i,                      // salida_pid x1000
            pwmcmd_i,                      // pwm_comando x100
            pwmsat_i,                      // pwm_saturado x100
            (unsigned int)datos->flag_saturacion,                      // sat
            datos->motor_derecho_velocidad,            // motor_der
            datos->motor_izquierdo_velocidad,             // motor_izq
            pitch_i,                       // pitch x1000
            ax_i, ay_i, az_i,              // accel raw
            gx_i, gy_i, gz_i               // gyro raw
        );

        if (len > 0 && (size_t)len < sizeof(buf)) {
            USB_DebugSend((uint8_t*)buf, (uint16_t)len);
        }
    }
}
