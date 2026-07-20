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
    LINE_STATE_PERP_ROTATE,    // Los 4 ADC en negro sin manipulación: cruce perpendicular, gira 90° (sentido del último esquive)
    // NOTA (2026-07-16): OBJ_ESPERA_REVERSA, OBJ_RETROCESO y OBJ_ARC eliminados
    // (muertos desde 2026-07-14). Esto RENUMERA los estados OBJ_* que viajan a
    // Qt en WifiOdomData_t.line_state — si Qt interpreta el valor, actualizar su mirror.
    LINE_STATE_OBJ_FRENO_REVERSA,      // Fase STOP: entra DIRECTO al detectar objeto — frena la inercia y lleva/sostiene la distancia (A6/A8 en banda) hasta estabilizarse → giro
    LINE_STATE_OBJ_GIRO_ESQUIVE,     // Objeto detectado: girando 90° por encoders (sentido según obj_esquive_dir)
    LINE_STATE_OBJ_PAUSA_GIRO,          // Post-rotación: balance estático espera 2s
    LINE_STATE_OBJ_BUSCAR_PARED, // Avanza despacio (2°) hasta encontrar la pared en el lateral activo (ADC7/ADC5)
    LINE_STATE_OBJ_BORDEAR_PARED,      // Wall-following: avanza mientras el lateral activo ve la pared
    LINE_STATE_OBJ_PARED_LIBRE,    // Perdió la pared en WALL_FWD: avanza un poco más antes de girar (no chocar la esquina)
    LINE_STATE_OBJ_GIRO_PARED,     // Wall-following: pivot hacia la pared hasta re-verla en el lateral activo
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
#define ADC_HISTORY_SIZE 	32U   // 8ms de historial a 4kHz para mediana anti-picos

#define BAR_COUNT    		8
#define BAR_SPACING  		2
#define SCREEN_W     		SSD1306_WIDTH
#define SCREEN_H    		SSD1306_HEIGHT

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// PID
#define KP     		4.0f   // Ganancia proporcional en PWM directo
#define KD     		0.12f   // Ganancia derivativa en PWM/(deg/s)
#define KI    		0.1f   // Ganancia integral en PWM/(deg*s)

#define SETPOINT_ANGLE 	0.0f

#define SOFT_ZONE_ANGLE_DEG   1.50f   // error a partir del cual el PID va al 100%
// 2026-07-10: 0.15 -> 0.35. Con 0.15, cerca del equilibrio el PID quedaba al
// 15% de autoridad y casi todo comando caia debajo de MOTOR_CMD_NEUTRAL (+-2):
// sin par hasta que el error ya era grande -> bamboleo lento de lado a lado.
#define SOFT_ZONE_MIN_SCALE   0.35f   // escala mínima cuando el error es ~0
// Zona de hold con histéresis: silencia P/I en el punto dulce de equilibrio.
// 2026-07-10: zona achicada (0.70/0.90 -> 0.25/0.45, gyro 4/10 -> 2/6) y el D
// ya NO se apaga dentro del hold (ver bloque del PID). Con la zona vieja el
// robot quedaba a la deriva casi 1 grado entero con motores muertos: un pendulo
// invertido SIEMPRE se cae en lazo abierto, asi que el hold ancho garantizaba
// un ciclo limite (caer -> atajar -> cruzar con impulso -> caer del otro lado)
// = el "se bambolea sin quedarse quieto". El anti-chatter real en parado lo da
// MOTOR_CMD_NEUTRAL, no este hold.
#define BALANCE_HOLD_ENTER_ANGLE_DEG  0.25f  // entra en hold si |error| <= este valor
#define BALANCE_HOLD_EXIT_ANGLE_DEG   0.45f  // sale de hold si |error| >= este valor
#define BALANCE_HOLD_ENTER_GYRO_DPS   2.0f   // entra en hold si |gyro| <= este valor
#define BALANCE_HOLD_EXIT_GYRO_DPS    6.0f   // sale de hold si |gyro| >= este valor

// Complementary Filter / PID timing
#define ALPHA 0.98f
#define DT_CTRL_FIXED 0.010f

// LOGGING MACROS
#define LOG_ENABLE 1
#define LOG_DECIM  5		// Frecuencia de envio de log csv mediante USB
#define LOG_WIFI_DECIM 10	// Frecuencia de envio de log binario mediante WIFI
#define WIFI_ODOM_PERIOD_MS 500  // Período del push de odometría/línea por WiFi (para graficar en Qt); no ligado a ACTIVATE_WIFI_LOG, arranca solo con la conexión

// Filter Control Parameters
#define I_MAX  100.0f       // Max Integral Term
#define DT_MIN 0.005f       // Min valid DT (5ms)
#define DT_MAX 0.05f        // Max valid DT (50ms)

// Fall detection (hysteresis)
#define FALL_ANGLE           	60.0f
#define RECOVER_ANGLE        	2.0f
#define UPSIDE_DOWN_ANGLE    	120.0f  // más agresivo para detectar boca abajo antes
#define DEAD_ZONE_ANGLE      	15.0f   // entre 35° y 120° → zona muerta, motores off
// Corte de emergencia exclusivo del modo línea. Usa velocidad filtrada y
// confirmación breve para no disparar por un solo escalón cuantizado del encoder.
#define LINE_SPEED_EMERGENCY_LIMIT       3.30f  // m/s: corte total de motores
#define LINE_SPEED_EMERGENCY_TRIP_CYC    2U     // 20ms consecutivos sobre el límite
#define LINE_SPEED_EMERGENCY_RESET_VEL   0.50f  // m/s: velocidad segura para rearmar
#define LINE_SPEED_EMERGENCY_RESET_CYC   50U    // 500ms seguro y vertical antes de volver

#define MOTOR_RIGHT_DEADBAND  	8   // offset sumado al motor derecho para compensar su mayor zona muerta (0 = sin compensación)
#define MOTOR_LEFT_DEADBAND   	8   // ídem motor izquierdo — ajustar si el izq. no arranca a PWM bajo
// 2026-07-10 (fix bamboleo, parte 2): los 8 de arriba compensan la friccion
// ESTATICA (arrancar la rueda parada). Con la rueda YA girando la friccion es
// cinetica (mucho menor) y sumar 8 igual convierte cada cruce por la zona
// neutra en una patada de ~11 PWM — un rele que, sobre una planta inestable,
// sostiene un ciclo limite (el bamboleo). Con ticks de encoder en los ultimos
// WHEEL_MOVING_WINDOW_MS se usa el offset cinetico; sin ticks (parada de
// verdad), el estatico completo para vencer el arranque.
#define MOTOR_DEADBAND_KINETIC  	4   // offset cuando la rueda ya esta girando
#define WHEEL_MOVING_WINDOW_MS  	80  // rueda "en movimiento" si tickeo hace menos de esto
// Zona neutra de comando: |comando| <= este valor => motor directamente en 0,
// SIN saltar a ±DEADBAND. Sin esto, cualquier chatter de ±1 PWM del PID cerca
// del equilibrio se convierte en golpes de ±9 PWM a 100 Hz (la compensación de
// deadband amplifica el ruido) — el "temblor" en balance. Subir de a 1 si sigue
// temblando; demasiado alto y el robot bambolea lento (el PID pierde autoridad
// fina cerca del equilibrio y corrige recién con error más grande).
#define MOTOR_CMD_NEUTRAL     	2
#define KV_BRAKE                0.8f  // ganancia base del freno traslacional
#define KV_BRAKE_STRONG         6.0f  // ganancia extra por encima del umbral
#define BRAKE_VEL_THRESHOLD     1.0f  // velocidad a partir de la cual se aplica el freno fuerte
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
// Yaw-assist (corrección de rumbo por gz en balance común y modo línea):
// zona muerta sobre gz crudo (SIN filtro por software — el MPU ya trae su DLPF
// de ~44 Hz): rotación < GZ_YAW_ASSIST_DB °/s => corrección exactamente 0, para
// que el ruido residual de gz no dispare la compensación de deadband de motor
// con el robot quieto. Subir si el yaw-assist sigue metiendo chatter; bajar si
// deja de corregir derivas de rumbo reales. La ganancia 0.23 no se toca.
#define GZ_YAW_ASSIST_DB        3.0f   // °/s (zona muerta, ajustar de a 1)
#define BRAKE_TILT_MAX          4.0f  // era 3.0 → 5.0 → 4.0: compromiso entre frenar de verdad y no sobrecorregir en balance normal
#define BRAKE_TILT_MAX_MANUAL   4.0f  // ídem
#define BRAKE_TILT_STEP_BAL     0.5f
#define BRAKE_TILT_STEP_MAN     0.5f
// Multiplicador extra sobre ComputeBrakeSetpointTarget, solo para LOST_BRAKE/
// EDGE_WAIT (el frenado justo al perder la línea del todo). Historia: se subió
// a 1.6 (2026-07-01) cuando el freno arrancaba 1s tarde, con el robot ya
// desacelerado solo. 2026-07-05: al adelantar el disparo de pérdida a 300ms,
// el freno ahora entra a velocidad de crucero plena y con 1.6 el tirón hacia
// atrás VOLCABA el robot — vuelto a 1.0 (freno estándar de balance, estable a
// velocidad). Si frena corto, subir de a 0.1, no volver a 1.6.
#define LOST_BRAKE_BOOST        1.0f
#define INTEGRAL_DECAY   		0.990f

// Steering PID (lazo cerrado por encoders)
#define STEER_KP        5.0f    // ajustar según respuesta real
#define STEER_KI        0.2f
#define STEER_KD        0.1f
#define STEER_I_MAX    15.0f
#define STEER_OUT_MAX  20.0f
// Velocity PI (lazo externo de velocidad en seguimiento de línea)
#define LINE_VEL_KP             3.0f   // ganancia proporcional vel PI (acelerando)
#define LINE_VEL_KP_BRAKE       8.0f   // sobrevelocidad: freno más firme sin endurecer la aceleración
#define LINE_VEL_KI             1.2f   // ganancia integral vel PI
#define LINE_VEL_I_MAX          2.5f   // anti-windup vel PI
#define LINE_BRAKE_ANGLE_MAX    3.0f   // inclinación máxima hacia atrás en seguimiento
#define LINE_SP_STEP_NORMAL     0.15f  // grados/ciclo al acelerar o aliviar el freno
#define LINE_SP_STEP_BRAKE      0.30f  // grados/ciclo al entrar en freno por sobrevelocidad
// Gobernador predictivo de velocidad: solo interviene cerca del tope GLOBAL.
// Proyecta la velocidad con la aceleración filtrada y empieza a frenar antes
// de LINE_SPEED_TARGET; no modifica el perfil de velocidad normal de las curvas.
#define LINE_SPEED_GUARD_MARGIN     0.15f  // m/s por debajo del tope donde empieza la guarda
#define LINE_SPEED_PREDICT_SEC      0.15f  // horizonte de predicción
#define LINE_ACCEL_FILTER_ALPHA     0.20f  // filtro de la derivada cuantizada de encoders
#define LINE_ACCEL_EST_MAX          8.0f   // m/s², rechazo de saltos imposibles de un tick
#define LINE_SPEED_GUARD_KP         6.0f   // grados de freno por m/s proyectado sobre la guarda
// Avance post-giro (LOST_FWD / EDGE_FWD): PI de velocidad.
// 2026-07-04: con TARGET=0.18 y ANGLE_MAX=1.0° (valores del avance "a ciegas",
// bajados cuando se aceleraba demasiado) el robot quedaba totalmente detenido en
// el retorno por odometría: 1° de techo, atenuado por la soft-zone del PID, no
// vence la fricción estática. Ahora que LOST_FWD navega a un punto conocido, se
// puede avanzar con más decisión.
#define LOST_FWD_SPEED_TARGET   0.40f  // m/s (0.18 no arrancaba desde parado)
#define LOST_FWD_KP             4.0f   // ganancia P acelerando
#define LOST_FWD_KP_BRAKE       14.0f  // ganancia P frenando (sobrevelocidad)
#define LOST_FWD_KI             1.0f   // ganancia I para sostener la velocidad de crucero
#define LOST_FWD_VEL_I_MAX      1.0f   // anti-windup del integral de velocidad
#define LOST_FWD_ANGLE_MAX      2.5f   // ángulo máximo de avance (°; 1.0 no movía el robot)
#define LOST_FWD_BRAKE_MAX      2.0f   // ángulo máximo de freno activo (°, se aplica en negativo)
// Anti-stall GENÉRICO (2026-07-10, ver AntiStall_Tick): los PIs de velocidad
// saturan en P+I con techos de 2-3 grados — si la fricción estática del
// momento (o la inclinación del piso, o un objeto) pide más, quedan clavados
// en el tope y el robot quieto para siempre. Todo estado que REQUIERE
// movimiento suma este boost en rampa POR ENCIMA del clamp de su PI: sin
// movimiento de encoders por WAIT ciclos crece STEP por ciclo hasta el tope
// del llamador; al moverse decae rápido (DECAY). Nada puede quedarse quieto.
#define ANTISTALL_VEL_THR    0.05f  // m/s: debajo de esto se considera quieto
#define ANTISTALL_WAIT_CYC   40     // ciclos quieto (400ms) antes de boostear
#define ANTISTALL_STEP       0.04f  // grados por ciclo de rampa del boost
#define ANTISTALL_DECAY      0.90f  // decaimiento por ciclo al volver a moverse
#define ANTISTALL_MAX_FWD    3.0f   // tope en avances (LOST_FWD/EDGE_FWD/pared)
#define ANTISTALL_MAX_REV    2.5f   // tope en reversas (RETROCESO/reversa de pared)
#define ANTISTALL_MAX_MAN    2.0f   // tope en control manual
// Retorno por odometría al punto de pérdida de línea (solo camino "centrado", LOST_FWD):
// después del giro de 180°, en vez de avanzar a ciegas, navega hacia la pose (x,y)
// donde se vio la línea por última vez. Steering P sobre el error de rumbo
// (bearing al objetivo - theta odométrico). Si el robot curva ALEJÁNDOSE del punto,
// primero verificar ODOM_THETA_SIGN (test del giro de 90°); si theta está bien y aun
// así corrige al revés, invertir el signo de LOST_RETURN_STEER_KP.
// Nota de escala: el formato compartido de motores aplica half_steer = steering*0.5,
// así que el efecto real por rueda es la MITAD de estos valores. Con KP=0.6/MAX=12
// (valores iniciales) el usuario reportó que "casi no corrige" — subidos 2026-07-05.
#define LOST_RETURN_STEER_KP    1.5f   // PWM de steering por grado de error de rumbo
#define LOST_RETURN_STEER_MAX  20.0f   // clamp del steering de navegación
#define LOST_RETURN_REACHED_M   0.10f  // distancia al punto para considerarlo alcanzado
// Sobrepaso: al llegar al punto (o pasarlo de costado) NO se rinde ahí mismo —
// los sensores necesitan pasar POR ENCIMA de la línea para verla, y el punto
// guardado es donde el robot la vio por última vez, no donde están los sensores
// ahora. Se sigue derecho OVERSHOOT_M más allá antes de dar por perdida la búsqueda.
// PASS_WIN_M: si el objetivo quedó claramente atrás (error de rumbo > 120°) a menos
// de esta distancia, se considera "pasado de costado" y también entra al sobrepaso
// (evita que quiera pegar la vuelta en U por un desvío lateral chico).
// 2026-07-10: 0.20→0.35 — con 0.20 (neto ~+0.10 tras el corte de REACHED_M) el
// robot se plantaba JUSTO sobre el punto de pérdida y no encontraba la línea
// ("se queda muy cerca"): entre el error de odometría acumulado y que los
// sensores están adelante, hace falta pasarse un poco más para cruzar la cinta.
#define LOST_RETURN_OVERSHOOT_M 0.35f  // metros extra en línea recta tras alcanzar el punto
#define LOST_RETURN_PASS_WIN_M  0.35f  // ventana de "lo pasé de costado"
#define LINE_ENC_CORR_KP        8.5f   // ganancia P: grados por (m/s de deficit) normalizado
#define LINE_ENC_CORR_MAX       4.0f   // angulo extra maximo por deficit de velocidad (grados)
#define LINE_REV_BOOST_MAX      1.4f   // extra de inclinacion si los encoders muestran reversa
#define LINE_REV_BOOST_UP       0.06f  // subida max por ciclo de control
#define LINE_REV_BOOST_DOWN     0.10f  // bajada max por ciclo de control
#define LINE_REV_VEL_START      0.05f  // m/s de reversa desde donde empieza a actuar
#define LINE_REV_VEL_FULL       0.35f  // m/s de reversa para aplicar boost maximo
// Corrección de rumbo sin línea: P simple sobre diferencia de velocidad de ruedas,
// en PWM absoluto (mismas unidades que steering_adjustment/pwm_sat). REV_STRAIGHT_MAX
// es chico a propósito: nunca puede acercarse a pwm_sat, así que ninguna rueda se
// queda sin margen. El plan original era arrancar con KP=0 e ir subiendo; el valor
// actual (-1.0, signo incluido) quedó de las pruebas en el robot — si la corrección
// empeora la asimetría en vez de corregirla, probar +1.0 (solo cambia el signo).
#define REV_STRAIGHT_KP   -1.0f   // PWM por rps de diferencia entre ruedas (0 = sin corrección)
#define REV_STRAIGHT_MAX   8.0f   // tope absoluto en PWM units
#define REV_STRAIGHT_SLEW  2.0f   // PWM/ciclo max de cambio (rampa)
// 2026-07-10 (fix zigzag de la reversa): P de rumbo sobre counts ACUMULADOS
// desde el inicio de la reversa (rev_dr - rev_dl ∝ desvío de rumbo real,
// 1 count ≈ 6.3mm de rueda) en lugar de la diferencia de velocidad
// instantánea, que a la velocidad de reversa está cuantizada en saltos de
// 3.57 rps (1 count/ciclo) y es casi puro ruido. Mismo signo/convención que
// REV_STRAIGHT_KP: si empeora el desvío en vez de corregirlo, probar +1.0.
// 2026-07-10 bis: KC bajado de -1.0 a -0.4 y corrección SOLO en movimiento:
// con el freno activo el robot pasa parte de la reversa casi quieto, y el P de
// rumbo a 1 PWM/count pivoteaba izquierda/derecha alrededor del rumbo inicial
// sin avanzar (reportado). Parado no hay rumbo que mantener: el target va a 0
// suave (por el mismo slew).
// 2026-07-13: el gate de "en movimiento" pasó de velocity_est_f > 0.12 a
// actividad de encoders (algún tick en los últimos ACT_MS). A velocidad de
// reversa el LPF de la velocidad cuantizada suele leer menos que 0.12 aunque
// el robot esté retrocediendo — la corrección quedaba apagada casi toda la
// reversa y el robot salía torcido (reportado).
#define REV_STRAIGHT_KC   -0.4f   // PWM por count de diferencia acumulada L/R
#define REV_STRAIGHT_ACT_MS 250U  // ms sin ticks de encoder para considerar "quieto" (sin corrección)
// Inner steering PI (lazo cerrado con encoder diferencial)
#define LINE_STEER_ENC_SCALE    0.12f  // escala steering_cmd [PWM] → diff_sp [rps]
#define LINE_STEER_MAX_RPS      2.0f   // límite del setpoint diferencial
#define LINE_STEER_FB_KP        5.0f   // inner steering PI — P
#define LINE_STEER_FB_KI        0.2f   // inner steering PI — I
#define LINE_STEER_FB_I_MAX     8.0f   // anti-windup inner steering

#define LINE_LOST_TIMEOUT_MS   2000    // ms sin línea antes de entrar en búsqueda
#define LINE_LOST_STEERING     12.0f   // steering suave para cuando recién se pierde la línea

#define OBJ_DETECT_THRESHOLD_VAL   3200.0f   // objeto detectado si ADC < este valor (sin objeto: ~4095, con objeto: <2000)
#define OBJ_DETECT_DEBOUNCE_CNT    10         // ciclos consecutivos (100 ms) para confirmar objeto
// Distancia objetivo al objeto: el hold del STOP (OBJ_FRENO_REVERSA) retrocede/
// avanza hasta que ADC6 y ADC8 (los frontales) lean dentro de la banda
// CLEAR..CLEAR+BAND — así la distancia final al objeto es siempre la misma,
// sin importar a qué distancia se lo detectó.
#define OBJ_REV_CLEAR_ADC          3500.0f   // piso de la banda de distancia en ADC6/ADC8
// Hold de distancia durante el STOP: si vuelve a acercarse (ADC < CLEAR)
// corrige hacia atrás; si se alejó de más (ADC > CLEAR+BAND) corrige hacia adelante;
// dentro de la banda queda quieto con freno de encoders.
#define OBJ_REV_HOLD_BAND          400.0f    // banda muerta por encima de CLEAR (3500..3900 = quieto; 300 era "muy ambiciosa", pedido 2026-07-10)
#define OBJ_REV_HOLD_FINE_BAND     60.0f     // banda muerta chica alrededor del centro: adentro no corrige, deja frenar/asentar
#define OBJ_REV_HOLD_ANGLE_MIN     0.7f      // corrección mínima cuando apenas se sale de la zona fina
#define OBJ_REV_HOLD_ANGLE_MAX     2.2f      // tope de corrección del hold (2026-07-14: 1.6→2.6, "intenta corregir pero el ángulo no le da"; el anti-stall sigue de respaldo)
#define OBJ_REV_HOLD_ERR_FOR_MAX   220.0f    // error (ADC) a partir del cual ya usa ANGLE_MAX
// 2026-07-10: damping por velocidad en el hold (el término D que le faltaba).
// El hold era P puro sobre distancia: el robot es un doble integrador
// (ángulo→aceleración→velocidad→posición) y un P de posición sin D oscila
// SIEMPRE — se pasaba, volvía, chocaba la pared. Este término inclina en
// contra del movimiento con la velocidad RÁPIDA de encoders (velocity_est_f;
// la lenta del freno general tarda ~0.5s y acá llega tarde): yendo hacia
// atrás inclina adelante, yendo hacia la pared inclina atrás — frena ANTES
// de pasarse de largo en ambos sentidos. Deadband 0.15 filtra el ruido de
// cuantización de un tick suelto (0.112 tras el LPF).
#define OBJ_REV_HOLD_KV            4.0f      // grados por m/s en contra del movimiento
#define OBJ_REV_HOLD_VEL_DB        0.15f     // deadband de velocidad (m/s)
#define OBJ_REV_HOLD_DAMP_MAX      2.5f      // tope del término de damping (grados)
// El giro de esquive recién arranca cuando la distancia estuvo DENTRO de la banda
// (3600..3900 en A6 y A8) durante STABLE_MS continuos — si la corrección lo está
// moviendo, el giro espera. Timeout total como colchón por si nunca se estabiliza.
#define OBJ_REV_STABLE_MS          2000U     // ms continuos dentro de la banda antes de permitir el giro
#define OBJ_REV_STOP_TIMEOUT_MS    10000U    // colchón: girar igual si en este tiempo total nunca se estabilizó
// Giro 90° por encoders: medir distancia entre centros de rueda y ajustar aquí
#define OBJ_ROTATE_TRACK_WIDTH     0.220f    // metros entre centros de ruedas (medido: 220 mm)
#define OBJ_HOLD_DURATION_MS       2000U     // ms de balance estatico en OBJ_HOLD antes de wall-following
// 2026-07-14: hold de POSICIÓN en la pausa post-giro ("que mantenga posición un
// poco mejor"). El freno por velocidad (ComputeBrakeSetpointTarget) usa la
// velocidad LENTA y solo frena — no devuelve la deriva ya recorrida. P suave
// sobre los counts netos de encoder desde la entrada a la pausa (1 count ≈ 6.3mm).
#define OBJ_PAUSA_POS_KP           0.06f     // grados por count de deriva (≈1° cada 10 cm)
#define OBJ_PAUSA_POS_MAX          1.5f      // tope de la corrección de posición (°)
#define OBJ_PAUSA_POS_DB_COUNTS    3.0f      // banda muerta (≈2 cm): adentro no corrige, deja asentar
#define OBJ_FINAL_TURN_WINDOW_MS   1500U     // la línea hallada al salir del bordeo debe disparar PERP dentro de esta ventana
// 2026-07-13: esquive alternado. `obj_esquive_dir` = +1 → giro 90° a la DERECHA
// y pared seguida con el lateral izquierdo ADC7 (adcAvg[6]) — el comportamiento
// de siempre. -1 → todo espejado: giro a la IZQUIERDA y pared con el lateral
// derecho ADC5 (adcAvg[4]). Se alterna en cada detección de objeto (la primera
// va a la derecha) y se rearma en derecha al entrar al modo línea. Mismos
// mecanismos, umbrales y constantes en ambos sentidos: solo cambian el signo de
// los pivots y el sensor lateral.
#define OBJ_WALL_ADC_IDX           ((obj_esquive_dir > 0) ? 6 : 4)   // ADC7 o ADC5 según sentido
#define OBJ_WALL_THRESHOLD         3600.0f   // ADC < umbral → objeto visible; > umbral → perdido (3750 dejaba que se alejara demasiado antes de girar a buscarlo)
#define OBJ_WALL_REVERSE_THOLD     600.0f    // ADC7 < este valor → demasiado cerca, reversa pareja
#define OBJ_WALL_TOO_CLOSE_THOLD   2100.0f   // ADC7 entre REVERSE_THOLD y este valor → pivot derecha
// 2026-07-10: reversa de pared por DISTANCIA fija de encoders (latch). Antes
// era "tilt fijo mientras ADC7 < THOLD": con 3.0° muchas veces ni vencía la
// fricción (se quedaba quieto) y apenas se despegaba unos mm ya salía de
// reversa y volvía a avanzar — nunca se alejaba de verdad. Ahora al disparar
// se retrocede OBJ_WALL_REV_COUNTS counts (≈6.3mm c/u) antes de devolver el
// control, con timeout de seguridad por si está trabado.
#define OBJ_WALL_REVERSE_ANGLE     4.5f      // grados de inclinación hacia atrás durante la reversa por pared (3.0 no vencía la fricción)
#define OBJ_WALL_REV_COUNTS        24        // counts de retroceso fijo (~15 cm) al disparar la reversa de pared
#define OBJ_WALL_REV_TIMEOUT_MS    3000U     // colchón: si en este tiempo no completó los counts, sale igual
// 2026-07-10: destrabe por atascamiento. Si en los estados de pared (avanza/
// pivot) los encoders no acumulan movimiento significativo en una ventana de
// STUCK_WIN_MS, el robot quedó ATORADO contra la pared (empujando sin moverse,
// o pivot trabado): se dispara el mismo latch de reversa pero con un objetivo
// LARGO (ESCAPE_COUNTS) para despegarlo de verdad antes de reintentar.
#define OBJ_WALL_STUCK_WIN_MS      2500U     // ventana de detección de atorado (5000 era mucho tiempo empujando la pared)
#define OBJ_WALL_STUCK_COUNTS      20        // menos que esto de movimiento en la ventana (~13 cm) = atorado
#define OBJ_WALL_REV_ESCAPE_COUNTS 60        // counts de la reversa de destrabe (~38 cm)
// 2026-07-11: si en BORDEAR_PARED/PARED_LIBRE/GIRO_PARED la pared (ADC7) no
// vuelve a verse en ningún momento durante este tiempo continuo, se aborta
// toda la secuencia a reposo (ROBOT_STATE_IDLE) en vez de seguir buscándola
// a ciegas y arriesgar un choque.
#define OBJ_WALL_MISSING_TIMEOUT_MS 5000U
// 2026-07-10: verificación del giro de esquive por ADC7. El giro de 90° por
// encoders/gyro puede quedar corto (la pared no entra en la vista lateral);
// buena detención = ADC7 < OBJ_ROT_ADC_GOOD al terminar. Si no bajó de ahí,
// una fase de ajuste fino sigue pivotando despacio hasta verla (con tope de
// counts extra y timeout para no pasarse de largo girando).
#define OBJ_ROT_ADC_GOOD           3200.0f   // ADC7 debajo de esto = giro bien terminado
#define OBJ_ROT_ADC_GOOD_CYC       3         // ciclos seguidos con ADC7 < GOOD para confirmar (30ms, anti-glitch)
#define OBJ_ROT_ADC_EXTRA_MAX      150.0f    // counts extra máximos del ajuste fino (~36°)
#define OBJ_ROT_ADC_EXTRA_MS       1500U     // timeout del ajuste fino
// Avance buscando la pared: mismo PI de velocidad que OBJ_WALL_FWD (más cauteloso,
// target más bajo) en vez de ángulo fijo sin ningún control -- se aceleraba sin límite
// hasta encontrar la pared.
#define OBJ_WALL_APPROACH_SPEED_TARGET 0.3f   // m/s, avance cauteloso buscando la pared
#define OBJ_WALL_APPROACH_ANGLE_MAX    1.5f   // ángulo máximo de avance buscando la pared (°)
#define OBJ_WALL_APPROACH_TIMEOUT  6000U     // ms máximos buscando la pared antes de rendirse
// Avance bordeando la pared: PI de velocidad (mismo patrón que LOST_FWD/EDGE_FWD) en vez
// de ángulo fijo + freno bang-bang por sobrevelocidad — regula la velocidad de crucero
// con precisión en lugar de acelerar a fondo hasta cruzar el umbral y frenar de golpe.
// 2026-07-14: perfil de avance bajado al del retorno por odometría (LOST_FWD,
// validado): con target 1.0 m/s y tope 3.5° el robot aceleraba fuerte bordeando
// (desde parado el deadband de velocidad hace leer el error completo → tilt
// clavado en el tope) y llegaba a la línea tan rápido que siempre se pasaba.
#define OBJ_WALL_SPEED_TARGET      0.85f     // m/s, avance moderado bordeando la pared (1.0 se pasaba de la línea; 0.45 demasiado lento)
#define OBJ_WALL_VEL_KP            4.0f      // ganancia P acelerando
#define OBJ_WALL_VEL_KP_BRAKE      22.0f     // ganancia P frenando (sobrevelocidad)
#define OBJ_WALL_VEL_KI            1.0f      // ganancia I para sostener la velocidad de crucero
#define OBJ_WALL_VEL_I_MAX         1.0f      // anti-windup del integral de velocidad
#define OBJ_WALL_FWD_ANGLE         3.3f      // ángulo máximo de avance en WALL_FWD (°; era 3.5 — acelerón; 2.5 le pareció poco al usuario)
#define OBJ_WALL_BRAKE_ANGLE       4.0f      // freno activo máximo por sobrevelocidad en WALL_FWD (°)
// Antes de arrancar la reversa por pared, si el robot todavía tiene velocidad residual
// (viniendo de avanzar), frena primero para no
// entorpecer la reversa con inercia hacia adelante.
#define OBJ_WALL_REVERSE_SETTLE_VEL 0.15f    // m/s por debajo del cual se considera "quieto"
#define OBJ_WALL_PIVOT_POWER       8.0f      // potencia del pivot en WALL_TURN/WALL_FWD
#define OBJ_WALL_LINE_IGNORE_MS    5000U     // ms desde la entrada al wall-following en que se ignora la línea (2026-07-14: 3s→5s, veía la línea al arrancar pared>avanza/gira y abortaba la esquiva)
#define OBJ_WALL_LINE_CONFIRM_CYC  5U        // lecturas consecutivas (5×10ms=50ms) antes de abandonar el bordeo por línea
#define OBJ_WALL_CLEAR_COUNTS      100       // counts de encoder a avanzar tras perder la pared, antes de girar

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

extern USBD_HandleTypeDef hUsbDeviceFS;
static uint8_t usb_tx_buf[USB_TX_BUF_SIZE];
static volatile uint16_t tx_head = 0;
static volatile uint16_t tx_tail = 0;
static volatile uint8_t usb_tx_busy = 0;
static volatile uint32_t usb_tx_dropped = 0;
static const char HEX_DIGITS[] = "0123456789ABCDEF";	// Tabla de dígitos hex para USB

//static uint8_t ema_initialized = 0;
static uint8_t aliveCounter, mpu6050Counter;
uint8_t mpuDataReady = 0;
uint8_t mpu_initialized = 0;
static volatile uint8_t mpu_req_pending = 0;
static float roll_deg = 0.0f;	// Ángulo de balanceo (eje Y, usado para el equilibrio)
static float pitch_deg = 0.0f;	// Ángulo de inclinación (eje X)

volatile uint16_t adcValues[8];
// Historial de secuencias DMA completas. La mediana de 32 barridos rechaza
// picos breves por EMI sin ocultar un nivel alto sostenido en el pin.
static volatile uint16_t adcHistory[ADC_HISTORY_SIZE][8] = {{0}};
static volatile uint8_t adc_history_write = 0;
static volatile uint8_t adc_history_count = 0;
static volatile uint32_t adc_snapshot_seq = 0;
// Cuarentena de sensores de línea enganchados (2026-07-10): un canal clavado
// en el tope (~4095, típico de soldadura fría — baja al presionarlo con el
// dedo — o emisor IR caído) finge "línea" permanente de ese lado: arrastra el
// centroide, impide detectar la pérdida de línea y ensucia all_black. Si un
// canal pasa LINE_STUCK_MS continuos por encima de LINE_STUCK_ADC se lo pone
// en cuarentena (contribuye 0 al centroide/detección y se marca con X en el
// display); vuelve solo apenas lee blanco de nuevo (< LINE_THRESHOLD).
// NOTA: es una DEFENSA, no un fix — el canal enganchado es un problema de
// hardware (revisar soldadura/conector del sensor marcado con X).
#define LINE_STUCK_ADC   3950.0f  // por encima de esto se considera "clavado en el tope"
#define LINE_STUCK_MS    8000U    // tiempo continuo arriba de STUCK_ADC para cuarentena
static uint32_t line_ch_stuck_t0[4]   = {0, 0, 0, 0};
static uint8_t  line_ch_quarantine[4] = {0, 0, 0, 0};
static uint16_t adcAvg[BAR_COUNT] = {0};

uint8_t i2c1_tx_busy = 0;

uint8_t dataTx, dataRx;
static uint8_t unerRxBuffer[RXBUFSIZE];
static uint8_t unerTxBuffer[TXBUFSIZE];
static _sRx    unerRx;
_sTx    unerTx;
static uint8_t esp01RxBuf[ESP01RXBUFAT];
static uint16_t esp01IwRx = 0;
static uint16_t esp01IrRx = 0;		/* Índice de lectura para el buffer UDP entrante */

// ── Perfiles de red WiFi ─────────────────────────────────────────────────
// Para cambiar de red: tocar SOLO el número de WIFI_PROFILE_ACTIVE.
// Para agregar una red nueva: sumar una fila a la tabla (y opcionalmente a
// CLAUDE.md). SSID/password/IP quedan sincronizados automáticamente, ya no
// hace falta descomentar/comentar bloques en más de un lugar.
typedef struct {
    const char *ssid;
    const char *password;
    const char *ip;
    const char *label;
} WifiProfile_t;

static const WifiProfile_t wifiProfiles[] = {
    /* 0 */ { "FCAL",                      "fcalconcordia.06-2019", "172.23.205.98", "FCAL / Universidad" },
    /* 1 */ { "MEGACABLE FIBRA-2.4G-ckd0", "djg19dlk",              "192.168.100.5", "Casa" },
    /* 2 */ { "Delco_Mendelevich",         "toyotakia",             "192.168.1.23",  "Delco Mendelevich" },
    /* 3 */ { "Wifi Habitaciones",         "toyotakia",             "192.168.1.48",  "Wifi Habitaciones" },
};

#define WIFI_PROFILE_ACTIVE  0        /* <<< NUMERO DE RED ELEGIDA >>> */

const char *wifiSSID;
const char *wifiPassword;
const char *wifiIp;

int16_t motorRightVelocity = 0;
int16_t motorLeftVelocity  = 0;

static float integral = 0.0f;
static float steering_adjustment = 0.0f;
static float filtered_roll_deg = 0.0f;
static float lat_tilt_f        = 0.0f;  // inclinación lateral (banking) filtrada — solo para la Vista 3D de Qt

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
static uint32_t last_wifi_odom_ms = 0;   // timestamp del último push de WifiOdomData_t
static uint16_t wifi_odom_seq     = 0;   // contador incremental para detectar pérdida de paquetes en Qt
uint8_t f_send_csv_log = 0;
uint8_t f_send_wifi_log = 0;
uint8_t f_change_display = 0;

static uint8_t f_wifi_connected = 0;
// Splash de conexión WiFi: al obtener IP propia, updateDisplay() muestra por
// unos segundos SSID + IP propia + IP destino, pisando la pantalla activa
// (sea cual sea) y volviendo sola a ella al vencer.
#define WIFI_SPLASH_MS  4000
static volatile uint32_t wifi_splash_until_ms = 0;
static uint8_t f_fallen = 0;   // 1 = caído, motores apagados
static uint8_t speed_limit_fault = 0; // caída forzada por exceso de velocidad en LINE

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
static uint8_t    line_seen_since_entry = 0; // evita búsqueda automática antes de adquirir línea
static float      line_search_dir  = 1.0f; // dirección de búsqueda (+1 o -1)
static float      last_line_dir    = 1.0f; // última dirección válida de la línea (+1 o -1)
static uint8_t    line_was_centered_on_lost = 1; // 1 = venía centrado (ADC del medio) al perder la línea, 0 = venía por los extremos
// 1 = el último sensor con señal fue un extremo (w[0]/w[3]) sin soporte del medio
static uint8_t    last_detected_edge_only = 0;
static float      line_error_f_d   = 0.0f;

static uint8_t upside_down_count = 0;
static uint8_t upright_count = 0;
static uint8_t fall_count = 0;

static uint8_t key_prev = 1;
static uint32_t key_last_ms = 0;
static uint32_t key_click_time = 0;
static uint8_t  key_click_count = 0;

static float manual_setpoint_ramped = 0.0f;  // último ángulo de avance aplicado en MANUAL (telemetría)
static float manual_vel_integral    = 0.0f;  // integral del PI de velocidad adelante/atrás en MANUAL
static float manual_straight_steer_f = 0.0f; // corrección de rumbo recto (mismo algoritmo que REV_STRAIGHT en OBJ_REVERSE) cuando no hay giro comandado en MANUAL

static float line_angle_ramped      = 0.0f;  // rampa de avance en line follower
static float line_enc_angle_corr     = 0.0f;  // corrección P de angulo por deficit de velocidad encoder
static float line_reverse_boost     = 0.0f;  // extra de angulo si encoders muestran reversa
static float line_vel_integral      = 0.0f;  // integral del PI de velocidad (idea 1)
static float line_steer_fb_int      = 0.0f;  // integral del inner steering PI (idea 2)
static float speed_right_rps_s      = 0.0f;  // velocidad rueda derecha, accesible fuera del bloque encoder
static float speed_left_rps_s       = 0.0f;  // velocidad rueda izquierda, accesible fuera del bloque encoder
static uint32_t wheel_r_last_tick_ms = 0;    // último tick de encoder derecho (deadband cinético/estático)
static uint32_t wheel_l_last_tick_ms = 0;    // ídem izquierdo
// Velocidad LENTA (tau ~0.5s) SOLO para el freno traslacional: meciéndose en el
// lugar la velocidad alterna de signo a ~1Hz y este filtro la promedia a ~0
// (no dispara el freno); una deriva real sostenida sí pasa (con ~0.5-1s de
// retardo, aceptable para frenar un empujón). Con el filtro rápido de antes
// (VEL_LPF_BETA=0.35) el freno inclinaba el setpoint alternando de signo con
// retardo y BOMBEABA el bamboleo en vez de frenarlo.
#define VEL_SLOW_BETA  0.02f
static float velocity_est_slow_f    = 0.0f;
static float line_error_disp        = 0.0f;  // último error de línea, para mostrar en pantalla
static uint8_t line_detected_disp   = 0;      // línea detectada en el ciclo actual, para telemetría WiFi
static float pwm_sat_prev = 0.0f;
static float prev_error = 0.0f;
static uint8_t balance_hold_active = 0;
static uint32_t obj_detect_ignore_until_ms = 0;  // ignora sensores de objeto hasta este tick
static uint8_t  prev_all_line_black        = 1;  // todos los sensores de línea en negro (robot en el aire)
static uint32_t all_black_start_ms         = 0;  // tick en que empezaron a verse todos negros
static uint32_t last_partial_line_ms       = 0;  // último ciclo con línea detectada SIN los 4 en negro (vista "parcial" real, imposible en el aire)
static uint32_t accel_settled_start_ms     = 0;  // tick desde que el acelerómetro dejó de moverse (0 = todavía moviéndose)
static uint8_t  f_in_air                   = 0;  // 1 = robot en el aire >2s → motores detenidos
static float    accel_motion_f             = 0.0f;  // variación reciente del acelerómetro (EMA), para distinguir "en la mano" de "quieto sobre blanco"
static uint32_t lrot_brake_start_ms        = 0;  // inicio del frenado previo al giro 180°
static uint8_t  line_quiet_cycles          = 0;  // ciclos consecutivos con velocidad cruda baja (gate de quietud de LOST_BRAKE/EDGE_WAIT)
static uint32_t lrot_settle_start_ms       = 0;  // inicio de la pausa de estabilización post-180°
static uint32_t edge_wait_start_ms         = 0;  // inicio del frenado previo al giro de 90° (perdida por un extremo)
static uint8_t  obj_rot_initialized  = 0;
static int32_t  obj_rot_r0           = 0;
static int32_t  obj_rot_l0           = 0;
static uint32_t obj_rot_start_ms     = 0;
static uint8_t  obj_rot_phase        = 0;    // 0=spin, 1=freno activo, 2=ajuste fino por ADC7 (solo GIRO_ESQUIVE)
static float    obj_rot_heading      = 0.0f; // ángulo gz acumulado durante giro
static uint32_t obj_rot_phase1_ms    = 0;    // inicio de fase 1
static float    obj_rot_p2_c0        = 0.0f; // counts al entrar al ajuste fino por ADC7
static uint32_t obj_rot_p2_ms        = 0;    // inicio del ajuste fino
static uint8_t  obj_rot_adc_good_cnt = 0;    // ciclos seguidos con ADC7 < OBJ_ROT_ADC_GOOD
static int32_t  obj_rev_r0           = 0;
static int32_t  obj_rev_l0           = 0;
static float    obj_rev_steer_f      = 0.0f;      // corrección de rumbo rampeada, PWM absoluto (sin línea), durante OBJ_REVERSE
static int32_t  obj_rev_last_counts  = 0;         // último rev_counts visto (gate de actividad del rumbo)
static uint32_t obj_rev_last_move_ms = 0;         // tick del último cambio de counts en la reversa
static int8_t   obj_esquive_dir      = 1;         // sentido del esquive ACTUAL: +1 derecha (ADC7), -1 izquierda (ADC5)
static int8_t   obj_esquive_next_dir = 1;         // sentido de la PRÓXIMA detección (se alterna en cada objeto)
// Latch de reversa de pared por counts (ver OBJ_WALL_REV_COUNTS)
static uint8_t  obj_wall_rev_latch    = 0;
static int32_t  obj_wall_rev_latch_r0 = 0;
static int32_t  obj_wall_rev_latch_l0 = 0;
static uint32_t obj_wall_rev_latch_ms = 0;
static int32_t  obj_wall_rev_goal     = OBJ_WALL_REV_COUNTS; // objetivo de la reversa activa (normal o escape)
static int32_t  obj_wall_stuck_r0     = 0;   // snapshot de la ventana de atorado
static int32_t  obj_wall_stuck_l0     = 0;
static uint32_t obj_wall_stuck_ms     = 0;   // 0 = ventana sin arrancar
static uint32_t obj_wall_missing_since_ms = 0; // 0 = pared visible / timer sin arrancar (ver OBJ_WALL_MISSING_TIMEOUT_MS)
static float    lost_fwd_vel_integral = 0.0f;     // integral del PI de velocidad en LOST_FWD/EDGE_FWD
static float    obj_wall_vel_integral = 0.0f;     // integral del PI de velocidad en OBJ_WALL_FWD
static uint32_t obj_brake_start_ms   = 0;         // inicio de OBJ_BRAKE (file-scope para reset en caída)
static uint32_t obj_pre_rotate_ms    = 0;         // inicio del wait post-freno antes de OBJ_ROTATE
static uint32_t obj_rev_band_enter_ms = 0;        // entrada a la banda 3600..3900 de A6/A8 en STOP (0 = fuera de banda)
static uint32_t obj_hold_start_ms    = 0;         // inicio de la pausa post-giro (PAUSA_GIRO, 2s)
static int32_t  obj_pausa_r0         = 0;         // ancla de posición del hold post-giro (PAUSA_GIRO)
static int32_t  obj_pausa_l0         = 0;
static uint32_t obj_final_turn_pending_ms = 0;     // línea reencontrada al salir del bordeo; espera el giro final PERP
static uint8_t  perp_from_obj_avoidance   = 0;     // el PERP_ROTATE actual es el giro final de un esquive
static uint32_t perp_obj_brake_start_ms   = 0;     // inicio real del contrafreno suave del giro final de esquive
static uint32_t obj_wall_approach_start_ms = 0;    // timestamp de entrada a WALL_APPROACH
static uint32_t obj_wall_fwd_start_ms = 0;         // timestamp de entrada a WALL_FWD (init de PI/estado, no se usa para ignorar línea)
static uint32_t obj_wall_lost_ms      = 0;         // (sin uso para ignorar línea, ver obj_wall_seq_start_ms)
static uint8_t  obj_wall_clear_initialized = 0;     // flag sesión actual de OBJ_WALL_CLEAR
static uint32_t obj_wall_seq_start_ms = 0;         // timestamp de entrada a TODA la secuencia de wall-following
                                                    // (BORDEAR_PARED/PARED_LIBRE/GIRO_PARED); se fija UNA sola vez
                                                    // al entrar desde BUSCAR_PARED y NO se resetea al ciclar entre
                                                    // esos 3 estados -- ver fix 2026-07-05 "ignora la línea todo el tiempo"
static int32_t  obj_wall_clear_r0 = 0;
static int32_t  obj_wall_clear_l0 = 0;
static uint8_t  obj_wall_line_confirm_cnt = 0;     // debounce compartido por los 3 estados de pared

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

// ── ODOMETRÍA DE POSE ─────────────────────────────────────────────────────
// Pose estimada del robot en el plano, integrada a 100 Hz en ControlStep10ms:
//  - distancia: promedio de encoders (misma escala ENC_CPR/ENC_VEL_SCALE que velocity_est)
//  - rumbo: gyro Z integrado (menos sensible al slip de ruedas que el diferencial de encoders)
// Marco de referencia: (0,0) y θ=0 en el punto/orientación donde se hizo el último
// reset (cambio de modo o comando RESET_ODOMETRY desde Qt). X = adelante inicial,
// Y = lateral, θ en grados (-180..180]. Se pausa la integración durante caídas.
// ODOM_THETA_SIGN: si al girar a la DERECHA θ se hace más POSITIVO, invertir a -1.0f
// (la convención deseada es la matemática estándar: antihorario = positivo).
#define ODOM_THETA_SIGN  (+1.0f)
// Mejoras de precisión (2026-07-09):
// - Gating de pivot: 1 count de encoder = ~6.3mm; girando en el lugar los ticks de
//   cada rueda caen en ciclos de 10ms DISTINTOS con el rumbo cambiando rápido, y esos
//   pasos de ±1 count integrados a rumbos diferentes no se cancelan → la posición
//   "camina" al rotar. Si el robot claramente está pivoteando (|gz| alto) y el avance
//   neto del ciclo es ≤1 count, se descarta la traslación de ese ciclo.
// - Bias residual de gyro Z aprendido en línea (solo para odometría): el bias fijo
//   de fábrica deriva con la temperatura; con el robot quieto (encoders sin ticks)
//   se estima el residuo con una EMA lenta y se resta al integrar θ.
// - Integración de 2º orden (rumbo de mitad de paso) para menos sesgo en curvas.
#define ODOM_PIVOT_GZ_DPS      40.0f   // |gz| por encima de esto = "está pivoteando"
#define ODOM_PIVOT_MAX_COUNTS  1       // avance neto (|dR+dL|) tolerado como ruido durante pivot
#define ODOM_GZ_STILL_LSB      150.0f  // |gz| crudo por debajo de esto puede ser bias (~1.5 dps)
#define ODOM_GZ_BIAS_ALPHA     0.005f  // EMA del bias (tau ~2s a 100Hz, solo quieto)
#define ODOM_GZ_BIAS_MAX_LSB   200.0f  // clamp de seguridad del bias aprendido (±2 dps)
#define ODOM_STILL_MIN_CYCLES  50      // ciclos quieto (500ms) antes de empezar a aprender
static float odom_x_m       = 0.0f;
static float odom_y_m       = 0.0f;
static float odom_theta_deg = 0.0f;
static float    odom_gz_bias_f   = 0.0f;  // bias residual de gz aprendido (LSB), solo odometría
static uint16_t odom_still_cnt   = 0;     // ciclos consecutivos con el robot quieto
// Pose del último punto donde se vio la línea (para el retorno por odometría en
// LOST_FWD). Se actualiza cada ciclo con línea detectada en FOLLOWING.
static float   line_loss_x_m       = 0.0f;
static float   line_loss_y_m       = 0.0f;
static uint8_t line_loss_pose_valid = 0;
// Fase de sobrepaso del retorno por odometría (LOST_FWD/EDGE_FWD): al alcanzar
// el punto de pérdida sigue derecho LOST_RETURN_OVERSHOOT_M más para que los
// sensores crucen la línea. Snapshot de la pose donde arrancó el sobrepaso.
static uint8_t lost_ret_ov_active = 0;
static float   lost_ret_ov_x      = 0.0f;
static float   lost_ret_ov_y      = 0.0f;

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
void USB_DebugSend(const uint8_t *data, uint16_t len);
void USB_DebugStr(const char *s);	// Envía una cadena C terminada en '\0' por USB.
void USB_DebugHex(uint8_t b);	// Envía un byte representado en dos dígitos hexadecimales ASCII.
void USB_Debug(const char *fmt, ...);	// Mini-printf para debug: soporta %s, %c, %d/%u, %X y %%.
uint8_t usb_enqueue_tx(const uint8_t *data, uint16_t len);
uint8_t usb_enqueue_tx_segments(const uint8_t *first, uint16_t first_len,
                                const uint8_t *second, uint16_t second_len);
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
        // Seqlock: impar mientras se publica, par cuando el historial es coherente.
        adc_snapshot_seq++;
        __DMB();
        uint8_t row = adc_history_write;
        for (uint8_t ch = 0; ch < 8; ++ch)
            adcHistory[row][ch] = adcValues[ch];
        adc_history_write = (uint8_t)((row + 1U) % ADC_HISTORY_SIZE);
        if (adc_history_count < ADC_HISTORY_SIZE)
            adc_history_count++;
        __DMB();
        adc_snapshot_seq++;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        ESP01_WriteRX(dataRx);      // Sube byte al buffer AT
        //ESP01_Task();    // dentro hace ESP01ATDecode()
        HAL_UART_Receive_IT(&huart1, &dataRx, 1);
    }
}

// Sin este callback, un error de overrun (ORE) aborta la recepción IT y nadie
// vuelve a armarla: el RX del ESP-01 queda muerto hasta el próximo reset.
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        HAL_UART_Receive_IT(&huart1, &dataRx, 1);
    }
}

void USBRxData(uint8_t *buf, int len) {
    if (buf == NULL || len <= 0) return;

    for (int i = 0; i < len; i++)
        UNER_PushByte(buf[i]);
}

// Encola el mensaje completo; usb_service_tx lo transmite en paquetes de hasta 64 bytes.
void USB_DebugSend(const uint8_t *data, uint16_t len) {
    if (data != NULL && len != 0U)
        (void)usb_enqueue_tx(data, len);
}

// Envía una cadena literal
void USB_DebugStr(const char *s) {
    if (s != NULL)
        USB_DebugSend((const uint8_t *)s, (uint16_t)strlen(s));
}

// Envía un byte como dos dígitos hex ASCII
void USB_DebugHex(uint8_t b) {
    char h[2] = { HEX_DIGITS[b >> 4], HEX_DIGITS[b & 0xF] };
    USB_DebugSend((const uint8_t *)h, sizeof(h));
}

static uint16_t USB_AppendUInt(char *dst, uint16_t pos, uint16_t capacity,
                               unsigned int value, unsigned int base) {
    char reversed[10];
    uint8_t count = 0;

    do {
        reversed[count++] = HEX_DIGITS[value % base];
        value /= base;
    } while (value != 0U && count < sizeof(reversed));

    while (count != 0U && pos < capacity)
        dst[pos++] = reversed[--count];

    return pos;
}

// Mini-printf acotado: construye cada log completo antes de encolarlo.
void USB_Debug(const char *fmt, ...) {
    char out[192];
    uint16_t pos = 0;
    va_list ap;

    if (fmt == NULL) return;
    va_start(ap, fmt);

    while (*fmt && pos < sizeof(out)) {
        if (*fmt == '%') {
            fmt++;
            if (*fmt == '\0') break;
            switch (*fmt) {
                case 's': {
                    const char *s = va_arg(ap, const char *);
                    if (s == NULL) s = "(null)";
                    while (*s && pos < sizeof(out)) out[pos++] = *s++;
                    break;
                }
                case 'c': {
                    out[pos++] = (char)va_arg(ap, int);
                    break;
                }
                case 'u': {
                    pos = USB_AppendUInt(out, pos, sizeof(out),
                                         va_arg(ap, unsigned int), 10U);
                    break;
                }
                case 'd': {
                    int value = va_arg(ap, int);
                    unsigned int magnitude;
                    if (value < 0) {
                        if (pos < sizeof(out)) out[pos++] = '-';
                        magnitude = 0U - (unsigned int)value;
                    } else {
                        magnitude = (unsigned int)value;
                    }
                    pos = USB_AppendUInt(out, pos, sizeof(out), magnitude, 10U);
                    break;
                }
                case 'X': {
                    pos = USB_AppendUInt(out, pos, sizeof(out),
                                         va_arg(ap, unsigned int), 16U);
                    break;
                }
                case '%':
                    out[pos++] = '%';
                    break;
                default:
                    if (pos < sizeof(out)) out[pos++] = '%';
                    if (pos < sizeof(out)) out[pos++] = *fmt;
                    break;
            }
        } else {
            out[pos++] = *fmt;
        }
        fmt++;
    }

    va_end(ap);
    USB_DebugSend((const uint8_t *)out, pos);
}


uint8_t usb_enqueue_tx(const uint8_t *data, uint16_t len) {
    return usb_enqueue_tx_segments(data, len, NULL, 0U);
}

uint8_t usb_enqueue_tx_segments(const uint8_t *first, uint16_t first_len,
                                const uint8_t *second, uint16_t second_len) {
    uint32_t primask;
    uint16_t used;
    uint16_t free_space;
    uint16_t total_len = (uint16_t)(first_len + second_len);

    if (total_len == 0U) return 1;
    if ((first_len != 0U && first == NULL) || (second_len != 0U && second == NULL))
        return 0;

    primask = __get_PRIMASK();
    __disable_irq();
    used = (uint16_t)((tx_head - tx_tail) & USB_TX_BUF_MASK);
    free_space = (uint16_t)(USB_TX_BUF_MASK - used);
    if (total_len > free_space) {
        usb_tx_dropped++;
        if (!primask) __enable_irq();
        return 0;
    }

    for (uint16_t i = 0; i < first_len; i++) {
        usb_tx_buf[tx_head] = first[i];
        tx_head = (uint16_t)((tx_head + 1U) & USB_TX_BUF_MASK);
    }
    for (uint16_t i = 0; i < second_len; i++) {
        usb_tx_buf[tx_head] = second[i];
        tx_head = (uint16_t)((tx_head + 1U) & USB_TX_BUF_MASK);
    }
    if (!primask) __enable_irq();

    if (usb_tx_busy == 0) {
        usb_service_tx();
    }
    return 1;
}

void usb_service_tx(void) {
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED || hcdc == NULL)
        return;

    if (usb_tx_busy && hcdc->TxState == 0) {
        usb_tx_busy = 0;
    }

    // 2) Si aún está ocupada la línea o no hay datos, no hacemos nada
    if (usb_tx_busy || tx_head == tx_tail) {
        return;
    }

    // 3) Preparamos el siguiente chunk y lo enviamos.
    // static: CDC_Transmit_FS solo guarda el puntero y la transferencia USB ocurre
    // después de retornar — un buffer de stack ya liberado corrompería los datos.
    static uint8_t chunk[64];
    uint16_t cnt = 0;
    uint16_t read_index = tx_tail;
    while (cnt < sizeof(chunk) && read_index != tx_head) {
        chunk[cnt++] = usb_tx_buf[read_index];
        read_index = (uint16_t)((read_index + 1U) & USB_TX_BUF_MASK);
    }

    // Solo consumir bytes si el driver aceptó realmente la transferencia.
    if (cnt && CDC_Transmit_FS(chunk, cnt) == USBD_OK) {
        tx_tail = read_index;
        usb_tx_busy = 1;
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

    // De % de potencia a registro de comparación del timer: el contador PWM
    // cuenta de 0 a ARR (959) y el pin está en alto mientras cuenta < CCR, así
    // que duty% = CCR/(ARR+1). Despejando: CCR = (ARR+1)·duty/100.
    // Ej.: 50% → CCR=480 → pin en alto 480 de cada 960 ticks del timer.
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

    uint8_t enqueued = I2C_Manager_EnqueuePriority(&req);

    if (!enqueued) {
        mpu_req_pending = 0;
        return -1;
    }

    // NO llamar Process() acá — solo marcar pending
    // El main loop lo va a llamar
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
    // Copia coherente de los últimos 32 barridos completos (~8ms). A diferencia
    // del filtro anterior, que elegía una sola de las ~40 secuencias disponibles
    // cada 10ms, la mediana usa el flujo real de 4kHz y rechaza pulsos aislados a
    // 4095 causados por conmutación/EMI. Un nivel alto sostenido no se oculta.
    uint16_t history_copy[ADC_HISTORY_SIZE][8];
    uint16_t channel_samples[ADC_HISTORY_SIZE];
    uint32_t seq_before, seq_after;
    uint8_t sample_count;
    for (;;) {
        seq_before = adc_snapshot_seq;
        if (seq_before & 1U) continue;
        __DMB();
        sample_count = adc_history_count;
        for (uint8_t i = 0; i < sample_count; ++i)
            for (uint8_t ch = 0; ch < 8; ++ch)
                history_copy[i][ch] = adcHistory[i][ch];
        __DMB();
        seq_after = adc_snapshot_seq;
        if (seq_before == seq_after && !(seq_after & 1U)) break;
    }

    if (sample_count == 0) return;

    for (uint8_t ch = 0; ch < BAR_COUNT; ++ch) {
        for (uint8_t i = 0; i < sample_count; ++i)
            channel_samples[i] = history_copy[i][ch];

        // Insertion sort: n<=32, costo pequeño a 100Hz y sin memoria dinámica.
        for (uint8_t i = 1; i < sample_count; ++i) {
            uint16_t key = channel_samples[i];
            int j = (int)i - 1;
            while (j >= 0 && channel_samples[j] > key) {
                channel_samples[j + 1] = channel_samples[j];
                j--;
            }
            channel_samples[j + 1] = key;
        }

        uint8_t mid = sample_count / 2U;
        adcAvg[ch] = (sample_count & 1U)
                   ? channel_samples[mid]
                   : (uint16_t)(((uint32_t)channel_samples[mid - 1U] +
                                 (uint32_t)channel_samples[mid]) / 2U);
    }
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
        wifi_splash_until_ms = HAL_GetTick() + WIFI_SPLASH_MS;  // splash SSID+IP en display
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

static uint8_t ObjWall_LineConfirmed(uint8_t line_detected, uint8_t line_ignore)
{
    if (line_ignore || !line_detected) {
        obj_wall_line_confirm_cnt = 0;
        return 0;
    }
    if (obj_wall_line_confirm_cnt < OBJ_WALL_LINE_CONFIRM_CYC)
        obj_wall_line_confirm_cnt++;
    return (obj_wall_line_confirm_cnt >= OBJ_WALL_LINE_CONFIRM_CYC);
}

// Anti-stall genérico (2026-07-10): boost de ángulo en rampa para estados que
// requieren movimiento (ver defines ANTISTALL_*). Una sola instancia global:
// los estados que lo usan son mutuamente excluyentes — `tag` identifica la
// maniobra y resetea el boost al cambiar de llamador o si nadie lo llamó hace
// >200ms (re-entrada al mismo estado tras un intervalo = maniobra nueva).
static float AntiStall_Tick(uint8_t tag, uint8_t wants_motion, float max_boost)
{
    static float    as_boost   = 0.0f;
    static uint16_t as_still   = 0;
    static uint8_t  as_tag     = 0xFF;
    static uint32_t as_last_ms = 0;

    uint32_t now = HAL_GetTick();
    if (tag != as_tag || (now - as_last_ms) > 200U) {
        as_tag   = tag;
        as_boost = 0.0f;
        as_still = 0;
    }
    as_last_ms = now;

    if (fabsf(velocity_est_f) < ANTISTALL_VEL_THR) {
        if (as_still < 65535U) as_still++;
    } else {
        as_still  = 0;
        as_boost *= ANTISTALL_DECAY;
    }
    if (wants_motion && as_still >= ANTISTALL_WAIT_CYC)
        as_boost = fminf(as_boost + ANTISTALL_STEP, max_boost);
    return as_boost;
}


static float ComputeBrakeFromVelocity(float velocity, float brake_tilt_max)
{
    float vel_for_brake = apply_deadbandf(velocity, BRAKE_VEL_DEADBAND);
    vel_for_brake = clampf_local(vel_for_brake, -BRAKE_VEL_MAX, BRAKE_VEL_MAX);
    float abs_vel = fabsf(vel_for_brake);
    float brake_mag = KV_BRAKE * abs_vel;
    if (abs_vel > BRAKE_VEL_THRESHOLD)
        brake_mag += KV_brake_value * (abs_vel - BRAKE_VEL_THRESHOLD);

    if (vel_for_brake < 0.0f)
        brake_mag = -brake_mag;

    return clampf_local(brake_mag, -brake_tilt_max, brake_tilt_max);
}

static float ComputeBrakeSetpointTarget(uint8_t state)
{
    if ((state == ROBOT_STATE_LINE_FOLLOWING) ||
        (state == ROBOT_STATE_BALANCE_AND_SPEED)) {
        return 0.0f;
    }

    // 2026-07-10: velocidad LENTA (tau ~0.5s) en lugar de velocity_est_f — el
    // vaivén en el lugar promedia a ~0 y ya no dispara el freno (que bombeaba
    // la oscilación); la deriva real sostenida sigue pasando. Ver VEL_SLOW_BETA.
    float brake_tilt_max = (state == ROBOT_STATE_MANUAL_CONTROL)
                         ? BRAKE_TILT_MAX_MANUAL
                         : BRAKE_TILT_MAX;
    return ComputeBrakeFromVelocity(velocity_est_slow_f, brake_tilt_max);
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
    float steer_err = sp - (speed_r - speed_l);

    float p = STEER_KP * steer_err;

    steer_pid_integral += steer_err * DT_CTRL_FIXED;
    if (steer_pid_integral >  STEER_I_MAX) steer_pid_integral =  STEER_I_MAX;
    if (steer_pid_integral < -STEER_I_MAX) steer_pid_integral = -STEER_I_MAX;
    float i = STEER_KI * steer_pid_integral;

    float d = STEER_KD * (steer_err - steer_pid_prev_error) / DT_CTRL_FIXED;
    steer_pid_prev_error = steer_err;

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

    // Acotar rangos explícitamente: el valor mostrado nunca necesita más de
    // 5 dígitos enteros, y frac_part ya es < scale — el módulo redundante
    // le deja claro el rango al compilador (evita -Wformat-truncation).
    if (int_part > 99999U) int_part = 99999U;

    switch (decimals) {
        case 0:
            snprintf(buf, buf_size, "%c%lu", sign, (unsigned long)int_part);
            break;
        case 1:
            snprintf(buf, buf_size, "%c%lu.%01lu", sign,
                     (unsigned long)int_part, (unsigned long)(frac_part % 10U));
            break;
        case 2:
            snprintf(buf, buf_size, "%c%lu.%02lu", sign,
                     (unsigned long)int_part, (unsigned long)(frac_part % 100U));
            break;
        default:
            snprintf(buf, buf_size, "%c%lu.%03lu", sign,
                     (unsigned long)int_part, (unsigned long)(frac_part % 1000U));
            break;
    }
}

// ─────────────────────────────────────────────────────────────────────
// Helpers de dibujo del display (usados solo por updateDisplay)
// ─────────────────────────────────────────────────────────────────────
static uint8_t oled_alive_phase = 0;   // parpadeo del punto "vivo" del header

// Dibuja un string en 5x7. OJO: SSD1306_DrawChar5x7 solo tiene glifos para
// 0-9 y A-Z — acá se mapean las minúsculas a mayúsculas y se dibuja la
// puntuación común a mano (píxeles), si no esos caracteres desaparecen.
static void OLED_Str5(uint16_t x, uint16_t y, const char *s)
{
    for (; *s; s++) {
        char c = *s;
        if (c >= 'a' && c <= 'z') c = (char)(c - 'a' + 'A');
        switch (c) {
            case ':':
                SSD1306_DrawPixel(x + 2, y + 2, SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(x + 2, y + 5, SSD1306_COLOR_WHITE);
                break;
            case '.':
                SSD1306_DrawPixel(x + 2, y + 6, SSD1306_COLOR_WHITE);
                break;
            case '-':
                SSD1306_DrawLine(x + 1, y + 3, x + 3, y + 3, SSD1306_COLOR_WHITE);
                break;
            case '+':
                SSD1306_DrawLine(x,     y + 3, x + 4, y + 3, SSD1306_COLOR_WHITE);
                SSD1306_DrawLine(x + 2, y + 1, x + 2, y + 5, SSD1306_COLOR_WHITE);
                break;
            case '*':
                SSD1306_DrawLine(x,     y + 1, x + 4, y + 5, SSD1306_COLOR_WHITE);
                SSD1306_DrawLine(x + 4, y + 1, x,     y + 5, SSD1306_COLOR_WHITE);
                break;
            case '/':
                SSD1306_DrawLine(x, y + 6, x + 4, y, SSD1306_COLOR_WHITE);
                break;
            case '\\':
                SSD1306_DrawLine(x, y, x + 4, y + 6, SSD1306_COLOR_WHITE);
                break;
            case '|':
                SSD1306_DrawLine(x + 2, y, x + 2, y + 6, SSD1306_COLOR_WHITE);
                break;
            case '<':
                SSD1306_DrawLine(x + 3, y + 1, x + 1, y + 3, SSD1306_COLOR_WHITE);
                SSD1306_DrawLine(x + 1, y + 3, x + 3, y + 5, SSD1306_COLOR_WHITE);
                break;
            case '>':
                SSD1306_DrawLine(x + 1, y + 1, x + 3, y + 3, SSD1306_COLOR_WHITE);
                SSD1306_DrawLine(x + 3, y + 3, x + 1, y + 5, SSD1306_COLOR_WHITE);
                break;
            case '^':
                SSD1306_DrawLine(x,     y + 3, x + 2, y + 1, SSD1306_COLOR_WHITE);
                SSD1306_DrawLine(x + 2, y + 1, x + 4, y + 3, SSD1306_COLOR_WHITE);
                break;
            case '=':
                SSD1306_DrawLine(x + 1, y + 2, x + 3, y + 2, SSD1306_COLOR_WHITE);
                SSD1306_DrawLine(x + 1, y + 4, x + 3, y + 4, SSD1306_COLOR_WHITE);
                break;
            case '!':
                SSD1306_DrawLine(x + 2, y, x + 2, y + 4, SSD1306_COLOR_WHITE);
                SSD1306_DrawPixel(x + 2, y + 6, SSD1306_COLOR_WHITE);
                break;
            default:
                SSD1306_DrawChar5x7(c, x, y);   // 0-9, A-Z; espacio y otros: nada
                break;
        }
        x += (uint16_t)(Font_5x7.FontWidth + 1);
    }
}

static uint16_t OLED_Str5W(const char *s)
{
    uint16_t w = 0;
    for (; *s; s++) w += (uint16_t)(Font_5x7.FontWidth + 1);
    return w;
}

static void OLED_Str5Centered(uint16_t y, const char *s)
{
    uint16_t w = OLED_Str5W(s);
    OLED_Str5((uint16_t)((w < SCREEN_W) ? (SCREEN_W - w) / 2 : 0), y, s);
}

static void OLED_Puts7CenteredX(const char *s, uint16_t x0, uint16_t x1, uint16_t y)
{
    uint16_t len = 0;
    for (const char *p = s; *p; p++) len++;
    uint16_t sw   = (len > 0) ? (uint16_t)(len * 8 - 1) : 0;
    uint16_t span = (uint16_t)(x1 - x0);
    uint16_t sx   = x0 + ((span > sw) ? (span - sw) / 2 : 0);
    SSD1306_GotoXY(sx, y);
    SSD1306_Puts(s, &Font_7x10, SSD1306_COLOR_WHITE);
}

static void OLED_WifiIcon7(uint16_t x, uint16_t y)   // 7x6 px
{
    if (f_wifi_connected) {
        SSD1306_DrawLine(x + 1, y,     x + 5, y,     SSD1306_COLOR_WHITE);
        SSD1306_DrawPixel(x,     y + 1, SSD1306_COLOR_WHITE);
        SSD1306_DrawPixel(x + 6, y + 1, SSD1306_COLOR_WHITE);
        SSD1306_DrawLine(x + 2, y + 2, x + 4, y + 2, SSD1306_COLOR_WHITE);
        SSD1306_DrawPixel(x + 1, y + 3, SSD1306_COLOR_WHITE);
        SSD1306_DrawPixel(x + 5, y + 3, SSD1306_COLOR_WHITE);
        SSD1306_DrawPixel(x + 3, y + 5, SSD1306_COLOR_WHITE);
    } else {
        SSD1306_DrawLine(x, y, x + 6, y + 5, SSD1306_COLOR_WHITE);
        SSD1306_DrawLine(x + 6, y, x, y + 5, SSD1306_COLOR_WHITE);
    }
}

// Header común a todas las pantallas: título a la izquierda; "F!" centrado
// si está caído; a la derecha modo actual, icono WiFi y spinner de actividad
// que gira mientras el display se sigue refrescando (loop vivo).
static void OLED_Header(const char *title)
{
    OLED_Str5(1, 1, title);

    const char *mode_str;
    switch (robot_state) {
        case ROBOT_STATE_BALANCE_ONLY:      mode_str = "BAL"; break;
        case ROBOT_STATE_BALANCE_AND_SPEED: mode_str = "SPD"; break;
        case ROBOT_STATE_LINE_FOLLOWING:    mode_str = "LIN"; break;
        case ROBOT_STATE_MANUAL_CONTROL:    mode_str = "MAN"; break;
        case ROBOT_STATE_MOTOR_TEST:        mode_str = "TST"; break;
        default:                            mode_str = "IDL"; break;
    }

    // Spinner de actividad tamaño carácter (| / - \): mucho más visible que
    // un punto — si deja de girar, el loop/display está congelado.
    // (vía OLED_Str5: DrawChar5x7 no tiene glifos para estos caracteres)
    static const char spin_chars[4] = { '|', '/', '-', '\\' };
    char spin_buf[2] = { spin_chars[oled_alive_phase & 0x03], '\0' };
    uint16_t x = SCREEN_W - 7;
    OLED_Str5(x, 1, spin_buf);

    x -= 10;                                  // icono wifi (7px + aire)
    OLED_WifiIcon7(x, 1);

    x -= (uint16_t)(OLED_Str5W(mode_str) + 3);
    OLED_Str5(x, 1, mode_str);

    // Indicador de caída centrado en el header (antes seguía la pila de la
    // derecha y se mezclaba con el modo/icono WiFi).
    if (f_fallen) {
        OLED_Str5Centered(1, "F!");
    }

    SSD1306_DrawLine(0, 9, SCREEN_W - 1, 9, SSD1306_COLOR_WHITE);
}

// Nombre corto del sub-estado del seguidor de línea (pantallas 1 y 6).
static const char *LineStateStr(void)
{
    if (robot_state != ROBOT_STATE_LINE_FOLLOWING) return "OFF";
    switch (line_state) {
        case LINE_STATE_FOLLOWING:          return "SIGUE";
        case LINE_STATE_LOST:               return "PERDI";
        case LINE_STATE_SEARCHING:          return "BUSCA";
        case LINE_STATE_LOST_BRAKE:         return "FRENA";
        case LINE_STATE_LOST_ROTATE:        return "GIRO";
        case LINE_STATE_LOST_SETTLE:        return "ESTAB";
        case LINE_STATE_LOST_FWD:           return "VUELVE";
        case LINE_STATE_EDGE_WAIT:          return "EFRENA";
        case LINE_STATE_EDGE_ROTATE:        return "EGIRO";
        case LINE_STATE_EDGE_SETTLE:        return "EESTAB";
        case LINE_STATE_EDGE_FWD:           return "EVUELV";
        case LINE_STATE_GIVEN_UP:           return "PARADO";
        case LINE_STATE_PERP_ROTATE:        return "PGIRO";
        case LINE_STATE_OBJ_FRENO_REVERSA:  return "STOP";
        case LINE_STATE_OBJ_GIRO_ESQUIVE:   return "ESQUIV";
        case LINE_STATE_OBJ_PAUSA_GIRO:     return "PAUSA";
        case LINE_STATE_OBJ_BUSCAR_PARED:   return "APRCH";
        case LINE_STATE_OBJ_BORDEAR_PARED:  return "PARED";
        case LINE_STATE_OBJ_PARED_LIBRE:    return "LIBRE";
        case LINE_STATE_OBJ_GIRO_PARED:     return "GIRAP";
        default:                            return "UNK";
    }
}

// Acción vigente del wall-following (pantalla 6), calculada directo de los
// umbrales crudos de ADC7 — exactamente lo que deciden los 3 estados de pared
// cada ciclo. Solo tiene sentido durante PARED/LIBRE/GIRAP.
static const char *ObjWallActionStr(void)
{
    float wall_adc = (float)adcAvg[OBJ_WALL_ADC_IDX];
    if (wall_adc < OBJ_WALL_REVERSE_THOLD)   return "REV";
    if (wall_adc < OBJ_WALL_TOO_CLOSE_THOLD) return "PIV";
    return "AVZ";
}

// Tiempo transcurrido en el sub-estado actual del seguidor. Se mide acá,
// al ritmo de refresco del display — suficiente para debug visual.
static uint32_t OLED_LineStateElapsedMs(void)
{
    static eLineState oled_prev_lstate = LINE_STATE_FOLLOWING;
    static uint32_t   oled_lstate_t0   = 0;
    if (line_state != oled_prev_lstate) {
        oled_prev_lstate = line_state;
        oled_lstate_t0   = HAL_GetTick();
    }
    return HAL_GetTick() - oled_lstate_t0;
}

// Barra horizontal con marco, relleno proporcional (escala ADC 0..4095) y
// ticks de umbral dibujados en color invertido respecto al relleno para
// que se vean tanto sobre la parte llena como sobre la vacía.
static void OLED_HBar(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                      uint16_t value, const float *ticks, uint8_t n_ticks)
{
    if (value > 4095) value = 4095;
    SSD1306_DrawRectangle(x, y, w, h, SSD1306_COLOR_WHITE);
    uint16_t fill = (uint16_t)((uint32_t)value * (uint32_t)(w - 2) / 4095U);
    if (fill > 0)
        SSD1306_DrawFilledRectangle(x + 1, y + 1, fill, h - 2, SSD1306_COLOR_WHITE);
    for (uint8_t i = 0; i < n_ticks; i++) {
        uint16_t tx = x + 1 + (uint16_t)((uint32_t)ticks[i] * (uint32_t)(w - 2) / 4095U);
        SSD1306_COLOR_t tc = (tx <= x + fill) ? SSD1306_COLOR_BLACK : SSD1306_COLOR_WHITE;
        SSD1306_DrawLine(tx, y + 1, tx, y + h - 2, tc);
    }
}

void updateDisplay(void) {
    SSD1306_Fill(SSD1306_COLOR_BLACK);
    oled_alive_phase++;

    char nbuf[16];
    char lbuf[24];

    // Alarma enclavada: prioridad sobre todas las pantallas y el splash WiFi.
    // El modo sigue siendo LINE; f_fallen solo mantiene los motores apagados.
    if (speed_limit_fault) {
        OLED_Header("LINEA");
        OLED_Puts7CenteredX("LIMITE DE", 0, SCREEN_W - 1, 14);
        OLED_Puts7CenteredX("VELOCIDAD", 0, SCREEN_W - 1, 27);
        OLED_Puts7CenteredX("EXCEDIDO", 0, SCREEN_W - 1, 40);
        OLED_Str5Centered(55, "MOTORES APAGADOS");
        SSD1306_RequestUpdate();
        return;
    }

    // ── Splash de conexión WiFi: pisa CUALQUIER pantalla durante WIFI_SPLASH_MS
    // y al vencer vuelve sola a la que estaba (f_change_display no se toca).
    if (wifi_splash_until_ms != 0) {
        if ((int32_t)(wifi_splash_until_ms - HAL_GetTick()) > 0) {
            OLED_Header("WIFI");
            OLED_Puts7CenteredX("CONECTADO", 0, SCREEN_W - 1, 14);

            // SSID truncado a 21 chars (128px / 6px por char en 5x7);
            // copia manual para no disparar -Wformat-truncation. Todas las
            // líneas centradas (pedido 2026-07-13).
            char ssid_buf[22];
            uint8_t si = 0;
            for (; wifiSSID[si] != '\0' && si < sizeof(ssid_buf) - 1; si++)
                ssid_buf[si] = wifiSSID[si];
            ssid_buf[si] = '\0';
            OLED_Str5Centered(30, "RED:");
            OLED_Str5Centered(38, ssid_buf);

            const char *lip = ESP01_GetLocalIP();   // IP propia (NULL si aún no está)
            char ip_buf[24];
            uint8_t bi = 0;
            const char *pfx = "IP: ";
            for (; *pfx; pfx++) ip_buf[bi++] = *pfx;
            for (const char *p = (lip != NULL) ? lip : "...";
                 *p != '\0' && bi < sizeof(ip_buf) - 1; p++)
                ip_buf[bi++] = *p;
            ip_buf[bi] = '\0';
            OLED_Str5Centered(48, ip_buf);

            bi = 0;                                 // IP destino (Qt), del perfil activo
            pfx = "PC: ";
            for (; *pfx; pfx++) ip_buf[bi++] = *pfx;
            for (const char *p = wifiIp; *p != '\0' && bi < sizeof(ip_buf) - 1; p++)
                ip_buf[bi++] = *p;
            ip_buf[bi] = '\0';
            OLED_Str5Centered(56, ip_buf);

            SSD1306_RequestUpdate();
            return;
        }
        wifi_splash_until_ms = 0;   // venció: seguir con la pantalla normal
    }

    if (f_change_display == 0) {
        // ───────────────────────────────────────────────────────────
        // PANTALLA 0 — BALANCE: gauge de roll + números clave del PID
        // ───────────────────────────────────────────────────────────
        OLED_Header("BALANCE");
        SSD1306_DrawLine(63, 11, 63, SCREEN_H - 1, SSD1306_COLOR_WHITE);

        // ── Izquierda: ángulo grande + inclinómetro horizontal ±15° ──
        OLED_Str5(2, 13, "ANG");
        FormatSignedFixed(nbuf, sizeof(nbuf), filtered_roll_deg, 1);
        SSD1306_GotoXY(2, 21);
        SSD1306_Puts(nbuf, &Font_7x10, SSD1306_COLOR_WHITE);

        {
            const uint16_t gx0 = 2, gy0 = 34, gw = 58, gh = 9;
            SSD1306_DrawRectangle(gx0, gy0, gw, gh, SSD1306_COLOR_WHITE);
            // tick central (0°) por encima del marco
            SSD1306_DrawLine(gx0 + gw / 2, gy0 - 2, gx0 + gw / 2, gy0 - 1, SSD1306_COLOR_WHITE);
            // marcador de roll actual (±15° a fondo de escala)
            float rr = clampf_local(filtered_roll_deg, -15.0f, 15.0f);
            int16_t moff = (int16_t)(rr * (float)(gw / 2 - 3) / 15.0f);
            uint16_t mx = (uint16_t)((int16_t)(gx0 + gw / 2) + moff);
            SSD1306_DrawFilledRectangle(mx - 1, gy0 + 2, 3, gh - 4, SSD1306_COLOR_WHITE);
            // marcador del setpoint (tick corto debajo del marco)
            float ss = clampf_local(dynamic_setpoint_f, -15.0f, 15.0f);
            int16_t soff = (int16_t)(ss * (float)(gw / 2 - 3) / 15.0f);
            uint16_t sxp = (uint16_t)((int16_t)(gx0 + gw / 2) + soff);
            SSD1306_DrawLine(sxp, gy0 + gh + 1, sxp, gy0 + gh + 2, SSD1306_COLOR_WHITE);
        }

        FormatSignedFixed(nbuf, sizeof(nbuf), dynamic_setpoint_f, 2);
        snprintf(lbuf, sizeof(lbuf), "SP:%s", nbuf);
        OLED_Str5(2, 49, lbuf);
        FormatSignedFixed(nbuf, sizeof(nbuf), setpoint_trim, 1);
        snprintf(lbuf, sizeof(lbuf), "TR:%s", nbuf);
        OLED_Str5(2, 57, lbuf);

        // ── Derecha: ganancias PID + velocidad + motores ──
        {
            const uint16_t rx = 67;
            FormatSignedFixed(nbuf, sizeof(nbuf), KP_value, 3);
            snprintf(lbuf, sizeof(lbuf), "KP:%s", nbuf + 1);  OLED_Str5(rx, 12, lbuf);
            FormatSignedFixed(nbuf, sizeof(nbuf), KI_value, 3);
            snprintf(lbuf, sizeof(lbuf), "KI:%s", nbuf + 1);  OLED_Str5(rx, 21, lbuf);
            FormatSignedFixed(nbuf, sizeof(nbuf), KD_value, 3);
            snprintf(lbuf, sizeof(lbuf), "KD:%s", nbuf + 1);  OLED_Str5(rx, 30, lbuf);
            FormatSignedFixed(nbuf, sizeof(nbuf), velocity_est_f, 2);
            snprintf(lbuf, sizeof(lbuf), "VE:%s", nbuf);      OLED_Str5(rx, 39, lbuf);

            if (robot_state == ROBOT_STATE_MANUAL_CONTROL) {
                uint8_t cmd = UNER_GetLastManualCmd();
                char c = (cmd == MOVE_FORWARD)  ? '^'
                       : (cmd == MOVE_BACKWARD) ? 'v'
                       : (cmd == MOVE_LEFT)     ? '<'
                       : (cmd == MOVE_RIGHT)    ? '>' : 'o';
                snprintf(lbuf, sizeof(lbuf), "CMD:%c", c);
            } else {
                // comando de motores (PWM firmado, R y L)
                snprintf(lbuf, sizeof(lbuf), "M%+03d%+03d",
                         (int)motorRightVelocity, (int)motorLeftVelocity);
            }
            OLED_Str5(rx, 57, lbuf);
        }

    }  else if (f_change_display == 1) {
        // ───────────────────────────────────────────────────────────
        // PANTALLA 1 — LINEA: barras ADC + posición de línea + estado
        // ───────────────────────────────────────────────────────────
        OLED_Header("LINEA");

        // ── Izquierda: 8 barras (1-4 línea con indicador B/N, 5-8 objeto) ──
        {
            const uint16_t bar_top   = 12;
            const uint16_t ind_h     = 4;
            const uint16_t digit_y   = 47;
            const uint16_t bar_width = 7;
            const uint16_t spacing   = 1;
            const uint16_t bar_line_top = bar_top + ind_h;

            for (uint8_t i = 0; i < 8; i++) {
                // ADC 1-4 en orden invertido (coincide con la vista del robot)
                uint8_t adc_idx = (i < 4) ? (3 - i) : i;
                uint16_t v  = adcAvg[adc_idx] > 4095 ? 4095 : adcAvg[adc_idx];
                uint16_t x0 = spacing + i * (bar_width + spacing);

                if (i < 4) {
                    uint16_t hmax = digit_y - 1 - bar_line_top;
                    uint16_t h = (uint32_t)v * hmax / 4095;
                    if (h > 0)
                        SSD1306_DrawFilledRectangle(x0, digit_y - 1 - h, bar_width, h, SSD1306_COLOR_WHITE);
                    // indicador negro/blanco: relleno = ve cinta negra
                    if (adcAvg[adc_idx] > (uint16_t)LINE_THRESHOLD)
                        SSD1306_DrawFilledRectangle(x0, bar_top, bar_width, ind_h - 1, SSD1306_COLOR_WHITE);
                    else
                        SSD1306_DrawRectangle(x0, bar_top, bar_width, ind_h - 1, SSD1306_COLOR_WHITE);
                } else {
                    uint16_t hmax = digit_y - 1 - bar_top;
                    uint16_t h = (uint32_t)v * hmax / 4095;
                    if (h > 0)
                        SSD1306_DrawFilledRectangle(x0, digit_y - 1 - h, bar_width, h, SSD1306_COLOR_WHITE);
                    // tick del umbral de objeto, en color invertido si la barra lo cubre
                    uint16_t hth = (uint16_t)((uint32_t)OBJ_DETECT_THRESHOLD_VAL * hmax / 4095U);
                    uint16_t yth = digit_y - 1 - hth;
                    SSD1306_COLOR_t tc = (h >= hth) ? SSD1306_COLOR_BLACK : SSD1306_COLOR_WHITE;
                    SSD1306_DrawLine(x0, yth, x0 + bar_width - 1, yth, tc);
                }
                if (adc_idx < 4 && line_ch_quarantine[adc_idx])
                    SSD1306_DrawChar5x7('X', x0 + 1, digit_y);   // sensor en cuarentena
                else
                    SSD1306_DrawChar5x7('1' + adc_idx, x0 + 1, digit_y);
            }

            // separador punteado entre grupo línea (1-4) y objeto (5-8)
            uint16_t sep_x = spacing + 4 * (bar_width + spacing) - 1;
            for (uint16_t py = bar_top; py < digit_y - 1; py += 3)
                SSD1306_DrawPixel(sep_x, py, SSD1306_COLOR_WHITE);
        }

        // ── Franja de posición de línea (centroide, ±0.6 a fondo de escala) ──
        {
            const uint16_t fx = 0, fy = 56, fw = 70, fh = 8;
            SSD1306_DrawRectangle(fx, fy, fw, fh, SSD1306_COLOR_WHITE);
            // tick central
            SSD1306_DrawLine(fx + fw / 2, fy + 2, fx + fw / 2, fy + fh - 3, SSD1306_COLOR_WHITE);
            uint8_t det = 0;
            for (uint8_t ch = 0; ch < 4; ch++)
                if (!line_ch_quarantine[ch] &&
                    adcAvg[ch] > (uint16_t)LINE_THRESHOLD) det = 1;
            if (det) {
                float e = clampf_local(line_error_disp, -0.6f, 0.6f);
                int16_t off = (int16_t)(e * (float)(fw / 2 - 4) / 0.6f);
                uint16_t cx = (uint16_t)((int16_t)(fx + fw / 2) + off);
                SSD1306_DrawFilledRectangle(cx - 1, fy + 1, 3, fh - 2, SSD1306_COLOR_WHITE);
            }
        }

        SSD1306_DrawLine(71, 11, 71, SCREEN_H - 1, SSD1306_COLOR_WHITE);

        // ── Derecha: estado + tiempo en estado + números clave ──
        OLED_Puts7CenteredX(LineStateStr(), 73, 127, 12);
        {
            uint32_t e = OLED_LineStateElapsedMs();
            snprintf(lbuf, sizeof(lbuf), "t:%lu.%01lus",
                     (unsigned long)(e / 1000U), (unsigned long)((e % 1000U) / 100U));
            OLED_Str5(74, 24, lbuf);
        }
        FormatSignedFixed(nbuf, sizeof(nbuf), line_error_disp, 2);
        snprintf(lbuf, sizeof(lbuf), "E:%s", nbuf);   OLED_Str5(74, 33, lbuf);
        FormatSignedFixed(nbuf, sizeof(nbuf), velocity_est_f, 2);
        snprintf(lbuf, sizeof(lbuf), "V:%s", nbuf);   OLED_Str5(74, 42, lbuf);
        snprintf(lbuf, sizeof(lbuf), "A%c:%u",
                 (obj_esquive_dir > 0) ? '7' : '5',
                 (unsigned)adcAvg[OBJ_WALL_ADC_IDX]);
        OLED_Str5(74, 51, lbuf);

    } else if (f_change_display == 2) {
        // ───────────────────────────────────────────────────────────
        // PANTALLA 2 — ADC BARRAS: 8 canales con umbral y disparo
        // ───────────────────────────────────────────────────────────
        OLED_Header("ADC BARRAS");

        const uint16_t bar_top   = 12;
        const uint16_t digit_y   = 56;
        const uint16_t bar_max_h = digit_y - bar_top - 1;
        const uint16_t spacing   = 2;
        const uint16_t bar_width = (SCREEN_W - (BAR_COUNT + 1) * spacing) / BAR_COUNT;

        for (uint8_t i = 0; i < BAR_COUNT; i++) {
            uint16_t v  = adcAvg[i] > 4095 ? 4095 : adcAvg[i];
            uint16_t h  = (uint32_t)v * bar_max_h / 4095;
            uint16_t x0 = spacing + i * (bar_width + spacing);
            if (h > 0)
                SSD1306_DrawFilledRectangle(x0, digit_y - 1 - h, bar_width, h, SSD1306_COLOR_WHITE);

            // tick de umbral (1-4: LINE_THRESHOLD, 5-8: OBJ_DETECT_THRESHOLD_VAL),
            // en color invertido si la barra ya lo cubre
            float th = (i < 4) ? LINE_THRESHOLD : OBJ_DETECT_THRESHOLD_VAL;
            uint16_t hth = (uint16_t)((uint32_t)th * bar_max_h / 4095U);
            uint16_t yth = digit_y - 1 - hth;
            SSD1306_COLOR_t tc = (h >= hth) ? SSD1306_COLOR_BLACK : SSD1306_COLOR_WHITE;
            SSD1306_DrawLine(x0, yth, x0 + bar_width - 1, yth, tc);

            // indicador de disparo: cuadradito arriba de la columna
            // (línea 1-4: v > umbral = cinta negra; objeto 5-8: v < umbral = objeto)
            uint8_t trig = (i < 4) ? (adcAvg[i] > (uint16_t)LINE_THRESHOLD)
                                   : ((float)adcAvg[i] < OBJ_DETECT_THRESHOLD_VAL);
            if (trig) {
                SSD1306_COLOR_t ic = (h >= bar_max_h - 7) ? SSD1306_COLOR_BLACK
                                                          : SSD1306_COLOR_WHITE;
                SSD1306_DrawFilledRectangle(x0 + bar_width / 2 - 2, bar_top + 1, 5, 5, ic);
            }

            uint16_t tx = x0 + (bar_width - Font_5x7.FontWidth) / 2;
            SSD1306_DrawChar5x7('1' + i, tx, digit_y);
        }
        SSD1306_DrawLine(0, digit_y - 1, SCREEN_W - 1, digit_y - 1, SSD1306_COLOR_WHITE);

    } else if (f_change_display == 3) {
        // ───────────────────────────────────────────────────────────
        // PANTALLA 3 — PARAM: ganancias de balance y de línea
        // ───────────────────────────────────────────────────────────
        OLED_Header("PARAM");
        SSD1306_DrawLine(63, 11, 63, SCREEN_H - 1, SSD1306_COLOR_WHITE);

        OLED_Str5(2, 12, "BALANCE");
        FormatSignedFixed(nbuf, sizeof(nbuf), KP_value, 3);
        snprintf(lbuf, sizeof(lbuf), "P:%s", nbuf + 1);   OLED_Str5(2, 21, lbuf);
        FormatSignedFixed(nbuf, sizeof(nbuf), KI_value, 3);
        snprintf(lbuf, sizeof(lbuf), "I:%s", nbuf + 1);   OLED_Str5(2, 30, lbuf);
        FormatSignedFixed(nbuf, sizeof(nbuf), KD_value, 3);
        snprintf(lbuf, sizeof(lbuf), "D:%s", nbuf + 1);   OLED_Str5(2, 39, lbuf);
        FormatSignedFixed(nbuf, sizeof(nbuf), setpoint_trim, 1);
        snprintf(lbuf, sizeof(lbuf), "TR:%s", nbuf);      OLED_Str5(2, 48, lbuf);
        FormatSignedFixed(nbuf, sizeof(nbuf), KV_brake_value, 1);
        snprintf(lbuf, sizeof(lbuf), "KV:%s", nbuf + 1);  OLED_Str5(2, 57, lbuf);

        OLED_Str5(66, 12, "LINEA");
        FormatSignedFixed(nbuf, sizeof(nbuf), KP_LINE, 2);
        snprintf(lbuf, sizeof(lbuf), "P:%s", nbuf + 1);   OLED_Str5(66, 21, lbuf);
        FormatSignedFixed(nbuf, sizeof(nbuf), KI_LINE, 2);
        snprintf(lbuf, sizeof(lbuf), "I:%s", nbuf + 1);   OLED_Str5(66, 30, lbuf);
        FormatSignedFixed(nbuf, sizeof(nbuf), KD_LINE, 2);
        snprintf(lbuf, sizeof(lbuf), "D:%s", nbuf + 1);   OLED_Str5(66, 39, lbuf);
        snprintf(lbuf, sizeof(lbuf), "TH:%lu", (unsigned long)(uint32_t)LINE_THRESHOLD);
        OLED_Str5(66, 48, lbuf);
        FormatSignedFixed(nbuf, sizeof(nbuf), LINE_SPEED_TARGET, 2);
        snprintf(lbuf, sizeof(lbuf), "VT:%s", nbuf + 1);  OLED_Str5(66, 57, lbuf);

    } else if (f_change_display == 4) {
        // ───────────────────────────────────────────────────────────
        // PANTALLA 4 — ADC VALORES: 8 canales numéricos + disparo (*)
        // ───────────────────────────────────────────────────────────
        OLED_Header("ADC VALORES");
        SSD1306_DrawLine(63, 11, 63, SCREEN_H - 1, SSD1306_COLOR_WHITE);

        const uint16_t rows4[4] = { 13, 26, 39, 52 };
        for (uint8_t i = 0; i < 4; i++) {
            uint8_t trig_l = (adcAvg[i] > (uint16_t)LINE_THRESHOLD);
            snprintf(lbuf, sizeof(lbuf), "%u:%4u%c", i + 1, adcAvg[i], trig_l ? '*' : ' ');
            OLED_Str5(2, rows4[i], lbuf);

            uint8_t trig_o = ((float)adcAvg[i + 4] < OBJ_DETECT_THRESHOLD_VAL);
            snprintf(lbuf, sizeof(lbuf), "%u:%4u%c", i + 5, adcAvg[i + 4], trig_o ? '*' : ' ');
            OLED_Str5(66, rows4[i], lbuf);
        }

    } else if (f_change_display == 5) {
        // ───────────────────────────────────────────────────────────
        // PANTALLA 5 — IMU: gyro/accel crudos + roll/omega/movimiento
        // ───────────────────────────────────────────────────────────
        OLED_Header("IMU");
        SSD1306_DrawLine(63, 11, 63, 45, SSD1306_COLOR_WHITE);

        OLED_Str5(2, 12, "GYRO");
        snprintf(lbuf, sizeof(lbuf), "X:%+d", (int)gx);  OLED_Str5(2, 20, lbuf);
        snprintf(lbuf, sizeof(lbuf), "Y:%+d", (int)gy);  OLED_Str5(2, 28, lbuf);
        snprintf(lbuf, sizeof(lbuf), "Z:%+d", (int)gz);  OLED_Str5(2, 36, lbuf);

        OLED_Str5(66, 12, "ACEL");
        snprintf(lbuf, sizeof(lbuf), "X:%+d", (int)ax);  OLED_Str5(66, 20, lbuf);
        snprintf(lbuf, sizeof(lbuf), "Y:%+d", (int)ay);  OLED_Str5(66, 28, lbuf);
        snprintf(lbuf, sizeof(lbuf), "Z:%+d", (int)az);  OLED_Str5(66, 36, lbuf);

        SSD1306_DrawLine(0, 46, SCREEN_W - 1, 46, SSD1306_COLOR_WHITE);

        FormatSignedFixed(nbuf, sizeof(nbuf), filtered_roll_deg, 1);
        snprintf(lbuf, sizeof(lbuf), "ROLL:%s", nbuf);   OLED_Str5(2, 49, lbuf);
        FormatSignedFixed(nbuf, sizeof(nbuf), gyro_f, 1);
        snprintf(lbuf, sizeof(lbuf), "W:%s", nbuf);      OLED_Str5(70, 49, lbuf);

        snprintf(lbuf, sizeof(lbuf), "MOV:%lu", (unsigned long)(uint32_t)accel_motion_f);
        OLED_Str5(2, 57, lbuf);
        FormatSignedFixed(nbuf, sizeof(nbuf), (float)gz / 100.0f, 0);
        snprintf(lbuf, sizeof(lbuf), "YAW:%s", nbuf);    OLED_Str5(70, 57, lbuf);

    } else if (f_change_display == 6) {
        // ───────────────────────────────────────────────────────────
        // PANTALLA 6 — OBJETO: estado de evasión + sensores con umbrales
        // ───────────────────────────────────────────────────────────
        OLED_Header("OBJETO");

        // En los estados de pared se agrega la acción vigente (AVZ/PIV/REV)
        // al nombre del sub-estado: "PARED>REV".
        {
            uint8_t in_wall = (robot_state == ROBOT_STATE_LINE_FOLLOWING) &&
                              (line_state == LINE_STATE_OBJ_BORDEAR_PARED ||
                               line_state == LINE_STATE_OBJ_PARED_LIBRE  ||
                               line_state == LINE_STATE_OBJ_GIRO_PARED);
            if (in_wall) {
                snprintf(lbuf, sizeof(lbuf), "%s>%s",
                         LineStateStr(), ObjWallActionStr());
                OLED_Puts7CenteredX(lbuf, 0, 127, 12);
            } else {
                OLED_Puts7CenteredX(LineStateStr(), 0, 127, 12);
            }
        }
        {
            uint32_t e = OLED_LineStateElapsedMs();
            snprintf(lbuf, sizeof(lbuf), "t:%lu.%01lus",
                     (unsigned long)(e / 1000U), (unsigned long)((e % 1000U) / 100U));
            uint16_t w = OLED_Str5W(lbuf);
            OLED_Str5((SCREEN_W - w) / 2, 24, lbuf);
        }

        // A6 (frontal, corte de la reversa inicial): ticks en detección (3200)
        // y en el corte de reversa (OBJ_REV_CLEAR_ADC=3600)
        {
            static const float ticks6[2] = { OBJ_DETECT_THRESHOLD_VAL,
                                             OBJ_REV_CLEAR_ADC };
            snprintf(lbuf, sizeof(lbuf), "A6:%4u", (unsigned)adcAvg[5]);
            OLED_Str5(0, 35, lbuf);
            OLED_HBar(44, 34, 83, 9, adcAvg[5], ticks6, 2);
        }
        // Lateral de pared activo (A7 esquivando a derecha, A5 a izquierda):
        // ticks en reversa / muy-cerca / pared visible
        {
            static const float ticks7[3] = { OBJ_WALL_REVERSE_THOLD,
                                             OBJ_WALL_TOO_CLOSE_THOLD,
                                             OBJ_WALL_THRESHOLD };
            snprintf(lbuf, sizeof(lbuf), "A%c:%4u",
                     (obj_esquive_dir > 0) ? '7' : '5',
                     (unsigned)adcAvg[OBJ_WALL_ADC_IDX]);
            OLED_Str5(0, 46, lbuf);
            OLED_HBar(44, 45, 83, 9, adcAvg[OBJ_WALL_ADC_IDX], ticks7, 3);
        }

        // A8: el corte de la reversa exige A6 Y A8 ≥ 3600 — si la reversa se
        // pasa de largo, mirar acá cuál de los dos es el que no despeja.
        FormatSignedFixed(nbuf, sizeof(nbuf), velocity_est_f, 2);
        snprintf(lbuf, sizeof(lbuf), "VE:%s", nbuf);     OLED_Str5(0, 57, lbuf);
        snprintf(lbuf, sizeof(lbuf), "A8:%4u", (unsigned)adcAvg[7]);
        OLED_Str5(70, 57, lbuf);

    } else if (f_change_display == 7) {
        // ───────────────────────────────────────────────────────────
        // PANTALLA 7 — ODOMETRIA: mapa de pose + números
        // Mapa: X odométrico hacia arriba, Y hacia la IZQUIERDA (θ positivo
        // antihorario, convención estándar — espejar Y aquí haría que la
        // flecha gire al revés de la realidad), origen al
        // centro. Autoescala para que la pose (y el punto de pérdida de
        // línea, si existe) siempre entren en el recuadro.
        // ───────────────────────────────────────────────────────────
        OLED_Header("ODOMETRIA");

        {
            const int16_t bx = 0, by = 11, bw = 53, bh = 53;
            const int16_t cx = bx + bw / 2, cy = by + bh / 2;
            SSD1306_DrawRectangle(bx, by, bw, bh, SSD1306_COLOR_WHITE);
            // cruz del origen
            SSD1306_DrawLine(cx - 2, cy, cx + 2, cy, SSD1306_COLOR_WHITE);
            SSD1306_DrawLine(cx, cy - 2, cx, cy + 2, SSD1306_COLOR_WHITE);

            float rng = 0.5f;
            if (fabsf(odom_x_m) > rng) rng = fabsf(odom_x_m);
            if (fabsf(odom_y_m) > rng) rng = fabsf(odom_y_m);
            if (line_loss_pose_valid) {
                if (fabsf(line_loss_x_m) > rng) rng = fabsf(line_loss_x_m);
                if (fabsf(line_loss_y_m) > rng) rng = fabsf(line_loss_y_m);
            }
            float k = (float)(bw / 2 - 3) / rng;

            // punto de pérdida de línea (cuadradito hueco)
            if (line_loss_pose_valid) {
                int16_t lx = cx - (int16_t)(line_loss_y_m * k);
                int16_t ly = cy - (int16_t)(line_loss_x_m * k);
                SSD1306_DrawRectangle(lx - 1, ly - 1, 3, 3, SSD1306_COLOR_WHITE);
            }

            // pose actual: punto lleno + rayo de rumbo (θ=0 → +X → arriba)
            int16_t px = cx - (int16_t)(odom_y_m * k);
            int16_t py = cy - (int16_t)(odom_x_m * k);
            float th = odom_theta_deg * ((float)M_PI / 180.0f);
            int16_t hx = px - (int16_t)(sinf(th) * 7.0f);
            int16_t hy = py - (int16_t)(cosf(th) * 7.0f);
            if (hx < bx + 1)      hx = bx + 1;
            if (hx > bx + bw - 2) hx = bx + bw - 2;
            if (hy < by + 1)      hy = by + 1;
            if (hy > by + bh - 2) hy = by + bh - 2;
            SSD1306_DrawLine(px, py, hx, hy, SSD1306_COLOR_WHITE);
            SSD1306_DrawFilledRectangle(px - 1, py - 1, 3, 3, SSD1306_COLOR_WHITE);
        }

        // ── Derecha: números ──
        {
            const uint16_t rx = 57;
            FormatSignedFixed(nbuf, sizeof(nbuf), odom_x_m, 2);
            snprintf(lbuf, sizeof(lbuf), "X:%sm", nbuf);   OLED_Str5(rx, 12, lbuf);
            FormatSignedFixed(nbuf, sizeof(nbuf), odom_y_m, 2);
            snprintf(lbuf, sizeof(lbuf), "Y:%sm", nbuf);   OLED_Str5(rx, 21, lbuf);
            FormatSignedFixed(nbuf, sizeof(nbuf), odom_theta_deg, 0);
            snprintf(lbuf, sizeof(lbuf), "TH:%s", nbuf);   OLED_Str5(rx, 30, lbuf);

            if (line_loss_pose_valid) {
                float ddx = line_loss_x_m - odom_x_m;
                float ddy = line_loss_y_m - odom_y_m;
                FormatSignedFixed(nbuf, sizeof(nbuf), sqrtf(ddx * ddx + ddy * ddy), 2);
                snprintf(lbuf, sizeof(lbuf), "D:%sm", nbuf + 1);
            } else {
                snprintf(lbuf, sizeof(lbuf), "D:----");
            }
            OLED_Str5(rx, 39, lbuf);

            FormatSignedFixed(nbuf, sizeof(nbuf), velocity_est_f, 2);
            snprintf(lbuf, sizeof(lbuf), "V:%s", nbuf);    OLED_Str5(rx, 48, lbuf);
            FormatSignedFixed(nbuf, sizeof(nbuf), (float)gz / 100.0f, 0);
            snprintf(lbuf, sizeof(lbuf), "GZ:%s", nbuf);   OLED_Str5(rx, 57, lbuf);
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

// ═══════════════════════════════════════════════════════════════════════════
//  LOOP DE CONTROL (100 Hz) — etapas de ControlStep10ms
//
//  ControlStep10ms (al final de este bloque) es solo el orquestador: llama a
//  las etapas en orden. Cada etapa es una función Ctrl_* de acá abajo, en el
//  mismo orden en que se ejecutan. Las variables de este bloque son el
//  "contexto del ciclo": se recalculan completas en cada ciclo de 10ms y
//  comunican los resultados de una etapa a las siguientes (eran locales de la
//  función original; mantienen exactamente los mismos nombres y semántica).
// ═══════════════════════════════════════════════════════════════════════════

// dt del control: fijo por diseño (desacopla el PID del jitter del sensor).
static const float dt_ctrl = DT_CTRL_FIXED;
// Vestigio: siempre 0 (ya no se detectan ciclos tardíos). Los `if (!late_cycle)`
// que quedan en el código son por lo tanto incondicionales; el compilador los elimina.
static const uint8_t late_cycle = 0;

// ── Contexto del ciclo (sensado) ──
static int32_t delta_right = 0, delta_left = 0;  // counts de encoder de ESTE ciclo
static int32_t enc_r = 0, enc_l = 0;             // conteos absolutos (foto atómica)
static float   gyro_rate_dps = 0.0f;             // gyro X en °/s (telemetría)
static float   accel_ang_deg = 0.0f;             // roll del accel (filtro + caída)
static float   steer_correction = 0.0f;          // steering PID de lazo cerrado (opcional)

// ── Contexto del ciclo (seguidor de línea) ──
static float   line_error = 0.0f;                // centroide [-1..1], izq = +
static uint8_t line_detected = 0;
static float   w_sum = 0.0f;                     // suma de pesos del centroide
static float   line_angle_cmd = 0.0f;            // ángulo pedido por el PI de velocidad
static float   line_desired_forward_vel = 0.0f;
static float   line_forward_vel = 0.0f;
static float   line_forward_accel_f = 0.0f;       // aceleración filtrada para guarda predictiva
static float   line_forward_vel_prev = 0.0f;
static uint8_t line_speed_predict_valid = 0;
static uint8_t manual_line_override = 0;         // comandos manuales durante LINE sin ver línea
static uint8_t line_pivot_active = 0;            // 1 = el sub-estado ya escribió los motores

// ── Contexto del ciclo (PID de balance → motores/telemetría) ──
static float   error = 0.0f;                     // setpoint - roll (°)
static float   p_term = 0.0f, i_term = 0.0f, d_term = 0.0f;
static float   output = 0.0f;                    // suma PID sin saturar
static float   pwm_cmd = 0.0f;                   // comando previo a saturación
static float   pwm_sat = 0.0f;                   // PWM común final (±50)
static uint8_t sat_flag = 0;
static float   log_p_line = 0.0f, log_i_line = 0.0f, log_d_line = 0.0f;

// ─────────────────────────────────────────────────────────────────────
// Etapa 1 — Lectura del IMU + métrica de manipulación.
// Devuelve 0 si el MPU todavía no tiene dato nuevo (el ciclo no corre).
// ─────────────────────────────────────────────────────────────────────
static uint8_t Ctrl_LeerIMU(void)
{
    if (!MPU6050_IsDataReady()) return 0;
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
    return 1;
}

// ─────────────────────────────────────────────────────────────────────
// Etapa 2 — Velocidad por encoders: counts acumulados por la IRQ de TIM2
// → delta del ciclo → m/s (velocity_est y sus versiones filtradas).
// ─────────────────────────────────────────────────────────────────────
static void Ctrl_VelocidadEncoders(void)
{
    static int32_t enc_right_prev = 0;
    static int32_t enc_left_prev  = 0;
    __disable_irq();
    enc_r = encoder_right;
    enc_l = encoder_left;
    __enable_irq();
    delta_right = enc_r - enc_right_prev;
    delta_left  = enc_l - enc_left_prev;
    enc_right_prev = enc_r;
    enc_left_prev  = enc_l;
    #define ENC_CPR        28  // 4x quadrature: ambos canales A y B, RISING+FALLING
    #define ENC_VEL_SCALE  0.17750f  // 2π × r_rueda (radio 2.825 cm) → m/s
    // De counts a velocidad, por análisis dimensional:
    //   delta [counts/ciclo] ÷ ENC_CPR [counts/vuelta] = vueltas por ciclo
    //   ÷ DT_CTRL_FIXED [s/ciclo]                      = vueltas por segundo (rps)
    //   × ENC_VEL_SCALE [m/vuelta]  (perímetro = 2πr)  = metros por segundo
    // Promedio de ambas ruedas = velocidad del centro del robot. El signo
    // negativo fija la convención global: AVANCE = velocity_est NEGATIVA
    // (los encoders cuentan al revés del sentido "adelante" del chasis).
    float speed_right_rps = (float)delta_right / (ENC_CPR * DT_CTRL_FIXED);
    float speed_left_rps  = (float)delta_left  / (ENC_CPR * DT_CTRL_FIXED);
    float vel_enc = -((speed_right_rps + speed_left_rps) * 0.5f) * ENC_VEL_SCALE;
    vel_enc = clampf_local(vel_enc, -20.0f, 20.0f);
    velocity_est    = vel_enc;
    // Filtro EMA (pasa-bajos de 1er orden): y += β·(x − y). Cada ciclo la
    // salida se acerca una fracción β al valor nuevo → constante de tiempo
    // τ ≈ dt/β. Con β=0.35 → τ≈29ms (rápido, para control); con β=0.02 →
    // τ≈0.5s (lento, para decisiones que no deben reaccionar a un tick suelto).
    velocity_est_f += VEL_LPF_BETA * (velocity_est - velocity_est_f);
    velocity_est_slow_f += VEL_SLOW_BETA * (velocity_est - velocity_est_slow_f);
    speed_right_rps_s = speed_right_rps;
    speed_left_rps_s  = speed_left_rps;
    if (delta_right != 0) wheel_r_last_tick_ms = HAL_GetTick();
    if (delta_left  != 0) wheel_l_last_tick_ms = HAL_GetTick();
}

// ─────────────────────────────────────────────────────────────────────
// Etapa 3 — Latches del wall-following: reversa por counts, detección de
// atorado y timeout de pared perdida. Solo hace algo en los 3 estados de
// pared; fuera de ellos deja todo desarmado.
// ─────────────────────────────────────────────────────────────────────
static void Ctrl_LatchesPared(void)
{
    // ── Latch de reversa de pared por counts (2026-07-10) ──
    // Se dispara con el lateral activo (ADC7/ADC5) < THOLD y queda activo hasta retroceder
    // OBJ_WALL_REV_COUNTS (o timeout). Lo consumen las ramas de setpoint
    // y de motores de los 3 estados de pared — reemplaza al chequeo
    // instantáneo de ADC que cortaba la reversa apenas se despegaba.
    if (line_state == LINE_STATE_OBJ_BORDEAR_PARED ||
        line_state == LINE_STATE_OBJ_PARED_LIBRE  ||
        line_state == LINE_STATE_OBJ_GIRO_PARED) {
        if (!obj_wall_rev_latch &&
            (float)adcAvg[OBJ_WALL_ADC_IDX] < OBJ_WALL_REVERSE_THOLD) {
            obj_wall_rev_latch    = 1;
            obj_wall_rev_goal     = OBJ_WALL_REV_COUNTS;
            obj_wall_rev_latch_r0 = enc_r;
            obj_wall_rev_latch_l0 = enc_l;
            obj_wall_rev_latch_ms = HAL_GetTick();
        }

        // Detección de atorado (2026-07-10): sin reversa activa, si en
        // OBJ_WALL_STUCK_WIN_MS el movimiento acumulado de encoders no llega
        // a STUCK_COUNTS, está trabado contra la pared (empuja sin avanzar o
        // pivot atorado) → reversa LARGA de destrabe (ESCAPE_COUNTS).
        if (!obj_wall_rev_latch) {
            if (obj_wall_stuck_ms == 0) {
                obj_wall_stuck_ms = HAL_GetTick();
                obj_wall_stuck_r0 = enc_r;
                obj_wall_stuck_l0 = enc_l;
            } else if (HAL_GetTick() - obj_wall_stuck_ms >= OBJ_WALL_STUCK_WIN_MS) {
                int32_t stuck_moved = (abs(enc_r - obj_wall_stuck_r0) +
                                       abs(enc_l - obj_wall_stuck_l0)) / 2;
                if (stuck_moved < OBJ_WALL_STUCK_COUNTS) {
                    obj_wall_rev_latch    = 1;
                    obj_wall_rev_goal     = OBJ_WALL_REV_ESCAPE_COUNTS;
                    obj_wall_rev_latch_r0 = enc_r;
                    obj_wall_rev_latch_l0 = enc_l;
                    obj_wall_rev_latch_ms = HAL_GetTick();
                }
                obj_wall_stuck_ms = 0;   // nueva ventana
            }
        } else {
            obj_wall_stuck_ms = 0;       // en reversa: la ventana no corre
        }

        if (obj_wall_rev_latch) {
            int32_t wall_rev_done = (abs(enc_r - obj_wall_rev_latch_r0) +
                                     abs(enc_l - obj_wall_rev_latch_l0)) / 2;
            // Timeout doble para la reversa de escape (recorrido más largo).
            uint32_t rev_tout = (obj_wall_rev_goal > OBJ_WALL_REV_COUNTS)
                              ? (OBJ_WALL_REV_TIMEOUT_MS * 2U)
                              : OBJ_WALL_REV_TIMEOUT_MS;
            if (wall_rev_done >= obj_wall_rev_goal ||
                (HAL_GetTick() - obj_wall_rev_latch_ms) > rev_tout)
                obj_wall_rev_latch = 0;
        }

        // ── Timeout de pared perdida (2026-07-11, 2026-07-20: IDLE → GIVEN_UP) ──
        // Si en toda la secuencia de pared no se vuelve a ver el objeto
        // (lateral activo < OBJ_WALL_THRESHOLD) durante OBJ_WALL_MISSING_TIMEOUT_MS
        // continuos, se abandona la búsqueda. Antes mandaba a ROBOT_STATE_IDLE,
        // que corta el balance de golpe y hace caer al robot (reportado). Ahora
        // pasa a LINE_STATE_GIVEN_UP (mismo "reposo total" que usan los timeouts
        // de LOST_FWD/EDGE_FWD): sigue en ROBOT_STATE_LINE_FOLLOWING, upright con
        // freno de encoders (ver Ctrl_SetpointDinamico), sin intentar más
        // búsqueda — y retoma solo si la línea vuelve a aparecer.
        // Durante la reversa de pared el timer no corre: retroceder aleja
        // la pared de la vista del lateral a propósito (la de escape dura
        // hasta 6s y dispararía el timeout en plena maniobra válida).
        if (obj_wall_rev_latch ||
            (float)adcAvg[OBJ_WALL_ADC_IDX] < OBJ_WALL_THRESHOLD) {
            obj_wall_missing_since_ms = 0;
        } else {
            if (obj_wall_missing_since_ms == 0) {
                obj_wall_missing_since_ms = HAL_GetTick();
            } else if (HAL_GetTick() - obj_wall_missing_since_ms >= OBJ_WALL_MISSING_TIMEOUT_MS) {
                line_state                 = LINE_STATE_GIVEN_UP;
                steering_adjustment        = 0.0f;
                obj_wall_missing_since_ms  = 0;
                obj_wall_rev_latch         = 0;
                obj_wall_stuck_ms          = 0;
            }
        }
    } else {
        obj_wall_rev_latch        = 0;
        obj_wall_stuck_ms         = 0;
        obj_wall_missing_since_ms = 0;
    }
}

// ─────────────────────────────────────────────────────────────────────
// Etapa 4 — Odometría de pose (x, y, θ) por dead-reckoning:
// distancia por encoders + rumbo por gyro Z. No integra durante caídas.
// ─────────────────────────────────────────────────────────────────────
static void Ctrl_Odometria(void)
{
    // ── ODOMETRÍA DE POSE ────────────────────────────────────────────
    // Distancia por encoders + rumbo por gyro Z, integrados cada ciclo.
    // Con la convención de signos actual, deltas positivos = avance
    // (velocity_est_f negativo = adelante, ver vel_enc arriba).
    // Durante caídas no se integra: las ruedas pueden girar sin traccionar.
    if (!f_fallen) {
        // Bias residual de gz aprendido con el robot quieto (ver defines ODOM_GZ_*):
        // encoders sin ticks + gz chico durante 500ms → EMA lenta del residuo.
        if (delta_right == 0 && delta_left == 0 &&
            fabsf((float)gz - odom_gz_bias_f) < ODOM_GZ_STILL_LSB) {
            if (odom_still_cnt < 65535) odom_still_cnt++;
            if (odom_still_cnt >= ODOM_STILL_MIN_CYCLES) {
                odom_gz_bias_f += ODOM_GZ_BIAS_ALPHA * ((float)gz - odom_gz_bias_f);
                odom_gz_bias_f = clampf_local(odom_gz_bias_f,
                                              -ODOM_GZ_BIAS_MAX_LSB, ODOM_GZ_BIAS_MAX_LSB);
            }
        } else {
            odom_still_cnt = 0;
        }
        float gz_odom_dps = ((float)gz - odom_gz_bias_f) / 100.0f;

        // Distancia recorrida este ciclo [m]: promedio de counts de ambas
        // ruedas (= avance del centro) ÷ counts/vuelta × metros/vuelta (2πr).
        // Misma conversión que la velocidad, pero sin dividir por dt.
        float odom_d = ((float)(delta_right + delta_left) * 0.5f / ENC_CPR) * ENC_VEL_SCALE;

        // Gating de pivot: rotando en el lugar, el avance neto de ±1 count por
        // ciclo es ruido de cuantización/timing, no traslación real — integrarlo
        // con el rumbo cambiando hace "caminar" la posición (ver defines arriba).
        if (fabsf(gz_odom_dps) > ODOM_PIVOT_GZ_DPS &&
            abs(delta_right + delta_left) <= ODOM_PIVOT_MAX_COUNTS) {
            odom_d = 0.0f;
        }

        // Integración de 2º orden: la traslación usa el rumbo de MITAD de paso
        // (θ + Δθ/2), no el rumbo final — menos sesgo sistemático en arcos.
        // La pose se acumula por dead-reckoning clásico:
        //   Δθ = ω·dt   (gyro Z [°/s] × 10ms)
        //   x += d·cos(θ_medio),  y += d·sin(θ_medio)
        // es decir, el paso 'd' se descompone en sus componentes según hacia
        // dónde apuntaba el robot. Usar el rumbo del MEDIO del paso equivale
        // a la regla del trapecio: en una curva, ni el rumbo viejo ni el
        // nuevo son representativos del arco — el promedio sí.
        float odom_dth = ODOM_THETA_SIGN * gz_odom_dps * DT_CTRL_FIXED;
        float odom_th_mid_rad = (odom_theta_deg + 0.5f * odom_dth) * (M_PI / 180.0f);
        odom_theta_deg += odom_dth;
        if      (odom_theta_deg >  180.0f) odom_theta_deg -= 360.0f;
        else if (odom_theta_deg < -180.0f) odom_theta_deg += 360.0f;
        odom_x_m += odom_d * cosf(odom_th_mid_rad);
        odom_y_m += odom_d * sinf(odom_th_mid_rad);
    }
}

// ─────────────────────────────────────────────────────────────────────
// Etapa 5 — Steering PID de lazo cerrado por encoders (opcional, según
// steer_pid_enabled). steer_correction se usa en los motores de BALANCE.
// ─────────────────────────────────────────────────────────────────────
static void Ctrl_SteeringLazoCerrado(void)
{
    if (steer_pid_enabled) {
        steer_correction = ComputeSteeringPID(speed_right_rps_s, speed_left_rps_s, 0.0f);
    } else {
        steer_correction = 0.0f;
        SteeringPID_Reset();
    }
}

// ─────────────────────────────────────────────────────────────────────
// Etapa 6 — Timing: dt real entre muestras del MPU (solo telemetría; el
// control usa dt fijo). Devuelve 0 en el primer ciclo (sin dt válido).
// ─────────────────────────────────────────────────────────────────────
static uint8_t Ctrl_TimingDt(void)
{
    static uint32_t last_mpu_us = 0;

    uint32_t sample_us = mpu_irq_timestamp_us;

    if (last_mpu_us == 0) {
        last_mpu_us = sample_us;
        dt_real = DT_CTRL_FIXED;
        return 0;   // primer ciclo: todavía no hay dt válido
    }

    uint32_t diff_us = sample_us - last_mpu_us;
    last_mpu_us = sample_us;

    dt_real = (float)diff_us * 1e-6f;
    // micros() envuelve cada ~44.7 s (CYCCNT/96 no es potencia de 2): en el ciclo
    // del wrap diff_us da basura enorme. Solo afecta telemetría (dt_ctrl es fijo).
    if (dt_real > 1.0f) dt_real = DT_CTRL_FIXED;
    return 1;
}

// ─────────────────────────────────────────────────────────────────────
// Etapa 7 — Filtro de actitud: gyro crudo → °/s, ángulo por accel,
// filtro complementario (filtered_roll_deg) y tilt lateral para Qt.
// ─────────────────────────────────────────────────────────────────────
static void Ctrl_FiltroIMU(void)
{
    const float ANG_SIGN = +1.0f;

    // Gyro crudo (int16 LSB) → velocidad angular en °/s: se divide por la
    // sensibilidad [LSB por °/s]. El valor teórico del datasheet para el
    // fondo de escala configurado sería 131 (±250 dps), pero 100 es el que
    // quedó calibrado empíricamente contra giros reales medidos en el robot
    // (ver Registro 2026-07-05/06 en CLAUDE.md — NO volver a 131).
    gyro_rate_dps = ANG_SIGN * ((float)gx / 100.0f);
    if (gyro_rate_dps >  250.0f) gyro_rate_dps =  250.0f;
    if (gyro_rate_dps < -250.0f) gyro_rate_dps = -250.0f;

    gyro_f = gyro_rate_dps;
    if (gyro_f >  180.0f) gyro_f =  180.0f;
    if (gyro_f < -180.0f) gyro_f = -180.0f;

    // Ángulo de roll desde el acelerómetro, por trigonometría de la gravedad:
    // con el robot quieto, el accel solo mide g. Parado vertical, g cae toda
    // en el eje Z (ay≈0, az≈+g) → ángulo 0°. Al inclinarse, parte de g se
    // proyecta en Y: tan(roll) = ay/az → roll = atan2(ay, az). Se usa atan2
    // (y no atan del cociente) porque maneja az=0 y los 4 cuadrantes sin
    // dividir por cero. × 180/π pasa de radianes a grados.
    // Vale solo en cuasi-estático: acelerando, el accel mide g + movimiento
    // y este ángulo miente — por eso el filtro complementario le cree poco.
    accel_ang_deg = ANG_SIGN * (atan2f((float)ay, (float)az) * (180.0f / M_PI));

    // Inicialización del filtro en la primera muestra — evita que arranque en 0°
    static uint8_t filter_init = 0;
    if (!filter_init) {
        filtered_roll_deg = accel_ang_deg;
        filter_init = 1;
    }

    // Filtro complementario (fusión gyro + accel):
    //   ángulo = 0.98 · (ángulo_anterior + gyro·dt) + 0.02 · ángulo_accel
    // El primer término integra el gyro (ángulo_anterior + velocidad×tiempo):
    // preciso a corto plazo pero acumula deriva. El segundo mezcla un 2% del
    // ángulo absoluto del accel: ruidoso pero sin deriva. El 98/2 equivale a
    // pasa-altos sobre el gyro + pasa-bajos sobre el accel con constante de
    // tiempo τ = α·dt/(1−α) ≈ 0.49s: la deriva del gyro se corrige en ~½s y
    // el ruido/aceleraciones del accel entra atenuado ~50 veces.
    filtered_roll_deg = ALPHA * (filtered_roll_deg + gyro_f * dt_ctrl)
                      + (1.0f - ALPHA) * accel_ang_deg;

    if (filtered_roll_deg >  180.0f) filtered_roll_deg =  180.0f;
    if (filtered_roll_deg < -180.0f) filtered_roll_deg = -180.0f;

    // Inclinación LATERAL (banking alrededor del eje de avance — el tercer
    // eje, 2026-07-10): accel-only con EMA suave. No participa de ningún
    // control, solo viaja en el push de odometría para que la Vista 3D de
    // Qt reaccione también a este eje. Misma fórmula que calculate_tilt().
    {
        // Misma idea que el roll del accel, pero para el eje X: el cateto
        // opuesto es la proyección de g en X y el adyacente es lo que queda
        // de g en el plano YZ (pitágoras: √(ay²+az²)) — así el ángulo lateral
        // no se contamina cuando el robot además está inclinado en roll.
        float lat_acc = atan2f(-(float)ax,
                               sqrtf((float)ay * (float)ay +
                                     (float)az * (float)az)) * (180.0f / M_PI);
        lat_tilt_f += 0.10f * (lat_acc - lat_tilt_f);
    }
}

// ─────────────────────────────────────────────────────────────────────
// Etapa 8 — Cuarentena de sensores de línea clavados en el tope
// (defensa contra soldadura fría/emisor caído, ver LINE_STUCK_*).
// ─────────────────────────────────────────────────────────────────────
static void Ctrl_CuarentenaLinea(void)
{
    for (uint8_t ch = 0; ch < 4; ch++) {
        if (adcAvg[ch] >= (uint16_t)LINE_STUCK_ADC) {
            if (line_ch_stuck_t0[ch] == 0) {
                line_ch_stuck_t0[ch] = HAL_GetTick();
            } else if (!line_ch_quarantine[ch] &&
                       (HAL_GetTick() - line_ch_stuck_t0[ch]) >= LINE_STUCK_MS) {
                line_ch_quarantine[ch] = 1;
            }
        } else {
            line_ch_stuck_t0[ch] = 0;
            if (adcAvg[ch] < (uint16_t)LINE_THRESHOLD)
                line_ch_quarantine[ch] = 0;   // leyó blanco: el sensor volvió
        }
    }
}

// ─────────────────────────────────────────────────────────────────────
// Etapa 9 — Entradas del seguidor de línea (solo en modo LINE_FOLLOWING):
// detección de obstáculo, centroide de línea, "todo negro" (cruce
// perpendicular vs. en el aire) y PI de velocidad → line_angle_cmd.
// ─────────────────────────────────────────────────────────────────────
static void Ctrl_EntradasLinea(void)
{
    line_error = 0.0f;
    line_angle_cmd = LINE_ANGLE;
    line_desired_forward_vel = 0.0f;
    line_forward_vel = 0.0f;
    line_detected = 0;
    w_sum = 0.0f;
    if (robot_state == ROBOT_STATE_LINE_FOLLOWING) {
        // Detección de objetos: sensores de largo alcance ADC 5-8 (índices 4-7).
        // Si cualquier sensor baja del umbral durante OBJ_DETECT_DEBOUNCE_CNT ciclos
        // consecutivos (10 ciclos = 100 ms), arranca la secuencia de evasión.
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
                // 2026-07-14: directo a la fase STOP, sin pasar por ESPERA_REVERSA.
                // El balance libre de ESPERA no tiene ancla de posición: llegando
                // con inercia el robot se iba un poco para atrás y perdía la pared
                // (reportado). El hold de distancia del STOP frena la inercia
                // (hold_damp) y sostiene/lleva la distancia a la banda 3500..3900
                // de A6/A8 desde el primer ciclo — no queda ningún período libre.
                line_state           = LINE_STATE_OBJ_FRENO_REVERSA;
                obj_brake_start_ms   = 0;
                obj_pre_rotate_ms    = 0;
                obj_rev_band_enter_ms = 0;
                obj_cnt              = 0;
                // Sentido alternado (2026-07-13): esta esquiva usa el sentido
                // programado y deja armado el opuesto para el próximo objeto.
                obj_esquive_dir      = obj_esquive_next_dir;
                obj_esquive_next_dir = (int8_t)-obj_esquive_next_dir;
                steering_adjustment = 0.0f;
                line_integral       = 0.0f;
                line_error_prev     = 0.0f;
                line_error_f_d      = 0.0f;
                obj_rev_steer_f     = 0.0f;
                obj_rot_initialized = 0;
                obj_rot_phase       = 0;
                obj_rot_heading     = 0.0f;
                obj_rot_phase1_ms   = 0;
                obj_wall_line_confirm_cnt = 0;
            }
        }

        // adcAvg[] es la mediana de los últimos 32 barridos DMA completos
        // (~8ms a 4kHz): rechaza picos EMI aislados sin agregar el retardo de
        // la antigua ventana de 40ms.
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
            w[i] = (!line_ch_quarantine[i] && s[i] > LINE_THRESHOLD)
                 ? fmaxf(signal, 0.0f) : 0.0f;
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
            // Tras dejar de mover el acelerómetro (lo soltaron/apoyaron), el timer de
            // all_black puede venir arrastrando segundos acumulados desde que estaba
            // en el aire — sin esto, apoyarlo sobre cualquier zona momentáneamente
            // oscura (o el punto de partida) disparaba PERP_ROTATE al instante. Exige
            // además un mínimo de quietud continua del acelerómetro antes de habilitar
            // el disparo de cruce perpendicular.
            const uint32_t ACCEL_SETTLE_MS = 400U;
            // Guardas anti "giro fantasma al apoyar el robot" (2026-07-05): un cruce
            // perpendicular REAL ocurre mientras se sigue la línea — los 4 sensores
            // pasan de "línea parcial" (línea + blanco a los costados) a todo negro
            // en un instante, y el todo-negro dura poco. Viniendo del aire, en cambio,
            // los 4 sensores llevan MUCHO tiempo en negro y nunca hubo vista parcial
            // (en el aire all_black implica line_detected, así que el timer de línea
            // no discrimina — esta variable sí).
            const uint32_t PERP_MAX_ELAPSED_MS  = 800U;  // todo-negro más viejo que esto NO es un cruce
            const uint32_t PERP_PARTIAL_WIN_MS  = 400U;  // el todo-negro debe empezar a menos de esto de la última vista parcial
            // Canal en cuarentena → all_black imposible (conservador: mejor
            // perderse un cruce perpendicular real que girar por un sensor roto).
            uint8_t all_black = (adcAvg[0] > (uint16_t)LINE_THRESHOLD && !line_ch_quarantine[0] &&
                                 adcAvg[1] > (uint16_t)LINE_THRESHOLD && !line_ch_quarantine[1] &&
                                 adcAvg[2] > (uint16_t)LINE_THRESHOLD && !line_ch_quarantine[2] &&
                                 adcAvg[3] > (uint16_t)LINE_THRESHOLD && !line_ch_quarantine[3]);
            if (line_detected && !all_black)
                last_partial_line_ms = HAL_GetTick();
            uint8_t accel_moving = (accel_motion_f > ACCEL_MOTION_THRESHOLD);
            if (accel_moving) {
                accel_settled_start_ms = 0;
            } else if (accel_settled_start_ms == 0) {
                accel_settled_start_ms = HAL_GetTick();
            }
            uint8_t accel_settled = (accel_settled_start_ms != 0) &&
                ((HAL_GetTick() - accel_settled_start_ms) > ACCEL_SETTLE_MS);
            if (all_black) {
                if (all_black_start_ms == 0) all_black_start_ms = HAL_GetTick();
                uint32_t all_black_elapsed = HAL_GetTick() - all_black_start_ms;
                if (accel_moving) {
                    if (all_black_elapsed > 2000U) f_in_air = 1;
                } else {
                    f_in_air = 0;
                    if (line_state == LINE_STATE_FOLLOWING &&
                        accel_settled &&
                        all_black_elapsed > PERP_DEBOUNCE_MS &&
                        all_black_elapsed < PERP_MAX_ELAPSED_MS &&
                        last_partial_line_ms != 0 &&
                        (all_black_start_ms - last_partial_line_ms) < PERP_PARTIAL_WIN_MS) {
                        uint32_t now_ms = HAL_GetTick();
                        perp_from_obj_avoidance =
                            (obj_final_turn_pending_ms != 0) &&
                            ((now_ms - obj_final_turn_pending_ms) <= OBJ_FINAL_TURN_WINDOW_MS);
                        obj_final_turn_pending_ms = 0;
                        perp_obj_brake_start_ms = 0;
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
            // Centroide = promedio de las posiciones de los sensores pesado
            // por cuánta línea ve cada uno: Σ(coef_i·w_i) / Σw_i. Los sensores
            // están en posiciones {-3,-1,+1,+3} (en unidades de 5.75mm); se usa
            // pos²·signo(pos) = {+9,+1,-1,-9} en vez de la posición lineal para
            // que los extremos pesen 9× (reacción fuerte apenas la línea se
            // escapa hacia una punta). Dividir por 9·w_sum normaliza: el peor
            // caso (toda la señal en un sensor extremo) da num = ±9·w_sum →
            // error = ±1. Resultado en [-1,+1], izquierda = positivo.
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
        line_error_disp    = line_error;
        line_detected_disp = (uint8_t)line_detected;

        // Velocidad deseada cae cuadráticamente con el error de línea.
        // La recta 1 − |e|/0.45 vale 1 centrado y llega a 0 con |e|=0.45
        // (línea ya bien afuera del centro); elevarla al cuadrado hace la
        // caída suave cerca del centro (0.1 de error solo quita ~40%) y
        // agresiva en curvas (0.3 de error quita ~90%): recto rápido, curva lento.
        // Floor 10%: en curva cerrada el robot frena casi al mínimo (era 25%).
        // Sin floor: speed_factor→0 → line_angle_cmd=0 → pwm_sat→0 → spin puro.
        float speed_factor = fmaxf(0.0f, 1.0f - fabsf(line_error) / 0.45f);
        speed_factor *= speed_factor;
        line_desired_forward_vel = line_detected
            ? fmaxf(LINE_SPEED_TARGET * 0.20f, LINE_SPEED_TARGET * speed_factor)
            : 0.0f;
        // Para el PI de línea el deadband debe ser de tipo "cero o valor real".
        // apply_deadbandf() resta 0.35 a toda lectura válida: a 2.50m/s hacía
        // creer al control que iba a 2.15m/s y seguía acelerando sobre el tope.
        float line_forward_vel_raw = fmaxf(0.0f, -velocity_est_f);
        line_forward_vel = (line_forward_vel_raw > BRAKE_VEL_DEADBAND)
                         ? line_forward_vel_raw : 0.0f;

        // Estimación de aceleración para anticipar el cruce del límite. Solo
        // se conserva mientras sigue una línea real; así una readquisición no
        // genera una derivada falsa por el salto desde cero.
        if (line_state == LINE_STATE_FOLLOWING && line_detected) {
            if (!line_speed_predict_valid) {
                line_forward_vel_prev   = line_forward_vel_raw;
                line_forward_accel_f    = 0.0f;
                line_speed_predict_valid = 1;
            } else {
                float accel_raw = (line_forward_vel_raw - line_forward_vel_prev) /
                                  DT_CTRL_FIXED;
                accel_raw = clampf_local(accel_raw,
                                         -LINE_ACCEL_EST_MAX, LINE_ACCEL_EST_MAX);
                line_forward_accel_f += LINE_ACCEL_FILTER_ALPHA *
                                        (accel_raw - line_forward_accel_f);
                line_forward_vel_prev = line_forward_vel_raw;
            }
        } else {
            line_forward_accel_f     = 0.0f;
            line_forward_vel_prev    = line_forward_vel_raw;
            line_speed_predict_valid = 0;
        }

        // PI de velocidad → inclinación de avance. Convierte un error de
        // velocidad [m/s] en un ángulo de setpoint [°]: ángulo = Kp·e + Ki·∫e.
        // Es el lazo EXTERNO de un control en cascada: este PI pide grados,
        // y el PID de balance (lazo interno) convierte esos grados en PWM.
        // Físicamente: más inclinación = más aceleración de avance (péndulo
        // invertido), así que "ir más rápido" se pide inclinándose más.
        if (line_state == LINE_STATE_FOLLOWING) {
            float vel_error = line_desired_forward_vel - line_forward_vel;
            float line_vel_kp_eff;
            float line_vel_i_term;
            if (vel_error > 0.0f) {
                // Acelerando: acumula integral solo en positivo
                line_vel_integral += vel_error * DT_CTRL_FIXED;
                line_vel_integral = clampf_local(line_vel_integral, 0.0f, LINE_VEL_I_MAX);
                line_vel_kp_eff = LINE_VEL_KP;
                line_vel_i_term = LINE_VEL_KI * line_vel_integral;
            } else {
                // Sobrevelocidad: la integral positiva servía para acelerar y no
                // debe oponerse al freno. Se descarga rápido y se excluye de la
                // salida hasta volver a quedar por debajo de la velocidad pedida.
                line_vel_integral *= 0.50f;
                line_vel_kp_eff = LINE_VEL_KP_BRAKE;
                line_vel_i_term = 0.0f;
            }
            line_angle_cmd = clampf_local(
                line_vel_kp_eff * vel_error + line_vel_i_term,
                -LINE_BRAKE_ANGLE_MAX, LINE_ANGLE
            );

            // Guarda independiente del PI: si la velocidad proyectada a 150ms
            // cruza la franja previa al tope global, fuerza un ángulo de freno.
            // Solo toma aceleración positiva; al desacelerar no interfiere.
            float predicted_forward_vel = line_forward_vel_raw +
                fmaxf(line_forward_accel_f, 0.0f) * LINE_SPEED_PREDICT_SEC;
            float guard_speed = fmaxf(0.0f,
                LINE_SPEED_TARGET - LINE_SPEED_GUARD_MARGIN);
            if (predicted_forward_vel > guard_speed) {
                float guard_brake = -clampf_local(
                    LINE_SPEED_GUARD_KP * (predicted_forward_vel - guard_speed),
                    0.0f, LINE_BRAKE_ANGLE_MAX);
                if (guard_brake < line_angle_cmd)
                    line_angle_cmd = guard_brake;
                // No conservar empuje integral mientras la guarda está frenando.
                line_vel_integral *= 0.50f;
            }
        } else {
            line_vel_integral *= 0.80f;
            line_angle_cmd = 0.0f;
        }

    } else if ((robot_state == ROBOT_STATE_BALANCE_AND_SPEED) ||
               (robot_state == ROBOT_STATE_BALANCE_ONLY)) {
        // velocity_est ya actualizado desde encoders al inicio del ciclo
    }
}

// ─────────────────────────────────────────────────────────────────────
// Etapa 10 — Override manual durante el modo línea.
// Comandos manuales durante LINE_FOLLOWING (WiFi/USB), SOLO si no ve la
// línea: reutiliza el mismo control que ROBOT_STATE_MANUAL_CONTROL (PI de
// velocidad + giro suave + corrección de rumbo recto) extendiendo las
// condiciones de esas dos ramas (setpoint y mezcla de motores) en vez de
// duplicar el algoritmo. En cuanto vuelve a ver la línea, se ignoran los
// comandos y el seguidor retoma el control normal (line_state ya queda
// forzado en LINE_STATE_FOLLOWING mientras el override está activo, así
// que retoma limpio). Se exige además que el comando haya llegado hace
// poco (<250ms): mientras se sigue la línea normalmente el decay de
// manual_setpoint_cmd/steering_cmd NO corre (vive en la rama MANUAL, que
// acá no se ejecuta) — sin este chequeo, un comando viejo (ya soltado)
// quedaría "congelado" y dispararía el override de golpe apenas se
// pierda la línea más tarde, sin que el usuario esté mandando nada.
// ─────────────────────────────────────────────────────────────────────
static void Ctrl_OverrideManualLinea(void)
{
    manual_line_override =
        (robot_state == ROBOT_STATE_LINE_FOLLOWING) && !line_detected &&
        (HAL_GetTick() - manual_cmd_last_ms) < 250U &&
        (fabsf(manual_setpoint_cmd) > 0.01f || fabsf(manual_steering_cmd) > 0.5f);
}

// ─────────────────────────────────────────────────────────────────────
// Etapa 11 — Cambios de estado del robot: al entrar/salir de cada modo
// se resetean integrales, filtros, odometría y display.
// ─────────────────────────────────────────────────────────────────────
static void Ctrl_CambiosDeEstado(void)
{
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
        velocity_est_slow_f = 0.0f;
        vel_from_accel      = 0.0f;
        line_vel_integral   = 0.0f;
        line_enc_angle_corr = 0.0f;
        line_reverse_boost  = 0.0f;
        balance_hold_active = 0;
        line_steer_fb_int   = 0.0f;
        speed_right_rps_s   = 0.0f;
        speed_left_rps_s    = 0.0f;
        line_state          = LINE_STATE_FOLLOWING;
        line_seen_since_entry = 0;
        obj_esquive_dir     = 1;   // esquive alternado: arranca por derecha
        obj_esquive_next_dir = 1;
        line_was_centered_on_lost = 1;
        last_detected_edge_only   = 0;
        dynamic_setpoint    = SETPOINT_ANGLE + setpoint_trim;
        dynamic_setpoint_f  = SETPOINT_ANGLE + setpoint_trim;
        base_setpoint_f     = SETPOINT_ANGLE + setpoint_trim;
        brake_setpoint_f    = 0.0f;
        line_lost_ms        = HAL_GetTick();
        line_angle_ramped   = 0.0f;
        lost_fwd_vel_integral = 0.0f;
        all_black_start_ms  = 0;
        last_partial_line_ms = 0;
        obj_final_turn_pending_ms = 0;
        perp_from_obj_avoidance   = 0;
        perp_obj_brake_start_ms   = 0;
        obj_wall_line_confirm_cnt = 0;
        odom_x_m            = 0.0f;   // odometría: origen = punto de entrada al modo
        odom_y_m            = 0.0f;
        odom_theta_deg      = 0.0f;
        line_loss_pose_valid = 0;     // snapshot viejo queda en el marco anterior
        lost_ret_ov_active   = 0;
    }

    if (robot_state != ROBOT_STATE_LINE_FOLLOWING && prev_robot_state == ROBOT_STATE_LINE_FOLLOWING) {
        f_change_display = display_before_line;  // restaurar display anterior
    }

    // Sub-estado línea: entrar/salir de modo evasión cambia display 1 ↔ 6
    {
        static eLineState prev_line_state_disp = LINE_STATE_FOLLOWING;
        if (robot_state == ROBOT_STATE_LINE_FOLLOWING) {
            int prev_obj = (prev_line_state_disp >= LINE_STATE_OBJ_FRENO_REVERSA);
            int curr_obj = (line_state           >= LINE_STATE_OBJ_FRENO_REVERSA);
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
        velocity_est_slow_f = 0.0f;
        vel_from_accel      = 0.0f;
        dynamic_setpoint    = SETPOINT_ANGLE + setpoint_trim;
        dynamic_setpoint_f  = SETPOINT_ANGLE + setpoint_trim;
        base_setpoint_f     = SETPOINT_ANGLE + setpoint_trim;
        brake_setpoint_f    = 0.0f;
        pwm_sat_prev        = 0.0f;

        odom_x_m       = 0.0f;   // odometría: origen = punto de entrada al modo
        odom_y_m       = 0.0f;
        odom_theta_deg = 0.0f;

        if (robot_state == ROBOT_STATE_MANUAL_CONTROL) {
            manual_setpoint_cmd     = 0.0f;
            manual_steering_cmd     = 0.0f;
            manual_cmd_last_ms      = HAL_GetTick();
            manual_setpoint_ramped  = 0.0f;
            manual_vel_integral     = 0.0f;
            manual_straight_steer_f = 0.0f;
            steering_adjustment     = 0.0f;
            obj_rev_steer_f         = 0.0f;
        }
    }

    prev_robot_state = robot_state;
}

// ─────────────────────────────────────────────────────────────────────
// Etapa 12 — Setpoint dinámico: cada modo/sub-estado decide cuántos
// grados inclinarse (base) más el freno traslacional (brake); el
// resultado rampa hacia dynamic_setpoint_f, la referencia del PID.
// Acá vive la "personalidad" del robot.
// ─────────────────────────────────────────────────────────────────────
static void Ctrl_SetpointDinamico(void)
{
    float base_setpoint_target  = SETPOINT_ANGLE + setpoint_trim;
    float brake_setpoint_target = 0.0f;

    if (robot_state == ROBOT_STATE_LINE_FOLLOWING && !manual_line_override) {
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
        } else if (line_state == LINE_STATE_FOLLOWING) {
            // La pérdida se confirma recién después de 150ms para no reaccionar
            // a parpadeos, pero el robot no puede seguir libre durante esa ventana.
            // Desde el PRIMER ciclo sin línea frena con la velocidad rápida; si la
            // lectura vuelve, el estado nunca cambia y el PI retoma normalmente.
            base_setpoint_target  = SETPOINT_ANGLE + setpoint_trim;
            brake_setpoint_target = ComputeBrakeFromVelocity(
                velocity_est_f, BRAKE_TILT_MAX);
            line_enc_angle_corr   = 0.0f;
            line_reverse_boost    = 0.0f;
        } else if (line_state == LINE_STATE_OBJ_FRENO_REVERSA) {
            // Frenado post-reversa + hold de distancia por ADC6/ADC8: la reversa
            // cortó en OBJ_REV_CLEAR_ADC; acá se sostiene esa distancia mientras
            // dura el STOP. Antes era bang-bang puro con ±2.75° fijos: se pasaba
            // hacia atrás/adelante y nunca lograba quedarse estable. Ahora la
            // corrección es proporcional y suave, con una zona fina muerta en el
            // centro para que pueda asentarse y cumplir la ventana de estabilidad.
            float front_min = fminf((float)adcAvg[5], (float)adcAvg[7]);
            float hold_center = OBJ_REV_CLEAR_ADC + 0.5f * OBJ_REV_HOLD_BAND;
            float hold_err    = hold_center - front_min;  // >0 = quedó cerca → retroceder
            float hold_abs    = fabsf(hold_err);

            // Término D del hold (2026-07-10): inclinación en contra del
            // movimiento con la velocidad RÁPIDA de encoders. Es lo que
            // frena la inercia ANTES de pasarse (en ambos sentidos: yendo
            // atrás tras el corte de la reversa, y volviendo hacia adelante
            // no se estrella contra la pared). Ver OBJ_REV_HOLD_KV.
            // velocity_est_f > 0 = reversa → damp positivo = inclina adelante.
            float hold_damp = clampf_local(
                OBJ_REV_HOLD_KV * apply_deadbandf(velocity_est_f, OBJ_REV_HOLD_VEL_DB),
                -OBJ_REV_HOLD_DAMP_MAX, OBJ_REV_HOLD_DAMP_MAX);

            if (hold_abs > OBJ_REV_HOLD_FINE_BAND) {
                float hold_mag = clampf_local(
                    (hold_abs - OBJ_REV_HOLD_FINE_BAND) / OBJ_REV_HOLD_ERR_FOR_MAX,
                    0.0f, 1.0f);
                float hold_angle = OBJ_REV_HOLD_ANGLE_MIN +
                    (OBJ_REV_HOLD_ANGLE_MAX - OBJ_REV_HOLD_ANGLE_MIN) * hold_mag;
                base_setpoint_target = SETPOINT_ANGLE + setpoint_trim +
                    ((hold_err > 0.0f) ? -hold_angle : hold_angle) + hold_damp;
                // Anti-stall (ver AntiStall_Tick): el STOP es la entrada al
                // detectar objeto y este hold cubre TODA la reversa hasta la
                // banda. Si la fricción puede con ANGLE_MAX, sin esto quedaría
                // clavado cerca del objeto hasta el timeout de 10s y giraría
                // demasiado cerca. Empuja en el sentido de la corrección.
                base_setpoint_target += ((hold_err > 0.0f) ? -1.0f : 1.0f) *
                    AntiStall_Tick(9, 1, ANTISTALL_MAX_REV);
                brake_setpoint_target = 0.0f;
            } else {
                // Dentro de la zona fina: solo el damping (el freno general
                // de ComputeBrakeSetpointTarget ahora usa la velocidad LENTA
                // —anti-bamboleo en parado— y acá llegaba ~0.5s tarde, justo
                // cuando la inercia post-reversa es máxima).
                base_setpoint_target  = SETPOINT_ANGLE + setpoint_trim + hold_damp;
                brake_setpoint_target = 0.0f;
            }
            line_enc_angle_corr   = 0.0f;
            line_reverse_boost    = 0.0f;
        } else if (line_state == LINE_STATE_LOST_FWD) {
            // Avance post-180°: PI de velocidad (ver LOST_FWD_* arriba), con deadband
            // (BRAKE_VEL_DEADBAND) y anti-windup igual que el PI de línea normal.
            {
                float lfwd_vel_raw = fmaxf(0.0f, -velocity_est_f);
                float lfwd_vel     = apply_deadbandf(lfwd_vel_raw, BRAKE_VEL_DEADBAND);
                float vel_error    = LOST_FWD_SPEED_TARGET - lfwd_vel;
                float kp           = (vel_error < 0.0f) ? LOST_FWD_KP_BRAKE : LOST_FWD_KP;
                if (!late_cycle) {
                    if (vel_error > 0.0f) {
                        lost_fwd_vel_integral += vel_error * dt_ctrl;
                        lost_fwd_vel_integral = clampf_local(
                            lost_fwd_vel_integral, 0.0f, LOST_FWD_VEL_I_MAX);
                    } else {
                        lost_fwd_vel_integral *= 0.80f;
                    }
                }
                base_setpoint_target = clampf_local(
                    kp * vel_error + LOST_FWD_KI * lost_fwd_vel_integral,
                    -LOST_FWD_BRAKE_MAX, LOST_FWD_ANGLE_MAX);

                // Anti-stall (ver AntiStall_Tick): nunca quedarse quieto.
                base_setpoint_target += AntiStall_Tick(1,
                    base_setpoint_target > 0.0f, ANTISTALL_MAX_FWD);
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
                ComputeBrakeFromVelocity(velocity_est_f, BRAKE_TILT_MAX) * LOST_BRAKE_BOOST,
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
                ComputeBrakeFromVelocity(velocity_est_f, BRAKE_TILT_MAX) * LOST_BRAKE_BOOST,
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
            // Avance post-giro: mismo control PI que LOST_FWD, mismo deadband,
            // mismo integral compartido (los dos estados son mutuamente excluyentes).
            {
                float efwd_vel_raw = fmaxf(0.0f, -velocity_est_f);
                float efwd_vel     = apply_deadbandf(efwd_vel_raw, BRAKE_VEL_DEADBAND);
                float vel_error    = LOST_FWD_SPEED_TARGET - efwd_vel;
                float kp           = (vel_error < 0.0f) ? LOST_FWD_KP_BRAKE : LOST_FWD_KP;
                if (!late_cycle) {
                    if (vel_error > 0.0f) {
                        lost_fwd_vel_integral += vel_error * dt_ctrl;
                        lost_fwd_vel_integral = clampf_local(
                            lost_fwd_vel_integral, 0.0f, LOST_FWD_VEL_I_MAX);
                    } else {
                        lost_fwd_vel_integral *= 0.80f;
                    }
                }
                base_setpoint_target = clampf_local(
                    kp * vel_error + LOST_FWD_KI * lost_fwd_vel_integral,
                    -LOST_FWD_BRAKE_MAX, LOST_FWD_ANGLE_MAX);
                // Anti-stall (ver AntiStall_Tick): nunca quedarse quieto.
                base_setpoint_target += AntiStall_Tick(2,
                    base_setpoint_target > 0.0f, ANTISTALL_MAX_FWD);
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
        } else if (line_state == LINE_STATE_OBJ_GIRO_ESQUIVE ||
                   line_state == LINE_STATE_LOST_ROTATE ||
                   line_state == LINE_STATE_EDGE_ROTATE ||
                   line_state == LINE_STATE_PERP_ROTATE) {
            // Rotación: upright sin avance, sin freno de encoders
            base_setpoint_target  = SETPOINT_ANGLE + setpoint_trim;
            brake_setpoint_target = 0.0f;
            line_enc_angle_corr   = 0.0f;
            line_reverse_boost    = 0.0f;
        } else if (line_state == LINE_STATE_OBJ_PAUSA_GIRO) {
            // Hold post-rotación: upright con freno traslacional, yaw-lock en switch.
            // 2026-07-14: + hold de POSICIÓN por encoders (ver OBJ_PAUSA_POS_*):
            // counts netos positivos = avance (ver signo de vel_enc) → inclina
            // hacia atrás para volver al punto donde terminó el giro, y viceversa.
            // Gate con obj_hold_start_ms != 0: el ancla se estampa en el case del
            // estado (que corre después en el ciclo) — el primer ciclo no corrige.
            float pausa_corr = 0.0f;
            if (obj_hold_start_ms != 0) {
                __disable_irq();
                int32_t pdr = encoder_right - obj_pausa_r0;
                int32_t pdl = encoder_left  - obj_pausa_l0;
                __enable_irq();
                float pausa_drift = (float)(pdr + pdl) * 0.5f;
                if (fabsf(pausa_drift) > OBJ_PAUSA_POS_DB_COUNTS) {
                    pausa_corr = clampf_local(-OBJ_PAUSA_POS_KP * pausa_drift,
                                              -OBJ_PAUSA_POS_MAX, OBJ_PAUSA_POS_MAX);
                }
            }
            base_setpoint_target  = SETPOINT_ANGLE + setpoint_trim + pausa_corr;
            brake_setpoint_target = ComputeBrakeSetpointTarget(ROBOT_STATE_BALANCE_ONLY);
            line_enc_angle_corr   = 0.0f;
            line_reverse_boost    = 0.0f;
        } else if (line_state == LINE_STATE_OBJ_BUSCAR_PARED) {
            // PI de velocidad (mismo patrón que OBJ_WALL_FWD, target más cauteloso)
            // en vez de ángulo fijo, que se aceleraba sin límite buscando la pared.
            {
                float wa_vel_raw = fmaxf(0.0f, -velocity_est_f);
                float wa_vel     = apply_deadbandf(wa_vel_raw, BRAKE_VEL_DEADBAND);
                float vel_error  = OBJ_WALL_APPROACH_SPEED_TARGET - wa_vel;
                float kp         = (vel_error < 0.0f) ? OBJ_WALL_VEL_KP_BRAKE : OBJ_WALL_VEL_KP;
                if (!late_cycle) {
                    if (vel_error > 0.0f) {
                        obj_wall_vel_integral += vel_error * dt_ctrl;
                        obj_wall_vel_integral = clampf_local(
                            obj_wall_vel_integral, 0.0f, OBJ_WALL_VEL_I_MAX);
                    } else {
                        obj_wall_vel_integral *= 0.80f;
                    }
                }
                base_setpoint_target = clampf_local(
                    kp * vel_error + OBJ_WALL_VEL_KI * obj_wall_vel_integral,
                    -OBJ_WALL_BRAKE_ANGLE, OBJ_WALL_APPROACH_ANGLE_MAX);
                // Anti-stall (ver AntiStall_Tick): nunca quedarse quieto.
                base_setpoint_target += AntiStall_Tick(3,
                    base_setpoint_target > 0.0f, ANTISTALL_MAX_FWD);
            }
            brake_setpoint_target = 0.0f;
            line_enc_angle_corr   = 0.0f;
            line_reverse_boost    = 0.0f;
        } else if (line_state == LINE_STATE_OBJ_BORDEAR_PARED) {
            // Demasiado cerca (ADC7 < REVERSE_THOLD): inclinación hacia atrás, misma
            // corrección que la reversa pareja (ver switch más abajo). Si todavía hay
            // velocidad residual de avance, frena primero (upright + freno de encoders)
            // en vez de arrancar la reversa de golpe con inercia hacia adelante.
            if (obj_wall_rev_latch) {   // reversa por counts, ver latch (2026-07-10)
                obj_wall_vel_integral = 0.0f;
                if (fabsf(apply_deadbandf(velocity_est_f, BRAKE_VEL_DEADBAND)) > OBJ_WALL_REVERSE_SETTLE_VEL) {
                    base_setpoint_target  = SETPOINT_ANGLE + setpoint_trim;
                    brake_setpoint_target = ComputeBrakeSetpointTarget(ROBOT_STATE_BALANCE_ONLY);
                } else {
                    // Anti-stall: mas inclinación si no arranca (tag 5 compartido).
                    base_setpoint_target  = -OBJ_WALL_REVERSE_ANGLE -
                        AntiStall_Tick(5, 1, ANTISTALL_MAX_REV);
                    brake_setpoint_target = 0.0f;
                }
            } else {
                // PI de velocidad (mismo patrón que LOST_FWD/EDGE_FWD): regula la
                // velocidad de crucero con precisión en vez de ángulo fijo + freno
                // bang-bang, que aceleraba a fondo hasta cruzar el umbral y frenaba de golpe.
                float wf_vel_raw = fmaxf(0.0f, -velocity_est_f);
                float wf_vel     = apply_deadbandf(wf_vel_raw, BRAKE_VEL_DEADBAND);
                float vel_error  = OBJ_WALL_SPEED_TARGET - wf_vel;
                float kp         = (vel_error < 0.0f) ? OBJ_WALL_VEL_KP_BRAKE : OBJ_WALL_VEL_KP;
                if (!late_cycle) {
                    if (vel_error > 0.0f) {
                        obj_wall_vel_integral += vel_error * dt_ctrl;
                        obj_wall_vel_integral = clampf_local(
                            obj_wall_vel_integral, 0.0f, OBJ_WALL_VEL_I_MAX);
                    } else {
                        obj_wall_vel_integral *= 0.80f;
                    }
                }
                base_setpoint_target = clampf_local(
                    kp * vel_error + OBJ_WALL_VEL_KI * obj_wall_vel_integral,
                    -OBJ_WALL_BRAKE_ANGLE, OBJ_WALL_FWD_ANGLE);
                // Anti-stall (ver AntiStall_Tick): nunca quedarse quieto.
                base_setpoint_target += AntiStall_Tick(4,
                    base_setpoint_target > 0.0f, ANTISTALL_MAX_FWD);
                brake_setpoint_target = 0.0f;
            }
            line_enc_angle_corr   = 0.0f;
            line_reverse_boost    = 0.0f;
        } else if (line_state == LINE_STATE_OBJ_PARED_LIBRE) {
            // Mismo esquema que OBJ_WALL_FWD (reversa si demasiado cerca, si no PI de
            // velocidad avanzando) -- ver comentarios ahí arriba.
            if (obj_wall_rev_latch) {   // reversa por counts, ver latch (2026-07-10)
                obj_wall_vel_integral = 0.0f;
                if (fabsf(apply_deadbandf(velocity_est_f, BRAKE_VEL_DEADBAND)) > OBJ_WALL_REVERSE_SETTLE_VEL) {
                    base_setpoint_target  = SETPOINT_ANGLE + setpoint_trim;
                    brake_setpoint_target = ComputeBrakeSetpointTarget(ROBOT_STATE_BALANCE_ONLY);
                } else {
                    // Anti-stall: mas inclinación si no arranca (tag 5 compartido).
                    base_setpoint_target  = -OBJ_WALL_REVERSE_ANGLE -
                        AntiStall_Tick(5, 1, ANTISTALL_MAX_REV);
                    brake_setpoint_target = 0.0f;
                }
            } else {
                float wf_vel_raw = fmaxf(0.0f, -velocity_est_f);
                float wf_vel     = apply_deadbandf(wf_vel_raw, BRAKE_VEL_DEADBAND);
                float vel_error  = OBJ_WALL_SPEED_TARGET - wf_vel;
                float kp         = (vel_error < 0.0f) ? OBJ_WALL_VEL_KP_BRAKE : OBJ_WALL_VEL_KP;
                if (!late_cycle) {
                    if (vel_error > 0.0f) {
                        obj_wall_vel_integral += vel_error * dt_ctrl;
                        obj_wall_vel_integral = clampf_local(
                            obj_wall_vel_integral, 0.0f, OBJ_WALL_VEL_I_MAX);
                    } else {
                        obj_wall_vel_integral *= 0.80f;
                    }
                }
                base_setpoint_target = clampf_local(
                    kp * vel_error + OBJ_WALL_VEL_KI * obj_wall_vel_integral,
                    -OBJ_WALL_BRAKE_ANGLE, OBJ_WALL_FWD_ANGLE);
                // Anti-stall (ver AntiStall_Tick): nunca quedarse quieto.
                base_setpoint_target += AntiStall_Tick(6,
                    base_setpoint_target > 0.0f, ANTISTALL_MAX_FWD);
                brake_setpoint_target = 0.0f;
            }
            line_enc_angle_corr   = 0.0f;
            line_reverse_boost    = 0.0f;
        } else if (line_state == LINE_STATE_OBJ_GIRO_PARED) {
            // Wall-following girando: upright puro, motores controlados por line_pivot_active.
            // Demasiado cerca (ADC7 < REVERSE_THOLD): inclinación hacia atrás (reversa pareja),
            // con la misma espera de estabilidad que en OBJ_WALL_FWD antes de arrancar.
            if (obj_wall_rev_latch) {   // reversa por counts, ver latch (2026-07-10)
                if (fabsf(apply_deadbandf(velocity_est_f, BRAKE_VEL_DEADBAND)) > OBJ_WALL_REVERSE_SETTLE_VEL) {
                    base_setpoint_target  = SETPOINT_ANGLE + setpoint_trim;
                    brake_setpoint_target = ComputeBrakeSetpointTarget(ROBOT_STATE_BALANCE_ONLY);
                } else {
                    // Anti-stall: mas inclinación si no arranca (tag 5 compartido).
                    base_setpoint_target  = -OBJ_WALL_REVERSE_ANGLE -
                        AntiStall_Tick(5, 1, ANTISTALL_MAX_REV);
                    brake_setpoint_target = 0.0f;
                }
            } else {
                base_setpoint_target  = SETPOINT_ANGLE + setpoint_trim;
                brake_setpoint_target = 0.0f;
            }
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
        // su propio control de frenado; excluye rotaciones y LINE_STATE_FOLLOWING (que ya
        // regula su propia velocidad con el PI de línea).
        if (brake_setpoint_target == 0.0f &&
            base_setpoint_target  >  0.0f &&
            line_state != LINE_STATE_FOLLOWING &&
            line_state != LINE_STATE_LOST_ROTATE &&
            line_state != LINE_STATE_OBJ_GIRO_ESQUIVE  &&
            line_state != LINE_STATE_EDGE_ROTATE &&
            line_state != LINE_STATE_PERP_ROTATE &&
            line_state != LINE_STATE_OBJ_GIRO_PARED &&
            line_state != LINE_STATE_OBJ_BORDEAR_PARED &&
            line_state != LINE_STATE_OBJ_PARED_LIBRE &&
            line_state != LINE_STATE_OBJ_BUSCAR_PARED) {
            float global_fwd_vel = fmaxf(0.0f, -velocity_est_f);
            if (global_fwd_vel > LINE_SPEED_TARGET * 0.90f) {
                float overspeed_brake = ComputeBrakeSetpointTarget(ROBOT_STATE_BALANCE_ONLY);
                if (overspeed_brake < base_setpoint_target)
                    base_setpoint_target = overspeed_brake;
            }
        }

        line_angle_ramped = base_setpoint_target; // mantener variable para telemetría
    } else if (robot_state == ROBOT_STATE_MANUAL_CONTROL || manual_line_override) {
        // Control manual normal (joystick/comandos WiFi-USB). El banco de pruebas
        // del giro de 90° que vivía acá (activado con el joystick en reposo) se
        // eliminó a pedido del usuario: MANUAL ahora es exclusivamente para mover
        // el robot con comandos, sin ningún ciclo automático. También se entra
        // acá con manual_line_override=1 (LINE_FOLLOWING sin ver la línea +
        // comando activo) — mismo control, reutilizado tal cual.
        if (HAL_GetTick() - manual_cmd_last_ms > 60) {
            manual_setpoint_cmd *= 0.96f;
            manual_steering_cmd *= 0.90f;
            if (fabsf(manual_setpoint_cmd) < 0.01f) manual_setpoint_cmd = 0.0f;
            if (fabsf(manual_steering_cmd) < 0.01f) manual_steering_cmd = 0.0f;
        }

        // Control de velocidad adelante/atrás por PI (mismo patrón que el
        // seguidor de línea: LINE_VEL_KP/LINE_VEL_KP_BRAKE/LINE_VEL_KI) en vez
        // del mapeo directo a un ángulo fijo de antes (4°, casi nunca alcanzaba
        // a vencer la fricción estática — "no logra avanzar ni retroceder").
        // manual_setpoint_cmd es ahora la velocidad deseada en m/s con signo
        // (+ adelante, − atrás), seteada por MOVE_FORWARD/BACKWARD en UNER.c
        // (antes mandaban ±4.0f como grados; ahora mandan ±1.0f como m/s).
        const float MANUAL_SPEED_MAX    = 1.0f;   // m/s, tope de velocidad
        const float MANUAL_ANGLE_MAX    = 6.0f;   // °, tope de inclinación de avance
        const float MANUAL_VEL_KP       = 6.0f;   // ganancia P acelerando
        const float MANUAL_VEL_KP_BRAKE = 10.0f;  // ganancia P frenando/revirtiendo
        const float MANUAL_VEL_KI       = 2.0f;
        const float MANUAL_VEL_I_MAX    = 2.0f;

        float manual_desired_vel = clampf_local(manual_setpoint_cmd, -MANUAL_SPEED_MAX, MANUAL_SPEED_MAX);
        float manual_actual_vel  = -velocity_est_f; // + adelante, misma convención que el resto del archivo
        float manual_vel_error   = manual_desired_vel - manual_actual_vel;

        // "Acelerando" = el error empuja en el mismo sentido que la velocidad
        // deseada: P floja + acumula integral. Si no (frenando o revirtiendo),
        // P más fuerte y la integral decae — igual que en el seguidor de línea.
        uint8_t manual_accelerating = (manual_desired_vel >= 0.0f) ? (manual_vel_error >= 0.0f)
                                                                    : (manual_vel_error <= 0.0f);
        float manual_vel_kp_eff;
        if (manual_accelerating) {
            manual_vel_integral += manual_vel_error * DT_CTRL_FIXED;
            manual_vel_integral  = clampf_local(manual_vel_integral, -MANUAL_VEL_I_MAX, MANUAL_VEL_I_MAX);
            manual_vel_kp_eff = MANUAL_VEL_KP;
        } else {
            manual_vel_integral *= 0.80f;
            manual_vel_kp_eff = MANUAL_VEL_KP_BRAKE;
        }

        float manual_angle_cmd = clampf_local(
            manual_vel_kp_eff * manual_vel_error + MANUAL_VEL_KI * manual_vel_integral,
            -MANUAL_ANGLE_MAX, MANUAL_ANGLE_MAX
        );

        // Anti-stall (ver AntiStall_Tick): con comando activo y el robot sin
        // moverse, boost en el sentido comandado (el clamp global de 6.0 manda).
        if (fabsf(manual_setpoint_cmd) > 0.01f && manual_angle_cmd != 0.0f) {
            float man_boost = AntiStall_Tick(8, 1, ANTISTALL_MAX_MAN);
            manual_angle_cmd += (manual_angle_cmd > 0.0f) ? man_boost : -man_boost;
        }
        manual_setpoint_ramped = manual_angle_cmd; // telemetría/consistencia con el resto del archivo
        base_setpoint_target   = SETPOINT_ANGLE + setpoint_trim + manual_angle_cmd;
        // ComputeBrakeSetpointTarget devuelve 0 directo si el estado es
        // LINE_FOLLOWING (ver la función) — acá "robot_state" sigue siendo
        // LINE_FOLLOWING durante manual_line_override, así que hay que forzar
        // MANUAL_CONTROL explícitamente para que SÍ aplique el freno traslacional.
        brake_setpoint_target = ComputeBrakeSetpointTarget(
            manual_line_override ? ROBOT_STATE_MANUAL_CONTROL : robot_state
        );
    } else if (robot_state == ROBOT_STATE_BALANCE_AND_SPEED) {
        base_setpoint_target = SETPOINT_ANGLE + setpoint_trim;
    } else if (robot_state == ROBOT_STATE_BALANCE_ONLY) {
        // BALANCE_ONLY: tilt setpoint against velocity to brake post-push drift
        base_setpoint_target  = SETPOINT_ANGLE + setpoint_trim;
        brake_setpoint_target = ComputeBrakeSetpointTarget(robot_state);
    } else {
        base_setpoint_target = SETPOINT_ANGLE + setpoint_trim;
    }

    // Clamp global de setpoint. MANUAL_CONTROL necesita su propia excepción
    // (6.0, ver MANUAL_ANGLE_MAX arriba) — sin esto, este clamp recortaba en
    // silencio el tope manual a 5.0 sin que el usuario se enterara.
    float sp_limit = (robot_state == ROBOT_STATE_BALANCE_AND_SPEED) ? 2.0f
                    : (robot_state == ROBOT_STATE_MANUAL_CONTROL || manual_line_override) ? 6.0f
                    : 5.0f;
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

        if (robot_state == ROBOT_STATE_MANUAL_CONTROL || manual_line_override) {
            sp_step_max = 0.1f;
        } else if (robot_state == ROBOT_STATE_LINE_FOLLOWING) {
            // El freno del PI de línea necesita llegar antes al ángulo pedido.
            // Mantener la rampa anterior en aceleración evita endurecer el arranque.
            sp_step_max = (line_state == LINE_STATE_FOLLOWING &&
                           line_angle_cmd < 0.0f &&
                           base_setpoint_target < base_setpoint_f)
                        ? LINE_SP_STEP_BRAKE
                        : LINE_SP_STEP_NORMAL;
        } else {
            sp_step_max = 0.1f;
        }

        float sp_delta = base_setpoint_target - base_setpoint_f;

        if (sp_delta >  sp_step_max) sp_delta =  sp_step_max;
        if (sp_delta < -sp_step_max) sp_delta = -sp_step_max;

        base_setpoint_f += sp_delta;
    }

    {
        float brake_step_max = (robot_state == ROBOT_STATE_MANUAL_CONTROL || manual_line_override)
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
    // En modo línea el setpoint nunca supera el límite de freno hacia atrás.
    if (robot_state == ROBOT_STATE_LINE_FOLLOWING) {
        if (dynamic_setpoint_f < -LINE_BRAKE_ANGLE_MAX)
            dynamic_setpoint_f = -LINE_BRAKE_ANGLE_MAX;
    }
    brake_setpoint_f   = dynamic_setpoint_f - base_setpoint_f;
}

// ─────────────────────────────────────────────────────────────────────
// Etapa 13 — Detección de caída/recuperación con histéresis y contadores
// de debounce. Al caer o recuperarse resetea todo el estado de control.
// ─────────────────────────────────────────────────────────────────────
static void Ctrl_DeteccionCaida(void)
{
    float abs_roll_filt = fabsf(filtered_roll_deg);
    float abs_roll_raw  = fabsf(accel_ang_deg);

    uint8_t upright_now     = (abs_roll_raw < RECOVER_ANGLE);
    uint8_t upside_down_now = (abs_roll_raw > UPSIDE_DOWN_ANGLE);

    static uint8_t dead_zone_count       = 0;
    static uint8_t speed_trip_count      = 0;
    static uint8_t speed_safe_count      = 0;

    // Emergencia de velocidad: no cambia robot_state ni line_state. Enclava la
    // misma salida segura de una caída para que Ctrl_SalidaMotores corte ambos
    // motores. Dos muestras filtradas consecutivas evitan falsos positivos.
    float abs_speed_filtered = fabsf(velocity_est_f);
    float abs_speed_raw      = fabsf(velocity_est);
    if (!speed_limit_fault && robot_state == ROBOT_STATE_LINE_FOLLOWING) {
        if (abs_speed_filtered > LINE_SPEED_EMERGENCY_LIMIT) {
            if (speed_trip_count < LINE_SPEED_EMERGENCY_TRIP_CYC)
                speed_trip_count++;
            if (speed_trip_count >= LINE_SPEED_EMERGENCY_TRIP_CYC) {
                speed_limit_fault = 1;
                speed_safe_count  = 0;
            }
        } else {
            speed_trip_count = 0;
        }
    } else if (!speed_limit_fault) {
        speed_trip_count = 0;
    }

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
        if (speed_limit_fault || fall_by_angle || fall_upside_down || in_dead_zone) {
            f_fallen = 1;
            integral            = 0.0f;
            balance_hold_active = 0;
            velocity_est        = 0.0f;
            velocity_est_f      = 0.0f;
            velocity_est_slow_f = 0.0f;
            vel_from_accel      = 0.0f;
            line_integral       = 0.0f;
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
            obj_rev_steer_f     = 0.0f;
            lost_fwd_vel_integral = 0.0f;
            obj_wall_vel_integral = 0.0f;
            manual_vel_integral   = 0.0f;
            manual_straight_steer_f = 0.0f;
            obj_rot_initialized = 0;
            obj_rot_start_ms    = 0;
            obj_rot_phase       = 0;
            obj_rot_heading     = 0.0f;
            obj_rot_phase1_ms   = 0;
            obj_brake_start_ms   = 0;
            obj_pre_rotate_ms    = 0;
            obj_rev_band_enter_ms = 0;
            obj_hold_start_ms    = 0;
            obj_wall_approach_start_ms = 0;
            obj_wall_fwd_start_ms      = 0;
            obj_wall_lost_ms           = 0;
            obj_wall_seq_start_ms      = 0;
            obj_wall_clear_initialized = 0;
            lrot_brake_start_ms        = 0;
            lrot_settle_start_ms       = 0;
            edge_wait_start_ms         = 0;
            line_quiet_cycles          = 0;
            lost_ret_ov_active         = 0;
            line_forward_accel_f       = 0.0f;
            line_forward_vel_prev      = 0.0f;
            line_speed_predict_valid   = 0;
            // Si se cayó durante la evasión (LINE_STATE_OBJ_*, últimos del enum, de ahí
            // el ">="), abandona la evasión y vuelve a FOLLOWING en vez de retomarla.
            if (line_state >= LINE_STATE_OBJ_FRENO_REVERSA) {
                line_state                 = LINE_STATE_FOLLOWING;
                line_seen_since_entry      = 0;
                line_was_centered_on_lost  = 1;
                last_detected_edge_only    = 0;
                obj_detect_ignore_until_ms = HAL_GetTick() + 5000U;
            }
        }
    } else {
        // Una caída por velocidad NO puede recuperarse solo porque el robot
        // sigue vertical durante los primeros milisegundos. Primero debe quedar
        // por debajo de RESET_VEL durante 500ms; luego usa el rearme normal por
        // ángulo y continúa en el mismo ROBOT_STATE_LINE_FOLLOWING.
        if (speed_limit_fault) {
            // Durante f_fallen los filtros se ponen a cero en cada ciclo; para
            // el rearme se usa la lectura cruda recién calculada del encoder.
            if (abs_speed_raw < LINE_SPEED_EMERGENCY_RESET_VEL) {
                if (speed_safe_count < LINE_SPEED_EMERGENCY_RESET_CYC)
                    speed_safe_count++;
            } else {
                speed_safe_count = 0;
            }
            if (speed_safe_count >= LINE_SPEED_EMERGENCY_RESET_CYC &&
                recover_by_angle && !fall_upside_down && !in_dead_zone) {
                speed_limit_fault = 0;
                speed_trip_count  = 0;
                speed_safe_count  = 0;
            }
        }

        if (!speed_limit_fault && recover_by_angle && !fall_upside_down && !in_dead_zone) {
            f_fallen = 0;
            accel_roll_f      = accel_ang_deg;
            filtered_roll_deg = accel_ang_deg;
            integral                  = 0.0f;
            balance_hold_active       = 0;
            velocity_est              = 0.0f;
            velocity_est_f            = 0.0f;
            velocity_est_slow_f       = 0.0f;
            vel_from_accel            = 0.0f;
            line_integral             = 0.0f;
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
            velocity_est_slow_f = 0.0f;
            vel_from_accel     = 0.0f;
            dynamic_setpoint   = SETPOINT_ANGLE + setpoint_trim;
            dynamic_setpoint_f = SETPOINT_ANGLE + setpoint_trim;
            base_setpoint_f    = SETPOINT_ANGLE + setpoint_trim;
            brake_setpoint_f   = 0.0f;
            steering_adjustment = 0.0f;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────
// Etapa 14 — PID de balance: error de ángulo → pwm_sat (PWM común).
// Incluye balance hold, soft zone, saturaciones, slew-rate y anti-windup.
// ─────────────────────────────────────────────────────────────────────
static void Ctrl_PIDBalance(void)
{
    error   = dynamic_setpoint_f - filtered_roll_deg;
    float control_error = error;
    p_term  = 0.0f;
    i_term  = 0.0f;
    d_term  = 0.0f;
    output  = 0.0f;
    pwm_cmd = 0.0f;
    pwm_sat = 0.0f;
    sat_flag = 0;

    log_p_line = 0.0f;
    log_i_line = 0.0f;
    log_d_line = 0.0f;

    if (f_fallen) return;

    {
        // PID clásico: output = Kp·e + Ki·∫e·dt + Kd·de/dt
        // Unidades: error en grados, salida en %PWM (±50 tras saturar).
        //   P (Kp=4 %PWM/°): reacción inmediata proporcional a la inclinación.
        //     Solo con P el robot oscila: siempre llega tarde.
        //   I (Ki=0.1 %PWM/(°·s)): acumula el error sostenido y corrige
        //     offsets constantes (centro de masa corrido, piso inclinado)
        //     que el P solo no puede anular. 'integral' se acumula más abajo.
        //   D (Kd=0.12 %PWM/(°/s)): reacciona a la VELOCIDAD del error —
        //     frena la caída apenas empieza, antes de que el ángulo crezca.
        //     Es el término que amortigua la oscilación del P.
        p_term = KP_value * control_error;
        i_term = KI_value * integral;

        // Derivada numérica por diferencia finita: de/dt ≈ (e_k − e_k−1)/dt.
        // Amplifica ruido (un salto de 0.1° en 10ms = 10°/s), por eso se
        // recorta dos veces: la derivada a ±500°/s y el término D a ±15 %PWM.
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

            // Excluye también LOST_FWD/EDGE_FWD/OBJ_BUSCAR_PARED: sin
            // esto, el hold se activa con error chico y silencia el PID justo cuando
            // debe empujar. OBJ_BUSCAR_PARED (avance post-giro de 90° buscando la
            // pared) se agregó porque "casi no avanza" — mismo patrón de bug.
            // MANUAL_CONTROL nunca había tenido esta exclusión: con un comando de
            // adelante/atrás activo, apenas la inclinación real se acercaba al
            // setpoint (error chico) el hold entraba y ponía balance_pi_scale=0,
            // matando la autoridad del PID justo cuando tenía que sostener el
            // avance — la causa real de "el comando llega pero no logra avanzar
            // ni retroceder". Solo se excluye mientras hay un comando activo
            // (manual_setpoint_cmd != 0); con el robot realmente idle en MANUAL,
            // el hold sigue sosteniendo el balance estático como antes.
            if (robot_state == ROBOT_STATE_LINE_FOLLOWING &&
                (line_detected ||
                 line_state == LINE_STATE_LOST_FWD ||
                 line_state == LINE_STATE_EDGE_FWD ||
                 // STOP post-reversa: el hold de distancia corrige con solo ±2° —
                 // si el hold de balance silencia el PID ahí, la corrección no mueve.
                 line_state == LINE_STATE_OBJ_FRENO_REVERSA ||
                 line_state == LINE_STATE_OBJ_BUSCAR_PARED ||
                 // Estados de pared: la reversa pareja arranca desde parado con
                 // solo 3° — si el hold silencia el PID ahí, el robot queda quieto
                 // (mismo patrón de bug ya arreglado en RETROCESO/BUSCAR_PARED).
                 line_state == LINE_STATE_OBJ_BORDEAR_PARED ||
                 line_state == LINE_STATE_OBJ_PARED_LIBRE ||
                 line_state == LINE_STATE_OBJ_GIRO_PARED))
                balance_hold_active = 0;
            else if ((robot_state == ROBOT_STATE_MANUAL_CONTROL || manual_line_override) &&
                     fabsf(manual_setpoint_cmd) > 0.01f)
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

            {
                // Soft zone: interpolación lineal de la ganancia efectiva.
                // x = |error|/1.5° (recortado a 1) es "qué tan lejos del
                // equilibrio estoy" en escala 0..1; soft_scale recorre la
                // recta de 0.35 (error 0) a 1.0 (error ≥ 1.5°). Cerca del
                // equilibrio el PID trabaja al 35% para no vibrar por ruido
                // de sensor; lejos recupera autoridad completa.
                float x = abs_error / SOFT_ZONE_ANGLE_DEG;
                if (x > 1.0f) x = 1.0f;
                float soft_scale = SOFT_ZONE_MIN_SCALE + (1.0f - SOFT_ZONE_MIN_SCALE) * x;

                if (balance_hold_active) {
                    // Hold: silencia P/I (sin chatter por offset residual de
                    // angulo) pero deja el D VIVO — parado de verdad D~0 igual
                    // (gyro en cero), y apenas el robot empieza a caerse el D
                    // frena la caida ANTES de que el error cruce el umbral de
                    // salida del hold. Con D tambien apagado (version previa,
                    // hasta 2026-07-10) el hold era "motores muertos hasta que
                    // la caida ya tomo impulso" -> bamboleo permanente.
                    balance_pi_scale = 0.0f;
                    balance_d_scale  = soft_scale;
                    integral *= 0.98f;
                } else {
                    balance_pi_scale = soft_scale;
                    balance_d_scale  = soft_scale;
                }
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
            line_state == LINE_STATE_FOLLOWING && !manual_line_override) {
            if (pwm_sat >  40.0f) pwm_sat =  40.0f;
            if (pwm_sat < -40.0f) pwm_sat = -40.0f;
        }

        if (robot_state == ROBOT_STATE_MANUAL_CONTROL || manual_line_override) {
            if (pwm_sat >  55.0f) pwm_sat =  55.0f;
            if (pwm_sat < -55.0f) pwm_sat = -55.0f;
        }

        float pwm_limit;
        if (robot_state == ROBOT_STATE_LINE_FOLLOWING && !manual_line_override) {
            pwm_limit = 40.0f;
        } else {
            pwm_limit = 100.0f;
        }

        {
            // Limitador de slew-rate: el PWM no puede cambiar más de 8
            // unidades por ciclo (8%/10ms = 800%/s, alcanza el tope ±50 en
            // ~60ms). Recorta el ESCALÓN, no el valor: suaviza golpes
            // mecánicos y picos de corriente sin limitar la autoridad final.
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

        // Acumulación de la integral (∫e·dt ≈ Σ e_k·dt) con anti-windup por
        // integración condicional: si el PWM ya está saturado, seguir
        // acumulando no empuja más fuerte (el motor no da más) — solo infla
        // 'integral', y al volver el error esa carga acumulada tarda en
        // descargarse y produce un sobrepaso enorme. Regla: saturado, solo
        // se integra si el error tiene el signo CONTRARIO al comando (o sea,
        // si integrar ayuda a salir de la saturación, no a hundirse más).
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
    }
}

// ═════════════════════════════════════════════════════════════════════
//  Handlers por sub-estado del seguidor de línea (etapa 15, modo LINEA).
//  Cada función es el cuerpo del case correspondiente del switch de
//  Ctrl_MotoresLinea (más abajo): decide steering_adjustment y, si toma
//  control directo de las ruedas (pivots/frenos), setea line_pivot_active.
// ═════════════════════════════════════════════════════════════════════

// FOLLOWING: PID de línea sobre el centroide → steering; si la línea
// desaparece >150ms, decide a qué secuencia de búsqueda saltar.
static void LineState_Following(void)
{
    if (line_detected) {
        line_seen_since_entry = 1;
        line_lost_ms = HAL_GetTick();

        // Snapshot de pose para el retorno por odometría: si más
        // adelante se pierde la línea, este es el punto al que volver.
        line_loss_x_m        = odom_x_m;
        line_loss_y_m        = odom_y_m;
        line_loss_pose_valid = 1;

        // Segundo PID del sistema, mismo esquema que el de
        // balance pero sobre OTRA variable: entrada = error
        // de línea [-1..1] (centroide), salida = diferencia
        // de PWM entre ruedas (steering, ±20). Kp=10 → con
        // la línea a mitad de camino de un extremo (e=0.5)
        // pide 5 de diferencial. Los dos PID no compiten:
        // balance decide el PWM COMÚN, este el DIFERENCIAL.
        float p_line = KP_LINE * line_error;

        if (!late_cycle) {
            line_integral += line_error * dt_ctrl;
        }

        if (line_integral >  5.0f) line_integral =  5.0f;
        if (line_integral < -5.0f) line_integral = -5.0f;

        float i_line = KI_LINE * line_integral;

        // EMA (β=0.4, τ≈25ms) ANTES de derivar: la derivada
        // amplifica ruido, así que se deriva la versión
        // filtrada del error y no el error crudo.
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
            return;
        }
        uint32_t ms_sin_linea = HAL_GetTick() - line_lost_ms;

        // Sin línea: conservar steering_adjustment (último valor conocido)
        // para que el robot siga corrigiendo hacia la línea.

        // 150ms (1000→300→150): confirma la pérdida antes de cambiar de
        // estado/buscar. El freno ya actúa desde el primer ciclo sin línea en
        // Ctrl_SetpointDinamico; esta espera solo filtra parpadeos del sensor.
        if (ms_sin_linea > 150) {
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
}

// LOST/SEARCHING: sin línea, frenar primero y luego girar 180°.
static void LineState_Lost(void)
{
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
}

// LOST_BRAKE: frena hasta velocidad baja sostenida (o timeout) antes del 180°.
static void LineState_LostBrake(void)
{
    // Frena hasta velocidad baja SOSTENIDA (o timeout) antes del giro 180°.
    // 2026-07-05: se compara la velocidad CRUDA sostenida N ciclos, no la
    // deadbandeada — restar el deadband (0.35) corría el umbral efectivo a
    // ~0.5 m/s real y el robot arrancaba el pivot rodando rápido → caída.
    // Exigirla baja 15 ciclos seguidos filtra los picos de cuantización
    // (un pico aislado resetea el contador un instante, un robot rodando
    // nunca la mantiene baja 150ms).
    const uint32_t LBRAKE_TIMEOUT      = 2500U;   // colchón, no corte normal
    const float    LBRAKE_VEL_THR      = 0.25f;   // m/s crudos
    const uint8_t  LBRAKE_QUIET_CYCLES = 15U;     // 150ms sostenidos

    if (lrot_brake_start_ms == 0) {
        lrot_brake_start_ms = HAL_GetTick();
        line_quiet_cycles   = 0;
    }

    if (line_detected) {
        line_integral        = 0.0f;
        line_error_prev      = 0.0f;
        line_lost_ms         = HAL_GetTick();
        line_steer_fb_int    = 0.0f;
        line_state           = LINE_STATE_FOLLOWING;
        lrot_brake_start_ms  = 0;
        return;
    }

    if (fabsf(velocity_est_f) < LBRAKE_VEL_THR) {
        if (line_quiet_cycles < 255) line_quiet_cycles++;
    } else {
        line_quiet_cycles = 0;
    }

    uint32_t elapsed = HAL_GetTick() - lrot_brake_start_ms;
    if (line_quiet_cycles >= LBRAKE_QUIET_CYCLES ||
        elapsed >= LBRAKE_TIMEOUT) {
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
}

// LOST_ROTATE: giro de 180° por encoders+gyro para buscar la línea perdida.
static void LineState_LostRotate(void)
{
    // Giro 180° derecha para buscar la línea perdida.
    // Misma lógica gz+encoder que OBJ_ROTATE y MANUAL, escalada a 180°.
    const float  LROT_ENC_TARGET   = 960.0f;  // antes 880 — quedaba corto (~165° reales), reescalado 880*(180/165)
    const float  LROT_PIVOT        = 24.0f;  // antes 14→20→24: arranques dispares por pwm_sat residual compitiendo con el pivot
    const float  LROT_BRAKE        = 10.0f;  // antes 16 — torque fijo sin medir velocidad remanente, a veces pasaba de frenada y desestabilizaba
    const float  LROT_SLOWDOWN_DEG = 55.0f;
    const float  LROT_ABS_TARGET   = 180.0f;
    // P0_MAX/P1_MAX son colchón de seguridad (peor caso ~2.3s), no el corte normal
    const uint32_t LROT_P0_MAX     = 1500U;
    const uint32_t LROT_P1_MAX     = 800U;
    const float  LROT_ENC_FRAC     = 0.60f;  // frena al 60% del recorrido

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

    // El giro no sale anticipadamente al ver línea a mitad de camino (podría
    // ser una detección espuria); siempre completa por encoder.
    float lrot_gz = (float)gz / 100.0f;
    obj_rot_heading += lrot_gz * DT_CTRL_FIXED;

    __disable_irq();
    int32_t lrot_dr = encoder_right - obj_rot_r0;
    int32_t lrot_dl = encoder_left  - obj_rot_l0;
    __enable_irq();
    float lrot_counts    = (fabsf((float)lrot_dr) + fabsf((float)lrot_dl)) * 0.5f;
    float lrot_enc_deg   = lrot_counts * (LROT_ABS_TARGET / LROT_ENC_TARGET);
    float lrot_abs_hdg   = fmaxf(fabsf(obj_rot_heading), lrot_enc_deg);
    float lrot_enc_thr   = LROT_ENC_TARGET * LROT_ENC_FRAC;

    // 2026-07-05: con gz/100 (resolución nueva, 31% más alta que /131) el
    // gyro dentro del fmaxf hace pensar que ya se completó el 70% del giro
    // antes de que el encoder lo confirme físicamente — entra temprano a la
    // fase de terminación (mucho más débil) y el resto del giro se arrastra
    // lento y a pulsos. Fix mínimo: la transición de fase se decide SOLO por
    // encoder (la fuente que no cambió de escala); el gyro (lrot_abs_hdg)
    // queda únicamente para la red de seguridad de sobregiro más abajo.
    if (obj_rot_phase == 0) {
        uint32_t elapsed = HAL_GetTick() - obj_rot_start_ms;
        if (lrot_counts  >= lrot_enc_thr ||
            elapsed      >= LROT_P0_MAX) {
            obj_rot_phase     = 1;
            obj_rot_phase1_ms = HAL_GetTick();
        } else {
            // Rampa de frenado interna también solo por encoder (mismo
            // motivo que la transición de fase, ver comentario arriba) —
            // con el gyro inflado, "remaining" se achicaba antes de tiempo
            // y bajaba el pivot (ya de por sí solo 14 PWM) durante buena
            // parte del giro, no solo cerca del final.
            float remaining = LROT_ABS_TARGET - lrot_enc_deg;
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
        // El freno arranca antes de llegar al 100% (antes esperaba a
        // enc_done exacto = actuaba de golpe justo al final, rompiendo el
        // equilibrio). Ahora entra al 85% del recorrido; el corte final
        // (enc_done + duración) no cambia, así que no acorta el giro.
        int enc_near_done = (lrot_counts >= LROT_ENC_TARGET * 0.85f);
        int overshoot     = (lrot_abs_hdg > LROT_ABS_TARGET * 1.2f);
        // Freno fuerte a torque fijo por tiempo corto y acotado, sin
        // buscar una velocidad de frenado exacta.
        const uint32_t LROT_BRAKE_DURATION = 250U;  // antes 400 — frenaba bien y luego "volvía" un poco (sobrefrenaba en reversa el tiempo que le quedaba)
        if ((enc_done && p1e >= LROT_BRAKE_DURATION) || p1e >= LROT_P1_MAX || overshoot) {
            // Bypass de LOST_SETTLE: arranca a avanzar sin esperar. Si ya
            // quedó sobre la línea, directo a FOLLOWING.
            if (line_detected) {
                line_seen_since_entry = 1;
                line_integral         = 0.0f;
                line_error_prev       = 0.0f;
                line_lost_ms          = HAL_GetTick();
                line_state            = LINE_STATE_FOLLOWING;
            } else {
                line_state          = LINE_STATE_LOST_FWD;
                line_lost_ms        = HAL_GetTick();
                lost_fwd_vel_integral = 0.0f;
            }
            obj_rot_initialized = 0;
            obj_rot_phase       = 0;
            obj_rot_heading     = 0.0f;
            steering_adjustment = 0.0f;
        } else if (!enc_near_done) {
            // Encoder aún no llegó al 85%: seguir pivoteando suave
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
}

// LOST_SETTLE: pausa de estabilización post-180° antes de avanzar.
static void LineState_LostSettle(void)
{
    // Post-180°: pausa de estabilización fuerte (upright + freno de
    // encoders) antes de arrancar a avanzar. Sale por tiempo mínimo
    // cumplido Y velocidad/inclinación estabilizadas. El timeout es
    // solo un colchón de seguridad (3s, no un corte normal) — antes
    // (1200ms) cortaba antes de frenar de verdad y el robot arrancaba
    // a avanzar con velocidad residual del giro.
    const uint32_t LSETTLE_MIN_MS   = 400U;
    const uint32_t LSETTLE_TIMEOUT  = 3000U;
    const float    LSETTLE_VEL_THR  = 0.25f;   // m/s crudos (ver comentario abajo)
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
        return;
    }

    uint32_t elapsed  = HAL_GetTick() - lrot_settle_start_ms;
    float tilt_err    = fabsf(filtered_roll_deg - dynamic_setpoint_f);
    // velocidad cruda con umbral 0.25 (deadbandeada con 0.10 equivalía a
    // ~0.45 m/s reales — demasiado permisivo para "estabilizado")
    int settled       = (elapsed >= LSETTLE_MIN_MS) &&
                        (fabsf(velocity_est_f) < LSETTLE_VEL_THR) &&
                        (tilt_err < LSETTLE_TILT_THR);

    if (settled || elapsed >= LSETTLE_TIMEOUT) {
        line_state           = LINE_STATE_LOST_FWD;
        line_lost_ms         = HAL_GetTick();
        lrot_settle_start_ms = 0;
        lost_fwd_vel_integral = 0.0f;
    }
}

// LOST_FWD: retorno por odometría al punto donde se perdió la línea.
static void LineState_LostFwd(void)
{
    // Post-180°: RETORNO POR ODOMETRÍA — en vez de avanzar a ciegas,
    // navega hacia la pose donde se vio la línea por última vez
    // (line_loss_x/y_m): steering P sobre el error de rumbo entre el
    // bearing al objetivo y theta odométrico. La velocidad la sigue
    // regulando el PI del bloque de setpoints (LOST_FWD_*).
    // Salidas: línea detectada → FOLLOWING; punto alcanzado sin línea
    // → GIVEN_UP; timeout como colchón de seguridad (10s, subido de
    // 5s a pedido del usuario para dar más tiempo de búsqueda).
    const uint32_t LOST_FWD_TIMEOUT = 10000U;
    if (line_detected) {
        line_seen_since_entry = 1;
        line_integral     = 0.0f;
        line_error_prev   = 0.0f;
        line_lost_ms      = HAL_GetTick();
        line_state        = LINE_STATE_FOLLOWING;
        steering_adjustment = 0.0f;
        lost_ret_ov_active  = 0;
    } else if ((HAL_GetTick() - line_lost_ms) > LOST_FWD_TIMEOUT) {
        line_state = LINE_STATE_GIVEN_UP;
        steering_adjustment = 0.0f;
        lost_ret_ov_active  = 0;
    } else if (line_loss_pose_valid) {
        float ret_dx   = line_loss_x_m - odom_x_m;
        float ret_dy   = line_loss_y_m - odom_y_m;
        float ret_dist = sqrtf(ret_dx * ret_dx + ret_dy * ret_dy);
        float ret_bearing = atan2f(ret_dy, ret_dx) * (180.0f / M_PI);
        float ret_herr    = ret_bearing - odom_theta_deg;
        if      (ret_herr >  180.0f) ret_herr -= 360.0f;
        else if (ret_herr < -180.0f) ret_herr += 360.0f;

        // Punto alcanzado (o pasado de costado): NO rendirse ahí
        // mismo — sobrepasar en línea recta para que los sensores
        // crucen la cinta (ver LOST_RETURN_OVERSHOOT_M).
        if (!lost_ret_ov_active &&
            (ret_dist < LOST_RETURN_REACHED_M ||
             (ret_dist < LOST_RETURN_PASS_WIN_M &&
              fabsf(ret_herr) > 120.0f))) {
            lost_ret_ov_active = 1;
            lost_ret_ov_x      = odom_x_m;
            lost_ret_ov_y      = odom_y_m;
        }

        if (lost_ret_ov_active) {
            float ov_dx = odom_x_m - lost_ret_ov_x;
            float ov_dy = odom_y_m - lost_ret_ov_y;
            if (sqrtf(ov_dx * ov_dx + ov_dy * ov_dy) >=
                LOST_RETURN_OVERSHOOT_M) {
                // Sobrepaso agotado sin línea: reposo (GIVEN_UP
                // retoma solo si la línea vuelve a aparecer).
                line_state = LINE_STATE_GIVEN_UP;
                lost_ret_ov_active = 0;
            }
            // Derecho, sin navegación: el yaw-assist por gyro
            // (steering==0) mantiene el rumbo.
            steering_adjustment = 0.0f;
        } else {
            // Signo: steering positivo genera gz negativo (misma
            // convención que el amortiguador gz*0.3), y theta integra
            // ODOM_THETA_SIGN*gz — de ahí el -ODOM_THETA_SIGN.
            steering_adjustment = clampf_local(
                -ODOM_THETA_SIGN * LOST_RETURN_STEER_KP * ret_herr,
                -LOST_RETURN_STEER_MAX, LOST_RETURN_STEER_MAX);
        }
    } else {
        // Sin snapshot válido (recién entrado al modo): avance recto
        // con el comportamiento anterior (yaw-assist por gyro).
        steering_adjustment = 0.0f;
    }
}

// EDGE_WAIT: perdida por un extremo (curva) — frena antes del giro de 45°.
static void LineState_EdgeWait(void)
{
    // Perdida por un extremo (curva): frena hasta velocidad baja (o
    // timeout) antes del giro de 90°. La entrada a este estado ya
    // ocurrió >=1s después de perder la línea (ver salida de FOLLOWING),
    // así que solo falta esperar a que el robot esté quieto.
    // Mismo gate de quietud sostenida que LOST_BRAKE (ver comentario ahí).
    const uint32_t EWAIT_TIMEOUT       = 2500U;
    const float    EWAIT_VEL_THR       = 0.25f;  // m/s crudos
    const uint8_t  EWAIT_QUIET_CYCLES  = 15U;    // 150ms sostenidos

    if (edge_wait_start_ms == 0) {
        edge_wait_start_ms = HAL_GetTick();
        line_quiet_cycles  = 0;
    }
    steering_adjustment = 0.0f;

    if (fabsf(velocity_est_f) < EWAIT_VEL_THR) {
        if (line_quiet_cycles < 255) line_quiet_cycles++;
    } else {
        line_quiet_cycles = 0;
    }

    uint32_t elapsed = HAL_GetTick() - edge_wait_start_ms;
    if (line_quiet_cycles >= EWAIT_QUIET_CYCLES ||
        elapsed >= EWAIT_TIMEOUT) {
        line_state          = LINE_STATE_EDGE_ROTATE;
        obj_rot_initialized = 0;
        obj_rot_phase       = 0;
        obj_rot_heading     = 0.0f;
        obj_rot_phase1_ms   = 0;
        obj_rot_start_ms    = 0;
        edge_wait_start_ms  = 0;
    }
}

// EDGE_ROTATE: gira 45° hacia el lado por el que se fue la línea.
static void LineState_EdgeRotate(void)
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

    float erot_gz = (float)gz / 100.0f;
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
}

// PERP_ROTATE: cruce perpendicular (4 sensores en negro) — gira 90° hacia
// el lado del último esquive y retoma FOLLOWING.
static void LineState_PerpRotate(void)
{
    // Cruce perpendicular detectado (los 4 ADC en negro sin manipular
    // el robot): gira 90° fijo y vuelve directo a FOLLOWING, sin pasar
    // por ningún estado de avance — se asume que quedó sobre la línea.
    // Sin información de hacia qué lado está la línea (los 4 sensores
    // vieron lo mismo), la dirección sigue a `obj_esquive_dir`
    // (2026-07-14): reencontrando la línea después de bordear el
    // obstáculo, girar hacia el MISMO lado del esquive aleja del
    // objeto — girar al contrario lo devolvía directo al obstáculo.
    // Sin esquive previo obj_esquive_dir vale +1 (se rearma al entrar
    // al modo línea), así que el caso normal conserva la convención
    // histórica de girar a la derecha. Mismo esquema de freno fijo
    // corto que LOST_ROTATE/EDGE_ROTATE (2026-07-01): no sale
    // anticipadamente ante ninguna condición, siempre completa el
    // giro por encoder y luego un freno fijo breve.
    const float  PROT_ENC_TARGET  = 440.0f; // calibración base de 90°
    const float  PROT_PIVOT       = 14.0f;
    const float  PROT_BRAKE       = 16.0f;
    const float  PROT_OBJ_BRAKE   = 8.0f;  // contrafreno suave solo para el giro final de esquive
    const float  PROT_SLOWDOWN_DEG = 55.0f;
    const float  PROT_ABS_TARGET  = 90.0f;
    const uint32_t PROT_P0_MAX    = 1500U;
    const uint32_t PROT_P1_MAX    = 800U;
    const uint32_t PROT_BRAKE_DURATION = 400U;
    const uint32_t PROT_OBJ_BRAKE_RAMP_MS = 200U;
    const uint32_t PROT_OBJ_BRAKE_DURATION = 300U;
    const float  PROT_ENC_FRAC    = 0.60f;
    const float  PROT_DIR         = (float)obj_esquive_dir; // sentido del último esquive (+1 derecha por defecto)

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
        perp_obj_brake_start_ms = 0;
    }

    float prot_gz = (float)gz / 100.0f;
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
        uint32_t obj_brake_elapsed = 0;
        if (enc_done && perp_from_obj_avoidance) {
            if (perp_obj_brake_start_ms == 0)
                perp_obj_brake_start_ms = HAL_GetTick();
            obj_brake_elapsed = HAL_GetTick() - perp_obj_brake_start_ms;
        }
        int brake_done = perp_from_obj_avoidance
            ? (enc_done && obj_brake_elapsed >= PROT_OBJ_BRAKE_DURATION)
            : (enc_done && p1e >= PROT_BRAKE_DURATION);
        int phase_timeout = perp_from_obj_avoidance
            ? (!enc_done && p1e >= PROT_P1_MAX)
            : (p1e >= PROT_P1_MAX);
        if (brake_done || phase_timeout || overshoot) {
            // 2026-07-14: antes retomaba FOLLOWING directo — llegando
            // del esquive con velocidad, arrancaba lanzado y se
            // Sin pausa posterior: el giro final lento ya entrega
            // el robot directamente al seguidor de línea.
            line_state = LINE_STATE_FOLLOWING;
            perp_from_obj_avoidance = 0;
            perp_obj_brake_start_ms = 0;
            line_seen_since_entry = 1;
            line_integral       = 0.0f;
            line_error_prev     = 0.0f;
            line_lost_ms        = HAL_GetTick();
            obj_rot_initialized = 0;
            obj_rot_phase       = 0;
            obj_rot_heading     = 0.0f;
            steering_adjustment = 0.0f;
            // Sin este reset, si los sensores siguen en negro después del
            // giro (no habia linea real), el timer ya vencido re-dispara
            // otro PERP_ROTATE apenas vuelva a FOLLOWING -> giro en círculos.
            all_black_start_ms  = 0;
        } else if (!enc_done) {
            perp_obj_brake_start_ms = 0;
            float remaining = PROT_ENC_TARGET - prot_counts;
            float ramp = fminf(remaining / (PROT_ENC_TARGET * 0.15f), 1.0f);
            float pivot = PROT_DIR * PROT_PIVOT * 0.4f * fmaxf(ramp, 0.2f);
            line_pivot_active  = 1;
            motorRightVelocity = (int16_t)clampf_local(-(pwm_sat + pivot), -60.0f, 60.0f);
            motorLeftVelocity  = (int16_t)clampf_local(-(pwm_sat - pivot), -60.0f, 60.0f);
        } else {
            float brake_power = PROT_BRAKE;
            if (perp_from_obj_avoidance) {
                float brake_ramp = fminf(
                    (float)obj_brake_elapsed / (float)PROT_OBJ_BRAKE_RAMP_MS,
                    1.0f);
                brake_power = PROT_OBJ_BRAKE * brake_ramp;
            }
            float pivot = PROT_DIR * brake_power;
            line_pivot_active  = 1;
            motorRightVelocity = (int16_t)clampf_local(-(pwm_sat - pivot), -60.0f, 60.0f);
            motorLeftVelocity  = (int16_t)clampf_local(-(pwm_sat + pivot), -60.0f, 60.0f);
        }
    }
}

// EDGE_SETTLE: pausa de estabilización post-45° antes de avanzar.
static void LineState_EdgeSettle(void)
{
    // Post-90°: pausa de estabilización idéntica a LOST_SETTLE (mismo
    // fix 2026-07-01: timeout es colchón de seguridad, no corte normal).
    const uint32_t ESETTLE_MIN_MS   = 400U;
    const uint32_t ESETTLE_TIMEOUT  = 3000U;
    const float    ESETTLE_VEL_THR  = 0.25f;  // m/s crudos
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
        return;
    }

    uint32_t elapsed  = HAL_GetTick() - lrot_settle_start_ms;
    float tilt_err    = fabsf(filtered_roll_deg - dynamic_setpoint_f);
    // ídem LOST_SETTLE: cruda con umbral 0.25
    int settled       = (elapsed >= ESETTLE_MIN_MS) &&
                        (fabsf(velocity_est_f) < ESETTLE_VEL_THR) &&
                        (tilt_err < ESETTLE_TILT_THR);

    if (settled || elapsed >= ESETTLE_TIMEOUT) {
        line_state           = LINE_STATE_EDGE_FWD;
        line_lost_ms         = HAL_GetTick();
        lrot_settle_start_ms = 0;
        lost_fwd_vel_integral = 0.0f;
    }
}

// EDGE_FWD: retorno por odometría, mismo mecanismo que LOST_FWD.
static void LineState_EdgeFwd(void)
{
    // Post-45°: RETORNO POR ODOMETRÍA, mismo mecanismo que LOST_FWD
    // (ver ahí) — navega hacia la pose donde se vio la línea por última
    // vez (line_loss_x/y_m, snapshot compartido, se graba sin importar
    // si la pérdida fue centrada o por el extremo). 10s de timeout como
    // colchón de seguridad → reposo total (GIVEN_UP).
    const uint32_t EDGE_FWD_TIMEOUT = 10000U;
    if (line_detected) {
        line_seen_since_entry = 1;
        line_integral     = 0.0f;
        line_error_prev   = 0.0f;
        line_lost_ms      = HAL_GetTick();
        line_state        = LINE_STATE_FOLLOWING;
        steering_adjustment = 0.0f;
        lost_ret_ov_active  = 0;
    } else if ((HAL_GetTick() - line_lost_ms) > EDGE_FWD_TIMEOUT) {
        // (ídem LOST_FWD: f_fallen acá era inalcanzable)
        line_state = LINE_STATE_GIVEN_UP;
        steering_adjustment = 0.0f;
        lost_ret_ov_active  = 0;
    } else if (line_loss_pose_valid) {
        float ret_dx   = line_loss_x_m - odom_x_m;
        float ret_dy   = line_loss_y_m - odom_y_m;
        float ret_dist = sqrtf(ret_dx * ret_dx + ret_dy * ret_dy);
        float ret_bearing = atan2f(ret_dy, ret_dx) * (180.0f / M_PI);
        float ret_herr    = ret_bearing - odom_theta_deg;
        if      (ret_herr >  180.0f) ret_herr -= 360.0f;
        else if (ret_herr < -180.0f) ret_herr += 360.0f;

        // Mismo sobrepaso que LOST_FWD (ver ahí).
        if (!lost_ret_ov_active &&
            (ret_dist < LOST_RETURN_REACHED_M ||
             (ret_dist < LOST_RETURN_PASS_WIN_M &&
              fabsf(ret_herr) > 120.0f))) {
            lost_ret_ov_active = 1;
            lost_ret_ov_x      = odom_x_m;
            lost_ret_ov_y      = odom_y_m;
        }

        if (lost_ret_ov_active) {
            float ov_dx = odom_x_m - lost_ret_ov_x;
            float ov_dy = odom_y_m - lost_ret_ov_y;
            if (sqrtf(ov_dx * ov_dx + ov_dy * ov_dy) >=
                LOST_RETURN_OVERSHOOT_M) {
                line_state = LINE_STATE_GIVEN_UP;
                lost_ret_ov_active = 0;
            }
            steering_adjustment = 0.0f;
        } else {
            steering_adjustment = clampf_local(
                -ODOM_THETA_SIGN * LOST_RETURN_STEER_KP * ret_herr,
                -LOST_RETURN_STEER_MAX, LOST_RETURN_STEER_MAX);
        }
    } else {
        // Sin snapshot válido (recién entrado al modo): avance recto
        // con el comportamiento anterior (yaw-assist por gyro).
        steering_adjustment = 0.0f;
    }
}

// GIVEN_UP: reposo total balanceado hasta que la línea vuelva a aparecer.
static void LineState_GivenUp(void)
{
    steering_adjustment = 0.0f;
    if (line_detected) {
        line_seen_since_entry = 1;
        line_integral       = 0.0f;
        line_error_prev     = 0.0f;
        line_lost_ms        = HAL_GetTick();
        line_state          = LINE_STATE_FOLLOWING;
    }
}

// OBJ_FRENO_REVERSA ("STOP"): entrada directa al detectar objeto — frena la
// inercia, mantiene la distancia (hold de A6/A8) y espera estabilidad → giro.
static void LineState_ObjFrenoReversa(void)
{
    // Fase 1: espera a que la velocidad caiga. Fase 2: wait antes de rotar.
    // Además, el giro exige distancia ESTABLE: A6 y A8 dentro de la banda
    // 3600..3900 durante OBJ_REV_STABLE_MS continuos — mientras el hold de
    // distancia (ver setpoints) esté corrigiendo, el giro espera. Colchón:
    // si en OBJ_REV_STOP_TIMEOUT_MS total nunca se estabilizó, gira igual.
    const float    BRAKE_VEL_THR  = 0.05f;
    const uint32_t BRAKE_TIMEOUT  = 1500U;
    const uint32_t PRE_ROTATE_MS  = 3000U;
    if (obj_brake_start_ms == 0) {
        obj_brake_start_ms = HAL_GetTick();
        // Rumbo recto (2026-07-14): desde que el STOP es la entrada
        // al detectar objeto, su hold hace también la reversa hasta
        // la banda de A6/A8 — y salía chueca (reportado) porque no
        // tenía ninguna corrección de rumbo. P sobre la diferencia
        // ACUMULADA de counts (ver REV_STRAIGHT_*): ancla el rumbo
        // de entrada y lo mantiene durante toda la fase, en ambos
        // sentidos de corrección del hold.
        __disable_irq();
        obj_rev_r0 = encoder_right;
        obj_rev_l0 = encoder_left;
        __enable_irq();
        obj_rev_steer_f      = 0.0f;
        obj_rev_last_counts  = 0;
        obj_rev_last_move_ms = HAL_GetTick();
    }

    {
        __disable_irq();
        int32_t rev_dr = encoder_right - obj_rev_r0;
        int32_t rev_dl = encoder_left  - obj_rev_l0;
        __enable_irq();
        int32_t rev_counts = (abs(rev_dr) + abs(rev_dl)) / 2;
        if (rev_counts != obj_rev_last_counts) {
            obj_rev_last_counts  = rev_counts;
            obj_rev_last_move_ms = HAL_GetTick();
        }
        // Solo corrige mientras hay movimiento real (gate por
        // actividad de encoders);
        // quieto, el target decae a 0 por el slew (anti-pivoteo).
        float corr_target = 0.0f;
        if ((HAL_GetTick() - obj_rev_last_move_ms) < REV_STRAIGHT_ACT_MS) {
            corr_target = clampf_local(
                REV_STRAIGHT_KC * (float)(rev_dr - rev_dl),
                -REV_STRAIGHT_MAX, REV_STRAIGHT_MAX);
        }
        float corr_delta = corr_target - obj_rev_steer_f;
        if (corr_delta >  REV_STRAIGHT_SLEW) corr_delta =  REV_STRAIGHT_SLEW;
        if (corr_delta < -REV_STRAIGHT_SLEW) corr_delta = -REV_STRAIGHT_SLEW;
        obj_rev_steer_f += corr_delta;
        steering_adjustment = obj_rev_steer_f;
    }

    // Fase 1: frenar hasta quieto o timeout (cruda: la deadbandeada con
    // umbral 0.05 equivalía a ~0.40 m/s reales)
    if (obj_pre_rotate_ms == 0) {
        if (fabsf(velocity_est_f) < BRAKE_VEL_THR + 0.10f ||
            (HAL_GetTick() - obj_brake_start_ms) > BRAKE_TIMEOUT) {
            obj_pre_rotate_ms = HAL_GetTick();  // arranca fase 2
        }
    }

    // Tracking de estabilidad de distancia: dentro de banda arranca/sigue
    // el timer; fuera de banda se resetea (los 2s deben ser CONTINUOS).
    {
        float front_min = fminf((float)adcAvg[5], (float)adcAvg[7]);
        uint8_t in_band = (front_min >= OBJ_REV_CLEAR_ADC &&
                           front_min <= OBJ_REV_CLEAR_ADC + OBJ_REV_HOLD_BAND);
        if (!in_band)                          obj_rev_band_enter_ms = 0;
        else if (obj_rev_band_enter_ms == 0)   obj_rev_band_enter_ms = HAL_GetTick();
    }
    uint8_t dist_stable = (obj_rev_band_enter_ms != 0 &&
                           (HAL_GetTick() - obj_rev_band_enter_ms) >= OBJ_REV_STABLE_MS);

    // Fase 2: wait quieto + distancia estable 2s (o timeout total) antes de girar
    if (obj_pre_rotate_ms != 0 &&
        (HAL_GetTick() - obj_pre_rotate_ms) >= PRE_ROTATE_MS &&
        (dist_stable ||
         (HAL_GetTick() - obj_brake_start_ms) >= OBJ_REV_STOP_TIMEOUT_MS)) {
        line_state          = LINE_STATE_OBJ_GIRO_ESQUIVE;
        obj_rot_initialized = 0;
        obj_rot_phase       = 0;
        obj_rot_heading     = 0.0f;
        obj_rot_phase1_ms   = 0;
        steering_adjustment = 0.0f;
        obj_brake_start_ms  = 0;
        obj_pre_rotate_ms   = 0;
        obj_rev_band_enter_ms = 0;
    }
}

// OBJ_GIRO_ESQUIVE: giro de 90° para esquivar (sentido según obj_esquive_dir),
// con ajuste fino por el lateral si el giro quedó corto.
static void LineState_ObjGiroEsquive(void)
{
    // Giro 90° para esquivar objeto: MISMA lógica y constantes
    // que el banco de pruebas de 90° en MANUAL, ya validado (pivot fijo
    // con slowdown por encoder, freno arrancando al 85% del recorrido,
    // continuidad del freno al terminar). Sentido según obj_esquive_dir
    // (2026-07-13): +1 derecha (histórico), -1 izquierda (espejado).
    const float  LINE_ROT_ENC_TARGET   = 372.0f;  // recortado más: 460→394→372 (seguía quedando grande)
    const float  LINE_ROT_PIVOT        = 24.0f;
    const float  LINE_ROT_BRAKE         = 10.0f;
    const float  LINE_ROT_SLOWDOWN_DEG = 28.0f;
    const float  LINE_ROT_ABS_TARGET   = 90.0f;
    const uint32_t LINE_ROT_P0_MAX     = 1500U;
    const uint32_t LINE_ROT_P1_MAX     = 800U;
    const float  LINE_ROT_ENC_FRAC     = 0.60f;
    const uint32_t LINE_ROT_BRAKE_DURATION = 250U;

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

    float obj_gz_dps = (float)gz / 100.0f;
    obj_rot_heading += obj_gz_dps * DT_CTRL_FIXED;

    // Espeja pivot y freno según el sentido del esquive; el heading
    // se compara siempre en valor absoluto, no necesita signo.
    float rot_dir = (float)obj_esquive_dir;

    __disable_irq();
    int32_t dr = encoder_right - obj_rot_r0;
    int32_t dl = encoder_left  - obj_rot_l0;
    __enable_irq();
    float rot_counts   = (fabsf((float)dr) + fabsf((float)dl)) * 0.5f;
    float enc_heading_deg = rot_counts * (LINE_ROT_ABS_TARGET / LINE_ROT_ENC_TARGET);
    // Transición de fase solo por encoder (ver LOST_ROTATE); abs_heading
    // (con gyro) queda solo para el overshoot.
    float abs_heading   = fmaxf(fabsf(obj_rot_heading), enc_heading_deg);
    float enc_phase0_thr = LINE_ROT_ENC_TARGET * LINE_ROT_ENC_FRAC;

    if (obj_rot_phase == 0) {
        uint32_t elapsed = HAL_GetTick() - obj_rot_start_ms;
        if (rot_counts >= enc_phase0_thr ||
            elapsed    >= LINE_ROT_P0_MAX) {
            obj_rot_phase     = 1;
            obj_rot_phase1_ms = HAL_GetTick();
        } else {
            // Rampa de frenado interna también solo por encoder.
            float remaining = LINE_ROT_ABS_TARGET - enc_heading_deg;
            float slowdown  = (remaining < LINE_ROT_SLOWDOWN_DEG)
                            ? (remaining / LINE_ROT_SLOWDOWN_DEG)
                            : 1.0f;
            float pivot = LINE_ROT_PIVOT * fmaxf(slowdown, 0.0f) * rot_dir;
            line_pivot_active = 1;
            motorRightVelocity = (int16_t)clampf_local(
                -(pwm_sat + pivot), -60.0f, 60.0f);
            motorLeftVelocity  = (int16_t)clampf_local(
                -(pwm_sat - pivot), -60.0f, 60.0f);
        }
    }

    if (obj_rot_phase == 1) {
        uint32_t p1_elapsed = HAL_GetTick() - obj_rot_phase1_ms;
        int enc_done      = (rot_counts >= LINE_ROT_ENC_TARGET);
        // Freno arranca al 85% del recorrido, no recién al 100% (ver
        // LOST_ROTATE) — el corte final sigue dependiendo de enc_done.
        int enc_near_done = (rot_counts >= LINE_ROT_ENC_TARGET * 0.85f);
        int overshoot     = (abs_heading > LINE_ROT_ABS_TARGET * 1.2f);
        if ((enc_done && p1_elapsed >= LINE_ROT_BRAKE_DURATION) ||
            p1_elapsed >= LINE_ROT_P1_MAX || overshoot) {
            // Verificación por el lateral activo (2026-07-10): buena
            // detención = la pared quedó a la vista (< 3200). Si no
            // bajó de ahí, el giro quedó corto → fase 2 de ajuste fino
            // (sigue pivotando despacio hasta verla). El overshoot NO
            // entra al ajuste: girar más solo lo empeoraría.
            if ((float)adcAvg[OBJ_WALL_ADC_IDX] < OBJ_ROT_ADC_GOOD ||
                overshoot) {
                line_state          = LINE_STATE_OBJ_PAUSA_GIRO;
                obj_rot_initialized = 0;
                obj_rot_phase       = 0;
                obj_rot_heading     = 0.0f;
                obj_rot_start_ms    = 0;
            } else {
                obj_rot_phase        = 2;
                obj_rot_p2_c0        = rot_counts;
                obj_rot_p2_ms        = HAL_GetTick();
                obj_rot_adc_good_cnt = 0;
            }
            // Seguir aplicando el mismo freno diferencial un ciclo más al
            // terminar (ver fix en banco MANUAL) — soltarlo de golpe a 0
            // daba un empujón justo en la transición.
            line_pivot_active = 1;
            motorRightVelocity = (int16_t)clampf_local(
                -(pwm_sat - LINE_ROT_BRAKE * rot_dir), -60.0f, 60.0f);
            motorLeftVelocity  = (int16_t)clampf_local(
                -(pwm_sat + LINE_ROT_BRAKE * rot_dir), -60.0f, 60.0f);
        } else if (!enc_near_done) {
            float remaining = LINE_ROT_ENC_TARGET - rot_counts;
            float ramp = fminf(remaining / (LINE_ROT_ENC_TARGET * 0.15f), 1.0f);
            float pivot = LINE_ROT_PIVOT * 0.4f * fmaxf(ramp, 0.2f) * rot_dir;
            line_pivot_active = 1;
            motorRightVelocity = (int16_t)clampf_local(
                -(pwm_sat + pivot), -60.0f, 60.0f);
            motorLeftVelocity  = (int16_t)clampf_local(
                -(pwm_sat - pivot), -60.0f, 60.0f);
        } else {
            line_pivot_active = 1;
            motorRightVelocity = (int16_t)clampf_local(
                -(pwm_sat - LINE_ROT_BRAKE * rot_dir), -60.0f, 60.0f);
            motorLeftVelocity  = (int16_t)clampf_local(
                -(pwm_sat + LINE_ROT_BRAKE * rot_dir), -60.0f, 60.0f);
        }
    } else if (obj_rot_phase == 2) {
        // Ajuste fino por el lateral activo (ADC7 girando a derecha,
        // ADC5 a izquierda; 2026-07-10): los 90° por encoders
        // terminaron pero la pared no quedó a la vista lateral
        // (≥ 3200) — le falta un poco más. Pivot suave en el mismo
        // sentido del giro hasta que el lateral baje de
        // OBJ_ROT_ADC_GOOD durante GOOD_CYC ciclos seguidos, con
        // tope de counts extra y timeout (por si la pared quedó
        // fuera de alcance).
        uint8_t rot_adc_good =
            ((float)adcAvg[OBJ_WALL_ADC_IDX] < OBJ_ROT_ADC_GOOD);
        if (rot_adc_good) {
            if (obj_rot_adc_good_cnt < 255) obj_rot_adc_good_cnt++;
        } else {
            obj_rot_adc_good_cnt = 0;
        }
        float extra_counts = rot_counts - obj_rot_p2_c0;

        if (obj_rot_adc_good_cnt >= OBJ_ROT_ADC_GOOD_CYC ||
            extra_counts >= OBJ_ROT_ADC_EXTRA_MAX ||
            (HAL_GetTick() - obj_rot_p2_ms) > OBJ_ROT_ADC_EXTRA_MS) {
            obj_rot_initialized = 0;
            obj_rot_phase       = 0;
            obj_rot_heading     = 0.0f;
            obj_rot_start_ms    = 0;
            // 2026-07-14: SIEMPRE sigue a PAUSA_GIRO, aunque el lateral
            // no haya visto la pared. El aborto a LOST_FWD que había acá
            // (2026-07-10, "detección fantasma") volvía a la línea al
            // instante — reportado en el esquive a IZQUIERDA: "si ve la
            // línea durante el giro se sale del modo esquive" y el robot
            // regresaba directo al obstáculo. Con la entrada por la fase
            // STOP (hold de distancia contra A6/A8 durante segundos) el
            // objeto está confirmado de sobra: fantasma es casi imposible.
            // Si la pared realmente no aparece bordeando, el timeout de
            // pared perdida (5s → reposo) ya cubre ese caso sin volver
            // hacia el obstáculo.
            line_state = LINE_STATE_OBJ_PAUSA_GIRO;
            // Mismo freno de continuidad que la salida de fase 1.
            line_pivot_active = 1;
            motorRightVelocity = (int16_t)clampf_local(
                -(pwm_sat - LINE_ROT_BRAKE * rot_dir), -60.0f, 60.0f);
            motorLeftVelocity  = (int16_t)clampf_local(
                -(pwm_sat + LINE_ROT_BRAKE * rot_dir), -60.0f, 60.0f);
        } else {
            float pivot = LINE_ROT_PIVOT * 0.5f * rot_dir;   // suave: es un retoque
            line_pivot_active = 1;
            motorRightVelocity = (int16_t)clampf_local(
                -(pwm_sat + pivot), -60.0f, 60.0f);
            motorLeftVelocity  = (int16_t)clampf_local(
                -(pwm_sat - pivot), -60.0f, 60.0f);
        }
    }
}

// OBJ_PAUSA_GIRO: balance estático 2s con hold de posición, luego a bordear.
static void LineState_ObjPausaGiro(void)
{
    // Balance estático 2s, luego pasa directo a OBJ_BORDEAR_PARED
    // (se prueba saltear OBJ_BUSCAR_PARED, ver Registro de Cambios 2026-07-05).
    steering_adjustment = 0.0f;
    if (obj_hold_start_ms == 0) {
        obj_hold_start_ms = HAL_GetTick();
        // Ancla del hold de posición (ver setpoint de PAUSA_GIRO).
        __disable_irq();
        obj_pausa_r0 = encoder_right;
        obj_pausa_l0 = encoder_left;
        __enable_irq();
    }
    if (f_fallen) {
        line_state        = LINE_STATE_FOLLOWING;
        line_integral     = 0.0f;
        line_error_prev   = 0.0f;
        line_error_f_d    = 0.0f;
        line_lost_ms      = HAL_GetTick();
        obj_hold_start_ms = 0;
        obj_detect_ignore_until_ms = HAL_GetTick() + 5000U;
    } else if ((HAL_GetTick() - obj_hold_start_ms) >= OBJ_HOLD_DURATION_MS) {
        line_state                 = LINE_STATE_OBJ_BORDEAR_PARED;
        obj_hold_start_ms          = 0;
        obj_wall_fwd_start_ms      = HAL_GetTick();
        obj_wall_seq_start_ms      = HAL_GetTick();
        obj_wall_vel_integral      = 0.0f;
    }
}

// OBJ_BUSCAR_PARED: avanza despacio hasta ver la pared en el lateral activo.
static void LineState_ObjBuscarPared(void)
{
    // Avanza despacio (PI de velocidad) hasta que el lateral activo detecta la pared.
    // Timeout OBJ_WALL_APPROACH_TIMEOUT → vuelve a FOLLOWING si no encuentra pared.
    uint8_t wall_found = ((float)adcAvg[OBJ_WALL_ADC_IDX] < OBJ_WALL_THRESHOLD);
    if (f_fallen) {
        line_state                 = LINE_STATE_FOLLOWING;
        obj_wall_approach_start_ms = 0;
        obj_detect_ignore_until_ms = HAL_GetTick() + 5000U;
    } else if (wall_found) {
        line_state                 = LINE_STATE_OBJ_BORDEAR_PARED;
        obj_wall_approach_start_ms = 0;
        obj_wall_fwd_start_ms      = HAL_GetTick();
        obj_wall_seq_start_ms      = HAL_GetTick();
        obj_wall_vel_integral      = 0.0f;
    } else if ((HAL_GetTick() - obj_wall_approach_start_ms) >= OBJ_WALL_APPROACH_TIMEOUT) {
        line_state                 = LINE_STATE_OBJ_BORDEAR_PARED;
        obj_wall_approach_start_ms = 0;
        obj_wall_fwd_start_ms      = HAL_GetTick();
        obj_wall_seq_start_ms      = HAL_GetTick();
        obj_wall_vel_integral      = 0.0f;
    }
    // Mientras avanza: steering neutro, setpoint manejado por bloque de setpoints.
    steering_adjustment = 0.0f;
}

// OBJ_BORDEAR_PARED: wall-following — avanza mientras el lateral ve la pared.
static void LineState_ObjBordearPared(void)
{
    // Wall-following: avanza mientras el lateral activo ve el objeto.
    // Si pierde el objeto → pivot hacia la pared. Si ve línea (tras 3s) → FOLLOWING.
    // Lateral < REVERSE_THOLD → reversa pareja. Entre REVERSE_THOLD y
    // TOO_CLOSE_THOLD → pivot alejándose de la pared.
    if (obj_wall_fwd_start_ms == 0)
        obj_wall_fwd_start_ms = HAL_GetTick();
    // Ignora línea solo los primeros OBJ_WALL_LINE_IGNORE_MS desde que
    // se entró a la secuencia de wall-following (obj_wall_seq_start_ms,
    // fijo, no se resetea al ciclar FWD/CLEAR/TURN) -- antes se usaba
    // obj_wall_fwd_start_ms, que se reseteaba cada vez que se volvía a
    // este estado desde GIRO_PARED, dejando la línea ignorada indefinidamente
    // mientras el robot oscilaba entre bordear/girar buscando la pared.
    uint8_t line_ignore   = ((HAL_GetTick() - obj_wall_seq_start_ms) < OBJ_WALL_LINE_IGNORE_MS);
    float wall_adc        = (float)adcAvg[OBJ_WALL_ADC_IDX];
    uint8_t wall_visible  = (wall_adc < OBJ_WALL_THRESHOLD);
    uint8_t wall_reverse  = obj_wall_rev_latch;   // por counts (2026-07-10)
    uint8_t too_close     = (!wall_reverse) && (wall_adc < OBJ_WALL_TOO_CLOSE_THOLD);
    if (ObjWall_LineConfirmed(line_detected, line_ignore)) {
        line_seen_since_entry = 1;
        obj_wall_line_confirm_cnt = 0;
        obj_final_turn_pending_ms = HAL_GetTick();
        line_state          = LINE_STATE_FOLLOWING;
        line_integral       = 0.0f;
        line_error_prev     = 0.0f;
        line_error_f_d      = 0.0f;
        line_lost_ms        = HAL_GetTick();
        steering_adjustment = 0.0f;
        obj_wall_fwd_start_ms = 0;
        obj_wall_seq_start_ms = 0;
        obj_detect_ignore_until_ms = HAL_GetTick() + 5000U;
    } else if (wall_reverse) {
        // Demasiado cerca: reversa pareja (mismo P de rumbo recto,
        // ver REV_STRAIGHT_KP/MAX/SLEW). line_pivot_active queda en 0: el
        // formato compartido de motores aplica half_steer=steering_adjustment*0.5.
        float rate_diff = speed_right_rps_s - speed_left_rps_s;
        float corr_target = clampf_local(
            REV_STRAIGHT_KP * rate_diff,
            -REV_STRAIGHT_MAX, REV_STRAIGHT_MAX
        );
        float corr_delta = corr_target - obj_rev_steer_f;
        if (corr_delta >  REV_STRAIGHT_SLEW) corr_delta =  REV_STRAIGHT_SLEW;
        if (corr_delta < -REV_STRAIGHT_SLEW) corr_delta = -REV_STRAIGHT_SLEW;
        obj_rev_steer_f += corr_delta;
        steering_adjustment = obj_rev_steer_f;
    } else if (too_close) {
        // Demasiado cerca: pivot alejándose de la pared (según sentido)
        line_pivot_active  = 1;
        motorRightVelocity = (int16_t)clampf_local(
            -(pwm_sat + OBJ_WALL_PIVOT_POWER * (float)obj_esquive_dir), -30.0f, 30.0f);
        motorLeftVelocity  = (int16_t)clampf_local(
            -(pwm_sat - OBJ_WALL_PIVOT_POWER * (float)obj_esquive_dir), -30.0f, 30.0f);
    } else if (!wall_visible) {
        // Perdió la pared: en vez de girar ya mismo, avanza un poco más
        // (OBJ_WALL_CLEAR_COUNTS) para no quedar pivoteando pegado a la
        // esquina/pared.
        line_state                 = LINE_STATE_OBJ_PARED_LIBRE;
        steering_adjustment        = 0.0f;
        obj_wall_fwd_start_ms      = 0;
        obj_wall_clear_initialized = 0;
        obj_wall_lost_ms           = HAL_GetTick();  // arranca la ventana de 2s de ignorar línea en WALL_CLEAR/WALL_TURN
    } else {
        // Avanza hacia adelante con ángulo fijo; yaw-lock se aplica al calcular motores.
        steering_adjustment = 0.0f;
        obj_rev_steer_f      = 0.0f;
    }
}

// OBJ_PARED_LIBRE: perdió la pared — avanza unos counts más antes de girar.
static void LineState_ObjParedLibre(void)
{
    // Avanza OBJ_WALL_CLEAR_COUNTS counts de encoder más después de perder
    // la pared en WALL_FWD, antes de girar -- para no quedar pivoteando
    // pegado a la esquina/pared. Misma reversa/pivot de seguridad que
    // WALL_FWD si en el medio se acerca demasiado.
    if (!obj_wall_clear_initialized) {
        __disable_irq();
        obj_wall_clear_r0 = encoder_right;
        obj_wall_clear_l0 = encoder_left;
        __enable_irq();
        obj_wall_clear_initialized = 1;
    }
    __disable_irq();
    int32_t clear_dr = encoder_right - obj_wall_clear_r0;
    int32_t clear_dl = encoder_left  - obj_wall_clear_l0;
    __enable_irq();
    int32_t clear_counts = (abs(clear_dr) + abs(clear_dl)) / 2;

    float wall_adc       = (float)adcAvg[OBJ_WALL_ADC_IDX];
    uint8_t wall_reverse = obj_wall_rev_latch;   // por counts (2026-07-10)
    uint8_t too_close    = (!wall_reverse) && (wall_adc < OBJ_WALL_TOO_CLOSE_THOLD);
    // Misma ventana que BORDEAR_PARED, medida desde la entrada a TODA
    // la secuencia (obj_wall_seq_start_ms) — ver fix 2026-07-05.
    uint8_t line_ignore2 = ((HAL_GetTick() - obj_wall_seq_start_ms) < OBJ_WALL_LINE_IGNORE_MS);

    if (ObjWall_LineConfirmed(line_detected, line_ignore2)) {
        line_seen_since_entry = 1;
        obj_wall_line_confirm_cnt = 0;
        obj_final_turn_pending_ms = HAL_GetTick();
        line_state          = LINE_STATE_FOLLOWING;
        line_integral       = 0.0f;
        line_error_prev     = 0.0f;
        line_error_f_d      = 0.0f;
        line_lost_ms        = HAL_GetTick();
        steering_adjustment = 0.0f;
        obj_wall_clear_initialized = 0;
        obj_wall_seq_start_ms = 0;
        obj_detect_ignore_until_ms = HAL_GetTick() + 5000U;
    } else if (wall_reverse) {
        float rate_diff = speed_right_rps_s - speed_left_rps_s;
        float corr_target = clampf_local(
            REV_STRAIGHT_KP * rate_diff,
            -REV_STRAIGHT_MAX, REV_STRAIGHT_MAX
        );
        float corr_delta = corr_target - obj_rev_steer_f;
        if (corr_delta >  REV_STRAIGHT_SLEW) corr_delta =  REV_STRAIGHT_SLEW;
        if (corr_delta < -REV_STRAIGHT_SLEW) corr_delta = -REV_STRAIGHT_SLEW;
        obj_rev_steer_f += corr_delta;
        steering_adjustment = obj_rev_steer_f;
    } else if (too_close) {
        line_pivot_active  = 1;
        motorRightVelocity = (int16_t)clampf_local(
            -(pwm_sat + OBJ_WALL_PIVOT_POWER * (float)obj_esquive_dir), -30.0f, 30.0f);
        motorLeftVelocity  = (int16_t)clampf_local(
            -(pwm_sat - OBJ_WALL_PIVOT_POWER * (float)obj_esquive_dir), -30.0f, 30.0f);
    } else if (clear_counts >= OBJ_WALL_CLEAR_COUNTS) {
        line_state                 = LINE_STATE_OBJ_GIRO_PARED;
        steering_adjustment        = 0.0f;
        obj_wall_clear_initialized = 0;
    } else {
        steering_adjustment = 0.0f;
        obj_rev_steer_f      = 0.0f;
    }
}

// OBJ_GIRO_PARED: pivot hacia la pared hasta re-verla en el lateral activo.
static void LineState_ObjGiroPared(void)
{
    // Pivot hacia la pared hasta re-verla en el lateral activo o detectar línea.
    // Lateral < REVERSE_THOLD → reversa pareja. Entre REVERSE_THOLD y
    // TOO_CLOSE_THOLD → pivot alejándose en cambio.
    float wall_adc       = (float)adcAvg[OBJ_WALL_ADC_IDX];
    uint8_t wall_visible = (wall_adc < OBJ_WALL_THRESHOLD);
    uint8_t wall_reverse = obj_wall_rev_latch;   // por counts (2026-07-10)
    uint8_t too_close    = (!wall_reverse) && (wall_adc < OBJ_WALL_TOO_CLOSE_THOLD);
    // Misma ventana que BORDEAR_PARED/PARED_LIBRE, medida desde la
    // entrada a toda la secuencia (obj_wall_seq_start_ms).
    uint8_t line_ignore2 = ((HAL_GetTick() - obj_wall_seq_start_ms) < OBJ_WALL_LINE_IGNORE_MS);
    if (ObjWall_LineConfirmed(line_detected, line_ignore2)) {
        line_seen_since_entry = 1;
        obj_wall_line_confirm_cnt = 0;
        obj_final_turn_pending_ms = HAL_GetTick();
        line_state          = LINE_STATE_FOLLOWING;
        line_integral       = 0.0f;
        line_error_prev     = 0.0f;
        line_error_f_d      = 0.0f;
        line_lost_ms        = HAL_GetTick();
        steering_adjustment = 0.0f;
        line_pivot_active   = 0;
        obj_wall_seq_start_ms = 0;
        obj_detect_ignore_until_ms = HAL_GetTick() + 5000U;
    } else if (wall_reverse) {
        // Demasiado cerca: reversa pareja (mismo P de rumbo recto,
        // ver REV_STRAIGHT_KP/MAX/SLEW). line_pivot_active queda en 0.
        float rate_diff = speed_right_rps_s - speed_left_rps_s;
        float corr_target = clampf_local(
            REV_STRAIGHT_KP * rate_diff,
            -REV_STRAIGHT_MAX, REV_STRAIGHT_MAX
        );
        float corr_delta = corr_target - obj_rev_steer_f;
        if (corr_delta >  REV_STRAIGHT_SLEW) corr_delta =  REV_STRAIGHT_SLEW;
        if (corr_delta < -REV_STRAIGHT_SLEW) corr_delta = -REV_STRAIGHT_SLEW;
        obj_rev_steer_f += corr_delta;
        steering_adjustment = obj_rev_steer_f;
    } else if (too_close) {
        // Demasiado cerca: pivot alejándose de la pared (según sentido)
        line_pivot_active  = 1;
        motorRightVelocity = (int16_t)clampf_local(
            -(pwm_sat + OBJ_WALL_PIVOT_POWER * (float)obj_esquive_dir), -30.0f, 30.0f);
        motorLeftVelocity  = (int16_t)clampf_local(
            -(pwm_sat - OBJ_WALL_PIVOT_POWER * (float)obj_esquive_dir), -30.0f, 30.0f);
    } else if (wall_visible) {
        line_state            = LINE_STATE_OBJ_BORDEAR_PARED;
        line_pivot_active     = 0;
        obj_wall_fwd_start_ms = HAL_GetTick();
        obj_wall_vel_integral = 0.0f;
    } else {
        // Pivot hacia el lado de la pared (opuesto al giro de esquive):
        // izquierda si se esquivó a derecha, y viceversa.
        line_pivot_active  = 1;
        motorRightVelocity = (int16_t)clampf_local(
            -(pwm_sat - OBJ_WALL_PIVOT_POWER * (float)obj_esquive_dir), -30.0f, 30.0f);
        motorLeftVelocity  = (int16_t)clampf_local(
            -(pwm_sat + OBJ_WALL_PIVOT_POWER * (float)obj_esquive_dir), -30.0f, 30.0f);
    }
}

// ─────────────────────────────────────────────────────────────────────
// Etapa 15a — Motores en modo LINEA: despacha el handler del sub-estado
// activo y aplica el formato común de motores (pwm_sat ± steering) si
// ningún handler tomó control directo de las ruedas (line_pivot_active).
// ─────────────────────────────────────────────────────────────────────
static void Ctrl_MotoresLinea(void)
{
    line_pivot_active = 0;

    switch (line_state) {
        case LINE_STATE_FOLLOWING:         LineState_Following();       break;
        case LINE_STATE_LOST:
        case LINE_STATE_SEARCHING:         LineState_Lost();            break;
        case LINE_STATE_LOST_BRAKE:        LineState_LostBrake();       break;
        case LINE_STATE_LOST_ROTATE:       LineState_LostRotate();      break;
        case LINE_STATE_LOST_SETTLE:       LineState_LostSettle();      break;
        case LINE_STATE_LOST_FWD:          LineState_LostFwd();         break;
        case LINE_STATE_EDGE_WAIT:         LineState_EdgeWait();        break;
        case LINE_STATE_EDGE_ROTATE:       LineState_EdgeRotate();      break;
        case LINE_STATE_EDGE_SETTLE:       LineState_EdgeSettle();      break;
        case LINE_STATE_EDGE_FWD:          LineState_EdgeFwd();         break;
        case LINE_STATE_GIVEN_UP:          LineState_GivenUp();         break;
        case LINE_STATE_PERP_ROTATE:       LineState_PerpRotate();      break;
        case LINE_STATE_OBJ_FRENO_REVERSA: LineState_ObjFrenoReversa(); break;
        case LINE_STATE_OBJ_GIRO_ESQUIVE:  LineState_ObjGiroEsquive();  break;
        case LINE_STATE_OBJ_PAUSA_GIRO:    LineState_ObjPausaGiro();    break;
        case LINE_STATE_OBJ_BUSCAR_PARED:  LineState_ObjBuscarPared();  break;
        case LINE_STATE_OBJ_BORDEAR_PARED: LineState_ObjBordearPared(); break;
        case LINE_STATE_OBJ_PARED_LIBRE:   LineState_ObjParedLibre();   break;
        case LINE_STATE_OBJ_GIRO_PARED:    LineState_ObjGiroPared();    break;
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

        // En LOST/SEARCHING/OBJ_HOLD/LOST_FWD/EDGE_FWD/WALL_FWD/WALL_CLEAR (solo
        // cuando avanzan derecho, steering_adjustment==0): corrección yaw igual
        // que BALANCE_ONLY. Cuando esos mismos estados están corrigiendo una
        // reversa (steering_adjustment != 0, ver los cases), se respeta esa
        // corrección en vez de pisarla con el término de gyro -- antes este
        // override se aplicaba siempre que el line_state coincidiera, sin
        // importar si había una corrección real activa, y la anulaba en
        // silencio. OBJ_ROTATE usa steering_adjustment directo (ver el case).
        if (steering_adjustment == 0.0f &&
            (line_state == LINE_STATE_LOST     ||
             line_state == LINE_STATE_SEARCHING ||
             line_state == LINE_STATE_OBJ_PAUSA_GIRO ||
             line_state == LINE_STATE_OBJ_BUSCAR_PARED ||
             line_state == LINE_STATE_OBJ_BORDEAR_PARED ||
             line_state == LINE_STATE_OBJ_PARED_LIBRE ||
             line_state == LINE_STATE_LOST_FWD ||
             line_state == LINE_STATE_EDGE_FWD)) {
            // 0.23 = 0.3*(100/131): misma restauración de ganancia efectiva
            // que la corrección de yaw de balance común (ver ahí). gz crudo
            // con zona muerta (sin filtro — ver GZ_YAW_ASSIST_DB).
            half_steer = apply_deadbandf((float)gz / 100.0f, GZ_YAW_ASSIST_DB) * 0.23f;
        }

        float mR = pwm_sat - half_steer;
        float mL = pwm_sat + half_steer;

        if (mR >  40.0f) mR =  40.0f;
        if (mR < -40.0f) mR = -40.0f;
        if (mL >  40.0f) mL =  40.0f;
        if (mL < -40.0f) mL = -40.0f;

        motorRightVelocity = -(int16_t)lroundf(mL);
        motorLeftVelocity  = -(int16_t)lroundf(mR);
    }
}

// ─────────────────────────────────────────────────────────────────────
// Etapa 15b — Motores en modo MANUAL (o override manual durante LINEA):
// giro rampeado o rumbo recto por encoders, mezclados con el PWM común.
// ─────────────────────────────────────────────────────────────────────
static void Ctrl_MotoresManual(void)
{
    line_integral       = 0.0f;
    line_error_prev     = 0.0f;
    line_state          = LINE_STATE_FOLLOWING;

    // Control manual normal (comandos por WiFi/USB). El banco de pruebas
    // del giro de 90° que se activaba con el joystick/comandos en reposo
    // se eliminó a pedido del usuario: MANUAL ahora solo mueve el robot en
    // respuesta a comandos, sin ningún ciclo automático de rotación. También
    // se entra acá con manual_line_override=1 (LINE_FOLLOWING sin ver la
    // línea + comando activo) — mismo control, reutilizado tal cual; el
    // `line_state = LINE_STATE_FOLLOWING` de arriba mantiene la máquina de
    // estados del seguidor limpia mientras dura el override.
    uint8_t manual_turning = fabsf(manual_steering_cmd) > 0.5f;

    if (manual_turning) {
        // Giro: rampa suave hacia manual_steering_cmd. Bajado a 1/4 de la
        // rampa anterior (0.25→0.0625/ciclo) porque, aun con la magnitud del
        // comando también reducida a 1/4 (±60→±15 en MOVE_LEFT/RIGHT,
        // UNER.c), el giro seguía sintiéndose demasiado fuerte — "siempre
        // movimientos suaves".
        const float STEER_RATE = 0.0625f;
        float steer_delta = manual_steering_cmd - steering_adjustment;
        if (steer_delta >  STEER_RATE) steer_delta =  STEER_RATE;
        if (steer_delta < -STEER_RATE) steer_delta = -STEER_RATE;
        steering_adjustment += steer_delta;
        manual_straight_steer_f = steering_adjustment; // sincronizado para la transición a "sin giro"
    } else {
        // Sin giro comandado: corrección de rumbo recto para que
        // adelante/atrás no se curve por asimetría mecánica entre ruedas —
        // mismo algoritmo P sobre diferencia de velocidad de ruedas ya
        // usado y calibrado en la reversa recta de OBJ_REVERSE/wall-following
        // (ver REV_STRAIGHT_KP/MAX/SLEW), reutilizado tal cual.
        float rate_diff = speed_right_rps_s - speed_left_rps_s;
        float corr_target = clampf_local(
            REV_STRAIGHT_KP * rate_diff,
            -REV_STRAIGHT_MAX, REV_STRAIGHT_MAX
        );
        float corr_delta = corr_target - manual_straight_steer_f;
        if (corr_delta >  REV_STRAIGHT_SLEW) corr_delta =  REV_STRAIGHT_SLEW;
        if (corr_delta < -REV_STRAIGHT_SLEW) corr_delta = -REV_STRAIGHT_SLEW;
        manual_straight_steer_f += corr_delta;
        steering_adjustment = manual_straight_steer_f;
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

    motorRightVelocity = -(int16_t)lroundf(mL);
    motorLeftVelocity  = -(int16_t)lroundf(mR);
}

// ─────────────────────────────────────────────────────────────────────
// Etapa 15c — Motores en BALANCE/IDLE-activo: PWM común ± corrección de
// rumbo (yaw-assist por gyro Z, o el steering PID de lazo cerrado).
// ─────────────────────────────────────────────────────────────────────
static void Ctrl_MotoresBalance(void)
{
    line_integral       = 0.0f;
    line_error_prev     = 0.0f;
    steering_adjustment = 0.0f;
    line_state          = LINE_STATE_FOLLOWING;

    // steer_pid_enabled=0: corrección open-loop por giroscopio Z (comportamiento original)
    // steer_pid_enabled=1: corrección de lazo cerrado por encoders
    // Ganancia 0.23 = 0.3*(100/131): restaura la ganancia EFECTIVA que estaba
    // calibrada antes del fix de escala del gyro (2026-07-04). gz crudo con
    // zona muerta (sin filtro por software — ver GZ_YAW_ASSIST_DB): giros
    // chicos por ruido no generan corrección ni disparan la compensación
    // de deadband de motor.
    float correction = steer_pid_enabled
                     ? steer_correction
                     : (-apply_deadbandf((float)gz / 100.0f, GZ_YAW_ASSIST_DB) * 0.23f);

    float mR = pwm_sat + correction;
    float mL = pwm_sat - correction;

    if (mR >  100.0f) mR =  100.0f;
    if (mR < -100.0f) mR = -100.0f;
    if (mL >  100.0f) mL =  100.0f;
    if (mL < -100.0f) mL = -100.0f;

    motorRightVelocity = -(int16_t)lroundf(mL);
    motorLeftVelocity  = -(int16_t)lroundf(mR);
}

// ─────────────────────────────────────────────────────────────────────
// Etapa 15 — Mezcla de motores: elige la rama según el modo. Caído no se
// toca nada (la salida final ya corta los motores). En MOTOR_TEST el
// comando SETMOTORSPEED controla directo y acá no se pisa.
// ─────────────────────────────────────────────────────────────────────
static void Ctrl_MezclaMotores(void)
{
    if (f_fallen) return;

    if (robot_state == ROBOT_STATE_LINE_FOLLOWING && !manual_line_override) {
        Ctrl_MotoresLinea();
    } else if (robot_state == ROBOT_STATE_MANUAL_CONTROL || manual_line_override) {
        Ctrl_MotoresManual();
    } else if (robot_state != ROBOT_STATE_MOTOR_TEST) {
        Ctrl_MotoresBalance();
    }
}

// ─────────────────────────────────────────────────────────────────────
// Etapa 16 — Telemetría: log binario WiFi, push de odometría y CSV USB.
// ─────────────────────────────────────────────────────────────────────
static void Ctrl_Telemetria(void)
{
    roll_deg = filtered_roll_deg;

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
        wlog.adc1                = adcAvg[0];
        wlog.adc2                = adcAvg[1];
        wlog.adc3                = adcAvg[2];
        wlog.adc4                = adcAvg[3];

        UNER_SendWifiLogData(&wlog);
    }

    // Push de odometría/línea por WiFi para graficar (mapa XY, franja de línea) en
    // Qt: independiente de ACTIVATE_WIFI_LOG, arranca solo con f_wifi_connected y a
    // un ritmo bajo (WIFI_ODOM_PERIOD_MS) para no competir por ancho de banda/CPU
    // con la telemetría de control ya existente.
    if (f_wifi_connected && (uint32_t)(HAL_GetTick() - last_wifi_odom_ms) >= WIFI_ODOM_PERIOD_MS) {
        last_wifi_odom_ms = HAL_GetTick();

        WifiOdomData_t odata;
        odata.seq           = wifi_odom_seq++;
        odata.t_ms          = HAL_GetTick();
        odata.x_m           = odom_x_m;
        odata.y_m           = odom_y_m;
        odata.theta_deg     = odom_theta_deg;
        odata.line_error    = line_error_disp;
        odata.line_detected = line_detected_disp;
        odata.robot_state   = robot_state;
        odata.line_state    = (uint8_t)line_state;
        odata.adc5          = adcAvg[4];  // sensores de objeto: menos = más cerca,
        odata.adc6          = adcAvg[5];  // ~4095 = nada adelante — para graficar
        odata.adc7          = adcAvg[6];  // la barrera/cuerpo frente al robot en Qt
        odata.adc8          = adcAvg[7];
        odata.roll_deg      = filtered_roll_deg;  // balanceo → Vista 3D de Qt
        odata.lat_deg       = lat_tilt_f;          // banking lateral → Vista 3D de Qt

        UNER_SendWifiOdomData(&odata);
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
}

// ─────────────────────────────────────────────────────────────────────
// Etapa 17 — Salida a motores: paro por "en el aire", compensación de
// zona muerta (estática/cinética según la rueda gire o no) y MotorControl.
// ─────────────────────────────────────────────────────────────────────
static void Ctrl_SalidaMotores(void)
{
    // En el aire >2s: detener motores hasta que los sensores de línea vean superficie.
    if (robot_state == ROBOT_STATE_LINE_FOLLOWING && f_in_air) {
        motorRightVelocity = 0;
        motorLeftVelocity  = 0;
    }

    if (robot_state == ROBOT_STATE_MOTOR_TEST) {
        // Modo test: SETMOTORSPEED controla directamente, sin PID ni compensación
        MotorControl(motorRightVelocity, -motorLeftVelocity);
    } else if ((robot_state != ROBOT_STATE_IDLE) && !f_fallen) {
        // Zona neutra: comandos chicos van a 0 en vez de saltar a ±DEADBAND
        // (ver MOTOR_CMD_NEUTRAL — anti-temblor cerca del equilibrio).
        // Deadband adaptativo (2026-07-10): offset estático completo solo con la
        // rueda realmente parada (sin ticks de encoder hace WHEEL_MOVING_WINDOW_MS);
        // girando, offset cinético menor — mata las patadas de ±11 PWM en cada
        // cruce por la zona neutra que sostenían el bamboleo (relé sobre planta
        // inestable). Ver comentario en los defines MOTOR_DEADBAND_KINETIC.
        uint32_t now_db = HAL_GetTick();
        int16_t dbR = (now_db - wheel_r_last_tick_ms < WHEEL_MOVING_WINDOW_MS)
                    ? MOTOR_DEADBAND_KINETIC : MOTOR_RIGHT_DEADBAND;
        int16_t dbL = (now_db - wheel_l_last_tick_ms < WHEEL_MOVING_WINDOW_MS)
                    ? MOTOR_DEADBAND_KINETIC : MOTOR_LEFT_DEADBAND;

        int16_t mR_comp = motorRightVelocity;
        if (mR_comp >= -MOTOR_CMD_NEUTRAL && mR_comp <= MOTOR_CMD_NEUTRAL) mR_comp = 0;
        if      (mR_comp > 0)  mR_comp = (int16_t)( mR_comp + dbR);
        else if (mR_comp < 0)  mR_comp = (int16_t)( mR_comp - dbR);
        if (mR_comp >  100) mR_comp =  100;
        if (mR_comp < -100) mR_comp = -100;

        int16_t mL_comp = -motorLeftVelocity;
        if (mL_comp >= -MOTOR_CMD_NEUTRAL && mL_comp <= MOTOR_CMD_NEUTRAL) mL_comp = 0;
        if      (mL_comp > 0)  mL_comp = (int16_t)( mL_comp + dbL);
        else if (mL_comp < 0)  mL_comp = (int16_t)( mL_comp - dbL);
        if (mL_comp >  100) mL_comp =  100;
        if (mL_comp < -100) mL_comp = -100;

        MotorControl(mR_comp, mL_comp);
    } else {
        MotorControl(0, 0);
    }
}

// ─────────────────────────────────────────────────────────────────────
// ORQUESTADOR — un ciclo completo de control (100 Hz, llamado por TIM1).
// El orden de las etapas es el contrato: cada una consume lo que dejaron
// las anteriores en las variables de ciclo.
// ─────────────────────────────────────────────────────────────────────
static void ControlStep10ms(void)
{
    if (!Ctrl_LeerIMU())       return;   // sin dato nuevo del MPU: el ciclo no corre
    Ctrl_VelocidadEncoders();
    Ctrl_LatchesPared();
    Ctrl_Odometria();
    Ctrl_SteeringLazoCerrado();
    if (!Ctrl_TimingDt())      return;   // primer ciclo: todavía no hay dt válido
    Ctrl_FiltroIMU();
    Ctrl_CuarentenaLinea();
    Ctrl_EntradasLinea();
    Ctrl_OverrideManualLinea();
    Ctrl_CambiosDeEstado();
    Ctrl_SetpointDinamico();
    Ctrl_DeteccionCaida();
    Ctrl_PIDBalance();
    Ctrl_MezclaMotores();
    Ctrl_Telemetria();
    Ctrl_SalidaMotores();
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

  wifiSSID     = wifiProfiles[WIFI_PROFILE_ACTIVE].ssid;
  wifiPassword = wifiProfiles[WIFI_PROFILE_ACTIVE].password;
  wifiIp       = wifiProfiles[WIFI_PROFILE_ACTIVE].ip;

  CDC_Attach_Rx(USBRxData);
  HAL_UART_Receive_IT(&huart1, &dataRx, 1);

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcValues, 8) != HAL_OK) {
      Error_Handler();
  }
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
  UNER_Init(&unerRx, &unerTx);
  static const UNER_Bindings_t unerBindings = {
      .adc = adcAvg, .adc_len = 8,
      .motor_right_velocity = &motorRightVelocity,
      .motor_left_velocity = &motorLeftVelocity,
      .ax = &ax, .ay = &ay, .az = &az,
      .gx = &gx, .gy = &gy, .gz = &gz,
      .roll = &roll_deg, .pitch = &pitch_deg,
      .kp = &KP_value, .kd = &KD_value, .ki = &KI_value,
      .kv_brake = &KV_brake_value,
      .steering = &steering_adjustment,
      .robot_state = &robot_state,
      .reset_mass_center = &f_resetMassCenter,
      .send_csv_log = &f_send_csv_log,
      .send_wifi_log = &f_send_wifi_log,
      .change_display = &f_change_display,
      .kp_line = &KP_LINE, .kd_line = &KD_LINE, .ki_line = &KI_LINE,
      .line_threshold = &LINE_THRESHOLD, .line_speed = &LINE_SPEED_TARGET,
      .manual_setpoint = &manual_setpoint_cmd,
      .manual_steering = &manual_steering_cmd,
      .manual_timeout_ms = &manual_cmd_last_ms,
      // rotation_target_deg/rotation_trigger quedan NULL: comandos de giro remoto no-op.
      .odom_x = &odom_x_m, .odom_y = &odom_y_m, .odom_theta = &odom_theta_deg,
      .setpoint_trim = &setpoint_trim,
  };
  UNER_RegisterBindings(&unerBindings);

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

	  if (I2C_Manager_IsProcessPending()) {
	      I2C_Manager_Process();
	  }

	  if(is10ms) {
	      is10ms = 0;

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
	                      f_change_display = (f_change_display + 1) % 8;  // 8 pantallas (6=OBJ, 7=ODOM)
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

	  if (I2C_Manager_IsProcessPending()) {
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
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
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
