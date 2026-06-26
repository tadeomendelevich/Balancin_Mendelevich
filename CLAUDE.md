# Balancin STM32 — Memoria del Proyecto
> Este archivo es la memoria persistente del firmware para Claude Code.
> Mantenerlo actualizado al final de cada sesión de trabajo.

---

## 🤖 Instrucciones Permanentes para Claude

**Estas reglas aplican en cada sesión, automáticamente y sin que te lo pida:**

### Al modificar cualquier archivo de código:
- Agregá una fila en "Registro de Cambios" con fecha actual (YYYY-MM-DD),
  archivo(s) tocado(s), qué cambió y por qué
- Si el cambio afecta parámetros PID, protocolo UART/UDP u otra sección
  del CLAUDE.md → actualizá esa sección también
- Si resolviste un bug → pasalo de "Pendientes" a "Funcionalidades completas"
- Si apareció un bug nuevo → agregalo en "Pendientes / bugs conocidos"
- Si tomaste una decisión de diseño importante → agregala en "Decisiones de Diseño"

### Al iniciar sesión:
- Leé este CLAUDE.md completo antes de hacer cualquier cosa
- Usalo como contexto del proyecto, no preguntes lo que ya está documentado acá

### Si un cambio afecta al otro proyecto (Qt ↔ STM32):
- Avisame explícitamente qué hay que cambiar en el otro proyecto
- Indicá el archivo exacto que necesita modificación

**Nunca rompas la estructura de este CLAUDE.md.**

---

## Descripción General
Firmware en **STM32CubeIDE** para el péndulo invertido "Balancín Mendelevich".
Implementa el control PID de estabilización, la lectura del IMU, el manejo de motores
y la comunicación bidireccional con la interfaz Qt (USB CDC + WiFi UDP).

---

## Rutas del Proyecto
| Proyecto | Ruta |
|----------|------|
| STM32 (este) | `C:\Users\tadeo\STM32CubeIDE\workspace_1.18.1\Balancin_Mendelevich` |
| Qt (interfaz) | `C:\Microcontroladores\BalancinQT` |

---

## Hardware
| Componente | Modelo | Descripción |
|------------|--------|-------------|
| Microcontrolador | STM32F411CEU6 (UFQFPN48) | MCU principal, 96 MHz (HSE 25 MHz + PLL) |
| IMU / Giroscopio | MPU-6050 | 6 ejes accel+gyro, I2C Fast, lectura DMA de 14 bytes, DLPF ~44 Hz |
| Driver de motores | Sin denominación en código (PWM directo) | Control PWM via TIM3/TIM4, 2 canales cada uno (dirección + velocidad) |
| Módulo WiFi | ESP-01 (ESP8266) | AT commands via USART1 a 115200 baud, UDP socket |
| Display | SSD1306 OLED 128×64 | I2C1, driver no bloqueante con DMA |
| Encoders | Cuadratura (modelo desconocido) | 4x quadrature vía EXTI: PA8/PB13 (derecho), PB14/PB15 (izquierdo). 12 CPR físicos → 24 conteos/rev con ambos canales RISING+FALLING |

---

## Arquitectura de Archivos
```
Balancin_Mendelevich/
├── CLAUDE.md                        ← memoria del proyecto (este archivo)
├── Balancin_Mendelevich.ioc         ← configuración CubeMX (pines, periféricos)
├── Core/
│   ├── Src/
│   │   ├── main.c                   ← loop principal, init, PID, state machine, complementary filter
│   │   ├── MPU6050.c                ← driver IMU MPU-6050 (DMA, fixed-point, calibración/bias)
│   │   ├── ESP01.c                  ← driver WiFi ESP-01 (AT commands, UDP, watchdog)
│   │   ├── UNER.c                   ← protocolo binario UNER (parser RX, encoder TX, 37+ comandos)
│   │   ├── i2c_manager.c            ← gestor I2C no bloqueante con cola (size 8) y DMA
│   │   ├── ssd1306.c                ← driver display OLED SSD1306 no bloqueante
│   │   ├── fonts.c                  ← fuentes bitmap (7x10, 5x7) y logo UNER
│   │   ├── stm32f4xx_hal_msp.c      ← init periféricos HAL (ADC, I2C, TIM, UART)
│   │   ├── stm32f4xx_it.c           ← handlers de interrupción
│   │   ├── system_stm32f4xx.c       ← init sistema y reloj
│   │   ├── sysmem.c                 ← gestión de memoria newlib (_sbrk)
│   │   └── syscalls.c               ← syscalls newlib
│   └── Inc/
│       ├── main.h                   ← defines globales, pin LED (PB10), MPU_INT (PB12)
│       ├── MPU6050.h
│       ├── ESP01.h
│       ├── UNER.h                   ← enum comandos, structs LogData_t / WifiLogData_t
│       ├── i2c_manager.h
│       ├── ssd1306.h
│       ├── fonts.h
│       ├── stm32f4xx_it.h
│       └── stm32f4xx_hal_conf.h
├── USB_DEVICE/
│   ├── App/
│   │   ├── usb_device.c/h
│   │   ├── usbd_cdc_if.c/h          ← interfaz CDC USB (RX → UNER parser, TX → telemetría CSV)
│   │   └── usbd_desc.c/h
│   └── Target/
│       └── usbd_conf.c/h
├── tests/
│   └── test_ESP01.c                 ← tests unitarios del driver ESP-01
├── Drivers/
│   └── ...                          ← HAL STM32F4 V1.28.3, CMSIS
└── Middlewares/
    └── ...                          ← ST USB Device Library (CDC)
```
> Actualizar si se agregan o renombran archivos.

---

## Comunicación con Qt
### Canal 1 — USB CDC
| Campo | Valor |
|-------|-------|
| Periférico STM32 | USB OTG FS (PA11=DM, PA12=DP), clase CDC |
| Baudrate | N/A (USB CDC, velocidad nativa USB FS) |
| Formato de trama RX | Protocolo UNER binario: `"UNER"` + nBytes + `':'` + cmd + payload + checksum |
| Formato de trama TX | Mismo protocolo UNER; telemetría CSV decimada (LOG_DECIM=5 → ~20 Hz a 100 Hz loop) |

### Canal 2 — WiFi UDP
| Campo | Valor |
|-------|-------|
| IP del PC destino (Qt) | **Variable según ubicación** — ver tabla de perfiles abajo |
| Puerto de escucha STM32 (RX) | `30000` (LocalPORT en `ESP01_StartUDP`) |
| Puerto de envío al PC (TX) | `30010` (RemotePORT en `ESP01_StartUDP`) |
| Frecuencia de telemetría | ~10 Hz (LOG_WIFI_DECIM=10 sobre loop de 100 Hz) |
| Formato paquete WiFi | `WifiLogData_t` binario packed: t_ms, roll, output, PID terms, mR, mL, dt, dyn_sp, line data, 4×ADC |

> ⚠️ **La IP del PC destino cambia según la red donde se trabaje.** Antes de flashear, verificar
> que `wifiIp` en `main.c` (línea ~244) coincida con la IP actual del PC con Qt.
> Perfiles disponibles en el código (descomentar el que corresponda):
>
> | Red | SSID | IP del PC |
> |-----|------|-----------|
> | Casa (activo) | `MEGACABLE FIBRA-2.4G-ckd0` | `192.168.100.5` |
> | FCAL / Universidad | `FCAL` | `172.23.205.98` |
> | Delco Mendelevich | `Delco_Mendelevich` | `192.168.1.55` |
> | Wifi Habitaciones | `Wifi Habitaciones` | `192.168.1.48` |
>
> Si agregás una red nueva → añadí una fila acá y un perfil comentado en `main.c`.

---

## Control PID
| Parámetro | Variable en código | Valor actual |
|-----------|-------------------|--------------|
| Kp | `KP` / `KP_value` | `4.0` |
| Ki | `KI` / `KI_value` | `0.1` |
| Kd | `KD` / `KD_value` | `0.12` |
| Setpoint (ángulo °) | `SETPOINT_ANGLE` | `0.0°` (+ `setpoint_trim` ajustable en runtime) |
| Frecuencia de control | TIM1 (Prescaler=9599, Period=99) | **100 Hz** (10 ms/ciclo) |
| Filtro de derivada | Sin filtro explícito en derivada; zona suave (soft-zone) | SOFT_ZONE_ANGLE=1.5°, scale_min=0.15 |
| Integral anti-windup | `I_MAX` | ±100.0 PWM units; decay=0.990 por ciclo |
| Freno por velocidad | `KV_BRAKE` / `KV_BRAKE_STRONG` | 0.0 / 5.0 (umbral BRAKE_VEL_THRESHOLD=1.5) — velocidad calculada por encoders |

**Sensor de ángulo:**
- Fuente: Filtro complementario (α=0.98) entre acelerómetro y giroscopio del MPU-6050
- Eje de control: **roll** (inclinación lateral del péndulo)
- Bias del MPU hardcodeado (`MPU_USE_FIXED_BIAS`): ax=-46, ay=4400, az=1980, gx=-441, gy=-107, gz=-54

**Detección de caída (histéresis):**
- Caída: |roll| > 60°, recuperación: |roll| < 2°
- Boca abajo: |roll| > 120°; zona muerta (motores off): 35°–120°

---

## Periféricos STM32 Usados
| Periférico | Función | Pin(es) | Configuración |
|------------|---------|---------|---------------|
| TIM1 | Interrupción control loop 100 Hz | — (interno) | Prescaler=9599, Period=99 |
| TIM2 | Trigger ADC cada 250 µs (4 kHz) | — (interno) | Prescaler=95, Period=249, TRGO |
| TIM3 | PWM motor (2 canales) | PB4=CH1, PB5=CH2 | Prescaler=0, Period=959 → ~100 kHz |
| TIM4 | PWM motor (2 canales) | PB6=CH1, PB7=CH2 | Prescaler=0, Period=959 → ~100 kHz |
| TIM5 | Interrupción 2 ms (tick auxiliar) | — (interno) | Prescaler=95, Period=1999 |
| I2C1 | IMU MPU-6050 + Display SSD1306 | PB8=SCL, PB9=SDA | Fast mode (400 kHz), DMA RX/TX |
| USART1 | Módulo WiFi ESP-01 (AT commands) | PA9=TX, PA10=RX | 115200 baud, async, IT RX byte a byte |
| USB OTG FS | Comunicación CDC con Qt | PA11=DM, PA12=DP | Device Only, CDC FS |
| ADC1 | 8 canales sensores (línea + analógicos) | PA1–PA7, PB0 | DMA circular, trigger TIM2, 15 ciclos/canal |
| GPIO PB10 | LED_BLINKER | PB10 | Output |
| GPIO PB12 | MPU_INT (EXTI12) | PB12 | Input, interrupción data-ready |
| GPIO PA8 | Encoder derecho canal A (EXTI8) | PA8 | Input pull-up, EXTI RISING+FALLING, EXTI9_5_IRQn prio 5 |
| GPIO PB13 | Encoder derecho canal B (EXTI13) | PB13 | Input pull-up, EXTI RISING+FALLING, EXTI15_10_IRQn |
| GPIO PB14 | Encoder izquierdo canal A (EXTI14) | PB14 | Input pull-up, EXTI RISING+FALLING, EXTI15_10_IRQn |
| GPIO PB15 | Encoder izquierdo canal B (EXTI15) | PB15 | Input pull-up, EXTI RISING+FALLING, EXTI15_10_IRQn |
| GPIO PB2 | CH_PD ESP-01 (enable módulo) | PB2 | Output |
| GPIO PA0 | KEY (botón usuario) | PA0 | Input pull-up |
| GPIO PC13 | LED integrado | PC13 | Output |

> **IMPORTANTE:** No modificar pines sin actualizar el `.ioc` en CubeMX primero.

---

## Estado Actual
- **Etapa:** Casi terminado
- **Última sesión:** 2026-05-08

### Funcionalidades completas ✅
- PID de estabilización (balance) con zona suave y anti-windup
- Lectura IMU MPU-6050 vía DMA con bias hardcodeado (arranque instantáneo)
- Filtro complementario (α=0.98) acelerómetro + giroscopio
- Display OLED SSD1306 no bloqueante (actualización asíncrona via DMA)
- Máquina de estados del robot (IDLE, BALANCE_ONLY, BALANCE_AND_SPEED, LINE_FOLLOWING, MANUAL_CONTROL, MOTOR_TEST)
- Comunicación USB CDC con protocolo UNER binario (37+ comandos)
- Comunicación WiFi UDP via ESP-01 con watchdog y reconexión automática
- Telemetría en tiempo real: CSV por USB (~20 Hz) y binario por WiFi (~10 Hz)
- Tuneo en tiempo real de Kp, Ki, Kd, setpoint, steering desde Qt
- Seguidor de línea con 8 sensores ADC, PID de línea (Kp=8, Kd=2, Ki=0), velocidad en lazo cerrado por encoders y steering directo proporcional al error
- Detección de objetos en modo línea: ADC 5-8 (largo alcance) con debounce 30 ms → pasa a IDLE automáticamente al detectar obstáculo
- Control manual remoto (FORWARD/BACKWARD/LEFT/RIGHT/STOP)
- Freno dinámico por velocidad de encoders (KV_BRAKE_STRONG=5.0, umbral 1.5 m/s)
- Detección de caída y recuperación con histéresis
- Gestor I2C no bloqueante con cola (evita bloquear el loop de control)
- Encoders de cuadratura 4x: PA8/PB13 (derecho), PB14/PB15 (izquierdo) vía EXTI con masking anti-storm en TIM5
- Velocidad real de ruedas desde encoders (reemplaza estimación accel+gyro)

### Cambios recientes (2026-05-27)
- **6 bugs de stale-state corregidos** en la máquina de evasión: `obj_rev_initialized`, `obj_brake_start_ms`, `obj_rot_initialized`/`obj_rot_start_ms`, `line_error_f_d`, `line_obj_rev_vel_integral` — todos causaban saltos prematuros de estado al recuperarse de una caída
- `KV_brake_value` ahora efectivamente usado en `ComputeBrakeSetpointTarget` (antes el slider de Qt no tenía efecto)
- `obj_hold_initialized` (dead code) eliminado
- **Wall-following implementado**: OBJ_ARC reemplazado por `OBJ_WALL_FWD` + `OBJ_WALL_TURN`. Usa ADC7 (`adcAvg[6]`) como sensor lateral. Cadena activa: FOLLOWING → WAIT(1s) → REVERSE(10c) → BRAKE → ROTATE(325c) → HOLD(2s) → WALL_FWD ↔ WALL_TURN → FOLLOWING.
- `OBJ_WALL_FWD`: ángulo fijo `5.5°`, avanza mientras ADC7 < 3750. Si ADC7 < 300 ("demasiado cerca") → pivot derecha. Si ADC7 > 3750 → WALL_TURN.
- `OBJ_WALL_TURN`: pivot izquierda (potencia 20) hasta re-ver ADC7 < 3750. Si ADC7 < 300 → pivot derecha en cambio.
- `OBJ_DETECT_THRESHOLD_VAL` subido de `2000` → `3200` (sensores hardware actualizados con nuevas resistencias, mayor alcance)
- `OBJ_WALL_PIVOT_POWER` = 8.0 (subido de 5 que era insuficiente, bajado de 20 que era demasiado agresivo)
- Línea ignorada durante `OBJ_WALL_LINE_IGNORE_MS=3000ms` al entrar en WALL_FWD (evita volver a FOLLOWING prematuramente al cruzar la línea original)
- `rev_target_counts` reducido de `300` → `10` counts; hold pre-reversa reducido de 2s → 1s

### Pendientes / bugs conocidos 🔧
- El SSID/IP WiFi está hardcodeado en `main.c` (líneas ~244); cambiar manualmente según red
- `USBRxData` tiene el `UNER_PushByte` comentado — los comandos USB desde Qt no se procesan por esa ruta
- Código de debug activo en `ESP01.c` (printfs de estados AT) que genera tráfico USB extra
- **Freeze residual al mover el robot** — Causa raíz identificada (2026-05-18): el DMA del I2C queda en estado "en progreso" indefinidamente sin completar. `mpu_data_ready_for_ctrl` nunca se setea, `ControlStep10ms` nunca corre, `i2c1_tx_busy` queda en 1, display congelado, motores al último PWM. El `while(1)` SIGUE girando (IWDG se patea) pero el sistema es funcionalmente inútil. Fix: watchdog de software en `while(1)` — si pasan >150ms sin dato del MPU se llama `I2C1_Recover()` (9 pulsos SCL + HAL_I2C_Init + I2C_Manager_Init) y se relanza la lectura SIN resetear el MCU. Detectable por "I2C RECOVER\r\n" en USB.
- Resolución de velocidad limitada: mínimo detectable ~0.74 m/s (1 count cada 10ms con ENC_CPR=24). No apto para integración de posición

---

## 📋 Registro de Cambios
> **Instrucción para Claude:** Al finalizar cada sesión, agregar una fila con los cambios realizados.

| Fecha | Archivo(s) modificado(s) | Cambio realizado | Motivo / Observación |
|-------|--------------------------|------------------|----------------------|
| 2026-06-26 | Core/Src/stm32f4xx_it.c | `NMI_Handler`, `BusFault_Handler`, `MemManage_Handler`, `UsageFault_Handler`: agregado `NVIC_SystemReset()` antes del `while(1)`. Antes quedaban en loop infinito → IWDG expiraba después de 2s de freeze. Ahora cualquier fault produce reset inmediato. Causa probable durante acción de motores: caída VDD → glitch HSE → CSS dispara NMI, o EMI PWM → acceso DMA inválido → BusFault. | HardFault ya tenía reset; los otros 4 no → 2s de robot congelado con motores fijos. |
| 2026-06-26 | Core/Src/main.c | `LINE_STATE_LOST_ROTATE` (nuevo estado): cuando el robot pierde la línea y la velocidad de encoders cae por debajo de 0.08 m/s (mínimo 500ms en LOST), hace un giro de 180° derecha con la misma lógica gz+encoder que MANUAL/OBJ_ROTATE. Si detecta la línea durante el giro → FOLLOWING inmediato. Al terminar el giro → vuelve a LOST para re-buscar. Parámetros: ENC_TARGET=760 (380×2), PIVOT=15, BRAKE=5, SLOWDOWN=55°, P0_MAX=2400ms. Display: "LROT". | Robot se quedaba en LOST indefinidamente sin buscar activamente la línea. |
| 2026-06-26 | Core/Src/main.c | `LINE_STATE_OBJ_BRAKE`: `PRE_ROTATE_MS` 1000→2000ms. Pausa más larga post-reversa para que el robot se estabilice antes del giro. | El robot entraba al giro todavía con algo de movimiento residual. |
| 2026-06-26 | Core/Src/main.c | `LINE_STATE_OBJ_ROTATE`: parámetros igualados a MANUAL: `ENC_TARGET` 325→380, `PIVOT` 8→15, `BRAKE` 3→5, `SLOWDOWN_DEG` 35→55, `P0_MAX` 3000→1200ms, `P1_MAX` 500→300ms, clamp motores ±30→±60. Heading compuesto gz+encoder, slowdown ramp, overshoot detection. | Con PIVOT=8 el giro era demasiado suave y no llegaba a 90°. Parámetros idénticos a MANUAL garantizan comportamiento equivalente. |
| 2026-06-26 | Core/Src/main.c | `LINE_STATE_OBJ_REVERSE`: `rev_target_counts` 10→200 (~10 cm). Retroceso re-habilitado para dar espacio antes del giro y no chocar el objeto con la carrocería. | 10 counts era prácticamente nada. |
| 2026-06-25 | Core/Src/main.c | Fix giro izquierda 90°: (1) `phase0_max_ms` reducido 1000→600 ms para 90°: con gz≈0 el fallback ya no gira 1 s entero. (2) `gz_settled`: `gz<12` en fase 1 solo sale DESPUÉS de 300 ms mínimos (antes salía en la 1ra iteración sin frenar). `phase1_max_ms` 400→500 ms. | gz no acumula bien CCW → fase 0 duraba 1 s → fase 1 salía a los 0 ms → 370°+ de inercia sin frenar. |
| 2026-06-25 | Core/Src/main.c | Fix overshoot 180°: `enc_exit_frac = 0.85 - max(0,(angle-90)/90)*0.15` → 90°=0.85 sin cambio, 180°=0.70 entra al freno antes. Auto-ciclo extendido a 3 pasos: 90°R → 90°L → 180°R (% 3). `MANUAL_ROT_ENC_TARGET` calibrado empíricamente a 380 counts/90°. | 180° acumulaba el doble de momentum; entrar a fase 1 al 70% da tiempo de freno suficiente sin afectar los giros de 90°. |
| 2026-06-25 | Core/Src/main.c | Encoder feedback para heading en rotación MANUAL: `enc_heading_deg = avg(|dr|+|dl|)/2 × (90/325)` calculado en cada ciclo. Heading compuesto = `max(gz, enc)`. Usado en: (1) transición fase 0→1 al 55% — ahora funciona para giro izquierda donde gz≈0; (2) slowdown usa `abs_remaining = abs_target - abs_heading` con heading compuesto; (3) salida anticipada de fase 1 cuando `enc >= 95%` del target (mínimo 60 ms). 325 counts = 90° confirmado empíricamente desde OBJ_ROTATE. | Giro izquierda no tenía feedback angular (gz≈0 en CCW) y dependía solo del tiempo (500 ms fallback). Encoders dan precisión simétrica para ambos sentidos. |
| 2026-06-25 | Core/Src/main.c | Fix "va y vuelve" en giro automático MANUAL: `MANUAL_ROT_BRAKE_POWER` 15→5 (freno suave, diferencial 10 PWM en lugar de 30), `phase0_max_ms` 600→400 ms (menos momentum acumulado), `phase1_max_ms` 500→250 ms, mínimo gz_settled 300→120 ms. Causa raíz: con BRAKE=15 y 500 ms de contra-rotación, la fase de freno acumulaba suficiente momentum inverso para revertir el giro completo → el robot volvía a ~0° dando la ilusión de un segundo giro en sentido contrario. Con freno más suave el robot desacelera y se queda en la nueva orientación. | Síntoma: robot giraba derecha y volvía izquierda (overshoot de fase 1), no un segundo trigger del auto-ciclo. |
| 2026-06-25 | Core/Src/main.c | Fix colapso post-giro derecha: fase 1 ahora usa `-(pwm_sat ∓ dir×BRAKE)` en lugar de `±dir×BRAKE` puro. La diferencia entre ambos motores sigue siendo 2×BRAKE_POWER (frena inercia rotacional, constante independiente de pwm_sat), mientras que el modo común `pwm_sat` mantiene el balance PID activo durante los 500 ms de freno. | Con `±dir×BRAKE` puro (sin pwm_sat), el PID de balance calculaba pero no se aplicaba → robot se inclinaba durante el freno → caída al terminar el giro. |
| 2026-06-25 | Core/Src/main.c | Fix secuencia automática de giros: `manual_auto_rot_last_ms` (elapsed desde último trigger) reemplazado por `manual_seq_next_ms` (deadline absoluto). La pausa de 2.5 s ahora se setea como `HAL_GetTick() + 2500` exactamente cuando el giro TERMINA (en ambas ramas de salida: fase 1 normal y timeout). El trigger de respaldo al disparar se setea a `+10000` para que nunca compita con el deadline real. Al entrar a MANUAL_CONTROL se setea `+2500` para dar pausa inicial. | Con la variable elapsed, la pausa se podía medir desde el trigger (no desde el fin), acortando efectivamente la pausa visible entre giros. |
| 2026-06-23 | Core/Src/main.c | Fix giro izquierda modo MANUAL (3ra iteración): fase 1 ahora usa freno puro sin `pwm_sat`. Causa raíz: en la fórmula `motorR = -(pwm_sat + dir*pivot)`, cuando `pwm_sat` es grande, ambos motores terminan con el mismo signo → TIM3/TIM4 ambos en modo "dirección forward" → efecto traslacional puro, sin diferencial de rotación → freno inefectivo. Con `motorR = dir*BRAKE, motorL = -dir*BRAKE` el freno es independiente del balance y siempre produce el diferencial correcto. El robot puede inclinarse levemente durante el freno pero la fase 1 es corta (<1s). |
| 2026-06-23 | Core/Src/main.c | Fix giro izquierda modo MANUAL: (1) `MANUAL_ROT_SLOWDOWN_DEG` 35°→55° — el slowdown nunca se activaba porque la transición fase 0→1 dispara a 49.5° (55% de 90°) pero el slowdown empieza a 90-35=55°, es decir DESPUÉS de la transición. Con 55° empieza a 35°, dentro de la zona activa. (2) Condición de salida de fase 1: `crossed_zero` corregido a `reversed` (la versión anterior era inversa y salía mientras el robot AÚN giraba en la dirección original). Ahora: `reversed = (gz_dps * gz_sign < 0)` = verdadero cuando gz realmente cambió de signo (freno sobrecompensó). Overshoot guard bajado a 120% (era 150%). Salida: `|gz|<12 || overshoot || (reversed && |gz|<40)`. (3) Comentario del ciclo auto actualizado (no hay 180° en el ciclo actual). | `crossed_zero` invertido causaba: derecha=queda corto (salía con robot AÚN girando CW), izquierda=hace 180° (overshoot a 135° + inercia) |
| 2026-06-02 | Core/Src/main.c | Fix sobrepaso de giro preciso: `MANUAL_ROT_PIVOT_POWER` 12→7, `MANUAL_ROT_SLOWDOWN_DEG` 15→35°, mínimo de potencia 0.3→0.0. Agregado `MANUAL_ROT_BRAKE_POWER=5` y `manual_rot_phase`: fase 0=avance (para al 72% del ángulo), fase 1=contra-pivot activo hasta que `gz < 15 dps` y `heading ≤ target + 3°`. Fix balance MANUAL≡BALANCE_ONLY: `pwm_limit` MANUAL 55→100 (anti-windup ahora idéntico), `BRAKE_TILT_STEP_MAN` 0.3→0.5 | Sobrepaso excesivo en giro de 90°; balance MANUAL diferente a BALANCE_ONLY en anti-windup e integral |
| 2026-06-02 | Core/Src/main.c | Giro automático en modo MANUAL: cada 5 s (desde entrada al modo o desde fin del giro anterior) dispara `manual_rot_target_deg=90°` + `manual_rot_trigger=1` si no hay giro activo ni caída. Variable `manual_auto_rot_last_ms` reseteada al entrar en MANUAL y al finalizar cada giro. | Demo/prueba de giros precisos sin intervención desde Qt |
| 2026-06-02 | Core/Inc/UNER.h, Core/Src/UNER.c, Core/Src/main.c | Giro preciso en modo MANUAL: 5 comandos UNER nuevos (`ROTATE_90_RIGHT=0xD5`, `ROTATE_90_LEFT=0xD6`, `ROTATE_180_RIGHT=0xD7`, `ROTATE_180_LEFT=0xD8`, `ROTATE_CUSTOM=0xD9` con float 4 bytes). Sensor primario: giroscopio gz integrado a 100 Hz. Slowdown en los últimos `MANUAL_ROT_SLOWDOWN_DEG=15°` (baja de 100% a 30% de `MANUAL_ROT_PIVOT_POWER=12`). Timeout dinámico proporcional al ángulo. Registro: `UNER_RegisterRotationCmd(&manual_rot_target_deg, &manual_rot_trigger)`. Qt debe informar mismo protocolo | Necesidad de giros precisos desde Qt sin tocar el seguidor de línea |
| 2026-06-02 | Core/Src/main.c | Detección de "puesto en el piso" en modo LINE: si los 4 sensores de línea (`adcAvg[0..3]`) estaban todos por encima de `LINE_THRESHOLD` (robot en el aire) y alguno baja, se activa `obj_detect_ignore_until_ms = HAL_GetTick() + 3000` para ignorar obstáculos 3s. Evita que la mano al soltar el robot dispare la evasión. Variable `prev_all_line_black` (file-scope, init=1) lleva el estado del ciclo anterior | La mano bajo los sensores al soltar el robot disparaba obj_detect constantemente |
| 2026-06-01 | Core/Src/main.c, Core/Src/stm32f4xx_it.c | ADC CH5–CH8 (`adcAvg[4..7]`, sensores de objeto) cambiados a `ADC_SAMPLETIME_144CYCLES` (antes 15 ciclos) para mayor sensibilidad/rango de detección de objetos. Fix crítico 1: CubeMX reseteó prioridades DMA1_S0/S1, DMA2_S0 y EXTI15_10 a 0 → I2C DMA freeze; corregido forzando prioridades en USER CODE BEGIN 2. Fix crítico 2: CubeMX borró `EXTI9_5_IRQHandler` completo y mutiló `EXTI15_10_IRQHandler` (sin manejo de PB13/14/15) → CPU en Default_Handler loop infinito al primer pulso de encoder; restaurados manualmente en stm32f4xx_it.c | Tras cualquier regeneración de CubeMX: `git diff Core/Src/stm32f4xx_it.c` primero |
| 2026-05-28 | Core/Src/main.c | `OBJ_WALL_FWD` dejó de regular velocidad fina con PI: ahora usa `OBJ_WALL_FWD_ANGLE=4.5f` constante mientras `ADC7` está entre `500` y `3750`, frena solo si `velocity_est_f` supera `OBJ_WALL_OVERSPEED_VEL=0.90f`, y aplica corrección yaw con giroscopio igual que hold/lost. `OBJ_WALL_TOO_CLOSE_THOLD` ajustado a `500.0f` | Los encoders son demasiado cuantizados para controlar `0.25 m/s`; el PI alternaba avance/freno y el robot bamboleaba sin avanzar recto |
| 2026-05-28 | Core/Src/main.c | `OBJ_WALL_FWD_ANGLE` aumentado de `2.5f` a `4.5f` | El ángulo máximo anterior era demasiado chico y el robot no avanzaba durante `OBJ_WALL_FWD` |
| 2026-05-27 | Core/Src/main.c | `OBJ_DETECT_THRESHOLD_VAL` cambiado de `2000.0f` → `3200.0f`. El sensor detecta objeto cuando `adcAvg[4..7] < 3200`. Se alargó la distancia de sensado cambiando resistencias en hardware, por lo que el ADC ahora satura antes y el umbral viejo quedaba corto | Hardware actualizado: sensores de objeto ahora responden a mayor distancia |
| 2026-05-27 | Core/Src/main.c | `rev_target_counts` en `LINE_STATE_OBJ_REVERSE` reducido de `300` → `150` counts (exactamente la mitad). Reduce el recorrido de retroceso antes del giro de evasión | Recorrido anterior era excesivo para la nueva distancia de detección; la mitad es suficiente para despejar el obstáculo |
| 2026-05-27 | Core/Src/main.c | OBJ_REVERSE y OBJ_BRAKE deshabilitados temporalmente: `OBJ_PRE_REVERSE_HOLD` ahora transiciona directo a `OBJ_ROTATE` en lugar de `OBJ_REVERSE`. Cadena activa: FOLLOWING → WAIT(2s) → ROTATE → HOLD(2s) → ARC → FOLLOWING. Los estados OBJ_REVERSE y OBJ_BRAKE permanecen en el enum y en el código pero nunca se alcanzan | Simplificar la evasión para probar sin reversa; se puede re-habilitar en una línea |
| 2026-05-27 | Core/Src/main.c | `OBJ_ARC_DIFF_TARGET` corregido de `-0.5f` → `+0.5f`. Validación empírica mostró que la convención `speed_right_rps_s - speed_left_rps_s` tiene signo opuesto al esperado en esta plataforma: positivo produce giro a la izquierda, negativo a la derecha. Comentarios en el código actualizados para reflejar la convención real | El robot giraba a la derecha con `-0.5f`; la corrección de signo lo invierte a la izquierda |
| 2026-05-27 | Core/Src/main.c | `OBJ_ARC_STEER` (open-loop, constante) reemplazado por PI cerrado sobre diferencial de encoders. Nuevo parámetro tunable `OBJ_ARC_DIFF_TARGET` [rps] = diferencial deseado `(speed_right - speed_left)`; positivo = gira izquierda. Ganancias: `OBJ_ARC_KP=15.0`, `OBJ_ARC_KI=3.0`, saturación ±`OBJ_ARC_STEER_MAX=30`. Anti-windup: si satura, deshace el último paso de integral. Variable de estado `obj_arc_steer_int` file-scope, reseteada en caída y al salir del estado | Open-loop era inconsistente: el radio del arco variaba con la batería y asimetrías mecánicas; el PI lo regula midiendo la diferencia real de encoders |
| 2026-05-27 | Core/Src/main.c | Agregado `LINE_STATE_OBJ_ARC`: avanza en arco con PI de diferencial de encoders hasta re-detectar línea → vuelve a FOLLOWING con integrales limpios. OBJ_HOLD modificado: ahora tiene timer `OBJ_HOLD_DURATION_MS=2000ms` y sale a OBJ_ARC (antes se quedaba indefinidamente). `obj_hold_start_ms` file-scope, reseteado en caída. Parámetros: `OBJ_ARC_DIFF_TARGET=0.5f` (izquierda), `OBJ_ARC_ANGLE=1.0°`. Cadena completa: FOLLOWING → WAIT → REVERS → BRAKE → ROTATE → HOLD(2s) → ARC → FOLLOWING | El robot se quedaba en HOLD para siempre sin reincorporarse a la línea; el arco completa la evasión y re-engancha el seguidor |
| 2026-05-27 | Core/Src/main.c | `obj_rev_initialized` ahora se resetea a 0 en el bloque de detección de caída (f_fallen=1). Sin este fix, si el robot caía durante OBJ_REVERSE y se recuperaba, el snapshot de encoder era viejo y `rev_counts >= 300` inmediatamente → saltaba a OBJ_BRAKE sin retroceder. Mismo patrón del bug ya corregido en OBJ_ROTATE | Bug: encoder snapshot stale post-caída durante OBJ_REVERSE |
| 2026-05-27 | Core/Src/main.c | `brake_start_ms` en `OBJ_BRAKE` promovido de `static` local a variable de archivo `obj_brake_start_ms`. Reseteado a 0 en: detección de caída, transición OBJ_REVERSE→OBJ_BRAKE, y al salir del estado. Sin fix: si el robot caía durante OBJ_BRAKE, al recuperarse el tiempo transcurrido superaba el timeout (1500ms) y saltaba a OBJ_ROTATE sin frenar | Bug idéntico al que afectaba a OBJ_ROTATE con `rot_initialized` |
| 2026-05-27 | Core/Src/main.c | `obj_rot_initialized` y `obj_rot_start_ms` ahora se resetean en el bloque de caída. Sin fix: si el robot caía durante OBJ_ROTATE, los counts acumulados durante el rodamiento sumaban al snapshot previo → el conteo podía superar target_counts=325 inmediatamente al recuperarse → saltaba a OBJ_HOLD sin haber girado | Mismo patrón de bug: snapshot de encoder stale post-caída en OBJ_ROTATE |
| 2026-05-27 | Core/Src/main.c | `KV_brake_value` (registrado con UNER para el slider "KV" en Qt) ahora se usa en `ComputeBrakeSetpointTarget` en lugar de la constante compilada `KV_BRAKE_STRONG`. Antes el slider de Qt enviaba valores a `KV_brake_value` pero la función de freno ignoraba la variable y usaba siempre 5.0f | Bug funcional: el ajuste de freno desde Qt no tenía efecto en runtime |
| 2026-05-27 | Core/Src/main.c | `line_error_f_d` agregado al bloque de caída y al bloque de recuperación (antes solo se limpiaba en las transiciones de estado). Sin esto, al retomar OBJ_REVERSE después de una caída, el término derivado del PID de línea tenía un spike de hasta ±16 en el primer ciclo | Previene spike de steering en primer ciclo post-caída en OBJ_REVERSE |
| 2026-05-27 | Core/Src/main.c | `line_obj_rev_vel_integral` y `line_error_f_d` agregados al bloque de recuperación (f_fallen→0). El bloque de caída ya los limpiaba, pero durante los ciclos de caída el integral se re-acumulaba (el bloque de velocidad corre antes de la detección de caída). Al recuperarse, ahora se garantiza que el integral arranca desde 0 | Consistencia: garantiza estado limpio al arrancar de recuperación, no solo en el ciclo de caída |
| 2026-05-27 | Core/Src/main.c | `obj_hold_initialized` eliminado: era una variable de archivo declarada (línea ~379) y reseteada en OBJ_ROTATE→OBJ_HOLD, pero nunca leída ni escrita en el case OBJ_HOLD — dead code puro | Limpieza: la variable fue parte de un hold personalizado que se simplificó y quedó sin uso |
| 2026-05-27 | Core/Src/main.c | Agregado `LINE_STATE_OBJ_ARC`: avanza en arco con `OBJ_ARC_STEER=20` y `OBJ_ARC_ANGLE=1.2°` hasta re-detectar línea → vuelve a FOLLOWING. OBJ_HOLD ahora tiene timer `OBJ_HOLD_DURATION_MS=2000ms` y sale a OBJ_ARC. `obj_hold_start_ms` file-scope, reseteado en caída. Cadena completa: FOLLOWING→WAIT→REVERS→BRAKE→ROTATE→HOLD→ARC→FOLLOWING | El robot se quedaba en HOLD indefinidamente sin reincorporarse a la línea |
| 2026-05-27 | Core/Src/main.c | Bloque de recuperación (f_fallen→0): corregida indentación extra en `line_integral = 0.0f` y variables reordenadas para consistencia visual con el bloque de caída | Cosmético |
| 2026-05-27 | Core/Src/main.c | `OBJ_WALL_FWD`: PI de velocidad eliminado (era la causa del "no avanza" — el robot en WALL_TURN con PIVOT_POWER=5 no salía del sitio, y el PI en WALL_FWD nunca llegaba a empujar). Reemplazado por ángulo fijo `OBJ_WALL_FWD_ANGLE=5.5°`. El PID de balance regula la inclinación directamente. Eliminados defines `OBJ_WALL_SPEED_TARGET`, `OBJ_WALL_VEL_KP`, `OBJ_WALL_VEL_KI`, `OBJ_WALL_VEL_I_MAX`. `obj_wall_vel_int` mantenida pero solo se resetea. | Robot se quedaba completamente detenido en WF_FWD |
| 2026-05-27 | Core/Src/main.c | `OBJ_WALL_PIVOT_POWER` aumentado de 5.0 → 20.0. Con 5.0 PWM el diferencial era insuficiente para rotar (por debajo del umbral efectivo de los motores). Con 20 el robot pivotea con claridad. | WALL_TURN no giraba — potencia insuficiente |
| 2026-05-27 | Core/Src/main.c | Detección "demasiado cerca" en WALL_FWD y WALL_TURN: si `adcAvg[6] < OBJ_WALL_TOO_CLOSE_THOLD=300`, pivotea derecha (mismo signo que OBJ_ROTATE) en lugar de avanzar/izquierda. Prioridad: `line_detected` > `too_close` > `!wall_visible` > avance. Nuevo define `OBJ_WALL_TOO_CLOSE_THOLD=300.0f`. | Evitar que el robot choque con el obstáculo cuando se acerca demasiado |
| 2026-05-27 | Core/Src/main.c | `steering_adjustment = 0.0f` explícito en rama de avance de WALL_FWD (cuando wall_visible y no too_close). Antes: valor residual de OBJ_HOLD podía causar desvío no deseado. | Prevenir desvío lateral al avanzar junto al objeto |
| 2026-05-27 | Core/Src/main.c | `obj_wall_vel_int = 0.0f` agregado en transición OBJ_HOLD→WALL_FWD. | Estado limpio al entrar en wall-following |
| 2026-05-27 | Core/Src/main.c | `OBJ_WALL_PIVOT_POWER` ajustado de 20.0 → 8.0 (20 era demasiado agresivo). `OBJ_WALL_LINE_IGNORE_MS=3000` agregado: al entrar a WALL_FWD se ignora `line_detected` durante 3s para no volver a FOLLOWING al pasar sobre la línea original. `obj_wall_fwd_start_ms` nuevo file-scope, inicializado al entrar desde OBJ_HOLD o WALL_TURN, reseteado al salir. | Robot volvía a FOLLOWING inmediatamente al cruzar la línea y el pivot era excesivo |
| 2026-05-26 | Core/Src/main.c | Detección de objetos re-habilitada: `obj_now` restaurado a `adcAvg[4..7] < OBJ_DETECT_THRESHOLD_f` (estaba forzado a 0). Cadena de evasión FOLLOWING→OBJ_REVERSE→OBJ_BRAKE→OBJ_ROTATE→OBJ_HOLD activa nuevamente | Se había deshabilitado para trabajar solo en seguimiento de línea |
| 2026-05-26 | Core/Src/main.c | OBJ_REVERSE: cuando no hay línea visible, `steering_adjustment` ahora usa feedback de encoders `(rev_dr - rev_dl) × REV_ENC_KP(0.05)` en lugar de 0. Compensa asimetría mecánica de la rueda izquierda que retrocede menos que la derecha | Reversa torcida: motor izquierdo tiene mayor resistencia en ese sentido |
| 2026-05-26 | Core/Src/main.c | `OBJ_REVERSE` reemplaza el setpoint fijo `-1.5°` por PI de velocidad hacia atrás (`LINE_OBJ_REV_*`) y usa PID de línea con error invertido, limitado por `LINE_OBJ_REV_STEER_MAX`. El clamp negativo de modo línea permite `-LINE_OBJ_REV_TILT_MAX` solo en reversa por obstáculo | Mantenerse sobre la línea al retroceder ante obstáculos, inclinando hacia atrás como el seguidor inclina hacia adelante |
| 2026-05-26 | Core/Src/main.c | Agregado `LINE_STATE_OBJ_PRE_REVERSE_HOLD`: al detectar obstáculo espera `OBJ_PRE_REVERSE_HOLD_MS=2000` en balance estático con freno por encoders antes de pasar a `OBJ_REVERSE`. Display muestra `WAIT` durante esa pausa | Suavizar la transición entre seguimiento de línea y marcha atrás |
| 2026-05-04 | CLAUDE.md | Lectura inicial completa del proyecto: hardware, periféricos, PID, protocolo UNER, arquitectura real de archivos | Primera documentación desde el código fuente |
| 2026-05-07 | Core/Src/main.c | Agregado soporte de encoders por cuadratura vía EXTI: variables `encoder_right/left`, GPIO init PA8/PB13/PB14/PB15, callbacks EXTI y cálculo de `speed_right/left_rps` en `ControlStep10ms` | Medición real de velocidad de ruedas con encoders físicos |
| 2026-05-07 | Core/Src/main.c | `velocity_est`/`velocity_est_f` ahora alimentados desde encoders (`vel_enc = promedio(speed_right_rps, speed_left_rps) × ENC_VEL_SCALE`). Eliminadas las 3 llamadas a `UpdateVelocityEstimate` (accel+gyro). Display "VE:" ya mostraba `velocity_est_f` sin cambios. `ENC_VEL_SCALE=0.17750f` (radio 2.825 cm). | Velocidad real de ruedas en lugar de estimación por fusión inercial |
| 2026-05-07 | Core/Src/main.c | Agregada infraestructura para steering de lazo cerrado por encoders: `ComputeSteeringPID()`, `SteeringPID_Reset()`, flag `steer_pid_enabled`. Por defecto deshabilitado (=0, comportamiento idéntico al anterior con corrección por gyro Z). Activar con `steer_pid_enabled=1` y ajustar `STEER_KP/KI/KD`. | Base para control de dirección preciso usando diferencia de velocidades de encoders |
| 2026-05-08 | Core/Src/main.c | Motor izquierdo invertido: negado en las dos llamadas a `MotorControl` (líneas con `motorLeftVelocity`). Los encoders también tenían signo invertido: `vel_enc = -((speed_r + speed_l) * 0.5f) * ENC_VEL_SCALE` | Motores nuevos de 1000 RPM tenían polaridad opuesta al lado izquierdo |
| 2026-05-08 | Core/Src/stm32f4xx_it.c | Agregado `EXTI9_5_IRQHandler` (faltaba → CPU caía en Default_Handler). Corregido `EXTI15_10_IRQHandler`: limpia flags de líneas no usadas con `EXTI->PR` (write-1-to-clear) y llama HAL para PB12, PB13, PB14, PB15. Esto elimina el re-ingreso infinito al handler que trababa el CPU | Fix definitivo del HardFault por interrupt storm de encoders |
| 2026-05-08 | Core/Src/main.c | Masking anti-storm en callback de encoders: tras cada pulso se enmascara el EXTI (`EXTI->IMR &= ~pin`). TIM5 (2ms) re-habilita todos los pines del encoder. Limita a 500 Hz max por canal | Previene freeze por rafagas de interrupciones del encoder |
| 2026-05-08 | Core/Src/main.c | Encoder 4x quadrature: PB13 y PB15 pasan de input mudo a EXTI RISING+FALLING. Callback maneja 4 canales: A con `(a==b)`, B con `(a!=b)`. `ENC_CPR` 12→24. TIM5 re-habilita los 4 pines | Duplica resolución efectiva: de 12 a 24 conteos/rev |
| 2026-05-08 | Core/Src/main.c | `KV_BRAKE` puesto en 0.0f (base velocidad baja). `KV_BRAKE_STRONG` ajustado a 5.0f (era 8.0f). `KV_brake_value` inicializado a `KV_BRAKE_STRONG`. Slider "KV" en Qt controla `KV_brake_value` en runtime | Adaptación a motores 1000 RPM (25% más rápidos); freno fuerte funciona bien con velocidad de encoders |
| 2026-05-08 | Core/Src/main.c | Seguidor de línea mejorado: (1) PI de velocidad externo (`LINE_VEL_KP=2.5`, `LINE_VEL_KI=0.8`) reemplaza `line_speed_scale` + `KV_LINE_BRAKE`. (2) Inner steering PI con encoder feedback (`LINE_STEER_FB_KP=5.0`, `LINE_STEER_ENC_SCALE=0.12`) reemplaza steering open-loop. (3) Reducción cuadrática de velocidad con `|line_error|` reemplaza `forward_factor` lineal. Variables `speed_right/left_rps_s` expuestas como statics. `LINE_SPEED_TARGET=0.25 m/s` ajustable en runtime. | Control fluido y robusto: la velocidad y el giro se regulan por realimentación real, no heurísticas abiertas |
| 2026-05-09 | Core/Src/main.c | `LINE_ANGLE` cambiado de 0.5° → 3.0°. Bug: 0.5° coincidía con `BALANCE_HOLD_ENTER_ANGLE_DEG=0.5°`, activando hold casi siempre → `pwm_sat≈0` → el steering (5–20 PWM) superaba el avance → ruedas retrocedían. Con 3.0° el `pwm_sat≈12`, suficiente para superar el steering en línea recta. | Fix de "cuesta ganar ángulo para avanzar" y "cuando ve la línea se va hacia atrás" |
| 2026-05-09 | Core/Src/main.c | `desired_vel` en el PI de velocidad del seguidor de línea ahora tiene un mínimo del 25% de `LINE_SPEED_TARGET` mientras la línea sea visible. Antes: `speed_factor=0` cuando `line_error>0.45` (sensores de borde) → `desired_vel=0` → PI comandaba ángulo negativo de hasta -3° → retroceso. Ahora `desired_vel >= 0.25 * LINE_SPEED_TARGET` en todo momento con línea visible. | Fix de "saca tanto ángulo con sensores de borde que se va hacia atrás" |
| 2026-05-09 | Core/Src/main.c | I2C Bus Recovery en USER CODE BEGIN 2: antes de cualquier transacción I2C, de-init hi2c1, pines PB8/PB9 como GPIO OD, 9 pulsos SCL manuales + STOP condition, restaurar AF4, HAL_I2C_Init. Causa del freeze-en-boot: IWDG cortaba una transacción I2C → esclavo quedaba con SDA=LOW → siguiente boot se bloqueaba en HAL_I2C → loop IWDG→freeze→IWDG. Solo el corte de alimentación liberaba el bus. | Fix del freeze permanente post-IWDG-reset |
| 2026-05-09 | Core/Src/main.c | IWDG watchdog: timeout ~2s (LSI /256, reload=250). Init en USER CODE BEGIN 2, patada (`0xAAAA`) en cada iteración del while(1). Cubre cualquier freeze: I2C lockup, interrupt loop infinito, HardFault, etc. El IWDG no se puede detener una vez arrancado. | Auto-recuperación de freeze sin intervención manual |
| 2026-05-09 | Core/Src/stm32f4xx_it.c, main.c | Fix freeze/HardFault: (1) `HardFault_Handler` → `NVIC_SystemReset()` para auto-recuperación. (2) `EXTI15_10_IRQHandler`: guarda `if(EXTI->IMR & pin)` antes de llamar al handler de cada encoder — evita que el MPU (PB12) procese bits pendientes de encoders enmascarados. (3) `TIM5_IRQn` prioridad rebajada de 0→6 en USER CODE BEGIN 2 para que no preempte handlers de encoder (EXTI9_5 prio 5, EXTI15_10 prio 1). | HardFault/freeze al mover el robot físicamente |
| 2026-05-09 | Core/Src/main.c | Comportamiento sin línea: FOLLOWING→LOST threshold 1s→2s. LINE_STATE_LOST y LINE_STATE_SEARCHING unificados: `steering_adjustment=0`, setpoint con freno de posición (BALANCE_ONLY). Robot queda quieto hasta que vuelve a ver la línea. | Sin girar como loco cuando pierde la línea |
| 2026-05-09 | Core/Src/main.c | Detección de reversa en modo línea: si `velocity_est_f > 0.05` (reversa, vel positiva = atrás) acumula distancia; al superar 0.10 m (10 cm) sale a BALANCE_ONLY. Contador se resetea cuando avanza. | Límite de 10 cm en reversa durante seguidor de línea |
| 2026-05-09 | Core/Src/main.c | Regulación de velocidad en modo línea por encoder: `vel_scale = max(0, 1 - |velocity_est_f| / LINE_SPEED_TARGET)`. A mayor velocidad medida → menor ángulo de avance → robot frena naturalmente. Reemplaza el límite de pwm_sat por error (que causaba caídas) y el turn_scale por error de línea. |: `turn_pwm_limit = max(8, 40*(1-|line_error|*0.8))`. Recto: límite=40. Error=0.5: límite=24. Error=1 (borde): límite=8. Efecto inmediato sin esperar rampa del setpoint. | Aceleración excesiva en curvas del seguidor de línea |
| 2026-05-09 | Core/Src/main.c | Velocidad al girar: `base_setpoint_target = line_angle_cmd * turn_scale` donde `turn_scale = max(0.2, 1 - |line_error|*0.8)`. Centrado=100% velocidad, sensor borde=20%. Línea perdida (LINE_STATE_LOST/SEARCHING): `base_setpoint_target = trim`, `brake_setpoint_target = ComputeBrakeSetpointTarget(BALANCE_ONLY)` para quedarse quieto con control de posición. | Velocidad excesiva al girar + quedarse quieto cuando pierde línea |
| 2026-05-09 | Core/Src/main.c | Inner steering PI de encoders eliminado. `steering_adjustment = clamp(steering_cmd, ±20)` directo. El inner PI (diff_sp→diff_actual→LINE_STEER_FB_KP) anulaba la corrección si el signo de los encoders no era exacto. Ahora el giro es proporcional a `line_error` vía KP_LINE, ajustable en runtime desde Qt. | Fix de "no dobla cuando se va de la línea" |
| 2026-05-09 | Core/Src/main.c | Setpoint en modo línea simplificado: eliminadas doble rampa y suma de setpoint_trim. Ahora `base_setpoint_target = line_angle_cmd` directamente (0 si no ve línea). Tres clamps anteriores reducidos a uno en el PI de velocidad. Evita retroceso por trim negativo y simplifica el flujo. | Simplificación del setpoint en modo línea |
| 2026-05-09 | Core/Src/main.c | `MOTOR_LEFT_DEADBAND` agregado (igual estructura que `MOTOR_RIGHT_DEADBAND`). Motor izquierdo no se movía en balance con pwm_sat bajo porque no tenía compensación de zona muerta. Fix: aplica offset ±LEFT_DEADBAND al comando del motor izquierdo en el mismo punto que el derecho. Ajustar el valor si el izq. sigue sin arrancar. | Motor izquierdo inmóvil en balance con corrección pequeña |
| 2026-05-09 | Core/Src/main.c | `dynamic_setpoint_f` clampeado a ≥ 0 en modo línea (después del cálculo final). Bug: `setpoint_trim` negativo + `line_angle_ramped` pequeño → suma negativa → retroceso, aunque `line_angle_cmd` y `base_setpoint_target` estuviesen clampeados a ≥ 0. El `base_setpoint_f` (versión rampeada) podía quedar negativo durante la transición. | Fix definitivo de retroceso en modo línea |
| 2026-05-09 | Core/Src/main.c | `line_angle_cmd` y `base_setpoint_target` clampeados a `[0, LINE_ANGLE]` en modo línea (antes `[-LINE_ANGLE, LINE_ANGLE]`). El ángulo nunca va negativo → robot nunca retrocede durante el seguimiento. | Fix de retroceso esporádico en modo línea |
| 2026-05-09 | Core/Src/main.c | Lectura de sensores de línea: reemplazado `adcValues[]` (snapshot DMA crudo) por `adcAvg[]` (promedio móvil de 15 muestras a 4kHz). Eliminado filtro EMA redundante (ADC_BETA=0.8) que retrasaba la detección un ciclo (80% del valor en primer sample → no alcanzaba LINE_THRESHOLD). `LINE_THRESHOLD` bajado de 3000→1500 para calibrar contra adcAvg en lugar del valor crudo. | Fix de "pasa por encima de la línea sin corregir" |
| 2026-05-09 | Core/Src/main.c | `LINE_ANGLE` bajado de 3.0° → 1.5°. Con 3.0° el robot iba a fondo porque el PI de vel está en retroalimentación positiva y siempre satura al máximo. 1.5° mantiene suficiente avance para superar el steering pero limita la velocidad tope. Ajustable desde Qt en runtime. | Reducción de velocidad del seguidor de línea |
| 2026-05-15 | Core/Src/main.c, CLAUDE.md | PI de velocidad del seguidor de línea ahora regula velocidad de avance positiva (`forward_vel = max(0, -velocity_est_f)`) y compara contra `desired_forward_vel`. No cambia `turn_scale` ni mínimos de avance. | Evitar que una velocidad positiva de reversa reduzca el setpoint a cero y haga oscilar/retroceder |
| 2026-05-18 | Core/Src/main.c, CLAUDE.md | Revertida la protección anti-retroceso que cortaba `pwm_sat` negativo a 0 y bloqueaba ruedas internas negativas con línea visible. | Ese enfoque hacía que al ver la línea el robot se fuera directamente hacia adelante y no mantuviera equilibrio |
| 2026-05-18 | Core/Src/main.c, CLAUDE.md | Suavizado del avance en modo línea: `LINE_VEL_KP` 5.5→2.5, `LINE_VEL_KI` 0.8→0.25, `LINE_VEL_I_MAX` 3.0→1.0, rampa de setpoint de línea 1.0°→0.06° por ciclo y factor `stability_scale` según inclinación/gyro antes de pedir avance. | Reducir oscilación adelante/atrás cuando detecta la línea sin tocar el PWM final del balance |
| 2026-05-18 | Core/Src/main.c, CLAUDE.md | Ajuste para recuperar avance sin volver a oscilar: `vel_scale` tiene piso 0.35, `stability_scale` tiene piso 0.50 y cae más lento, rampa de setpoint de línea 0.06°→0.12° por ciclo. | El cambio anterior estabilizó pero quedó demasiado conservador y no ganaba inclinación hacia adelante |
| 2026-05-18 | Core/Src/main.c, CLAUDE.md | Agregado `line_forward_boost`: boost de inclinación basado en encoders. Si `line_forward_vel` queda por debajo de `LINE_FWD_STALL_FRAC` de la velocidad deseada, suma gradualmente hasta `LINE_FWD_BOOST_MAX=0.8°`; si recupera avance, lo descarga. | Usar encoders para pedir más ángulo cuando no avanza, sin tocar PWM final ni pelearse con el PID de balance |
| 2026-05-18 | Core/Src/main.c, CLAUDE.md | Agregado `line_reverse_boost`: si `velocity_est_f` indica reversa (`> LINE_REV_VEL_START`), suma gradualmente hasta `LINE_REV_BOOST_MAX=1.4°` y evita que `stability_scale` baje de 0.75 mientras corrige reversa. | Controlar explícitamente cuando el robot se va hacia atrás, usando encoders y modificando solo el setpoint de inclinación |
| 2026-05-18 | Core/Src/main.c, CLAUDE.md | Tres fixes para "se queda parado" en modo línea: (1) `vel_scale` eliminado (→1.0 constante) — el PI de velocidad ya regula la velocidad, vel_scale era un segundo freno redundante que recortaba el ángulo al 65%. (2) `sp_step_max` en LINE 0.12→0.25°/ciclo — rampa 2× más rápida. (3) Anti-windup al PI de velocidad: no acumula integral cuando la salida está saturada y `vel_error>0`, evita que el integral wind-up retrasa la frenada al superar el target. | Robot se quedaba parado en algunos puntos por rampa lenta y vel_scale recortando ángulo en el peor momento |
| 2026-05-18 | Core/Src/main.c, CLAUDE.md | Fix doble que impedía ganar velocidad en modo línea: (1) `stability_scale` ahora usa `base_setpoint_f` como referencia en lugar de `SETPOINT_ANGLE+trim` (0°) — antes penalizaba la inclinación correcta de avance; umbral subido 0.8°→1.5°, piso 0.50→0.60, gyro 12→20 dps. (2) `vel_scale` floor 0.35→0.65 para que no recorte el ángulo en crucero. (3) `LINE_VEL_KI` 0.25→1.2 y `LINE_VEL_I_MAX` 1.0→2.5 para mantener ángulo en estado estacionario. | Con referencia errónea (0°), stability_scale reducía el ángulo a 52% cuando el robot inclinaba correctamente a 2°; combinado con vel_scale=0.35, el ángulo llegaba a 18% del comando |
| 2026-05-18 | Core/Src/main.c, CLAUDE.md | `line_forward_boost` (rampas stall/recover) reemplazado por corrección P directa basada en encoders: `line_enc_angle_corr = LINE_ENC_CORR_KP(2.5) × (deficit_vel / desired_vel)`, máx `LINE_ENC_CORR_MAX=2.0°`. Respuesta instantánea al déficit de velocidad vs. rampas lentas (35ms/° anterior). | Robot no ganaba ángulo al ver la línea desde parado: la corrección es proporcional y directa al dato de encoders |
| 2026-05-18 | Core/Src/main.c, CLAUDE.md | Balance hold deshabilitado cuando `line_detected` en modo LINE_FOLLOWING: `if (robot_state == LINE_FOLLOWING && line_detected) balance_hold_active = 0`. También se limpia `balance_hold_active=0` en la transición de entrada al modo línea. | El hold silenciaba el PID durante ~80ms al arrancar el seguimiento (error < 0.5° mientras el setpoint rampea desde 0°), impidiendo que el robot ganara ángulo para avanzar sobre la línea |
| 2026-05-18 | Core/Src/main.c, CLAUDE.md | Frenado activo por encoders: `line_angle_cmd` clampea hasta -1.5° (antes 0). Freno separado de `vel_scale` (cuando `line_angle_cmd < 0` no se multiplica por vel_scale=0 que lo anulaba). Integral del PI de velocidad solo acumula positivo; en sobrevelocidad decae ×0.95 sin acumular negativo → elimina retroceso por windup. `vel_scale` corta avance al 90% del target. `ADC_AVERAGE_SIZE` 15→4: retraso detección 7.5ms→0.5ms. Freno máximo -1.5° (suavizado desde -3°) para evitar oscilación acelera/frena. | Robot aceleraba sin límite, pasaba por encima de la línea sin detectarla, y oscilaba indefinidamente |
| 2026-05-18 | Core/Src/main.c, CLAUDE.md | Detección de objetos + evasión por encoders: objeto detectado → `LINE_STATE_OBJ_ROTATE`. Giro derecha (`steering_adjustment=-20`) midiendo con encoders. `target_counts = (π/4 × OBJ_ROTATE_TRACK_WIDTH × ENC_CPR) / ENC_VEL_SCALE`. Métrica: `(|Δright| + |Δleft|) / 2`. Timeout 3s. `OBJ_ROTATE_TRACK_WIDTH=0.220m` (medido). Display muestra "ROTATE". | Reemplaza condición sensor-based (poco confiable) por medición odométrica de encoders |
| 2026-05-19 | Core/Src/main.c, CLAUDE.md | Fix de rotación en `LINE_STATE_OBJ_ROTATE`: reemplazado `steering_adjustment=-20` (que producía MotorControl(+10,+10) con pwm_sat≈0, sin diferencial real) por asignación directa de motores con `line_pivot_active=1`. Formula: `motorRightVelocity = -(pwm_sat + PIVOT_POWER)`, `motorLeftVelocity = -(pwm_sat - PIVOT_POWER)` con `PIVOT_POWER=30`. Esto produce MotorControl(-(pwm_sat+30), -(pwm_sat-30)) — ambos canales en la misma dirección relativa, creando spin real independiente del valor de pwm_sat. | Bug: la lógica half_steer falla a pwm_sat≈0 porque ambos motores terminan con el mismo signo positivo y el PID de balance lo cancela; el robot no rotaba y el timeout de 3s disparaba siempre |
| 2026-05-19 | Core/Src/main.c, CLAUDE.md | Fix de entrada inmediata a HOLD: variables `rot_initialized`, `rot_r0`, `rot_l0`, `rot_start_ms` del case OBJ_ROTATE eran `static` locales. Al caer el robot durante la rotación, el bloque LINE_FOLLOWING se salteaba (f_fallen=1) sin resetear esas vars. Al recuperarse, `dr/dl` acumulaban los counts de la caída → `rot_counts >= target_counts` inmediatamente → salto a OBJ_HOLD. Fix: variables promovidas a scope de archivo (`obj_rot_*`) y `obj_rot_initialized=0` se resetea en la transición FOLLOWING→OBJ_ROTATE. | Las statics locales no se pueden resetear desde fuera del case; se necesita scope de archivo para controlarlo explícitamente |
| 2026-05-19 | Core/Src/main.c, CLAUDE.md | Agregado `LINE_STATE_OBJ_REVERSE`: reversa de 300 counts con steering invertido basado en `line_error` (×0.3, clamped ±12) para mantener centrado sobre la línea mientras retrocede. Setpoint = trim − 1.5° para lean suave hacia atrás. Sin línea detectada: retrocede recto. Corregido `target_counts` de OBJ_ROTATE: fórmula geométrica daba 23 pero en la práctica son 350 → valor empírico. Cadena: FOLLOWING → OBJ_REVERSE → OBJ_BRAKE → OBJ_ROTATE → OBJ_HOLD. | Fórmula ignoraba reducción de engranajes reales; steering en reversa usa signo negado del error de línea. |
| 2026-05-19 | Core/Src/main.c, CLAUDE.md | Agregado `LINE_STATE_OBJ_BRAKE`: estado intermedio post-reversa que usa `ComputeBrakeSetpointTarget` para frenar activamente por encoders y espera a que `fabsf(velocity_est_f) < 0.05 m/s` antes de iniciar el giro. Timeout de 1500 ms. Evita que la inercia de la reversa distorsione el giro. | Sin este estado, el robot entraba al giro con inercia residual y no giraba en el lugar. |
| 2026-05-18 | Core/Src/main.c, CLAUDE.md | `LINE_STATE_OBJ_HOLD` simplificado: usa exactamente las mismas características que BALANCE_ONLY (setpoint=trim, freno traslacional por encoders, corrección yaw=gz×0.3 igual que LOST/SEARCHING). Sale únicamente cuando `f_fallen=1` → transiciona a `LINE_STATE_FOLLOWING` para retomar la línea al recuperarse. Display muestra "HOLD". | Reutiliza el balance estático probado en lugar de lógica custom; sale solo por caída |
| 2026-05-18 | Core/Src/main.c, CLAUDE.md | Ventana de ignorar objeto al activar modo línea por botón (doble click): `obj_detect_ignore_until_ms = HAL_GetTick() + 4000`. La detección de objeto no dispara mientras `HAL_GetTick() < obj_detect_ignore_until_ms`. Solo aplica al activar por botón físico, no por comando Qt. | Evitar falsa detección cuando el dedo pasa frente a los ADC 5-8 al presionar el botón |
| 2026-05-18 | Core/Src/main.c, CLAUDE.md | Pantalla 1 (LINE_FOLLOWING): panel izquierdo expandido de 4 a 8 barras ADC (bar_width 5→7, spacing 1, área 55→70px, divisor 57→71, contenido derecho 62→73). ADC 1-4 (línea) en orden invertido; ADC 5-8 (objeto) a continuación. Separador punteado en x=32. | Barras más anchas y legibles; panel derecho desplazado para dar espacio |
| 2026-05-18 | Core/Src/main.c, CLAUDE.md | Pantalla 4 (f_change_display==4): reemplazado bloque ADC1-4 con Font_7x10 por grid 4×2 con Font_5x7 mostrando ADC1-8. Columna izquierda (x=1): A1-A4; columna derecha (x=67): A5-A8. Divisor vertical en x=63. | Ver todos los ADC raw en una sola pantalla para calibrar umbrales de línea y objeto |
| 2026-05-18 | Core/Src/main.c, CLAUDE.md | Watchdog de software I2C/MPU: función `I2C1_Recover()` (DMA abort + 9 pulsos SCL + HAL_I2C_Init + I2C_Manager_Init) llamada desde while(1) si pasan >150ms sin dato nuevo del MPU. Watchdog integrado en el bloque `mpu_data_ready_for_ctrl` usando `last_ctrl_ms` estático. Imprime "I2C RECOVER" por USB para diagnóstico. | Fix del freeze total donde el while(1) seguía girando pero el DMA I2C quedaba colgado sin completar, dejando ControlStep10ms sin correr y todo congelado |

---

## Decisiones de Diseño
> Registrar el *por qué* de decisiones críticas del firmware.

- **Filtro complementario en lugar de Kalman:** menor costo computacional, suficiente para este sistema (α=0.98, dt=10ms)
- **Loop de control en callback de TIM1 a 100 Hz:** periodicidad exacta garantizada por hardware, independiente del loop main
- **Bias MPU hardcodeado (`MPU_USE_FIXED_BIAS`):** elimina calibración al arranque; el robot puede actuar en segundos sin necesidad de estar quieto
- **I2C no bloqueante con cola:** el MPU y el SSD1306 comparten I2C1; la cola evita colisiones y no bloquea el loop de 10ms
- **Protocolo UNER binario:** frame compacto con checksum, permite comandos bidireccionales y telemetría eficiente sobre USB CDC y UDP
- **USB CDC en lugar de UART para Qt:** mayor throughput y sin necesidad de conversor USB-UART externo
- **USART1 dedicado a ESP-01:** recepción byte-a-byte por interrupción, sin DMA para UART (el ESP-01 maneja su propia lógica AT)
- **PWM a ~100 kHz (TIM3/TIM4, Period=959):** frecuencia alta para reducir ruido audible y mejorar respuesta de motores DC
- **Masking de EXTI en encoder:** tras cada pulso se enmascara la línea EXTI y TIM5 la reactiva cada 2ms. Limita a 500 Hz/canal, evita freeze por rafagas. Solución más robusta que solo limpiar flags
- **4x quadrature por software:** ambos canales A y B con EXTI RISING+FALLING. Canal B usa lógica de dirección invertida `(a!=b)`. ENC_CPR=24. Sin modo encoder de hardware (requeriría cambio de pines)
- **`KV_brake_value` mapeado a slider "KV" en Qt:** permite ajustar el freno fuerte en runtime sin recompilar. Se inicializa desde `KV_BRAKE_STRONG`
- **Seguidor de línea con setpoint no negativo:** cuando la línea está visible, el setpoint de inclinación no debe ir a retroceso. No forzar `pwm_sat` a 0 ni bloquear ruedas internas negativas: esa prueba del 2026-05-17 hizo que el robot empujara hacia adelante sin poder equilibrarse y fue revertida el 2026-05-18.
- **Avance de línea subordinado a estabilidad:** el seguidor de línea solo modula el setpoint de inclinación. Si `filtered_roll_deg` se aleja del equilibrio o `gyro_f` sube, `stability_scale` reduce el avance pedido. El PWM final sigue bajo autoridad del PID de balance.
- **Boost de avance por encoders:** `line_forward_boost` aumenta solo el setpoint de inclinación cuando los encoders muestran que el robot no avanza lo suficiente en modo línea. No actúa sobre motores ni `pwm_sat`; el PID de balance conserva la autoridad final.
- **Escape de reversa por encoders:** si `velocity_est_f` es positiva (convención actual: reversa), `line_reverse_boost` suma inclinación hacia adelante de forma rampeada. Es más fuerte que el boost de stall, pero sigue actuando solo sobre setpoint.
- **Steering de línea directo:** el PI interno sobre diferencial de encoders quedó deshabilitado/eliminado del camino activo porque dependía de signos de encoder muy sensibles. El giro actual usa `steering_adjustment = clamp(KP_LINE*line_error + KI_LINE*I + KD_LINE*D, ±20)` y los encoders quedan para velocidad longitudinal.

---

## Dependencias con Qt
> Cambios en el firmware que requieren cambios **coordinados** en Qt:

- Si modificás el **formato de trama UNER** → actualizar el parser en `serialmanager.cpp` de Qt
- Si cambiás **campos de `WifiLogData_t`** → actualizar el display/plot en Qt (struct packed compartida)
- Si cambiás **puertos UDP** (30000/30010) → actualizar `udpmanager.cpp` en Qt
- Si agregás **nuevos comandos** al enum `_eCmd` → agregar handlers en Qt
- Consultar siempre: `C:\Microcontroladores\BalancinQT\CLAUDE.md`

---

## Comandos / Flujo de Trabajo en STM32CubeIDE
```
1. Abrir workspace: C:\Users\tadeo\STM32CubeIDE\workspace_1.18.1\
2. Proyecto: Balancin_Mendelevich
3. Para regenerar código HAL: abrir Balancin_Mendelevich.ioc → Generate Code
   ⚠️  NO sobreescribir secciones USER CODE BEGIN / USER CODE END
4. Compilar: Project → Build All (Ctrl+B)
5. Flashear: Run → Debug (F11) o Run (Ctrl+F11)
6. Monitor serie: Window → Show View → Console  (o usar Qt para debug)
```

## ⚠️ Advertencias Importantes
- **Nunca** modificar código fuera de bloques `/* USER CODE BEGIN */` y `/* USER CODE END */` — CubeMX los sobreescribirá.
- **Después de cada regeneración de CubeMX:** verificar con `git diff` que no se hayan borrado handlers críticos en `stm32f4xx_it.c`. CubeMX elimina `EXTI9_5_IRQHandler` (encoder PA8) y simplifica `EXTI15_10_IRQHandler` quitando el manejo de encoders PB13/14/15 — sin esos handlers el CPU cae en loop infinito al primer pulso de encoder. También verificar que las prioridades de interrupts en `MX_DMA_Init` y `MX_GPIO_Init` no hayan sido reseteadas a 0. CubeMX resetea silenciosamente `DMA1_Stream0/1_IRQn`, `DMA2_Stream0_IRQn` y `EXTI15_10_IRQn` a prio 0, lo que causa freeze I2C. Las prioridades correctas están forzadas en `USER CODE BEGIN 2` y sobreviven la regeneración — pero siempre confirmar con git diff.
- Antes de cambiar el `.ioc`, hacer commit en Git o guardar backup.
- El módulo WiFi puede tardar hasta 3s en conectarse al arranque — normal.
- El SSID/password/IP WiFi está hardcodeado en `main.c` líneas ~242-244 y **cambia según la red donde se trabaje** — ver tabla de perfiles en la sección "Canal 2 — WiFi UDP". Siempre verificar antes de flashear.
