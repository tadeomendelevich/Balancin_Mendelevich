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

### Cambios recientes (2026-05-19)
- Detección de objetos **deshabilitada temporalmente** (`obj_now = 0`) para trabajar solo en seguimiento de línea
- Centroide cuadrático implementado: coeficientes {+9,+1,-1,-9} / (9×w_sum), sensores a ±5.75mm y ±17.25mm del centro
- `ADC_BASELINE[4]` para sustracción de baseline — **completar con valores medidos sobre piso blanco**
- Cadena de evasión implementada pero pausada: FOLLOWING → OBJ_REVERSE → OBJ_BRAKE → OBJ_ROTATE → OBJ_HOLD

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
- Antes de cambiar el `.ioc`, hacer commit en Git o guardar backup.
- El módulo WiFi puede tardar hasta 3s en conectarse al arranque — normal.
- El SSID/password/IP WiFi está hardcodeado en `main.c` líneas ~242-244 y **cambia según la red donde se trabaje** — ver tabla de perfiles en la sección "Canal 2 — WiFi UDP". Siempre verificar antes de flashear.
