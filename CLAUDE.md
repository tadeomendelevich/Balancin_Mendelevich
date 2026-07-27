# Balancin STM32 — Memoria del Proyecto
> Este archivo es la memoria persistente del firmware para Claude Code.
> Mantenerlo actualizado al final de cada sesión de trabajo.

---

## 🤖 Instrucciones Permanentes para Claude

**Estas reglas aplican en cada sesión, automáticamente y sin que te lo pida:**

### Al modificar cualquier archivo de código:
- Agregá una fila en "Registro de Cambios" con fecha actual (YYYY-MM-DD),
  archivo(s) tocado(s), qué cambió y por qué
  (en CLAUDE.md quedan solo las ~10 filas mas recientes; las mas viejas se MUEVEN al tope de la tabla de `CHANGELOG.md` — la informacion nunca se borra, solo se muda de archivo)
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
| Encoders | Cuadratura (modelo desconocido) | 4x quadrature por muestreo periódico (polling) a 4 kHz vía TIM2: PA8/PB13 (derecho), PB14/PB15 (izquierdo). `ENC_CPR=28` conteos/rev |

---

## Arquitectura de Archivos
```
Balancin_Mendelevich/
├── CLAUDE.md                        ← memoria del proyecto (este archivo)
├── Balancin_Mendelevich.ioc         ← configuración CubeMX (pines, periféricos)
├── Core/
│   ├── Src/
│   │   ├── main.c                   ← loop principal, init, PID, state machine, complementary filter (desde 2026-07-16: ControlCiclo10ms —renombrada 2026-07-20, antes ControlStep10ms— es un orquestador de etapas Ctrl_* + handlers LineState_* por sub-estado)
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
│       ├── UNER.h                   ← enum comandos, structs LogData_t / WifiLogData_t / WifiOdomData_t
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
| Frecuencia de telemetría | ~10 Hz (LOG_WIFI_DECIM=10 sobre loop de 100 Hz), solo si `ACTIVATE_WIFI_LOG` está activo |
| Formato paquete WiFi | `WifiLogData_t` binario packed: t_ms, roll, output, PID terms, mR, mL, dt, dyn_sp, line data, 4×ADC |
| Push de odometría (2026-07-06, +ADC de objeto y roll 2026-07-08) | `WifiOdomData_t` (cmd `0xDC`, packed: seq, t_ms, x_m, y_m, theta_deg, line_error, line_detected, robot_state, line_state, adc5, adc6, adc7, adc8, roll_deg, lat_deg) enviado cada `WIFI_ODOM_PERIOD_MS=500ms` automáticamente en cuanto `f_wifi_connected=1` — **no depende de `ACTIVATE_WIFI_LOG`**, pensado para graficar mapa XY + posición de línea + barrera/obstáculo frente al robot + inclinación en la Vista 3D de Qt sin competir por ancho de banda/CPU con la telemetría de control a 10 Hz |

> ⚠️ **La IP del PC destino cambia según la red donde se trabaje.** Antes de flashear, verificar
> que el perfil activo coincida con la IP actual del PC con Qt.
> **Desde 2026-07-10, un solo lugar para cambiar de red:** en `main.c` (~línea 423) hay una tabla
> `wifiProfiles[]` (SSID + password + IP en una sola fila por red) y una macro
> `#define WIFI_PROFILE_ACTIVE <n>` — para cambiar de red alcanza con cambiar ese número, ya no
> hay que tocar/comentar SSID, password e IP por separado en tres lugares. `ESP01.c` tenía un
> `SERVER_IP` propio (fallback de reconexión, hoy código muerto) que se sacó ese mismo día — ya
> no hay ninguna otra IP hardcodeada en el firmware fuera de `wifiProfiles[]`.
>
> | # | Red | SSID | IP del PC |
> |---|-----|------|-----------|
> | 0 | FCAL / Universidad | `FCAL` | `172.23.205.98` |
> | 1 | Casa | `MEGACABLE FIBRA-2.4G-ckd0` | `192.168.100.5` |
> | 2 | Delco Mendelevich (activo) | `Delco_Mendelevich` | `192.168.1.23` |
> | 3 | Wifi Habitaciones | `Wifi Habitaciones` | `192.168.1.48` |
>
> Si agregás una red nueva → sumá una fila a `wifiProfiles[]` en `main.c` y una fila acá.

---

## Control PID
| Parámetro | Variable en código | Valor actual |
|-----------|-------------------|--------------|
| Kp | `KP` / `KP_value` | `4.0` |
| Ki | `KI` / `KI_value` | `0.1` |
| Kd | `KD` / `KD_value` | `0.12` |
| Setpoint (ángulo °) | `SETPOINT_ANGLE` | `0.0°` (+ `setpoint_trim` ajustable en runtime) |
| Frecuencia de control | TIM1 (Prescaler=9599, Period=99) | **100 Hz** (10 ms/ciclo) |
| Filtro de derivada | Sin filtro explícito en derivada; zona suave (soft-zone) | SOFT_ZONE_ANGLE=1.5°, scale_min=0.35 (0.15→0.35 el 2026-07-10) |
| Hold de equilibrio (anti-chatter) | `BALANCE_HOLD_*` | Achicado 2026-07-10: enter 0.25°/2°s, exit 0.45°/6°s (era 0.70/4 y 0.90/10); dentro del hold solo se silencian P/I — el D queda vivo |
| Integral anti-windup | `I_MAX` | ±100.0 PWM units; decay=0.990 por ciclo |
| Freno por velocidad | `KV_BRAKE` / `KV_BRAKE_STRONG` | 0.0 / 5.0 (umbral BRAKE_VEL_THRESHOLD=1.5) — velocidad calculada por encoders |
| Velocidad objetivo en línea | `LINE_SPEED_TARGET` | 2.50 m/s por defecto; ajustable en runtime con `MODIFY_LINE_SPEED=0xC4`, limitado a 0.20..4.00 m/s |

**Sensor de ángulo:**
- Fuente: Filtro complementario (α=0.98) entre acelerómetro y giroscopio del MPU-6050
- Eje de control: **roll** (inclinación lateral del péndulo)
- Bias del MPU hardcodeado (`MPU_USE_FIXED_BIAS`): ax=-46, ay=4950, az=1980, gx=-441, gy=-107, gz=-54

**Detección de caída (histéresis):**
- Caída: |roll| > 60°, recuperación: |roll| < 2°
- Boca abajo: |roll| > 120°; zona muerta (motores off): 35°–120°

---

## Periféricos STM32 Usados
| Periférico | Función | Pin(es) | Configuración |
|------------|---------|---------|---------------|
| TIM1 | Interrupción control loop 100 Hz | — (interno) | Prescaler=9599, Period=99 |
| TIM2 | Trigger ADC (TRGO) + muestreo de encoders cada 250 µs (4 kHz) | — (interno) | Prescaler=95, Period=249, TRGO. IT habilitada (`HAL_TIM_Base_Start_IT`), prio NVIC=3 |
| TIM3 | PWM motor (2 canales) | PB4=CH1, PB5=CH2 | Prescaler=0, Period=959 → ~100 kHz |
| TIM4 | PWM motor (2 canales) | PB6=CH1, PB7=CH2 | Prescaler=0, Period=959 → ~100 kHz |
| TIM5 | Sin uso desde 2026-07-01 (era el re-habilitador de EXTI de encoders, ya no aplica) | — (interno) | Inicializado (`MX_TIM5_Init`) pero nunca arrancado — no genera IT |
| I2C1 | IMU MPU-6050 + Display SSD1306 | PB8=SCL, PB9=SDA | Fast mode (400 kHz), DMA RX/TX |
| USART1 | Módulo WiFi ESP-01 (AT commands) | PA9=TX, PA10=RX | 115200 baud, async, IT RX byte a byte |
| USB OTG FS | Comunicación CDC con Qt | PA11=DM, PA12=DP | Device Only, CDC FS |
| ADC1 | 8 canales sensores (línea + analógicos) | PA1–PA7, PB0 | DMA circular, trigger TIM2, 15 ciclos/canal |
| GPIO PB10 | LED_BLINKER | PB10 | Output |
| GPIO PB12 | MPU_INT (EXTI12) | PB12 | Input, interrupción data-ready, EXTI15_10_IRQn prio 2 |
| GPIO PA8 | Encoder derecho canal A | PA8 | Input pull-up simple (sin EXTI desde 2026-07-01), leído por polling en TIM2 |
| GPIO PB13 | Encoder derecho canal B | PB13 | Input pull-up simple (sin EXTI desde 2026-07-01), leído por polling en TIM2 |
| GPIO PB14 | Encoder izquierdo canal A | PB14 | Input pull-up simple (sin EXTI desde 2026-07-01), leído por polling en TIM2 |
| GPIO PB15 | Encoder izquierdo canal B | PB15 | Input pull-up simple (sin EXTI desde 2026-07-01), leído por polling en TIM2 |
| GPIO PB2 | CH_PD ESP-01 (enable módulo) | PB2 | Output |
| GPIO PA0 | KEY (botón usuario) | PA0 | Input pull-up |
| GPIO PC13 | LED integrado | PC13 | Output |

> **IMPORTANTE:** No modificar pines sin actualizar el `.ioc` en CubeMX primero.

---

## Estado Actual
- **Etapa:** Casi terminado
- **Última sesión:** 2026-07-16 (refactor de legibilidad — pendiente compilar y validar en el robot)

### Funcionalidades completas ✅
- PID de estabilización (balance) con zona suave y anti-windup
- Lectura IMU MPU-6050 vía DMA con bias hardcodeado (arranque instantáneo)
- Filtro complementario (α=0.98) acelerómetro + giroscopio
- Display OLED SSD1306 no bloqueante (actualización asíncrona via DMA)
- Máquina de estados del robot (IDLE, BALANCE_ONLY, BALANCE_AND_SPEED, LINE_FOLLOWING, MANUAL_CONTROL, MOTOR_TEST)
- Comunicación USB CDC con protocolo UNER binario (37+ comandos)
- Comunicación WiFi UDP via ESP-01 con watchdog y reconexión automática
- Telemetría en tiempo real: CSV por USB (~20 Hz), binario por WiFi (`WifiLogData_t`, ~10 Hz, requiere `ACTIVATE_WIFI_LOG`) y push de odometría (`WifiOdomData_t`, 2 Hz, automático con solo tener WiFi conectado — ver Canal 2 más abajo)
- Tuneo en tiempo real de Kp, Ki, Kd, setpoint, steering desde Qt
- Seguidor de línea con 8 sensores ADC, PID de línea (Kp=10, Kd=2, Ki=0.5), velocidad en lazo cerrado por encoders y steering directo proporcional al error
- Detección de objetos en modo línea: ADC 5-8 (largo alcance) con debounce de 100 ms → **entra DIRECTO a la fase STOP (`OBJ_FRENO_REVERSA`, 2026-07-14)**: el hold de distancia frena la inercia y lleva/sostiene al robot en la banda 3500..3900 de A6/A8 (con anti-stall tag 9), estabiliza 2s en banda y recién ahí gira. `OBJ_ESPERA_REVERSA` y `OBJ_RETROCESO` (y el reservado `OBJ_ARC`) fueron **eliminados definitivamente el 2026-07-16** (recuperables del historial de git si hicieran falta)
- **Esquive alternado (2026-07-13)**: la secuencia de evasión alterna el sentido en cada objeto — la primera esquiva gira 90° a la DERECHA y bordea la pared con ADC7 (lateral izquierdo, comportamiento histórico), la siguiente gira a la IZQUIERDA y bordea con ADC5 (lateral derecho), y así. Mismos mecanismos/umbrales en ambos sentidos (`obj_esquive_dir` espeja pivots y `OBJ_WALL_ADC_IDX` elige el sensor). Al entrar al modo línea se rearma en derecha. El display (pantallas 1 y 6) muestra `A7:`/`A5:` según el sentido activo. **Desde 2026-07-14 el giro del cruce perpendicular (`PERP_ROTATE`) también sigue `obj_esquive_dir`**: al reencontrar la línea tras bordear, gira hacia el mismo lado del esquive (girar al contrario devolvía al obstáculo); sin esquive previo conserva la derecha histórica
- Control manual remoto (FORWARD/BACKWARD/LEFT/RIGHT/STOP): adelante/atrás por PI de velocidad (máx 1 m/s, ángulo máx 6°, mismo patrón que el PI de velocidad del seguidor de línea); giro suave y a 1/4 de fuerza (steering ±15, rampa ~0.0625/ciclo) en vez de un salto brusco; adelante/atrás van derecho por corrección de rumbo (mismo algoritmo P sobre diferencia de velocidad de ruedas que la reversa recta de `OBJ_REVERSE`) (2026-07-06). **También funciona durante `LINE_FOLLOWING` mientras no ve la línea** (`manual_line_override`, mismo control reutilizado, se ignora apenas la línea reaparece) (2026-07-06)
- Freno dinámico por velocidad de encoders (KV_BRAKE_STRONG=5.0, umbral 1.5 m/s)
- Detección de caída y recuperación con histéresis
- Gestor I2C no bloqueante con cola (evita bloquear el loop de control)
- Encoders de cuadratura 4x: PA8/PB13 (derecho), PB14/PB15 (izquierdo), decodificados por muestreo periódico a 4 kHz vía TIM2 (ver Registro de Cambios 2026-07-01)
- Velocidad real de ruedas desde encoders (reemplaza estimación accel+gyro)
- **Odometría de pose (x, y, θ)** integrada a 100 Hz: encoders (distancia) + gyro Z (rumbo). Comandos UNER `GET_ODOMETRY=0xDA` / `RESET_ODOMETRY=0xDB`; origen se resetea al cambiar de modo. **Pendiente verificar `ODOM_THETA_SIGN` en el robot físico (abierto desde 2026-07-04, aún sin confirmar)** — el proyecto Qt ya tiene las herramientas para hacerlo cómodo: pestaña "Odometría (WiFi)" con mapa XY navegable + flecha de rumbo (ver CLAUDE.md de Qt), y los comandos `GET_ODOMETRY`/`RESET_ODOMETRY` en el combo de comandos. Test: reset odometría, girar el robot 90° a la derecha a mano, `GET_ODOMETRY` y ver si θ salió positivo (~+90°, signo correcto) o negativo (invertir `ODOM_THETA_SIGN` en `main.c` ~línea 596)
- **Retorno por odometría al punto de pérdida de línea** (`LOST_FWD`, camino centrado post-180°): navega a la pose guardada del último punto con línea visible en vez de avanzar a ciegas; display "VUELVE" (2026-07-04, pendiente validar signo de steering en el robot)
- **Giro de 180°/90° (`LOST_ROTATE`/`EDGE_ROTATE`/`PERP_ROTATE`/`OBJ_GIRO_ESQUIVE`) revertidos 2026-07-05** a su forma previa a esa sesión (pivot fijo, freno fijo, heading por `fmaxf(gyro,encoder)`) tras una cadena de rediseños que terminó girando hacia ambos lados. **Corrección 2026-07-06: la escala del gyro Z en estos 4 bloques es `gz/100`, NO `gz/131`** — esta misma sección decía lo contrario hasta hoy; quedó desactualizada porque la reversión del 2026-07-05 restauró casi todo a su forma pre-sesión PERO el usuario pidió explícitamente mantener el fix de escala (`gz/100`) sobre esa base revertida (ver fila del Registro de Cambios 2026-07-05 "Único cambio sobre la reversión completa"). Verificado en vivo grepeando `main.c`: no queda ningún `gz/131` en el código activo, solo en comentarios históricos. **El banco de pruebas del giro de 90° en modo MANUAL (que existía para esto) fue eliminado el 2026-07-06** — MANUAL ahora es exclusivamente control por comandos WiFi/USB, sin ningún ciclo automático

> El "Snapshot historico — sesion 2026-05-27" (superseded) se movio a `CHANGELOG.md` el 2026-07-07.

### Pendientes / bugs conocidos 🔧
- **Estación por rueda (Opción A) — heading-hold por gyro agregado 2026-07-26, pendiente validar en el robot.** Historia del día: (1) knobs de rotación (`WHEEL_ROT_DB` 3→2, `WHEEL_ROT_KP` 0.15→0.25); (2) la TRASLACIÓN fallaba asimétrica (adelante nunca frenaba) → se movió del trim de PWM al SETPOINT de inclinación (`WheelStation_AngleCorr()`, patrón PAUSA_GIRO) — **VALIDADO en el robot: "el PID corrige muchísimo mejor"**; (3) quedaba un giro lento constante a la derecha (micro-patinaje invisible para los encoders + zona muerta de 3°/s del yaw-assist) → **heading-hold por gyro Z**: `wheel_yaw_deg` integra el rumbo desde el ancla y un P (`WHEEL_YAW_KP=2.0`, DB=1°, tope 12°/s eq) se suma al canal del yaw-assist con su mismo signo. (4) 3ª iteración: D siempre activa en la traslación (`WHEEL_STATION_DAMP=0.25 °/rps`) — **VALIDADA: "la deriva traslacional quedó bastante resuelta, se amortigua bastante"**. (5) 4ª iteración ("desde la base"): bias del gyro medido SOLO en reposo real (congelado durante balance) + hold PI — **VALIDADA: "quedó muy bien el tema del giro, no lo hace más"**. (6) 5ª iteración: warmup rápido del bias (~0.3–0.5s de IDLE alcanzan, validado) y DAMP 0.25→0.45 (mejoró pero seguía lento y temblaba en reposo). (7) 6ª/7ª iteración: latch de asentado, DAMP 0.45→0.70, banda angosta + re-anclaje — seguía sin amortiguar bien. (8) 8ª iteración — **rediseño final SIN ANCLA, VALIDADO ("funciona espectacular")**: D de velocidad con zona muerta de 0.8 rps + desplazamiento reciente con fuga τ≈2s ("ancla flotante"). (9) **Siesta de motores** (`MOTOR_SLEEP_*`, solo BALANCE_ONLY): PWM 0 real en el punto dulce hasta empujón/deriva. 2ª forma (ventana propia desacoplada del hold) validada ("mucho mejor"); 3ª pasada del 2026-07-26: entrada aún más fácil (gyro≤4°/s, 100ms sin ticks, |pwm|≤12, 150ms sostenido) y más amortiguación traslacional (`WHEEL_STATION_DAMP`=1.00, KP=0.08, tope 2.5°). 4ª pasada: zona ciega de la D corregida (`WHEEL_V_DB_RPS` 0.8→0.25, KP 0.08→0.05) — "mejoró muchísimo". 5ª pasada: quedaban ~15 cruces de llegada → siesta como asentador (ventanas cortas: gyro≤5, 60ms sin ticks, |pwm|≤15, 80ms sostenido — la pesca en los extremos del vaivén donde v≈0 y la fricción estática lo mata en 2-3 cruces) + DAMP 1.0→1.2 + `WHEEL_V_DB_RPS` 0.25→0.15. **6ª pasada (2026-07-27, a validar)**: la 5ª sobre-corrigió — reporte: "no entra casi nunca a la siesta y se va de a poco para los costados". Con V_DB 0.15 + DAMP 1.2, un count suelto pateaba el setpoint ~0.66° y cada tick reseteaba las ventanas de la siesta (círculo vicioso). Fix: `WHEEL_V_DB_RPS` 0.15→0.25 y `DAMP` 1.2→1.0 (vuelven a los valores validados; la siesta permisiva queda intacta y es quien asienta los cruces), y contra la deriva lenta invisible `WHEEL_DISP_LEAK` 0.995→0.998 (τ≈5s: con τ≈2s toda deriva <~5 mm/s se drenaba antes de superar `WHEEL_TRANS_DB` y nunca se corregía; ahora el umbral baja a ~2 mm/s). **7ª pasada (2026-07-27) — VALIDADA ("la mejora es una locura, amortigua los empujones muy rápido")**: la 6ª empeoró ("reacciona tarde") → causa raíz estructural: la corrección viajaba dentro de `base_setpoint_target` y la rampa de `sp_step_max=0.1°/ciclo` + la EMA de velocidad le metían ~150ms de retraso — la D llegaba en contrafase y bombeaba el vaivén (por eso subir DAMP siempre empeoraba). Fix: corrección aplicada POST-rampa directo sobre `dynamic_setpoint_f` al final de `Ctrl_SetpointDinamico`, con slew propio `WS_STEP=0.4°/ciclo`. Recién ahora la D actúa en fase — evaluar los knobs desde acá. Knobs: tiembla en movimiento → DAMP 1.0→0.9; siesta cicla dormido/despierto (despierta enseguida tras dormir) → `MOTOR_SLEEP_ENTER_MS` 80→120 o `MOTOR_SLEEP_ANG_ENTER` 0.8→0.6; sigue cruzando mucho → `WHEEL_STATION_ANG_KP` 0.05→0.03; si tras esto sigue derivando lento → `WHEEL_TRANS_DB` 3→2 o `WHEEL_DISP_LEAK` 0.998→0.999. Al validar todo el paquete del día: commit + tag (¿V27?). Todo commiteado y pusheado en **2dc5a7b** (2026-07-26); última versión validada completa en el robot: tag **V26** (ba790d5). Al validar esta pasada conviene taggear (¿V27?). Kill-switch `wheel_pi_enabled=0` apaga los tres mecanismos (setpoint, rot por encoders y heading-hold PI). Pendiente aparte: extender la estación a línea/manual si aplica.
- El SSID/IP WiFi está hardcodeado en `main.c` (líneas ~244); cambiar manualmente según red
- `MODIFY_BETA_G/A (0xBC/0xBD)` son vestigiales: los betas de los filtros EMA hoy son defines fijos en `main.c` (las variables runtime ya no existen) y sus punteros no se registran en `unerBindings`. Desde 2026-07-24 el firmware responde UNKNOWN (antes ACK mentiroso). Decidir: sacarlos del combo de Qt o volver a hacer los betas variables y registrarlos
- Código de debug activo en `ESP01.c` (printfs de estados AT) que genera tráfico USB extra
- **Freeze residual al mover el robot** — Causa raíz identificada (2026-05-18): el DMA del I2C queda en estado "en progreso" indefinidamente sin completar. `mpu_data_ready_for_ctrl` nunca se setea, `ControlCiclo10ms` nunca corre, `i2c1_tx_busy` queda en 1, display congelado, motores al último PWM. El `while(1)` SIGUE girando (IWDG se patea) pero el sistema es funcionalmente inútil. Fix: watchdog de software en `while(1)` — si pasan >150ms sin dato del MPU se llama `I2C1_Recover()` (9 pulsos SCL + HAL_I2C_Init + I2C_Manager_Init) y se relanza la lectura SIN resetear el MCU. Detectable por "I2C RECOVER\r\n" en USB. **2026-07-01: se sospecha que este freeze estaba agravado (no necesariamente causado) por el viejo sistema de encoders vía EXTI, que compartía prioridad NVIC con el DMA de I2C y podía dejar la transacción I2C pendiente en cola durante ráfagas de pulsos (ver Registro de Cambios 2026-07-01). Con encoders migrados a polling por TIM2 y el DMA de I2C subido a prioridad 0, debería ser mucho menos frecuente — pendiente de confirmar en el robot físico si desaparece del todo.**
- Resolución de velocidad limitada: piso de cuantización de `velocity_est` (calculado en `ControlCiclo10ms` sobre delta de counts cada 10ms, no sobre el muestreo de 4kHz de `SampleEncoders250us` que solo alimenta el conteo acumulado) ≈ 0.32 m/s con 1 count en una sola rueda por ciclo, ≈ 0.63 m/s si tiquean ambas ruedas. `BRAKE_VEL_DEADBAND=0.35` (2026-07-01) calibrado para filtrar ese piso. No apto para integración de posición ni para reaccionar a velocidades reales por debajo de ese umbral
- **Freeze + reset durante giro de 180° (LOST_ROTATE), reportado repetidamente pese a varios intentos de fix.** Sesión 2026-07-01: se probó primero bajar `LROT_PIVOT`/`LROT_BRAKE` (hipótesis de glitch eléctrico) + corregir prioridad NVIC I2C vs EXTI15_10 — el usuario confirmó que el problema **persistió igual**, descartando la hipótesis de corriente de motor como causa suficiente. El usuario migró el conteo de encoders de EXTI a polling por TIM2 (ver más abajo) sospechando que el bug estaba en el subsistema de encoders. Revisión de código no encontró un bug concreto en el nuevo esquema de polling — arquitectura correcta y con más margen que el EXTI viejo. **Pendiente de validar en el robot físico si el freeze+reset desaparece con el nuevo esquema de encoders.** Si persiste, revisar next: reversión brusca de motores (EMI) con un osciloscopio en VDD durante el pivot, o un logic analyzer en las líneas de encoder para confirmar que no hay rebote/ruido eléctrico real en el sensor.

---

## 📋 Registro de Cambios
> **Instruccion para Claude:** Al finalizar cada sesion, agregar una fila con los cambios realizados (la mas reciente arriba).
> Mantene aca solo las ~10 filas mas recientes: cuando agregues filas nuevas, move las que sobren al TOPE de la tabla de `CHANGELOG.md`. **Nunca borres una fila — siempre movela.**
> Historial completo del proyecto (2026-05-04 en adelante): ver `CHANGELOG.md` en la raiz del proyecto.

| Fecha | Archivo(s) modificado(s) | Cambio realizado | Motivo / Observación |
|-------|--------------------------|------------------|----------------------|
| 2026-07-27 | Core/Src/main.c | **7ª pasada — causa raíz del "reacciona tarde": la corrección de la estación pasaba por la RAMPA del setpoint.** La 6ª pasada no ayudó ("empeoró bastante, le cuesta quedarse en la zona dulce, reacciona tarde, sigue sin entrar a la siesta"). Diagnóstico estructural: `WheelStation_AngleCorr()` se sumaba a `base_setpoint_target`, que en BALANCE/IDLE rampa a `sp_step_max=0.1°/ciclo` (10°/s) hacia `base_setpoint_f` — una corrección de 1° tardaba 100ms en llegar, +~50ms de la EMA de velocidad (`WHEEL_SPD_BETA=0.20`). Con el período del vaivén (~1s), 150ms ≈ 50-90° de fase: la D llegaba con el movimiento ya invertido y en vez de amortiguar BOMBEABA — por eso subir DAMP siempre empeoraba y el robot nunca juntaba la quietud que pide la siesta. (La rampa existió siempre; con DAMP chico y zona muerta grande la D casi no actuaba y no se notaba.) Fix: la corrección ahora se aplica **POST-rampa** al final de `Ctrl_SetpointDinamico` — local `wheel_station_corr` se estampa en las ramas BALANCE_ONLY/IDLE y se suma directo a `dynamic_setpoint_f` (después de la contabilidad de `brake_setpoint_f`, que queda intacta) con un slew propio `WS_STEP=0.4°/ciclo`: sigue el movimiento real casi sin retraso y solo recorta el escalón instantáneo de un count suelto. Clamp final a ±sp_limit. Knobs de la 6ª (DAMP=1.0, V_DB=0.25, LEAK=0.998) quedan — recién ahora la D actúa en fase y se puede evaluar de verdad. NO compilado (el usuario compila). | Usuario: "este cambio no ayudó para nada, empeoró bastante — le cuesta quedarse en la zona de equilibrio dulce, es como que reacciona tarde, y sigue sin entrar al modo siesta, no llega a quedarse tan quieto" |
| 2026-07-27 | Core/Src/main.c | **6ª pasada de la estación: la 5ª sobre-corrigió — el robot no entraba casi nunca a la siesta y "se iba de a poco".** Dos causas: (1) `WHEEL_V_DB_RPS=0.15` + `WHEEL_STATION_DAMP=1.20` convertían el pico de ~0.7 rps de UN count suelto en un pateo de setpoint de ~0.66° → el PID respondía, caían más ticks, y cada tick reseteaba las ventanas de la siesta (60ms quiet + 80ms sostenido) — círculo vicioso: la propia estación mantenía al robot despierto y vagando. Fix: `WHEEL_V_DB_RPS` 0.15→**0.25** (vuelve al valor validado; el pico residual genera ≤0.45° breve y el vaivén real 0.3–0.7 rps sigue cubierto) y `DAMP` 1.20→**1.00** (el knob ya anotado para este síntoma; los ~15 cruces los asienta la siesta permisiva, que queda intacta). (2) La deriva lenta era invisible por diseño: con fuga τ≈2s y `WHEEL_TRANS_DB=3`, toda deriva <1.5 counts/s (~5 mm/s) se drenaba más rápido de lo que acumulaba y NUNCA superaba la banda → `WHEEL_DISP_LEAK` 0.995→**0.998** (τ≈5s): el umbral de deriva invisible baja a ~0.6 counts/s (~2 mm/s). Siesta sin cambios. NO compilado (el usuario compila). | Usuario: "en el punto de equilibrio al que llego tras un empujón le cuesta mantenerse, no se mete casi nunca al modo siesta, y se empieza a ir para los costados de a poco" |
| 2026-07-26 | Core/Src/main.c | **Estación por rueda — rediseño del eje de TRASLACIÓN: de trim de PWM a corrección de SETPOINT.** Probando en el robot los knobs de rotación de hoy, el usuario reportó que la traslación fallaba asimétrica: deriva hacia adelante que NUNCA frenaba y hacia atrás corregía mal. Causa: en un balancín el PWM de rueda es el actuador equivocado para posición — frenar las ruedas inclina el cuerpo hacia adelante y el PID de balance responde acelerando; el trim (±8%) pelea contra el balance y pierde. Rediseño: la traslación ahora corrige el **setpoint de inclinación** (helper `WheelStation_AngleCorr()` antes de la etapa 12, aplicado en las ramas BALANCE_ONLY e IDLE de `Ctrl_SetpointDinamico`): P sobre el exceso fuera de `WHEEL_TRANS_DB=10` counts, ganancia `WHEEL_STATION_ANG_KP=0.06°/count` y signo idénticos al hold ya validado de PAUSA_GIRO (counts+=avance → corrección negativa=inclina atrás), tope `WHEEL_STATION_ANG_MAX=2°`, sin D propio (el freno de `ComputeBrakeSetpointTarget` ya amortigua); gate con `wheel_pos_armed` (primer ciclo no corrige, el ancla se estampa en `Ctrl_MotoresBalance` que corre después — mismo patrón que PAUSA). La ROTACIÓN queda en PWM diferencial (no afecta el pitch, ahí el trim no pelea) — `Ctrl_MotoresBalance` quedó rotation-only (`WHEEL_TRANS_KP/KD` eliminados, `WHEEL_TRANS_DB` movido junto al helper). **Fix extra**: `wheel_pos_armed=0` al entrar a IDLE, al entrar a BALANCE/SPEED/MANUAL y al recuperarse de caída — antes el ancla quedaba vieja tras pasar por línea/caída y la estación intentaba "volver" a una posición anterior. NO compilado (el usuario compila). | Usuario: "existe demasiada deriva hacia adelante, se comienza a ir hacia adelante pero nunca frena; para atrás sí contrarresta pero no funciona muy bien el PID para mantenerlo en su lugar" |
| 2026-07-26 | Core/Src/main.c | **Estación por rueda: ajuste del eje de rotación contra el giro lento residual.** Aplicados los dos primeros knobs que quedaron anotados como pendiente el 2026-07-21: `WHEEL_ROT_DB` 3→2 counts (ataja el giro antes) y `WHEEL_ROT_KP` 0.15→0.25 %PWM/count (corrige más fuerte el exceso). `WHEEL_ROT_KD` queda en 0.60 — subirlo a 1.0 SOLO si al probar aparece temblor al corregir el giro. Si con esto la deriva rotacional persiste, el siguiente paso anotado es heading-hold por gyro Z/odometría θ (la asistencia de rumbo actual solo amortigua velocidad de giro, no sostiene rumbo). Ejes de traslación intactos. NO compilado (el usuario compila). | Usuario: "creo que había quedado una mínima deriva hacia un lado pero estaba más estable, podríamos corregir lo que faltaba así lo terminamos" |
| 2026-07-26 | Core/Src/main.c | **La siesta como asentador del vaivén final + más D.** Quedaba un vaivén de llegada (~15 cruces sobre el ancla antes de asentarse) y el usuario pidió una siesta aún más permisiva. Sinergia deliberada: en cada extremo de la oscilación la velocidad pasa por ~0 — con ventanas cortas la siesta lo pesca ahí, corta motores y la fricción estática mata la oscilación en 2-3 cruces. Siesta: `GYRO_ENTER` 4→5°/s, `ENTER_MS` 150→80, `CMD_MAX` 12→15, `ENC_QUIET_MS` 100→60. Amortiguación: `WHEEL_STATION_DAMP` 1.00→1.20 y `WHEEL_V_DB_RPS` 0.25→0.15 (la D llega más abajo). Si tiembla en movimiento, volver DAMP a 1.0. NO compilado (el usuario compila). | Usuario: "mejoró muchísimo pero va y viene como 15 veces en el ancla antes de quedar en equilibrio; y quiero ser aún más permisivo con la siesta" |
| 2026-07-26 | Core/Src/main.c | **Vaivén chico perpetuo resuelto: la zona muerta de la D (0.8 rps) era la zona CIEGA donde vivía la oscilación.** Tras subir DAMP/KP el robot amortiguaba traslaciones grandes (>0.8 rps, con esfuerzo) pero quedaba meciéndose eternamente en chico: el vaivén de 0.2–0.7 rps caía DEBAJO de `WHEEL_V_DB_RPS=0.8` (D inactiva) mientras el resorte del ancla flotante (KP=0.08) seguía inyectando energía — resorte sin amortiguador en ese régimen. Fix: `WHEEL_V_DB_RPS` 0.8→**0.25** (la D ahora cubre el régimen donde el resorte trabaja), `WHEEL_STATION_ANG_KP` 0.08→**0.05** (resorte más blando = menos energía inyectada) y `WHEEL_TRANS_DB` 2→3. El temblor en reposo que la zona muerta grande evitaba ya no es riesgo: el pico de un count suelto lo drena la EMA en ~50ms, `MOTOR_CMD_NEUTRAL` absorbe el comando chico resultante y la siesta corta todo en reposo real. NO compilado (el usuario compila). | Usuario: "está todo el tiempo en el vaivén sobre el ancla, amortigua traslaciones grandes con mucho esfuerzo pero las chicas nunca — se está moviendo tooodo el tiempo, no llega nunca al equilibrio" |
| 2026-07-26 | Core/Src/main.c | **Ajuste fino: siesta más fácil de entrar + más autoridad de amortiguación traslacional.** La 2ª forma de la siesta funcionó ("ahí está mucho mejor") pero el usuario pidió que cueste un poco menos entrar: `MOTOR_SLEEP_GYRO_ENTER` 3→4°/s, `MOTOR_SLEEP_ENTER_MS` 200→150, `MOTOR_SLEEP_CMD_MAX` 8→12, `MOTOR_SLEEP_ENC_QUIET_MS` 120→100 (ANG_ENTER queda 0.8 para no romper la histéresis con ANG_WAKE=1.0). Además la traslación seguía demorando en amortiguar: `WHEEL_STATION_DAMP` 0.70→**1.00** °/rps, `WHEEL_STATION_ANG_KP` 0.06→0.08 y tope `WHEEL_STATION_ANG_MAX` 2.0→2.5° (más autoridad de frenado). Si sobre-frena/tiembla al moverse, bajar DAMP de a 0.1. NO compilado (el usuario compila). | Usuario: "que le cueste un poooooco menos entrar a siesta (cuando entra queda muy eficiente); además demora bastante en amortiguar la traslación corrigiendo con ángulo" |
| 2026-07-26 | Core/Src/main.c | **Siesta de motores en el punto dulce (`MOTOR_SLEEP_*`, solo BALANCE_ONLY).** El rediseño sin ancla quedó **VALIDADO** ("funciona espectacular") y el usuario pidió eficiencia: en equilibrio el robot seguía corrigiendo montones de veces por segundo (desgaste + vibración). Nivel nuevo DEBAJO del hold, en `Ctrl_SalidaMotores`: con `balance_hold_active` + `|pwm_sat|<=4` + encoders sin ticks por 250ms, sostenido 300ms → **PWM 0 real** (ni el salto de deadband; el robot se para por equilibrio mecánico). Despierta al instante con CUALQUIERA de: salida del hold (error≥0.45°/gyro≥6°/s), un solo tick de encoder (= empujón, `enc_quiet` cae), o `|gyro_f|>=MOTOR_SLEEP_WAKE_GYRO=4°/s` (umbral más fino que el exit del hold, recupera autoridad antes de que la caída tome impulso). Respeta la lección del 2026-07-10 (motores muertos en zona ancha → bamboleo): despertar sensible por gyro/encoder. **2ª forma (mismo día)**: la 1ª (hold activo + 250ms sin un tick + |pwm|≤4) nunca entraba — la propia vibración de las correcciones impedía cumplirla (círculo vicioso) y con 0.3–0.5° de error residual el hold ni entraba. Ahora la siesta tiene ventana PROPIA desacoplada del hold: entra con |error|≤0.8° + |gyro|≤3°/s + 120ms sin ticks + |pwm_sat|≤8 sostenido 200ms; despierta con tick de encoder, gyro≥4°/s o |error|≥1.0°. El PID sigue corriendo dormido (pwm_sat/slew/integral con decay) — al despertar retoma sin discontinuidad. Knobs: si sigue sin dormir, subir `MOTOR_SLEEP_GYRO_ENTER` a 4 o `MOTOR_SLEEP_CMD_MAX` a 12; si bambolea dormido-despierto, bajar `MOTOR_SLEEP_WAKE_GYRO` a 3 o achicar `MOTOR_SLEEP_ANG_ENTER` a 0.6. NO compilado (el usuario compila). | Usuario: "funciona espectacular — mejoremos el punto dulce: en equilibrio sigue corrigiendo montones de veces por segundo sin necesidad, desgasta motores y vibra; desactivar motores hasta un empujón o deriva por setpoint o encoder" |
| 2026-07-26 | Core/Src/main.c | **8ª iteración — rediseño final de la traslación: SIN ANCLA ("quedate quieto donde estés").** El usuario reencuadró el objetivo: en balance puro no le importa ningún lugar de referencia — solo quedarse quieto donde sea, sin amplificar ruido ni empujones. `WheelStation_AngleCorr()` reescrita sin ancla, sin latch y sin resorte a punto fijo: (1) **D** = amortiguación de la velocidad EMA (`WHEEL_STATION_DAMP=0.70 °/rps`) con zona muerta `WHEEL_V_DB_RPS=0.8` — el pico de ~0.7 rps que mete un count suelto en la EMA queda por debajo y NO genera corrección → cero temblor en reposo, sin necesidad de latch; es quien mata los empujones. (2) **"P" flotante** = desplazamiento RECIENTE con fuga: `wheel_disp_f = disp·0.995 + Δcounts` (τ≈2s), corregido con `WHEEL_STATION_ANG_KP=0.06°/count` fuera de ±2 counts — un ancla que SIGUE al robot: frena el arrastre lento que se escapa bajo la zona muerta de la D, pero se olvida del lugar (tras un empujón queda donde murió la velocidad, no vuelve al origen; en reposo la fuga lo drena a 0 y la corrección desaparece sola). Eliminados `WHEEL_SETTLE_*`/`WHEEL_WAKE_DB` y el latch; los anclajes `wheel_pos_anchor_*` siguen existiendo pero solo para el lazo de ROTACIÓN por encoders (ahí el rumbo sí tiene referencia con sentido). NO compilado (el usuario compila). | Usuario: "no me importa dónde está el ancla — en balance puro quiero que se quede quieto en el lugar que sea, sin amplificar ruido o empujones; no debería haber ancla, concentrado en mantenerse lo más quieto posible con encoders y ángulo" |
| 2026-07-26 | Core/Src/main.c | **7ª iteración: banda del resorte angosta (10→2 counts) + latch con histéresis y re-anclaje.** Aun con DAMP=0.70 el vaivén post-empujón no moría y "no ayuda cambiando los knobs". Causa estructural: la banda muerta ancha de ±10 counts dejaba ~12 cm de "caída libre" — el robot volvía lanzado, el resorte se apagaba de golpe al entrar a la banda, la cruzaba entera (ahí solo actuaba la D, débil por cuantización), se pasaba al otro lado y el ciclo seguía. La banda ancha protegía el punto dulce, pero ese trabajo ya lo hace el latch → quedó obsoleta. Cambios en `WheelStation_AngleCorr()`: `WHEEL_TRANS_DB` 10→**2** counts (control PD continuo casi todo el recorrido); latch rediseñado con **histéresis**: se asienta cerca del ancla (`WHEEL_SETTLE_POS=8` counts ~5 cm) y quieto 300ms → **re-ancla donde quedó** (la posición final pasa a ser la nueva estación, simetriza el umbral de despertar y de paso re-cero-a la referencia del lazo de rotación — el rumbo lo sostiene el heading-hold por gyro, no le afecta) → corrección 0 exacto; despierta solo con desplazamiento real > `WHEEL_WAKE_DB=10` counts (~6 cm) desde el punto de asentado. NO compilado (el usuario compila). | Usuario: "sigo sin poder amortiguar los empujones, demora demasiado, deriva demasiado, y no ayuda cambiando solamente los rps del encoder" |

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
- **4x quadrature por software:** ambos canales A y B decodificados por transición de estados. ENC_CPR=28. Sin modo encoder de hardware (requeriría cambio de pines)
- **Encoders por polling (TIM2 @ 4kHz) en lugar de EXTI (2026-07-01):** el esquema anterior por interrupción de flanco con masking anti-storm (EXTI + TIM5 re-habilitando cada 2ms) compartía prioridad NVIC con el DMA de I2C y quedó sospechado de causar freezes/resets recurrentes durante el giro de 180°, posiblemente por ráfagas de interrupciones o una carrera de lectura-modificación-escritura sobre `EXTI->IMR` entre distintos niveles de prioridad. El muestreo periódico tiene una tasa de interrupción fija y acotada (4000/seg) independiente de la velocidad física de la rueda, eliminando esa clase de problema de raíz, a costa de una resolución temporal ligeramente menor (peor caso 250µs de retraso en detectar un flanco, insignificante frente a los 10ms del loop de control)
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
- **Odometría (2026-07-04, cerrado en Qt 2026-07-06)**: `GET_ODOMETRY=0xDA` (respuesta: 3 floats LE = x[m], y[m], θ[°]) y `RESET_ODOMETRY=0xDB` (ACK) ya tienen handlers en Qt (`mainwindow.h`/`mainwindow.cpp` — el proyecto Qt no usa `serialmanager.cpp`/`udpmanager.cpp` separados, todo el envío/parsing UNER está en `mainwindow.cpp`). Disponibles desde el combo `comboBox_CMD` ("GET ODOMETRY"/"RESET ODOMETRY"); la respuesta se loguea en `textEdit_PROCCES`
- **⚠️ `line_state` renumerado (2026-07-16)**: al borrar `OBJ_ESPERA_REVERSA`/`OBJ_RETROCESO`/`OBJ_ARC` del enum, los valores numéricos de los estados OBJ_* que viajan en `WifiOdomData_t.line_state` cambiaron (FRENO_REVERSA pasó de 15 a 13, y todos los posteriores bajan; los estados previos a OBJ no cambian). Si Qt interpreta ese número (p. ej. para la pared 3D o para mostrar el nombre del estado), actualizar el mirror del enum en `mainwindow.h`/`.cpp`
- **Push de odometría por WiFi (2026-07-06)**: `CMD_WIFI_ODOM_DATA=0xDC` (struct packed `WifiOdomData_t`, ver "Canal 2 — WiFi UDP") ya tiene mirror y parsing en Qt (`mainwindow.h`/`mainwindow.cpp`) — pestaña "Odometría (WiFi)" con mapa XY. Si se cambia el layout de `WifiOdomData_t` en `UNER.h`, hay que actualizar el mirror en `mainwindow.h` (mismo patrón que `WifiLogData_t`)
- **Clamps nuevos del 2026-07-24 (solo informativo, sin cambio obligatorio en Qt)**: `MODIFY_SETPOINT=0xC8` ahora se limita en firmware a ±10° y `MODIFY_KV_BRAKE=0xBF` a 0..100 (recién ahora funciona — antes ni siquiera tenía handler). Recomendado en Qt: acotar `spinBox_SETPOINT` (hoy ±180°, `mainwindow.ui`) a ±10° para que la UI refleje el rango real aceptado
- **`GETSPEED=0xA4` (2026-07-24)**: implementado en ambos lados EN LA MISMA SESIÓN — firmware responde 3 floats LE (vel [m/s, + = adelante], rueda D [rps], rueda I [rps]) y Qt ya tiene el case de parseo en `decodeData` (`mainwindow.cpp`, loguea en `textEdit_PROCCES` y detecta firmware viejo por longitud). Si se cambia el layout de la respuesta, tocar ambos
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
