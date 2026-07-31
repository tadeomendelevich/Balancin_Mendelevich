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
| Velocidad objetivo en línea | `LINE_SPEED_TARGET` | 2.50 m/s por defecto; ajustable en runtime con `MODIFY_LINE_SPEED=0xC4`, limitado a **0.20..8.00 m/s** (era 4.00 hasta 2026-07-27) |
| Tope de inclinación (setpoint dinámico) | `SP_LIMIT_DEG` | 5.0° por defecto; ajustable con `MODIFY_SP_LIMIT=0xC9`, limitado a 1..15°. Rige LINE_FOLLOWING/BALANCE_ONLY/IDLE (MANUAL usa 6°, BALANCE_AND_SPEED 2°). **No escala solo con la velocidad**: a velocidades altas conviene subirlo a mano |

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
- **Estación por rueda (Opción A) — heading-hold por gyro agregado 2026-07-26, pendiente validar en el robot.** Historia del día: (1) knobs de rotación (`WHEEL_ROT_DB` 3→2, `WHEEL_ROT_KP` 0.15→0.25); (2) la TRASLACIÓN fallaba asimétrica (adelante nunca frenaba) → se movió del trim de PWM al SETPOINT de inclinación (`WheelStation_AngleCorr()`, patrón PAUSA_GIRO) — **VALIDADO en el robot: "el PID corrige muchísimo mejor"**; (3) quedaba un giro lento constante a la derecha (micro-patinaje invisible para los encoders + zona muerta de 3°/s del yaw-assist) → **heading-hold por gyro Z**: `wheel_yaw_deg` integra el rumbo desde el ancla y un P (`WHEEL_YAW_KP=2.0`, DB=1°, tope 12°/s eq) se suma al canal del yaw-assist con su mismo signo. (4) 3ª iteración: D siempre activa en la traslación (`WHEEL_STATION_DAMP=0.25 °/rps`) — **VALIDADA: "la deriva traslacional quedó bastante resuelta, se amortigua bastante"**. (5) 4ª iteración ("desde la base"): bias del gyro medido SOLO en reposo real (congelado durante balance) + hold PI — **VALIDADA: "quedó muy bien el tema del giro, no lo hace más"**. (6) 5ª iteración: warmup rápido del bias (~0.3–0.5s de IDLE alcanzan, validado) y DAMP 0.25→0.45 (mejoró pero seguía lento y temblaba en reposo). (7) 6ª/7ª iteración: latch de asentado, DAMP 0.45→0.70, banda angosta + re-anclaje — seguía sin amortiguar bien. (8) 8ª iteración — **rediseño final SIN ANCLA, VALIDADO ("funciona espectacular")**: D de velocidad con zona muerta de 0.8 rps + desplazamiento reciente con fuga τ≈2s ("ancla flotante"). (9) **Siesta de motores** (`MOTOR_SLEEP_*`, solo BALANCE_ONLY): PWM 0 real en el punto dulce hasta empujón/deriva. 2ª forma (ventana propia desacoplada del hold) validada ("mucho mejor"); 3ª pasada del 2026-07-26: entrada aún más fácil (gyro≤4°/s, 100ms sin ticks, |pwm|≤12, 150ms sostenido) y más amortiguación traslacional (`WHEEL_STATION_DAMP`=1.00, KP=0.08, tope 2.5°). 4ª pasada: zona ciega de la D corregida (`WHEEL_V_DB_RPS` 0.8→0.25, KP 0.08→0.05) — "mejoró muchísimo". 5ª pasada: quedaban ~15 cruces de llegada → siesta como asentador (ventanas cortas: gyro≤5, 60ms sin ticks, |pwm|≤15, 80ms sostenido — la pesca en los extremos del vaivén donde v≈0 y la fricción estática lo mata en 2-3 cruces) + DAMP 1.0→1.2 + `WHEEL_V_DB_RPS` 0.25→0.15. **6ª pasada (2026-07-27, a validar)**: la 5ª sobre-corrigió — reporte: "no entra casi nunca a la siesta y se va de a poco para los costados". Con V_DB 0.15 + DAMP 1.2, un count suelto pateaba el setpoint ~0.66° y cada tick reseteaba las ventanas de la siesta (círculo vicioso). Fix: `WHEEL_V_DB_RPS` 0.15→0.25 y `DAMP` 1.2→1.0 (vuelven a los valores validados; la siesta permisiva queda intacta y es quien asienta los cruces), y contra la deriva lenta invisible `WHEEL_DISP_LEAK` 0.995→0.998 (τ≈5s: con τ≈2s toda deriva <~5 mm/s se drenaba antes de superar `WHEEL_TRANS_DB` y nunca se corregía; ahora el umbral baja a ~2 mm/s). **7ª pasada (2026-07-27) — VALIDADA ("la mejora es una locura, amortigua los empujones muy rápido")**: la 6ª empeoró ("reacciona tarde") → causa raíz estructural: la corrección viajaba dentro de `base_setpoint_target` y la rampa de `sp_step_max=0.1°/ciclo` + la EMA de velocidad le metían ~150ms de retraso — la D llegaba en contrafase y bombeaba el vaivén (por eso subir DAMP siempre empeoraba). Fix: corrección aplicada POST-rampa directo sobre `dynamic_setpoint_f` al final de `Ctrl_SetpointDinamico`, con slew propio `WS_STEP=0.4°/ciclo`. **VALIDADA** ("la mejora es una locura, amortigua los empujones muy rápido") — commit+push `6a286a6`. **8ª pasada (2026-07-27)**: quedaba brusquedad al llegar al punto dulce (se pasaba del equilibrio justo antes de la siesta) → rodilla suave `WHEEL_V_SOFT_KNEE=1.0` rps: debajo, la ganancia de la D escala lineal con \|v\| (cuadrática en chico, entera en grande — un empujón real recibe la D completa). **No alcanzó** ("sigue demasiado brusco y potente, no entra nunca a la siesta") — atacaba el término equivocado. **9ª pasada (2026-07-27, a validar)**: el que empujaba en el punto dulce era el **P saturado**: tras un empujón `wheel_disp_f` llega a ~90 counts (×0.05 = 4.5°, clampeado al tope general de 2.5°) y con la fuga de τ≈5s seguía inclinando 2.5° de vuelta durante segundos → error nunca < 0.8°. Fix: tope propio `WHEEL_STATION_P_MAX=0.5°` para el P (rol real: frenar deriva de mm/s; 0.5° alcanza y queda debajo del umbral de la siesta) — la D conserva sus ±2.5° validados contra empujones; la rodilla queda. **VALIDADA a medias** ("quedó suave") pero la siesta seguía sin entrar NUNCA. **10ª pasada (2026-07-27, a validar)**: el residuo del P (≤0.5°, fuga τ≈5s) mantenía al robot GATEANDO de vuelta durante segundos — cada tick reseteaba la ventana de la siesta (que en sí está bien calibrada, era alcanzable antes de la estación). Fix: drenaje rápido en reposo real en `WheelStation_AngleCorr()` — ambas ruedas >`WHEEL_DISP_CALM_MS=150`ms sin tick + \|v\| bajo la zona muerta → `wheel_disp_f` se drena con `WHEEL_DISP_FAST_LEAK=0.90` (τ≈100ms, muere en ~0.3s); la deriva lenta real (~cm/s) tiquea cada <150ms y no dispara el drenaje. **No alcanzó** — el reporte siguió igual. **11ª pasada (2026-07-27, a validar)**: fin del suavizado término-por-término (pasadas 8-10: rodilla, tope del P, drenaje — siempre quedaba un término activo perturbando) → **factor de CALMA global**: `calm = min(1, max(|gyro_f|/8°/s, |v_trans|/1.0rps))` multiplica TODA la corrección — quieto = estación muda (cero perturbación en el punto dulce), empujón = gyro dispara al instante → estación entera. Rodilla eliminada (el calm la subsume); tope del P y drenaje quedan. **No alcanzó** — con la estación ya muda el síntoma siguió: el culpable estaba en OTRA capa. **12ª pasada (2026-07-27, a validar)**: la PATADA de compensación de deadband en `Ctrl_SalidaMotores` — con rueda parada, un comando de 3 PWM saltaba a 11 (offset estático completo de golpe): relé oscilando en el punto dulce que reseteaba la siesta eternamente. Fixes: `MOTOR_CMD_NEUTRAL` 2→4 (chatter de 3-4 PWM → 0 real) + rampa `MOTOR_DB_RAMP_GAIN=2` (offset proporcional al exceso, pleno recién con \|cmd\|≥8). **VALIDADA** ("quedó mucho mejor"). **13ª pasada (2026-07-27, a validar)**: más suavidad pedida → `MOTOR_DB_RAMP_GAIN` 2→1 (offset pleno recién con \|cmd\|≥12). **14ª pasada (2026-07-27, a validar)**: la rueda derecha se movía sola sin parar (la izquierda muda) — el canal diferencial (contra-torque retenido del integrador de yaw + PD de rotación) dejaba una rueda siempre arriba de la zona neutra → **calma rotacional** (`rot_calm = min(1, max(\|yaw_rate\|/2°/s, \|v_rot\|/0.5rps))` multiplica yaw-hold y PD de rotación; el I solo acumula con calma>0.3). **15ª pasada (2026-07-27)**: empujón FUERTE lo volteaba → fade de supervivencia por gyro>40°/s. **FRACASÓ** ("no frena nunca"): toda recuperación normal supera 40°/s de gyro → estación siempre muda; NO reintentar gates por gyro alto. **16ª pasada (2026-07-27) — VALIDADA y cierre del paquete ("quedó muy bueno, se queda en el punto dulce montón de tiempo, me encanta")**: fade eliminado; el vuelco se resuelve por TECHO: `WHEEL_STATION_ANG_MAX` 2.5→1.5 — el freno de la D sigue proporcional a v (1°/rps, idéntico hasta 1.5 rps), solo se recorta el extremo que apilado con el freno clásico volcaba. Knobs: aún vuelca → 1.5→1.2; frena poco → 1.5→1.8. **Estado final validado en el robot: tag V27** (pasadas 7-16: post-rampa + calma traslacional/rotacional + zona neutra 4 + rampa de deadband + techo 1.5°). El cuadro de knobs de la ventana de siesta quedó explicado en la sesión del 2026-07-27; regla clave: `ANG_ENTER<ANG_WAKE`, y ojo al solape `GYRO_ENTER=5>WAKE_GYRO=4` (si cicla dormido/despierto: WAKE_GYRO→6 o GYRO_ENTER→3). Knobs restantes de suavidad: `MOTOR_CMD_NEUTRAL` 4→5, soft zone `scale_min` 0.35→0.30; retrocesos: bamboleo lento → NEUTRAL 4→3, flojo para arrancar la rueda → RAMP_GAIN 1→2, si vuelve el giro lento constante → `WHEEL_ROTCALM_GYRO_REF` 2→1. Al validar: commit + tag (¿V27?). Knobs: brusco al asentarse → rodilla 1.0→1.5; vaivén chico que no muere → rodilla 1.0→0.6; tiembla en movimiento → DAMP 1.0→0.9; siesta cicla dormido/despierto (despierta enseguida tras dormir) → `MOTOR_SLEEP_ENTER_MS` 80→120 o `MOTOR_SLEEP_ANG_ENTER` 0.8→0.6; sigue cruzando mucho → `WHEEL_STATION_ANG_KP` 0.05→0.03; si tras esto sigue derivando lento → `WHEEL_TRANS_DB` 3→2 o `WHEEL_DISP_LEAK` 0.998→0.999. Al validar todo el paquete del día: commit + tag (¿V27?). Todo commiteado y pusheado en **2dc5a7b** (2026-07-26); última versión validada completa en el robot: tag **V26** (ba790d5). Al validar esta pasada conviene taggear (¿V27?). Kill-switch `wheel_pi_enabled=0` apaga los tres mecanismos (setpoint, rot por encoders y heading-hold PI). Pendiente aparte: extender la estación a línea/manual si aplica.
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
| 2026-07-30 | Core/Src/main.c, Core/Src/UNER.c, Core/Src/MPU6050.c | **Comentarios de fin de línea nombrando la técnica/fórmula de cada cuenta (~90 líneas, solo comentarios — cero cambio funcional).** El usuario agregó a mano un `// Media movil exponencial` en el calibrador de bias del gyro y pidió replicar ese estilo (corto, al final de la línea, nombrando QUÉ es la fórmula) en la mayor cantidad de lugares posibles, porque le facilita muchísimo leer el firmware. Cubierto: **filtros** (EMA en velocidad rápida/lenta, accel_motion, lat_tilt, bias de odometría, error de línea, velocidad por rueda; filtro complementario con sus dos mitades pasa-altos/pasa-bajos; mediana + insertion sort del ADC); **PID/PI** (los cuatro lazos: balance, línea, steering por encoders, y los PI de velocidad de LOST_FWD/EDGE_FWD/OBJ_WALL×3/MANUAL — cada uno con P/I/D, integración rectangular `I += e*dt`, anti-windup por saturación y por integración condicional, descarga exponencial, ganancia asimétrica acelerar/frenar); **geometría** (atan2 del roll/pitch/lateral y por qué atan2 y no atan, Pitágoras, centroide como promedio ponderado + normalización, odometría: integración de Euler + regla del trapecio + descomposición polar→cartesiano + wrap a ±180°, navegación de retorno: distancia euclídea, rumbo y giro más corto); **conversiones** (análisis dimensional counts→rps→m/s, escalado del gyro, regla de tres duty→CCR y ADC→píxeles, punto fijo Q14 del MPU, recomposición big-endian); **utilidades de control** (saturación/clamp, zona muerta como "exceso", limitadores de slew-rate y sus rampas, interpolaciones lineales/lerp, decaimientos exponenciales, mezcla diferencial común±steering, semisuma=traslación / semidiferencia=rotación, calma traslacional y rotacional como normalización 0..1); **protocolo** (checksum XOR de RX y TX y por qué el orden no importa, buffer circular por máscara); **tabla de cuadratura** de los encoders (índice = 4·estado_anterior + estado_actual). NO compilado (el usuario compila). | Usuario: "hay comentarios que me ayudarían montón, como el de media exponencial que agregué acá, aclaran qué es y me ayudan a la interpretación — a la derecha luego de la línea, cortito y resumido, fijate dónde podés agregarlo en la mayor cantidad de lugares posibles" + "mientras más mejor!!!" |
| 2026-07-27 | Core/Inc/UNER.h, Core/Src/UNER.c, Core/Src/main.c, (Qt: mainwindow.h/.cpp) | **`MODIFY_SP_LIMIT=0xC9` nuevo (tope de inclinación ajustable desde Qt) + tope de velocidad de línea 4→8 m/s.** (1) **0xC9**: el `sp_limit` de `Ctrl_SetpointDinamico` (inclinación máxima para acelerar/frenar, hasta hoy la constante `5.0f`) pasó a la global `float SP_LIMIT_DEG = 5.0f`, enlazada por el binding nuevo `sp_limit` y modificable en runtime; el case en `UNER.c` valida con `getF32BoundedFromRx` a **1..15°** (`SP_LIMIT_CMD_MIN/MAX_DEG`) y responde UNKNOWN si no hay binding (patrón de `MODIFY_BETA_*`, no ACK mentiroso). Rige LINE_FOLLOWING/BALANCE_ONLY/IDLE; MANUAL (6°) y BALANCE_AND_SPEED (2°) conservan sus topes propios, y el freno hacia atrás en línea sigue acotado aparte por `LINE_BRAKE_ANGLE_MAX=3°`. (2) **`LINE_SPEED_CMD_MAX_MPS` 4.00→8.00 m/s**: el usuario quería pasar de 4 m/s y el clamp del firmware lo recortaba en silencio aunque Qt lo permitiera. Se queda en 8 y no más porque el corte de emergencia (`LINE_SPEED_EMERGENCY_LIMIT`) está en 10 m/s: pedir un objetivo ≥ a ese corte sería autodestructivo. **Contexto**: ninguno de los parámetros de frenado escala con la velocidad objetivo (ángulo, rampas `LINE_SP_STEP_*`, ganancias del PI, margen y horizonte de la guarda predictiva son absolutos), así que a velocidades altas el robot queda relativamente sub-frenado — de ahí que tenga sentido poder subir el tope de inclinación a mano. NO compilado (el usuario compila). | Usuario: "quiero que me agregues sp_limit al Qt para poder cambiarlo de ahí, agregá la lógica al archivo UNER también; y sacame el límite de 4 m/s que tengo en Qt para poder llevarlo a más" |
| 2026-07-27 | Core/Src/main.c | **BUG CRÍTICO del esquive corregido: "PARADO" instantáneo al perder la pared — el reset del timer de pared-perdida decía `10` en vez de `0`.** Reporte: en pleno esquive, apenas el lateral deja de ver la pared, salta de `PARED>AVZ` directo a `PARADO` (`LINE_STATE_GIVEN_UP`), en vez de girar hacia la pared. Causa (en `Ctrl_LatchesPared`, etapa 3): con la pared visible el timer se "reseteaba" a **10** — un valor NO nulo, y la rama de abajo usa `== 0` como "timer sin arrancar". Al perder la pared se salteaba el arranque del timer y se evaluaba `HAL_GetTick() - 10` (≈ el uptime completo) contra los 5000 ms → verdadero desde el segundo 5 de encendido → `GIVEN_UP` **en el mismo ciclo** en que se pierde la pared. Y como la etapa 3 corre ANTES de los handlers de `line_state` (etapa 15), el estado ya era GIVEN_UP cuando `LineState_ObjBordearPared` iba a hacer su transición legítima `!wall_visible → OBJ_PARED_LIBRE → OBJ_GIRO_PARED`: el giro nunca llegaba a ejecutarse. Fix: reset a `0` (+ comentario explicando por qué DEBE ser 0). **Origen: commit `165f1ac` (2026-07-24)**, donde el `0→10` entró suelto, sin relación con el propósito de ese commit (estación por rueda + auditoría UNER) — no fue de la sesión de hoy; el diagnóstico anterior (que el timeout saltaba por un giro lento a causa del bamboleo) era **incorrecto**. NO compilado (el usuario compila). | Usuario: "se queda en estado PARADO de golpe en el medio del esquive, va de PARED>AVANZA a PARADO ni bien pierde de vista la pared — debería girar al lado de la pared al toque" |
| 2026-07-27 | Core/Src/main.c | **Bamboleo post-giro de 90° del esquive: la salida suave era GLOBAL — ahora es solo de BALANCE_ONLY.** Reporte: tras el primer giro de 90° del esquive de objeto el robot bamboleaba adelante/atrás. Causa: la zona neutra 4 + rampa del offset (12ª/13ª) aplicaban en TODOS los modos — en línea, los comandos medianos de la recuperación post-pivot (5-11 PWM) salían debilitados (un cmd 5 pasó de salir 13 a salir 6) → correcciones flojas → sub-amortiguado. Fix en `Ctrl_SalidaMotores`: flag `out_soft = (robot_state==BALANCE_ONLY)` — solo ahí rigen `MOTOR_CMD_NEUTRAL=4` + rampa; los modos dinámicos (línea/manual/speed) usan `MOTOR_CMD_NEUTRAL_DYN=2` + offset completo de golpe (comportamiento histórico pre-2026-07-27, el que el seguidor tenía validado). NO compilado (el usuario compila). | Usuario: "luego del giro de 90° en el esquive de objeto se va bastante para atrás y adelante, se bambolea — ¿cambiamos algo de ahí?" |
| 2026-07-27 | Core/Src/main.c | **Auditoría de coherencia post-V27 entre modos + 2 fixes.** Pedido: verificar que los cambios del día (estación, zona neutra, rampa de deadband) no rompan línea/manual/speed. Mapa verificado: LINE→`Ctrl_MotoresLinea`, MANUAL/override→`Ctrl_MotoresManual`, resto (BALANCE_ONLY/**BALANCE_AND_SPEED**/IDLE)→`Ctrl_MotoresBalance`. **Fix 1**: el bloque `wheel_pi_enabled` de `Ctrl_MotoresBalance` (rotación por encoders + heading-hold yaw) corría TAMBIÉN en BALANCE_AND_SPEED — ahora gateado a BALANCE_ONLY/IDLE (mismo alcance que el eje de traslación; en SPEED anclaba el rumbo a la pose de entrada y pelearía con la maniobra). **Fix 2**: el slew post-rampa (`wheel_station_corr_f`, estático) arrastraba hasta 1.5° de corrección vieja durante ~40ms al setpoint del modo nuevo tras un cambio de modo en pleno frenado — ahora se resetea a 0 cuando `!wheel_pos_armed` (cambio de modo o caída). **Verificado sin cambios**: siesta gateada a BALANCE_ONLY (entrada Y despertar chequean `robot_state`); zona neutra 4 + rampa de deadband son globales pero coherentes (en línea/manual los comandos típicos son ≥12 → offset pleno igual que antes; en movimiento rige el db cinético 4 y el anti-stall por setpoint no pasa por PWM); `wheel_disp_f` arranca fresco en cada re-entrada (reset en `!armed`); `wheel_yaw_int` se resetea al re-anclar; `wheel_pos_armed=0` al entrar a IDLE/BALANCE/SPEED/MANUAL y al recuperarse de caída (fix 2026-07-26 intacto); en IDLE la salida corta motores más allá de lo que compute la estación. NO compilado (el usuario compila). | Usuario: "cambiamos el PID de una rueda a las dos — verificá que todos los estados hayan quedado coherentes (seguidor de línea, manual y demás) y corregí lo necesario" |
| 2026-07-27 | Core/Src/main.c | **16ª pasada — fade de supervivencia ELIMINADO (mataba el freno) y reemplazado por techo más bajo: `WHEEL_STATION_ANG_MAX` 2.5→1.5.** La 15ª quedó muy mal ("ahora no frena nunca"): el gate de gyro>40°/s estaba conceptualmente mal — CUALQUIER recuperación normal de empujón supera 40°/s de gyro (el vaivén de recuperación siempre es violento en gyro aunque el empujón sea moderado), así que `panic≈1` durante toda respuesta y la estación quedaba muda justo cuando tenía que frenar. Eliminados los `WHEEL_PANIC_*` y el factor `(1-panic)`; queda documentado en comentario para no reintentarlo. El pedido real ("frenar como antes pero proporcional a la fuerza del empujón") ya lo cumple la D por construcción — es lineal en v (1°/rps) — el problema del vuelco era solo el TECHO: con 2.5° de estación + freno clásico apilados el total volcaba. Ahora tope 1.5°: el freno conserva la proporcionalidad validada en todo el rango útil (hasta 1.5 rps da lo mismo que antes) y el balance conserva margen para sobrevivir el empujón fuerte. Knobs: aún vuelca con empujón fuerte → 1.5→1.2; frena poco → 1.5→1.8. NO compilado (el usuario compila). | Usuario: "quedó muy mal, ahora no frena nunca — debería frenar como antes pero de forma proporcional a lo fuerte del empujón" |
| 2026-07-27 | Core/Src/main.c | **15ª pasada — fade de SUPERVIVENCIA: en empujones fuertes la estación se caía a sí misma.** Reporte: con un empujón fuerte (sobre todo hacia adelante) la respuesta era tan agresiva que el robot se caía. Causa de diseño: el factor de calma de la 11ª usa el gyro para DESPERTAR la estación en un empujón — pero en uno fuerte la recuperación es violenta (gyro 50-100°/s, error grande) y eso mantenía la estación a plena autoridad (±2.5° de setpoint) justo cuando el PID de balance necesita la referencia limpia para sobrevivir; encima se APILA con el freno clásico de `ComputeBrakeSetpointTarget` (KV_BRAKE_STRONG) que también inclina para frenar → doble freno → vuelco. Fix en `WheelStation_AngleCorr()`: **`panic = clamp01(max((\|gyro_f\|-40)/(80-40), (\|error\|-3)/(6-3)))`** y la corrección se multiplica por `calm × (1-panic)`: entre 40 y 80°/s de gyro (o 3 a 6° de error) la estación se retira linealmente hasta quedar muda; el rescate del empujón fuerte lo hacen el PID de balance + el freno clásico (validado hace meses) y la estación retoma sola al moderarse la recuperación. Los empujones moderados (gyro < 40°/s) conservan exactamente la amortiguación validada. Knobs: si aún se cae con empujón fuerte → bajar `WHEEL_PANIC_GYRO_LO` 40→30 y `ANG_LO` 3→2.5; si los empujones moderados perdieron amortiguación → subir LO. NO compilado (el usuario compila). | Usuario: "cuando recibe un empujón se comporta demasiado agresivo, tanto que se cae por la respuesta — debería siempre mantener equilibrio; se cae más que nada cuando lo empujo fuerte hacia adelante" |
| 2026-07-27 | Core/Src/main.c | **14ª pasada — calma ROTACIONAL: el canal diferencial dejaba una rueda empujando sola (la derecha se movía sin parar, la izquierda muda).** Reporte: "el motor derecho se mueve todo el tiempo y el izquierdo no; el derecho rompe el equilibrio". Causa: el canal diferencial (heading-hold PI + PD de rotación en `Ctrl_MotoresBalance`) — el integrador de yaw conserva a propósito un contra-torque CONSTANTE (se congela dentro de `WHEEL_YAW_DB` "conservando el contra-torque") y ese diferencial permanente se suma a una rueda y se resta a la otra: con `MOTOR_CMD_NEUTRAL=4`, la rueda favorecida (derecha, vía mL) queda siempre arriba del umbral (empuja sin parar) y la otra abajo (0) — asimetría perpetua que rompe equilibrio y siesta. Fix (mismo patrón que la calma traslacional de la 11ª): **`rot_calm = min(1, max(\|yaw_rate\|/WHEEL_ROTCALM_GYRO_REF=2°/s, \|v_rot\|/WHEEL_ROTCALM_V_REF=0.5rps))`** multiplica el yaw-hold completo (P+I) y el PD de rotación; además el I solo acumula con `rot_calm > WHEEL_ROTCALM_INT_GATE=0.3` (anti-windup: antes, quieto con rumbo fuera de la DB, el I crecía hasta el tope y al despertar pegaba un diferencial grande). Sin giro real → canal mudo → las dos ruedas quietas; girando de verdad → canal entero (el heading-hold validado no pierde autoridad donde importa). Reordenado el bloque: drift/rot/v_rot se calculan antes del yaw-hold para alimentar la calma. NO compilado (el usuario compila). | Usuario: "el motor derecho se mueve todo el tiempo y el izquierdo no, se queda quieto el izquierdo, pero el derecho es el que rompe el equilibrio o balance" |
| 2026-07-27 | Core/Src/main.c | **13ª pasada — más suavidad aún: `MOTOR_DB_RAMP_GAIN` 2→1.** La 12ª quedó **VALIDADA** ("quedó mucho mejor") y el usuario pidió suavizar todavía más el punto dulce. La rampa de compensación de deadband ahora sube con pendiente 1: comando 5 → +1 (total 6), comando 8 → +4, offset estático completo recién con \|cmd\| ≥ 12 — toda la franja de correcciones chicas empuja mínimo; los comandos grandes de un empujón real no cambian. Si quedara flojo para arrancar la rueda en correcciones medianas (el robot "se cuelga" antes de reaccionar), volver a 2. Próximos knobs de suavidad si hiciera falta más: `MOTOR_CMD_NEUTRAL` 4→5, o el soft zone del PID (`scale_min` 0.35→0.30). NO compilado (el usuario compila). | Usuario: "ok, quedó mucho mejor — igual me gustaría suavizar más aún la respuesta en el punto dulce" |
| 2026-07-27 | Core/Src/main.c | **12ª pasada — el brusco del punto dulce NO era la estación: era la PATADA de compensación de deadband de los motores.** Con la estación ya muda en calma (11ª) el síntoma siguió → el culpable estaba en `Ctrl_SalidaMotores`: con la rueda parada (>80ms sin ticks) cualquier comando que superara `MOTOR_CMD_NEUTRAL=2` saltaba de golpe a comando+8 (offset estático) — una micro-corrección de 3 PWM del PID se convertía en una patada de **11 PWM** contra un robot quieto → sacudida → gyro>5°/s + ticks → ventana de siesta reseteada → el PID corregía la sacudida → otra patada: relé oscilando en el punto dulce, exactamente el mecanismo del bamboleo del 2026-07-10 pero en versión chica. Dos fixes: (1) `MOTOR_CMD_NEUTRAL` 2→**4** (el knob que el propio comentario del código documentaba para "si sigue temblando"): el chatter de 3-4 PWM va a 0 real — motores quietos, sin ticks, la siesta por fin tiene su ventana; el riesgo de bamboleo lento hoy lo atajan hold+siesta, si aparece volver a 3. (2) **Rampa de la compensación** (`MOTOR_DB_RAMP_GAIN=2`): apenas por encima de la zona neutra el offset entra proporcional al exceso (`comp=min(db,(\|cmd\|-4)×2)`) en vez de completo de golpe — offset pleno recién con \|cmd\|≥8; los comandos chicos empujan suave, los grandes (empujón real) no cambian. NO compilado (el usuario compila). | Usuario: "sigue sin entrar, no sé cuál será el problema — no llega a ser tan suave el PID en el punto dulce, no le da la suavidad" |

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
- **`MODIFY_SP_LIMIT=0xC9` (2026-07-27)**: implementado en ambos lados EN LA MISMA SESIÓN — firmware recibe float32 en grados (clamp 1..15) y Qt ya tiene la fila "Tope inclinacion" en el panel de línea, el ítem en `comboBox_CMD`, el case de envío y el parseo del ACK/UNKNOWN. Si se cambia el rango en `UNER.c` (`SP_LIMIT_CMD_MIN/MAX_DEG`), actualizar el `setRange` de `spLimitSpinBox` y el `QInputDialog::getDouble` en `mainwindow.cpp`
- **Tope de velocidad de línea 4→8 m/s (2026-07-27)**: `LINE_SPEED_CMD_MAX_MPS` en `UNER.c` y, del lado de Qt, `lineSpeedSpinBox->setRange` y los dos `QInputDialog::getDouble` del combo. **Los tres deben moverse juntos**: si Qt permite más de lo que acepta el firmware, el clamp recorta en silencio y el usuario no se entera
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
