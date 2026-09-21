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
| Encoders | Cuadratura (modelo desconocido) | 4x quadrature por muestreo periódico (polling) a 4 kHz vía TIM2: PA8/PB13 (derecho), PB14/PB15 (izquierdo). `ENCODER_COUNTS_POR_VUELTA=28` conteos/rev |

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
│   │   ├── display_oled.c               ← ocho pantallas, alertas, refresco y adaptador I2C/DMA del OLED
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
│       ├── display_oled.h               ← API del display y foto de datos de solo lectura
│       ├── robot_estados.h          ← enums compartidos, mismos valores del protocolo
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

## Wi-Fi Station / SoftAP - 2026-09-16

SoftAP seleccionable por KEY al arrancar o comando 0xDD en IDLE; Station por defecto. Driver AT secuencial, descubrimiento de cliente UDP, plazos reales, parser IPD y cola de ACK. Corrige campos WifiLogData_t desalineados del refactor local. Compilado Debug/Release; tests host ESP01/UNER. Pendiente prueba fisica. Ver SOFTAP.md.

El arranque normal es Station; KEY al encender selecciona SoftAP. Red Balancin, WPA2, IP 192.168.4.1. El selector de Qt elige busqueda; Aplicar por USB solicita el cambio al robot detenido. No hay modo hibrido ni fallback automatico. La seleccion dura hasta reiniciar.

---

## Comunicación con Qt
### Canal 1 — USB CDC
| Campo | Valor |
|-------|-------|
| Periférico STM32 | USB OTG FS (PA11=DM, PA12=DP), clase CDC |
| Baudrate | N/A (USB CDC, velocidad nativa USB FS) |
| Formato de trama RX | Protocolo UNER binario: `"UNER"` + nBytes + `':'` + cmd + payload + checksum |
| Formato de trama TX | Mismo protocolo UNER; telemetría CSV decimada (LOG_DECIMACION=5 → ~20 Hz a 100 Hz loop) |

### Canal 2 — WiFi UDP
| Campo | Valor |
|-------|-------|
| IP del PC destino (Qt) | **Variable según ubicación** — ver tabla de perfiles abajo |
| Puerto de escucha STM32 (RX) | `30000` (LocalPORT en `ESP01_StartUDP`) |
| Puerto de envío al PC (TX) | `30010` (RemotePORT en `ESP01_StartUDP`) |
| Frecuencia de telemetría | ~10 Hz (LOG_WIFI_DECIMACION=10 sobre loop de 100 Hz), solo si `ACTIVATE_WIFI_LOG` está activo |
| Formato paquete WiFi | `WifiLogData_t` binario packed: t_ms, roll, output, PID terms, mR, mL, dt, dyn_sp, line data, 4×ADC |
| Push de odometría (2026-07-06, +ADC de objeto y roll 2026-07-08) | `WifiOdomData_t` (cmd `0xDC`, packed: seq, t_ms, x_m, y_m, theta_deg, line_error, line_detected, robot_state, line_state, adc5, adc6, adc7, adc8, roll_deg, lat_deg) enviado cada `WIFI_ODOM_PERIODO_MS=500ms` automáticamente en cuanto `f_wifi_connected=1` — **no depende de `ACTIVATE_WIFI_LOG`**, pensado para graficar mapa XY + posición de línea + barrera/obstáculo frente al robot + inclinación en la Vista 3D de Qt sin competir por ancho de banda/CPU con la telemetría de control a 10 Hz |

> ⚠️ **La IP del PC destino cambia según la red donde se trabaje.** Antes de flashear, verificar
> que el perfil activo coincida con la red. Qt nuevo descubre el destino; la IP fija queda para Qt anterior.
> **Desde 2026-07-10, un solo lugar para cambiar de red:** en `main.c` (~línea 423) hay una tabla
> `wifiProfiles[]` (SSID + password + IP en una sola fila por red) y una macro
> `#define WIFI_PERFIL_ACTIVO <n>` — para cambiar de red alcanza con cambiar ese número, ya no
> hay que tocar/comentar SSID, password e IP por separado en tres lugares. `ESP01.c` tenía un
> `SERVER_IP` propio (fallback de reconexión, hoy código muerto) que se sacó ese mismo día — ya
> no hay ninguna otra IP hardcodeada en el firmware fuera de `wifiProfiles[]`.
>
> | # | Red | SSID | IP del PC |
> |---|-----|------|-----------|
> | 0 | FCAL / Universidad | `FCAL` | `172.23.205.98` |
> | 1 | Casa (activo) | `MEGACABLE FIBRA-2.4G-ckd0` | `192.168.100.5` |
> | 2 | Delco Mendelevich | `Delco_Mendelevich` | `192.168.1.23` |
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
| Integral anti-windup | `INTEGRAL_MAX` | ±100.0 PWM units; sin decay libre — solo `integral *= 0.98f` dentro del hold de equilibrio |
| Freno por velocidad | `KV_FRENO` / `KV_FRENO_FUERTE` | 0.8 / 6.0 (umbral FRENO_VEL_UMBRAL=1.0) — velocidad calculada por encoders |
| Velocidad objetivo en línea | `LINE_SPEED_TARGET` | 2.50 m/s por defecto; ajustable en runtime con `MODIFY_LINE_SPEED=0xC4`, limitado a **0.20..8.00 m/s** (era 4.00 hasta 2026-07-27) |
| Tope de inclinación (setpoint dinámico) | `SP_LIMIT_DEG` | 5.0° por defecto; ajustable con `MODIFY_SP_LIMIT=0xC9`, limitado a 1..15°. Rige LINE_FOLLOWING/BALANCE_ONLY/IDLE (MANUAL usa 6°, BALANCE_AND_SPEED 2°). **No escala solo con la velocidad**: a velocidades altas conviene subirlo a mano |

**Sensor de ángulo:**
- Fuente: Filtro complementario (α=0.98) entre acelerómetro y giroscopio del MPU-6050
- Eje de control: **roll** (inclinación lateral del péndulo)
- Bias del MPU hardcodeado (`MPU_USAR_BIAS_FIJO`): ax=-46, ay=4950, az=1980, gx=-441, gy=-107, gz=-54

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
- **Última sesión:** 2026-09-21 (display extraído a display_oled.c; Debug/Release y regresión host verificados; pendiente prueba física)

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
- **Esquive alternado (2026-07-13)**: la secuencia de evasión alterna el sentido en cada objeto — la primera esquiva gira 90° a la DERECHA y bordea la pared con ADC7 (lateral izquierdo, comportamiento histórico), la siguiente gira a la IZQUIERDA y bordea con ADC5 (lateral derecho), y así. Mismos mecanismos/umbrales en ambos sentidos (`obj_esquive_dir` espeja pivots y `OBJ_PARED_ADC_INDICE` elige el sensor). Al entrar al modo línea se rearma en derecha. El display (pantallas 1 y 6) muestra `A7:`/`A5:` según el sentido activo. **Desde 2026-07-14 el giro del cruce perpendicular (`PERP_ROTATE`) también sigue `obj_esquive_dir`**: al reencontrar la línea tras bordear, gira hacia el mismo lado del esquive (girar al contrario devolvía al obstáculo); sin esquive previo conserva la derecha histórica
- Control manual remoto (FORWARD/BACKWARD/LEFT/RIGHT/STOP): adelante/atrás por PI de velocidad (máx 1 m/s, ángulo máx 6°, mismo patrón que el PI de velocidad del seguidor de línea); giro suave y a 1/4 de fuerza (steering ±15, rampa ~0.0625/ciclo) en vez de un salto brusco; adelante/atrás van derecho por corrección de rumbo (mismo algoritmo P sobre diferencia de velocidad de ruedas que la reversa recta de `OBJ_REVERSE`) (2026-07-06). **También funciona durante `LINE_FOLLOWING` mientras no ve la línea** (`manual_line_override`, mismo control reutilizado, se ignora apenas la línea reaparece) (2026-07-06)
- Freno dinámico por velocidad de encoders (KV_FRENO=0.8 + KV_FRENO_FUERTE=6.0, umbral 1.0 m/s)
- Detección de caída y recuperación con histéresis
- Gestor I2C no bloqueante con cola (evita bloquear el loop de control)
- Encoders de cuadratura 4x: PA8/PB13 (derecho), PB14/PB15 (izquierdo), decodificados por muestreo periódico a 4 kHz vía TIM2 (ver Registro de Cambios 2026-07-01)
- Velocidad real de ruedas desde encoders (reemplaza estimación accel+gyro)
- **Odometría de pose (x, y, θ)** integrada a 100 Hz: encoders (distancia) + gyro Z (rumbo). Comandos UNER `GET_ODOMETRY=0xDA` / `RESET_ODOMETRY=0xDB`; origen se resetea al cambiar de modo. **Pendiente verificar `ODOM_THETA_SIGNO` en el robot físico (abierto desde 2026-07-04, aún sin confirmar)** — el proyecto Qt ya tiene las herramientas para hacerlo cómodo: pestaña "Odometría (WiFi)" con mapa XY navegable + flecha de rumbo (ver CLAUDE.md de Qt), y los comandos `GET_ODOMETRY`/`RESET_ODOMETRY` en el combo de comandos. Test: reset odometría, girar el robot 90° a la derecha a mano, `GET_ODOMETRY` y ver si θ salió positivo (~+90°, signo correcto) o negativo (invertir `ODOM_THETA_SIGNO` en `main.c` ~línea 596)
- **Retorno por odometría al punto de pérdida de línea** (`LOST_FWD`, camino centrado post-180°): navega a la pose guardada del último punto con línea visible en vez de avanzar a ciegas; display "VUELVE" (2026-07-04, pendiente validar signo de steering en el robot)
- **Giro de 180°/90° (`LOST_ROTATE`/`EDGE_ROTATE`/`PERP_ROTATE`/`OBJ_GIRO_ESQUIVE`) revertidos 2026-07-05** a su forma previa a esa sesión (pivot fijo, freno fijo, heading por `fmaxf(gyro,encoder)`) tras una cadena de rediseños que terminó girando hacia ambos lados. **Corrección 2026-07-06: la escala del gyro Z en estos 4 bloques es `gz/100`, NO `gz/131`** — esta misma sección decía lo contrario hasta hoy; quedó desactualizada porque la reversión del 2026-07-05 restauró casi todo a su forma pre-sesión PERO el usuario pidió explícitamente mantener el fix de escala (`gz/100`) sobre esa base revertida (ver fila del Registro de Cambios 2026-07-05 "Único cambio sobre la reversión completa"). Verificado en vivo grepeando `main.c`: no queda ningún `gz/131` en el código activo, solo en comentarios históricos. **El banco de pruebas del giro de 90° en modo MANUAL (que existía para esto) fue eliminado el 2026-07-06** — MANUAL ahora es exclusivamente control por comandos WiFi/USB, sin ningún ciclo automático

> El "Snapshot historico — sesion 2026-05-27" (superseded) se movio a `CHANGELOG.md` el 2026-07-07.

### Pendientes / bugs conocidos 🔧
- **Estación por rueda (Opción A) — heading-hold por gyro agregado 2026-07-26, pendiente validar en el robot.** Historia del día: (1) knobs de rotación (`RUEDA_ROTACION_ZONA_MUERTA` 3→2, `RUEDA_ROTACION_KP` 0.15→0.25); (2) la TRASLACIÓN fallaba asimétrica (adelante nunca frenaba) → se movió del trim de PWM al SETPOINT de inclinación (`Estacion_CorreccionAngulo()`, patrón PAUSA_GIRO) — **VALIDADO en el robot: "el PID corrige muchísimo mejor"**; (3) quedaba un giro lento constante a la derecha (micro-patinaje invisible para los encoders + zona muerta de 3°/s del yaw-assist) → **heading-hold por gyro Z**: `wheel_yaw_deg` integra el rumbo desde el ancla y un P (`RUEDA_RUMBO_KP=2.0`, DB=1°, tope 12°/s eq) se suma al canal del yaw-assist con su mismo signo. (4) 3ª iteración: D siempre activa en la traslación (`RUEDA_ESTACION_AMORTIGUACION_KD=0.25 °/rps`) — **VALIDADA: "la deriva traslacional quedó bastante resuelta, se amortigua bastante"**. (5) 4ª iteración ("desde la base"): bias del gyro medido SOLO en reposo real (congelado durante balance) + hold PI — **VALIDADA: "quedó muy bien el tema del giro, no lo hace más"**. (6) 5ª iteración: warmup rápido del bias (~0.3–0.5s de IDLE alcanzan, validado) y DAMP 0.25→0.45 (mejoró pero seguía lento y temblaba en reposo). (7) 6ª/7ª iteración: latch de asentado, DAMP 0.45→0.70, banda angosta + re-anclaje — seguía sin amortiguar bien. (8) 8ª iteración — **rediseño final SIN ANCLA, VALIDADO ("funciona espectacular")**: D de velocidad con zona muerta de 0.8 rps + desplazamiento reciente con fuga τ≈2s ("ancla flotante"). (9) **Siesta de motores** (`MOTOR_SLEEP_*`, solo BALANCE_ONLY): PWM 0 real en el punto dulce hasta empujón/deriva. 2ª forma (ventana propia desacoplada del hold) validada ("mucho mejor"); 3ª pasada del 2026-07-26: entrada aún más fácil (gyro≤4°/s, 100ms sin ticks, |pwm|≤12, 150ms sostenido) y más amortiguación traslacional (`RUEDA_ESTACION_AMORTIGUACION_KD`=1.00, KP=0.08, tope 2.5°). 4ª pasada: zona ciega de la D corregida (`RUEDA_VEL_ZONA_MUERTA_RPS` 0.8→0.25, KP 0.08→0.05) — "mejoró muchísimo". 5ª pasada: quedaban ~15 cruces de llegada → siesta como asentador (ventanas cortas: gyro≤5, 60ms sin ticks, |pwm|≤15, 80ms sostenido — la pesca en los extremos del vaivén donde v≈0 y la fricción estática lo mata en 2-3 cruces) + DAMP 1.0→1.2 + `RUEDA_VEL_ZONA_MUERTA_RPS` 0.25→0.15. **6ª pasada (2026-07-27, a validar)**: la 5ª sobre-corrigió — reporte: "no entra casi nunca a la siesta y se va de a poco para los costados". Con V_DB 0.15 + DAMP 1.2, un count suelto pateaba el setpoint ~0.66° y cada tick reseteaba las ventanas de la siesta (círculo vicioso). Fix: `RUEDA_VEL_ZONA_MUERTA_RPS` 0.15→0.25 y `DAMP` 1.2→1.0 (vuelven a los valores validados; la siesta permisiva queda intacta y es quien asienta los cruces), y contra la deriva lenta invisible `RUEDA_DESPLAZAMIENTO_FUGA` 0.995→0.998 (τ≈5s: con τ≈2s toda deriva <~5 mm/s se drenaba antes de superar `RUEDA_TRASLACION_ZONA_MUERTA_COUNTS` y nunca se corregía; ahora el umbral baja a ~2 mm/s). **7ª pasada (2026-07-27) — VALIDADA ("la mejora es una locura, amortigua los empujones muy rápido")**: la 6ª empeoró ("reacciona tarde") → causa raíz estructural: la corrección viajaba dentro de `setpoint_base_objetivo` y la rampa de `setpoint_paso_max=0.1°/ciclo` + la EMA de velocidad le metían ~150ms de retraso — la D llegaba en contrafase y bombeaba el vaivén (por eso subir DAMP siempre empeoraba). Fix: corrección aplicada POST-rampa directo sobre `dynamic_setpoint_f` al final de `Ctrl_SetpointDinamico`, con slew propio `WS_STEP=0.4°/ciclo`. **VALIDADA** ("la mejora es una locura, amortigua los empujones muy rápido") — commit+push `6a286a6`. **8ª pasada (2026-07-27)**: quedaba brusquedad al llegar al punto dulce (se pasaba del equilibrio justo antes de la siesta) → rodilla suave `WHEEL_V_SOFT_KNEE=1.0` rps: debajo, la ganancia de la D escala lineal con \|v\| (cuadrática en chico, entera en grande — un empujón real recibe la D completa). **No alcanzó** ("sigue demasiado brusco y potente, no entra nunca a la siesta") — atacaba el término equivocado. **9ª pasada (2026-07-27, a validar)**: el que empujaba en el punto dulce era el **P saturado**: tras un empujón `wheel_disp_f` llega a ~90 counts (×0.05 = 4.5°, clampeado al tope general de 2.5°) y con la fuga de τ≈5s seguía inclinando 2.5° de vuelta durante segundos → error nunca < 0.8°. Fix: tope propio `RUEDA_ESTACION_P_MAX=0.5°` para el P (rol real: frenar deriva de mm/s; 0.5° alcanza y queda debajo del umbral de la siesta) — la D conserva sus ±2.5° validados contra empujones; la rodilla queda. **VALIDADA a medias** ("quedó suave") pero la siesta seguía sin entrar NUNCA. **10ª pasada (2026-07-27, a validar)**: el residuo del P (≤0.5°, fuga τ≈5s) mantenía al robot GATEANDO de vuelta durante segundos — cada tick reseteaba la ventana de la siesta (que en sí está bien calibrada, era alcanzable antes de la estación). Fix: drenaje rápido en reposo real en `Estacion_CorreccionAngulo()` — ambas ruedas >`RUEDA_REPOSO_SIN_TICKS_MS=150`ms sin tick + \|v\| bajo la zona muerta → `wheel_disp_f` se drena con `RUEDA_DESPLAZAMIENTO_FUGA_REPOSO=0.90` (τ≈100ms, muere en ~0.3s); la deriva lenta real (~cm/s) tiquea cada <150ms y no dispara el drenaje. **No alcanzó** — el reporte siguió igual. **11ª pasada (2026-07-27, a validar)**: fin del suavizado término-por-término (pasadas 8-10: rodilla, tope del P, drenaje — siempre quedaba un término activo perturbando) → **factor de CALMA global**: `calm = min(1, max(|gyro_f|/8°/s, |v_trans|/1.0rps))` multiplica TODA la corrección — quieto = estación muda (cero perturbación en el punto dulce), empujón = gyro dispara al instante → estación entera. Rodilla eliminada (el calm la subsume); tope del P y drenaje quedan. **No alcanzó** — con la estación ya muda el síntoma siguió: el culpable estaba en OTRA capa. **12ª pasada (2026-07-27, a validar)**: la PATADA de compensación de deadband en `Ctrl_SalidaMotores` — con rueda parada, un comando de 3 PWM saltaba a 11 (offset estático completo de golpe): relé oscilando en el punto dulce que reseteaba la siesta eternamente. Fixes: `MOTOR_COMANDO_NEUTRO` 2→4 (chatter de 3-4 PWM → 0 real) + rampa `MOTOR_ZONA_MUERTA_RAMPA_GANANCIA=2` (offset proporcional al exceso, pleno recién con \|cmd\|≥8). **VALIDADA** ("quedó mucho mejor"). **13ª pasada (2026-07-27, a validar)**: más suavidad pedida → `MOTOR_ZONA_MUERTA_RAMPA_GANANCIA` 2→1 (offset pleno recién con \|cmd\|≥12). **14ª pasada (2026-07-27, a validar)**: la rueda derecha se movía sola sin parar (la izquierda muda) — el canal diferencial (contra-torque retenido del integrador de yaw + PD de rotación) dejaba una rueda siempre arriba de la zona neutra → **calma rotacional** (`rot_calma = min(1, max(\|yaw_velocidad\|/2°/s, \|v_rot\|/0.5rps))` multiplica yaw-hold y PD de rotación; el I solo acumula con calma>0.3). **15ª pasada (2026-07-27)**: empujón FUERTE lo volteaba → fade de supervivencia por gyro>40°/s. **FRACASÓ** ("no frena nunca"): toda recuperación normal supera 40°/s de gyro → estación siempre muda; NO reintentar gates por gyro alto. **16ª pasada (2026-07-27) — VALIDADA y cierre del paquete ("quedó muy bueno, se queda en el punto dulce montón de tiempo, me encanta")**: fade eliminado; el vuelco se resuelve por TECHO: `RUEDA_ESTACION_ANGULO_MAX` 2.5→1.5 — el freno de la D sigue proporcional a v (1°/rps, idéntico hasta 1.5 rps), solo se recorta el extremo que apilado con el freno clásico volcaba. Knobs: aún vuelca → 1.5→1.2; frena poco → 1.5→1.8. **Estado final validado en el robot: tag V27** (pasadas 7-16: post-rampa + calma traslacional/rotacional + zona neutra 4 + rampa de deadband + techo 1.5°). El cuadro de knobs de la ventana de siesta quedó explicado en la sesión del 2026-07-27; regla clave: `ANG_ENTER<ANG_WAKE`, y ojo al solape `GYRO_ENTER=5>WAKE_GYRO=4` (si cicla dormido/despierto: WAKE_GYRO→6 o GYRO_ENTER→3). Knobs restantes de suavidad: `MOTOR_COMANDO_NEUTRO` 4→5, soft zone `scale_min` 0.35→0.30; retrocesos: bamboleo lento → NEUTRAL 4→3, flojo para arrancar la rueda → RAMP_GAIN 1→2, si vuelve el giro lento constante → `RUEDA_ROTACION_CALMA_GIRO_REF` 2→1. Al validar: commit + tag (¿V27?). Knobs: brusco al asentarse → rodilla 1.0→1.5; vaivén chico que no muere → rodilla 1.0→0.6; tiembla en movimiento → DAMP 1.0→0.9; siesta cicla dormido/despierto (despierta enseguida tras dormir) → `MOTOR_SIESTA_ENTRA_MS` 80→120 o `MOTOR_SIESTA_ANGULO_ENTRA` 0.8→0.6; sigue cruzando mucho → `RUEDA_ESTACION_ANGULO_KP` 0.05→0.03; si tras esto sigue derivando lento → `RUEDA_TRASLACION_ZONA_MUERTA_COUNTS` 3→2 o `RUEDA_DESPLAZAMIENTO_FUGA` 0.998→0.999. Al validar todo el paquete del día: commit + tag (¿V27?). Todo commiteado y pusheado en **2dc5a7b** (2026-07-26); última versión validada completa en el robot: tag **V26** (ba790d5). Al validar esta pasada conviene taggear (¿V27?). Kill-switch `rueda_mecanismos_habilitados=0` apaga los tres mecanismos (setpoint, rot por encoders y heading-hold PI). Pendiente aparte: extender la estación a línea/manual si aplica.
- El SSID/IP WiFi está hardcodeado en `main.c` (líneas ~244); cambiar manualmente según red
- `MODIFY_BETA_G/A (0xBC/0xBD)` son vestigiales: los betas de los filtros EMA hoy son defines fijos en `main.c` (las variables runtime ya no existen) y sus punteros no se registran en `unerBindings`. Desde 2026-07-24 el firmware responde UNKNOWN (antes ACK mentiroso). Decidir: sacarlos del combo de Qt o volver a hacer los betas variables y registrarlos
- Código de debug activo en `ESP01.c` (printfs de estados AT) que genera tráfico USB extra
- **Freeze residual al mover el robot** — Causa raíz identificada (2026-05-18): el DMA del I2C queda en estado "en progreso" indefinidamente sin completar. `mpu_data_ready_for_ctrl` nunca se setea, `ControlCiclo10ms` nunca corre, `i2c1_tx_busy` queda en 1, display congelado, motores al último PWM. El `while(1)` SIGUE girando (IWDG se patea) pero el sistema es funcionalmente inútil. Fix: watchdog de software en `while(1)` — si pasan >150ms sin dato del MPU se llama `I2C1_Recover()` (9 pulsos SCL + HAL_I2C_Init + I2C_Manager_Init) y se relanza la lectura SIN resetear el MCU. Detectable por "I2C RECOVER\r\n" en USB. **2026-07-01: se sospecha que este freeze estaba agravado (no necesariamente causado) por el viejo sistema de encoders vía EXTI, que compartía prioridad NVIC con el DMA de I2C y podía dejar la transacción I2C pendiente en cola durante ráfagas de pulsos (ver Registro de Cambios 2026-07-01). Con encoders migrados a polling por TIM2 y el DMA de I2C subido a prioridad 0, debería ser mucho menos frecuente — pendiente de confirmar en el robot físico si desaparece del todo.**
- Resolución de velocidad limitada: piso de cuantización de `velocity_est` (calculado en `ControlCiclo10ms` sobre delta de counts cada 10ms, no sobre el muestreo de 4kHz de `SampleEncoders250us` que solo alimenta el conteo acumulado) ≈ 0.32 m/s con 1 count en una sola rueda por ciclo, ≈ 0.63 m/s si tiquean ambas ruedas. `FRENO_VEL_ZONA_MUERTA=0.35` (2026-07-01) calibrado para filtrar ese piso. No apto para integración de posición ni para reaccionar a velocidades reales por debajo de ese umbral
- **Freeze + reset durante giro de 180° (LOST_ROTATE), reportado repetidamente pese a varios intentos de fix.** Sesión 2026-07-01: se probó primero bajar `LROT_PIVOT`/`LROT_BRAKE` (hipótesis de glitch eléctrico) + corregir prioridad NVIC I2C vs EXTI15_10 — el usuario confirmó que el problema **persistió igual**, descartando la hipótesis de corriente de motor como causa suficiente. El usuario migró el conteo de encoders de EXTI a polling por TIM2 (ver más abajo) sospechando que el bug estaba en el subsistema de encoders. Revisión de código no encontró un bug concreto en el nuevo esquema de polling — arquitectura correcta y con más margen que el EXTI viejo. **Pendiente de validar en el robot físico si el freeze+reset desaparece con el nuevo esquema de encoders.** Si persiste, revisar next: reversión brusca de motores (EMI) con un osciloscopio en VDD durante el pivot, o un logic analyzer en las líneas de encoder para confirmar que no hay rebote/ruido eléctrico real en el sensor.

---

## 📋 Registro de Cambios
> **Instruccion para Claude:** Al finalizar cada sesion, agregar una fila con los cambios realizados (la mas reciente arriba).
> Mantene aca solo las ~10 filas mas recientes: cuando agregues filas nuevas, move las que sobren al TOPE de la tabla de `CHANGELOG.md`. **Nunca borres una fila — siempre movela.**
> Historial completo del proyecto (2026-05-04 en adelante): ver `CHANGELOG.md` en la raiz del proyecto.

| Fecha | Archivo(s) modificado(s) | Cambio realizado | Motivo / Observación |
|-------|--------------------------|------------------|----------------------|
| 2026-09-21 | Core/Src/main.c, Core/Src/display_oled.c, Core/Inc/display_oled.h, Core/Inc/robot_estados.h, Debug/Core/Src/subdir.mk, Debug/objects.list, tests/test_pantalla_regression.py | Display separado de main (archivos renombrados a display_oled.c/.h a pedido del usuario): ocho pantallas, avisos WiFi/caída/límite, logo, callbacks I2C/DMA y temporizadores. Foto estática de datos para no cargar el stack ni exponer variables del controlador. Se conservan intervalos 60/100 ms, prioridades visuales, pool DMA y consulta de flags IMU volatile tras avanzar el OLED. Enums trasladados sin renumerar. Debug/Release sin errores ni advertencias; 6000 cuadros comparados con el respaldo, pruebas de refresco/IMU/overflow. Control y secciones generadas CubeMX idénticos. | Respaldo previo publicado: commit 291e34f y tag backup/pre-display-2026-09-21. Sin cambios de Qt. Pendiente verificar en OLED físico. |
| 2026-09-16 | ESP01.c/.h, UNER.c/.h, main.c, tests, SOFTAP.md | SoftAP seleccionable por KEY al arrancar o comando 0xDD en IDLE; Station por defecto. Driver AT secuencial, descubrimiento de cliente UDP, plazos reales, parser IPD y cola de ACK. Corrige campos WifiLogData_t desalineados del refactor local. Compilado Debug/Release; tests host ESP01/UNER. Pendiente prueba fisica. Ver SOFTAP.md. | Pedido: integrar SoftAP y respaldar ambos repositorios. |
| 2026-08-30 | Core/Src/main.c, Core/Inc/MPU6050.h, Core/Src/ssd1306.c, CLAUDE.md | **Limpieza: 26 defines muertos borrados, 22 bloques de comentario historico reescritos en presente, y 2 renombres mas (`SETTLE`, `ComputeBrakeFromVelocity`). Cero cambio funcional.** (1) **26 `#define` sin un solo uso eliminados** (verificado en todo `Core/` y `USB_DEVICE/`): 19 de `main.c` — la familia de la velocidad por acelerometro previa a los encoders (`VEL_DECAIMIENTO`, `VEL_DECAIMIENTO_ACELEROMETRO`, `VEL_COMPLEMENTARIO_ALFA`, `VEL_ACELEROMETRO_ESCALA`), la del *inner steering PI* descartado (`LINEA_DIRECCION_ENCODER_ESCALA/_MAX_RPS/_REALIM_KP/_KI/_I_MAX`), los del `ciclo_tarde` que se saco hoy (`DT_MIN_VALIDO`, `DT_MAX_VALIDO`) y 8 sueltos —, 3 de `MPU6050.h` (`GYRO_XOUT_H_REG`, `GRAVEDAD`, `MULTIPLICADORFLOAT`) y 4 de `ssd1306.c` (`IS_BUSY`, `DELAY_MS`, `SSD1306_COLUMNADDR`, `SSD1306_PAGEADDR`), mas 2 comentarios de cabecera que quedaron huerfanos. Diff: 28 borrados, 0 agregados. (2) **22 bloques de comentario con fecha reescritos en presente** (154 -> 118 lineas): el usuario pidio que los comentarios digan **que pasa ahora**, no que pasaba antes. Se conserva el "por que es asi" (que es la parte que explica el codigo de hoy) y se tiran fechas, valores viejos y cronicas de bugs ya resueltos; la historia completa sigue en `CHANGELOG.md` y en git. **Se conservan a proposito** las advertencias del tipo "NO reintentar X", reformuladas en presente (ej. el "fade de supervivencia" por gyro alto). Quedan **0** comentarios con fecha en `main.c`. (3) **`SETTLE` erradicado** (0 ocurrencias): `LINE_STATE_LOST_SETTLE`/`EDGE_SETTLE` -> `_LOST_ASENTAR`/`_EDGE_ASENTAR`, `LineState_LostSettle`/`EdgeSettle` -> `LostAsentar`/`EdgeAsentar`, `LSETTLE_*`/`ESETTLE_*` -> `PERDIDA_ASENTAR_*`/`BORDE_ASENTAR_*` (y de paso `THR`->`UMBRAL`, `TILT`->`ANGULO`), `settled` -> `asentado`. Se eligio ASENTAR por consistencia con `accel_asentado`, renombrado en una sesion anterior. (4) **`ComputeBrakeFromVelocity` -> `Freno_AnguloPorVelocidad`** (7 ocurrencias): completa el par con `Freno_AnguloSegunModo` — los dos empiezan con `Freno_` y terminan diciendo de que depende el angulo que devuelven (del modo / de la velocidad). (5) **Doc corregida**: esta tabla decia que la integral de balance tiene `decay=0.990 por ciclo` — **ese decay no existe**: `INTEGRAL_DECAIMIENTO` era uno de los defines muertos, y el unico decaimiento real es `integral *= 0.98f` **dentro del hold de equilibrio**; fuera del hold la integral solo se satura en `INTEGRAL_MAX`. El enum de `line_state` mantiene el orden -> **Qt no requiere cambios**. | Usuario: "elimina los define que estan muertos" + "estos comentarios no me sirven mas, ya que no me interesan las cosas que pasaban antes, solo me interesa que pasa ahora" + "que significa SETTLE, cambiemoslo para mejorarlo" |
| 2026-08-30 | Core/Src/main.c, CLAUDE.md | **Limpieza de legibilidad: 3 renombres, `FWD` erradicado y codigo muerto eliminado. Cero cambio funcional (compilado y validado por el usuario).** (1) **`ComputeBrakeSetpointTarget` -> `Freno_AnguloSegunModo`** (17 ocurrencias): el nombre viejo sugeria que calculaba el setpoint, cuando en realidad es el *portero* de `ComputeBrakeFromVelocity` — decide SI el modo frena, con que tope y con que velocidad (la LENTA, tau~0.5s, para que el vaiven en el lugar no dispare el freno). Hace juego con `Estacion_CorreccionAngulo`. (2) **`FWD` erradicado del proyecto** (0 ocurrencias en `Core/` y `USB_DEVICE/`): `LINE_STATE_LOST_FWD`/`EDGE_FWD` -> `_LOST_AVANZA`/`_EDGE_AVANZA`, `LineState_LostFwd`/`EdgeFwd` -> `LostAvanza`/`EdgeAvanza`, `LOST_FWD_TIMEOUT`/`EDGE_FWD_TIMEOUT` -> `PERDIDA_AVANCE_TIMEOUT_MS`/`BORDE_AVANCE_TIMEOUT_MS` (engancha con la familia `PERDIDA_AVANCE_*` que ya existia), `lfwd_vel`/`efwd_vel` -> `perdida_avance_vel_neta`/`borde_avance_vel_neta` (se leen al lado de su `_cruda`: cruda -> neta = despues de la zona muerta), `vel_fwd` -> `vel_solo_avance`, `global_fwd_vel` -> `vel_avance_global`, y `WALL_FWD` en comentarios -> `OBJ_BORDEAR_PARED` (el nombre real del estado). **`LOST` se conserva a pedido del usuario.** El enum mantiene el orden -> **Qt no requiere cambios**. (3) **Codigo muerto eliminado**: `static const uint8_t ciclo_tarde = 0` era un vestigio de cuando se media el `dt` real y se salteaba la integracion si un ciclo llegaba tarde; al pasar a `dt` fijo (`DT_CTRL_FIJO`) quedo constante en 0, dejando 8 `if (!ciclo_tarde)` siempre verdaderos (el compilador ya los eliminaba). Borrados los 8 bloques (los 5 PI de velocidad de LOST/EDGE/pared, el anti-windup del PID de balance y los terminos I y D del PID de linea) con sus cuerpos desindentados, mas la variable. De yapa, `float d_line = 0.0f` seguido de asignacion incondicional quedo declarado con su valor. **Verificado**: `git diff -w` muestra los `ciclo_tarde` SOLO como borrados (ninguna linea de codigo se perdio), llaves -8/-8 con balance global 0, -19 lineas (8 `if` + 8 `}` + 3 de la variable), y los 4 integradores con el mismo numero de apariciones que antes. (4) **Doc corregida**: esta tabla decia `KV_FRENO=0.0` y `FRENO_VEL_UMBRAL=1.5`; en el codigo son **0.8** y **1.0** desde hace tiempo. **COMPILADO Y VALIDADO por el usuario** ("ahi compile y quedo perfecto"). | Usuario: "ComputeBrakeSetpointTarget es raro el nombre... podemos cambiarle el nombre?" + "se puede cambiar el nombre y poner algo mas claro a todos los que digan fwd? no se entiende esa abreviacion" + "dale, sacalos ya que es codigo muerto, pero asegurate de no romper nada, hace las revisiones correspondientes" |
| 2026-08-26 | Core/Src/main.c, CLAUDE.md | **Segunda tanda de legibilidad: 98 defines + 128 variables renombrados con una convencion unica, y 212 lineas de defines realineadas. Cero cambio funcional.** Reporte del usuario: "estan muy raros los nombres de los DEFINE, quedaron horribles, imposible estudiarlo asi". **Convencion aplicada**: `<SUBSISTEMA>_<MECANISMO>_<MAGNITUD>_<UNIDAD>`, sin abreviaturas (`REV`->`REVERSA`, `ROT`->`ROTACION`, `YAW`->`RUMBO`, `FB`->`REALIM`, `CMD`->`COMANDO`, `AMORT`->`AMORTIGUACION`, `ENC`->`ENCODER`, `DESPL`->`DESPLAZAMIENTO`, `ZM`->`ZONA_MUERTA`, `CIC`->`CICLOS`, `IDX`->`INDICE`, `ANG`->`ANGULO`, `SP`->`SETPOINT`); los limites de cambio por ciclo pasan a decir `RAMPA` (antes `PASO`, que parecia un valor cualquiera); unidad al final cuando es ambigua (`_GRADOS _RPS _DPS _COUNTS _ADC _METROS _MS`). Ejemplos: `LINEA_REV_EMPUJE_SUBE`->`LINEA_ESCAPE_REVERSA_RAMPA_SUBIDA`, `LINEA_ENC_CORR_KP`->`LINEA_DEFICIT_VEL_KP`, `ENC_CPR`->`ENCODER_COUNTS_POR_VUELTA`, `OBJ_REV_HOLD_*`->`OBJ_DISTANCIA_*`, `BIAS_FIJO_GX`->`BIAS_FIJO_GIRO_X`. **Hallazgo clave**: el sufijo `_f` significaba DOS cosas distintas (filtrado por EMA y ya-rampeado). Separado: `_f` = siempre EMA; las rampas ahora dicen `_rampeado`. Cadena de angulos renombrada para que se lea como secuencia: `linea_pi_angulo_pedido` + `linea_deficit_angulo_extra` + `linea_escape_reversa_angulo` -> `linea_angulo_total_avance`; y `setpoint_base_objetivo`/`_rampeado` -> `setpoint_dinamico_final` (el que recibe el PID; antes era `setpoint_dinamico_f`, indistinguible de `setpoint_dinamico`). **NO se toco**: `KP/KD/KI/SETPOINT_ANGLE` (universales), `M_PI`, ni los campos de struct de `UNER.h` (`adc_len`, `kv_brake`, `wheel_left_rps`, `wheel_right_rps`, `rotation_target_deg`) -> **Qt no requiere cambios**. Verificado por script: 0 nombres viejos, todos los nuevos con su `#define`, 0 duplicados, diff simetrico. NO compilado (el usuario compila). | Usuario: "quiero que mejores todos los nombres de los mismos para mejorar la comprension del codigo, imposible estudiarlo asi con esos nombres raros" + "sigo tratando de mejorar la interpretacion lo maximo posible" |
| 2026-08-26 | Core/Src/main.c, CLAUDE.md | **Legibilidad para estudio: renombres de identificadores locales, defines `RUEDA_ESTACION_*` movidos al bloque de arriba con comentarios recortados, y 26 comentarios nuevos de fin de linea nombrando la formula de cada cuenta. Cero cambio funcional.** (1) **Renombres** (todos verificados con 0 ocurrencias viejas): `LINE_THRESHOLD`->`LINEA_UMBRAL_ADC`; toda la familia `PERP`->`PERPENDICULAR` (enum `LINE_STATE_PERPENDICULAR_ROTATE`, funcion `LineState_PerpendicularRotate`, variables `perpendicular_desde_esquive`/`perpendicular_obj_freno_inicio_ms` y los 3 defines, renombrados ademas a `PERPENDICULAR_ANTIRREBOTE_MS`/`PERPENDICULAR_DURACION_MAX_MS`/`PERPENDICULAR_VISTA_PREVIA_MS`); locales del bloque de cruce perpendicular (`all_black`->`todo_negro`, `all_black_elapsed`->`todo_negro_transcurrido_ms`, `accel_moving`->`accel_moviendose`, `accel_settled`->`accel_asentado`, `ACCEL_MOTION_THRESHOLD`->`ACCEL_MOVIMIENTO_UMBRAL`, `ACCEL_SETTLE_MS`->`ACCEL_ASENTADO_MS`); locales del centroide (`s[]`->`linea_adc_crudo[]`, `w[]`->`linea_peso[]`); deteccion de objeto (`obj_now`->`obj_visto_ahora`, `obj_cnt`->`obj_confirma_cnt`, `obj_clear_cnt`->`obj_libera_cnt`, alineados con el patron `linea_detect_confirma_cnt`/`_libera_cnt`); y `WheelStation_AngleCorr()`->`Estacion_CorreccionAngulo()`. (2) **Defines movidos**: los 11 `RUEDA_ESTACION_*`/`RUEDA_TRASLACION_ZONA_MUERTA_COUNTS`/`RUEDA_V_ZM_RPS`/`RUEDA_CALMA_*`/`RUEDA_DESPL_*` pasaron de estar sueltos en la linea ~3586 al bloque de defines de balance/freno (lineas 290-302); sus comentarios pasaron de ~69 lineas de historia a 23 (uno corto por define). Se conservaron a proposito los knobs accionables y la advertencia de NO reintentar el "fade de supervivencia" por gyro alto. Tambien se recortaron los 2 comentarios largos que quedaban en la funcion (15 y 30 lineas -> 2 y 11). El detalle historico completo sigue en este CHANGELOG. (3) **26 comentarios** de fin de linea nuevos nombrando la tecnica: buffer circular por modulo/mascara, regla de tres duty->CCR, escalado LSB->deg/s, resta de timestamps, norma L1, semisuma=traslacion, superposicion avance+freno, extrapolacion de Taylor 1er orden, descomposicion polar->cartesiano, etc. Verificado: sin defines `RUEDA_*` duplicados ni usados-sin-definir. NO compilado (el usuario compila). | Usuario estudiando el firmware linea por linea: "estos nombres estan rarisimos para darme cuenta para que sirven", "estos define deberian estar arriba con los demas y no me gustan los comentarios extensos", "esos comentarios me ayudan mucho, se puede completar en los demas lugares?" |
| 2026-08-03 | Core/Src/main.c, CLAUDE.md | **Renombrado masivo de identificadores de inglés a español: 233 variables + 216 defines (solo nombres — cero cambio funcional).** El usuario estudia el firmware línea por línea y pidió nombres reconocibles en castellano. Dos pases scriptados con reemplazo por palabra completa y *lookbehind* que protege los accesos `.campo` / `->campo`. **Variables** (2384 reemplazos): `robot_state`→`estado_robot`, `line_*`→`linea_*`, `wheel_*`→`rueda_*`, `obj_wall_*`→`obj_pared_*`, `enc_r`/`enc_l`→`enc_der`/`enc_izq`, `ax..gz`→`accel_x..giro_z`, `f_fallen`→`f_caido`, `output`→`salida_pid`, `pwm_sat`→`pwm_saturado`, `steering_adjustment`→`ajuste_direccion`. **Defines** (717 en main.c + 55 en CLAUDE.md): `MOTOR_SLEEP_*`→`MOTOR_SIESTA_*`, `WHEEL_*`→`RUEDA_*`, `OBJ_WALL_*`→`OBJ_PARED_*`, `ANTISTALL_*`→`ANTIATASCO_*`, `LOST_FWD_*`→`PERDIDA_AVANCE_*`, `LOST_RETURN_*`→`RETORNO_*`, `BRAKE_TILT_*`→`FRENO_INCLINACION_*`, `STEER_*`→`DIRECCION_*`, `I_MAX`→`INTEGRAL_MAX`, `ALPHA`→`ALFA_COMPLEMENTARIO`. **NO se tocó** (a propósito): handles de CubeMX (se regeneran), defines de pines, `M_PI`, `KP/KD/KI/SETPOINT_ANGLE`, `ENCODER_COUNTS_POR_VUELTA`, los **campos de struct** de `UNER.h` (espejados en Qt) ni las constantes enlazadas a UNER (`KP_value`, `LINE_SPEED_TARGET`, `SP_LIMIT_DEG`, `KV_brake_value`) — por eso **Qt no requiere ningún cambio**. Verificado por script: 0 nombres viejos restantes, sin colisiones ni duplicados, y ambos diffs simétricos (2574 ins / 2574 del) confirmando renombrado puro sin pérdida de código. `CHANGELOG.md` NO se reescribió (es historia). **Pendiente**: las variables LOCALES dentro de funciones (`trabado_desplazamiento`, `gz_dps_cal`, `line_forward_boost`, `escala_estabilidad`, `wheel_disp_f`…) siguen en inglés. NO compilado (el usuario compila). | Usuario: "me encantaría que esos nombres de variables los hagas más reconocibles y sencillos, en español" + "hacé lo mismo para todos los DEFINE que no tienen nombre claro, que se entienda correctamente cuando los leo así los puedo estudiar fácil" |
| 2026-07-30 | Core/Src/main.c, Core/Src/UNER.c, Core/Src/MPU6050.c | **Comentarios de fin de línea nombrando la técnica/fórmula de cada cuenta (~90 líneas, solo comentarios — cero cambio funcional).** El usuario agregó a mano un `// Media movil exponencial` en el calibrador de bias del gyro y pidió replicar ese estilo (corto, al final de la línea, nombrando QUÉ es la fórmula) en la mayor cantidad de lugares posibles, porque le facilita muchísimo leer el firmware. Cubierto: **filtros** (EMA en velocidad rápida/lenta, accel_motion, lat_tilt, bias de odometría, error de línea, velocidad por rueda; filtro complementario con sus dos mitades pasa-altos/pasa-bajos; mediana + insertion sort del ADC); **PID/PI** (los cuatro lazos: balance, línea, steering por encoders, y los PI de velocidad de LOST_FWD/EDGE_FWD/OBJ_WALL×3/MANUAL — cada uno con P/I/D, integración rectangular `I += e*dt`, anti-windup por saturación y por integración condicional, descarga exponencial, ganancia asimétrica acelerar/frenar); **geometría** (atan2 del roll/pitch/lateral y por qué atan2 y no atan, Pitágoras, centroide como promedio ponderado + normalización, odometría: integración de Euler + regla del trapecio + descomposición polar→cartesiano + wrap a ±180°, navegación de retorno: distancia euclídea, rumbo y giro más corto); **conversiones** (análisis dimensional counts→rps→m/s, escalado del gyro, regla de tres duty→CCR y ADC→píxeles, punto fijo Q14 del MPU, recomposición big-endian); **utilidades de control** (saturación/clamp, zona muerta como "exceso", limitadores de slew-rate y sus rampas, interpolaciones lineales/lerp, decaimientos exponenciales, mezcla diferencial común±steering, semisuma=traslación / semidiferencia=rotación, calma traslacional y rotacional como normalización 0..1); **protocolo** (checksum XOR de RX y TX y por qué el orden no importa, buffer circular por máscara); **tabla de cuadratura** de los encoders (índice = 4·estado_anterior + estado_actual). NO compilado (el usuario compila). | Usuario: "hay comentarios que me ayudarían montón, como el de media exponencial que agregué acá, aclaran qué es y me ayudan a la interpretación — a la derecha luego de la línea, cortito y resumido, fijate dónde podés agregarlo en la mayor cantidad de lugares posibles" + "mientras más mejor!!!" |
| 2026-07-27 | Core/Inc/UNER.h, Core/Src/UNER.c, Core/Src/main.c, (Qt: mainwindow.h/.cpp) | **`MODIFY_SP_LIMIT=0xC9` nuevo (tope de inclinación ajustable desde Qt) + tope de velocidad de línea 4→8 m/s.** (1) **0xC9**: el `sp_limit` de `Ctrl_SetpointDinamico` (inclinación máxima para acelerar/frenar, hasta hoy la constante `5.0f`) pasó a la global `float SP_LIMIT_DEG = 5.0f`, enlazada por el binding nuevo `sp_limit` y modificable en runtime; el case en `UNER.c` valida con `getF32BoundedFromRx` a **1..15°** (`SP_LIMIT_CMD_MIN/MAX_DEG`) y responde UNKNOWN si no hay binding (patrón de `MODIFY_BETA_*`, no ACK mentiroso). Rige LINE_FOLLOWING/BALANCE_ONLY/IDLE; MANUAL (6°) y BALANCE_AND_SPEED (2°) conservan sus topes propios, y el freno hacia atrás en línea sigue acotado aparte por `LINEA_FRENO_ANGULO_MAX=3°`. (2) **`LINE_SPEED_CMD_MAX_MPS` 4.00→8.00 m/s**: el usuario quería pasar de 4 m/s y el clamp del firmware lo recortaba en silencio aunque Qt lo permitiera. Se queda en 8 y no más porque el corte de emergencia (`LINEA_VEL_CORTE_LIMITE`) está en 10 m/s: pedir un objetivo ≥ a ese corte sería autodestructivo. **Contexto**: ninguno de los parámetros de frenado escala con la velocidad objetivo (ángulo, rampas `LINE_SP_STEP_*`, ganancias del PI, margen y horizonte de la guarda predictiva son absolutos), así que a velocidades altas el robot queda relativamente sub-frenado — de ahí que tenga sentido poder subir el tope de inclinación a mano. NO compilado (el usuario compila). | Usuario: "quiero que me agregues sp_limit al Qt para poder cambiarlo de ahí, agregá la lógica al archivo UNER también; y sacame el límite de 4 m/s que tengo en Qt para poder llevarlo a más" |
| 2026-07-27 | Core/Src/main.c | **BUG CRÍTICO del esquive corregido: "PARADO" instantáneo al perder la pared — el reset del timer de pared-perdida decía `10` en vez de `0`.** Reporte: en pleno esquive, apenas el lateral deja de ver la pared, salta de `PARED>AVZ` directo a `PARADO` (`LINE_STATE_GIVEN_UP`), en vez de girar hacia la pared. Causa (en `Ctrl_LatchesPared`, etapa 3): con la pared visible el timer se "reseteaba" a **10** — un valor NO nulo, y la rama de abajo usa `== 0` como "timer sin arrancar". Al perder la pared se salteaba el arranque del timer y se evaluaba `HAL_GetTick() - 10` (≈ el uptime completo) contra los 5000 ms → verdadero desde el segundo 5 de encendido → `GIVEN_UP` **en el mismo ciclo** en que se pierde la pared. Y como la etapa 3 corre ANTES de los handlers de `line_state` (etapa 15), el estado ya era GIVEN_UP cuando `LineState_ObjBordearPared` iba a hacer su transición legítima `!pared_visible → OBJ_PARED_LIBRE → OBJ_GIRO_PARED`: el giro nunca llegaba a ejecutarse. Fix: reset a `0` (+ comentario explicando por qué DEBE ser 0). **Origen: commit `165f1ac` (2026-07-24)**, donde el `0→10` entró suelto, sin relación con el propósito de ese commit (estación por rueda + auditoría UNER) — no fue de la sesión de hoy; el diagnóstico anterior (que el timeout saltaba por un giro lento a causa del bamboleo) era **incorrecto**. NO compilado (el usuario compila). | Usuario: "se queda en estado PARADO de golpe en el medio del esquive, va de PARED>AVANZA a PARADO ni bien pierde de vista la pared — debería girar al lado de la pared al toque" |
---

## Decisiones de Diseño
> Registrar el *por qué* de decisiones críticas del firmware.

- **Display aislado (2026-09-21):** `display_oled.c` recibe `const PantallaDatos *`; la foto se actualiza desde el loop principal solo al vencer el refresco. Sus timers y callbacks son privados. Los umbrales se copian desde las constantes del control, sin duplicar su configuración. La foto vive en RAM estática para no sumar ~184 bytes a la pila del dibujado.
- **Filtro complementario en lugar de Kalman:** menor costo computacional, suficiente para este sistema (α=0.98, dt=10ms)
- **Loop de control en callback de TIM1 a 100 Hz:** periodicidad exacta garantizada por hardware, independiente del loop main
- **Bias MPU hardcodeado (`MPU_USAR_BIAS_FIJO`):** elimina calibración al arranque; el robot puede actuar en segundos sin necesidad de estar quieto
- **I2C no bloqueante con cola:** el MPU y el SSD1306 comparten I2C1; la cola evita colisiones y no bloquea el loop de 10ms
- **Protocolo UNER binario:** frame compacto con checksum, permite comandos bidireccionales y telemetría eficiente sobre USB CDC y UDP
- **USB CDC en lugar de UART para Qt:** mayor throughput y sin necesidad de conversor USB-UART externo
- **USART1 dedicado a ESP-01:** recepción byte-a-byte por interrupción, sin DMA para UART (el ESP-01 maneja su propia lógica AT)
- **PWM a ~100 kHz (TIM3/TIM4, Period=959):** frecuencia alta para reducir ruido audible y mejorar respuesta de motores DC
- **Masking de EXTI en encoder:** tras cada pulso se enmascara la línea EXTI y TIM5 la reactiva cada 2ms. Limita a 500 Hz/canal, evita freeze por rafagas. Solución más robusta que solo limpiar flags
- **4x quadrature por software:** ambos canales A y B decodificados por transición de estados. ENCODER_COUNTS_POR_VUELTA=28. Sin modo encoder de hardware (requeriría cambio de pines)
- **Encoders por polling (TIM2 @ 4kHz) en lugar de EXTI (2026-07-01):** el esquema anterior por interrupción de flanco con masking anti-storm (EXTI + TIM5 re-habilitando cada 2ms) compartía prioridad NVIC con el DMA de I2C y quedó sospechado de causar freezes/resets recurrentes durante el giro de 180°, posiblemente por ráfagas de interrupciones o una carrera de lectura-modificación-escritura sobre `EXTI->IMR` entre distintos niveles de prioridad. El muestreo periódico tiene una tasa de interrupción fija y acotada (4000/seg) independiente de la velocidad física de la rueda, eliminando esa clase de problema de raíz, a costa de una resolución temporal ligeramente menor (peor caso 250µs de retraso en detectar un flanco, insignificante frente a los 10ms del loop de control)
- **`KV_brake_value` mapeado a slider "KV" en Qt:** permite ajustar el freno fuerte en runtime sin recompilar. Se inicializa desde `KV_FRENO_FUERTE`
- **Seguidor de línea con setpoint no negativo:** cuando la línea está visible, el setpoint de inclinación no debe ir a retroceso. No forzar `pwm_saturado` a 0 ni bloquear ruedas internas negativas: esa prueba del 2026-05-17 hizo que el robot empujara hacia adelante sin poder equilibrarse y fue revertida el 2026-05-18.
- **Avance de línea subordinado a estabilidad:** el seguidor de línea solo modula el setpoint de inclinación. Si `roll_filtrado_grados` se aleja del equilibrio o `giro_dps_clampeado` sube, `escala_estabilidad` reduce el avance pedido. El PWM final sigue bajo autoridad del PID de balance.
- **Boost de avance por encoders:** `line_forward_boost` aumenta solo el setpoint de inclinación cuando los encoders muestran que el robot no avanza lo suficiente en modo línea. No actúa sobre motores ni `pwm_sat`; el PID de balance conserva la autoridad final.
- **Escape de reversa por encoders:** si `velocidad_est_ema` es positiva (convención actual: reversa), `linea_escape_reversa_angulo` suma inclinación hacia adelante de forma rampeada. Es más fuerte que el boost de stall, pero sigue actuando solo sobre setpoint.
- **Steering de línea directo:** el PI interno sobre diferencial de encoders quedó deshabilitado/eliminado del camino activo porque dependía de signos de encoder muy sensibles. El giro actual usa `ajuste_direccion = clamp(KP_LINE*linea_error + KI_LINE*I + KD_LINE*D, ±20)` y los encoders quedan para velocidad longitudinal.

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
