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
- **Estación por rueda (Opción A) — queda una traslación rotacional residual (abierto 2026-07-21).** El lazo de posición por rueda (`Ctrl_MotoresBalance`) ya deja al robot bien planto y sin vibrar en el punto dulce, PERO todavía se le escapa **un poco de giro/traslación rotacional lento** aun con el eje de rotación agregado (banda angosta `WHEEL_ROT_DB=3`, `WHEEL_ROT_KP=0.15`, `WHEEL_ROT_KD=0.60`). Pendiente de afinar mañana. Primeros knobs a probar (en orden): bajar `WHEEL_ROT_DB` a 2, subir `WHEEL_ROT_KP` a 0.25, y/o subir `WHEEL_ROT_KD` a 1.0 si aparece temblor al corregir el giro. Si con eso no alcanza, considerar sumar heading-hold por gyro Z/odometría θ (hoy la asistencia de rumbo por gyro solo amortigua la velocidad de giro, no sostiene el rumbo — su `GZ_YAW_ASSIST_DB` deja pasar giros lentos). **El cambio de hoy está SIN COMMITEAR y SIN TAG** (esperando validación) — la última versión funcional guardada es el tag **V26** (commit ba790d5). El feature tiene kill-switch `wheel_pi_enabled=0` para volver al comportamiento V26 sin recompilar la lógica.
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
| 2026-07-24 | Core/Inc/UNER.h, Core/Src/UNER.c, Core/Src/main.c, (Qt: mainwindow.cpp) | **`GETSPEED=0xA4` implementado punta a punta + validación centralizada de floats.** (1) **GETSPEED**: respuesta = 3 floats LE: velocidad global [m/s, **+ = adelante** — UNER.c invierte el signo de `velocity_est_f`, cuya convención interna es negativo=adelante], rueda derecha [rps], rueda izquierda [rps]. Bindings nuevos al final de `UNER_Bindings_t` (`velocity_mps`, `wheel_right_rps`, `wheel_left_rps` → `velocity_est_f`, `speed_right_rps_s`, `speed_left_rps_s`). En Qt (`decodeData` de `mainwindow.cpp`) case nuevo que loguea "VELOCIDAD -> X m/s | Rueda D/I rps" y detecta por longitud un firmware viejo sin el handler. Ojo: es la velocidad cuantizada de encoders (piso ≈0.32 m/s por tick, ver pendiente de resolución). (2) **Helper `getF32BoundedFromRx(rx,out,lo,hi)`**: rechaza NaN/Inf (no escribe) y recorta al rango; aplicado a TODOS los comandos float — KP/KD/KI balance y línea (0..200), LINE_THRES (0..4095), STEERING (±60), LINE_SPEED (0.20..4.00), SETPOINT (±10°), KV_BRAKE (0..100), ROTATE_CUSTOM (±360°), BETA_G/A (0..1). (3) **BETA_G/A (0xBC/0xBD) ahora responden UNKNOWN si no hay binding** (los betas hoy son defines fijos y los punteros quedan NULL — antes ACK mentiroso; ver Pendientes). NO compilado (el usuario compila ambos proyectos). | Usuario: "lo de velocidad lo podes agregar, y hace las mejoras que te parezcan necesarias para robustez" |
| 2026-07-24 | Core/Src/UNER.c | **Auditoría completa del flujo UNER (STM32 ↔ Qt) + 3 fixes.** (1) **`MODIFY_KV_BRAKE=0xBF` NO tenía case en `decodeCommand`**: Qt lo enviaba y el firmware respondía UNKNOWN sin tocar `KV_brake_value` — el ajuste de KV desde Qt estaba roto desde siempre (el puntero `p_KV_BRAKE` se registraba pero nunca se usaba). Agregado el case con isfinite + clamp 0..100 (`KV_BRAKE_CMD_MAX`, mismo rango que el diálogo de Qt) y ACK. (2) **`MODIFY_SETPOINT=0xC8` sin validación**: el spinbox de Qt permite ±180° y se aplicaba directo a `setpoint_trim` — un valor grande volteaba el robot. Ahora isfinite + clamp ±10° (`SETPOINT_TRIM_MAX_DEG`), mismo patrón que 0xC4. (3) **Respuesta `FIRMWARE=0xF1` declaraba 13 bytes pero enviaba 11** (frameLength 12 hardcodeado vs "UNER V1.0" de 9 chars): trama corta desincronizaba el parser de Qt; ahora el largo se calcula con sizeof. Verificado OK (sin cambios): trama de Qt para 0xC4 (checksum incluye header, NBYTES=6 ✓), clamp 0.20..4.00 coherente en ambos lados, mirror `WifiOdomData_t`/`WifiLogData_t` en Qt coincide byte a byte, `ODOM_LINE_STATE_OBJ_FIRST=13` en Qt ya coincide con `LINE_STATE_OBJ_FRENO_REVERSA=13` post-renumeración, `USBRxData` SÍ llama `UNER_PushByte` (el pendiente que decía lo contrario estaba obsoleto — removido). Vestigial detectado: `GETSPEED=0xA4` ("VELOCIDAD" en el combo de Qt) no tiene handler en STM32 ni parser de respuesta en Qt — muerto en ambos lados, se deja documentado. NO compilado (el usuario compila). | Usuario (goal): revisar el flujo de todos los comandos UNER, en especial los nuevos de velocidad del seguidor (0xC4) y setpoint/ángulo (0xC8), que hacía tiempo no se usaban |
| 2026-07-21 | Core/Inc/UNER.h, Core/Src/UNER.c | Formalizado `MODIFY_LINE_SPEED=0xC4` como velocidad objetivo real en m/s. STM32 recibe el float32 ya enlazado a `LINE_SPEED_TARGET`, descarta NaN/Inf y limita cualquier pedido al rango operativo 0.20..4.00 m/s antes de aplicarlo; conserva el ACK y la compatibilidad de protocolo. | Completar el control de velocidad solicitado desde la nueva interfaz Qt sin permitir valores peligrosos o absurdos. |
| 2026-07-21 | Core/Src/main.c | **Estación por rueda (Opción A) — lazo de POSICIÓN por rueda en modo BALANCE/IDLE.** Cada rueda tiene un lazo P PROPIO que mantiene su POSICIÓN (deriva neta `enc − ancla`), ENCIMA del balance común (`pwm_sat` intacto = autoridad principal). **Iteración del día**: primero se hizo lazo de VELOCIDAD (objetivo 0 rps, PI con integral propia por rueda). En el robot rechazaba empujones muy bien PERO la rueda derecha (más fricción) vibraba muchísimo: un lazo de velocidad=0 pelea con el vaivén natural del balance (la rueda TIENE que mecerse para no caerse) → oscilación creciente/stick-slip; el filtro EMA, la banda muerta de velocidad y la fuga de integral no alcanzaron (empeoró: oscilador de relajación). **Rediseño a POSICIÓN**: el vaivén del balance va y vuelve → desplazamiento neto ≈ 0 → el lazo NO lo ve; solo la deriva sostenida acumula posición. Zona muerta ancha `WHEEL_POS_DB=10` counts (~6 cm): DENTRO trim=0 exacto (punto dulce 100% libre), FUERA corrige solo el EXCESO (arranca en 0 en el borde, sin escalón). Primer intento fue **P puro** → oscilaba y "se iba para atrás" (resorte sin freno: P de posición sobre inercia sobrepasa). **Forma PD**: `KP=0.15` %PWM/count (resorte) + `KD=0.60` %PWM/rps (amortiguación por velocidad filtrada EMA, solo fuera de la zona), tope ±8% (`WHEEL_TRIM_MAX`). Tras el signo, el robot quedaba planto pero **giraba lento y constante**: la banda ancha por rueda dejaba rotar libre dentro de ±DB (la asistencia de rumbo por gyro amortigua la velocidad de giro pero no sostiene el rumbo). **Fix = descomposición común/diferencial** (mismo patrón que pwm_sat±correction): TRASLACIÓN (promedio, banda ANCHA `WHEEL_TRANS_DB=10` para ignorar meneos chicos sin vibrar) y ROTACIÓN (semidiferencia, banda ANGOSTA `WHEEL_ROT_DB=3` para atajar el giro), cada eje su PD; la descomposición es lineal → hereda el signo `+` verificado sin re-verificar. La derecha ve trans+rot, la izquierda trans−rot. **Estado al cierre del día: queda una traslación rotacional residual lenta — se dejó como PENDIENTE para 2026-07-22 (ver "Pendientes"), sin commitear ni tag** (la versión funcional guardada sigue siendo V26). Estáticas `wheel_pos_anchor_r/l` + `wheel_pos_armed` (recaptura el ancla al entrar a balance). Kill-switch `wheel_pi_enabled` (0 = comportamiento V26 exacto). SOLO en `Ctrl_MotoresBalance`; pendiente extender a línea/manual tras validar. **Signo verificado en el robot (2026-07-21)**: con `−(...)` amplificaba la deriva hacia adelante (realimentación positiva → se escapaba); la convención de encoder/motor de este chasis pide **`+(...)`** para que sea realimentación negativa. Si alguna vez se aleja del ancla, volver a `−(...)` en las 2 líneas de `wheel_trim_r/l`. Versión funcional previa etiquetada **V26** (commit ba790d5). NO compilado (el usuario compila). | Usuario: pidió un PID de balance por rueda (Opción A). Reportó vibración fuerte de la rueda derecha con el lazo de velocidad y sugirió ignorar desviaciones chicas en el punto dulce → se rediseñó a lazo de posición con zona muerta ancha. Pidió commit+push+tag V26 antes de empezar |
| 2026-07-20 | Core/Src/main.c, CHANGELOG.md, STM32F411CEUX_RAM.ld, STM32F411CEUX_FLASH.ld | **Rename `ControlStep10ms` → `ControlCiclo10ms`** (declaración, definición, llamada en el `while(1)` y comentarios de las etapas). Actualizadas también las referencias vigentes en la documentación (descripción de `main.c` en el árbol de archivos, las 2 filas de "Pendientes" que nombran la función, y el comentario de `_Min_Stack_Size` en ambos linker scripts). Las filas históricas del Registro de Cambios/CHANGELOG que mencionan el nombre viejo NO se tocan (son un registro de lo dicho en su momento). NO compilado (el usuario compila). | Usuario: pidió un nombre en español para el orquestador del loop de control ("controlador de flujo" en vez de "ControlStep10ms"); eligió `ControlCiclo10ms` entre las opciones sugeridas y pidió aplicarlo en todo el código |
| 2026-07-20 | Core/Src/main.c | **Timeout de "pared perdida" (esquive de objeto sin encontrar la pared) ya no manda a `ROBOT_STATE_IDLE` — pasa a `LINE_STATE_GIVEN_UP`.** En `Ctrl_LatchesPared()`, si tras `OBJ_WALL_MISSING_TIMEOUT_MS` (5s) nunca se vuelve a ver el lateral activo (ADC7/ADC5) durante el esquive, el robot se caía: `ROBOT_STATE_IDLE` corta el balance de golpe. Ahora usa el mismo patrón de "reposo total" que ya tenían los timeouts de `LOST_FWD`/`EDGE_FWD` (10s, cuando el retorno por odometría tampoco encuentra la línea): pasa a `LINE_STATE_GIVEN_UP` sin salir de `ROBOT_STATE_LINE_FOLLOWING` — sigue upright con freno de encoders (`Ctrl_SetpointDinamico`), sin más intentos de búsqueda, y retoma solo si la línea reaparece (`LineState_GivenUp`). NO compilado (el usuario compila). | Usuario: "en el esquive del objeto, si la pared nunca la encuentra, no quiero que se salga de modo seguidor de línea para ir derecho a IDLE porque se cae el robot — quiero que pase a modo REPOSO dentro de seguidor de línea, que siga manteniendo balance, como cuando pierdo la línea por mucho tiempo" |
| 2026-07-16 | Core/Src/main.c | **Refactor de legibilidad (goal): `ControlStep10ms` partido en 17 etapas + 18 handlers, y estados muertos eliminados. Sin cambios de lógica buscados; NO compilado (compila el usuario).** (1) **Estados muertos borrados**: `LINE_STATE_OBJ_ESPERA_REVERSA`, `LINE_STATE_OBJ_RETROCESO` (fuera de la cadena desde 2026-07-14) y `LINE_STATE_OBJ_ARC` (reservado, nunca usado) eliminados del enum, del switch, de `LineStateStr`, de las ramas de setpoint y de todas las listas de exclusión; defines/estáticas huérfanas fuera (`OBJ_PRE_REVERSE_*`, `OBJ_REV_CLEAR_CNT/MAX_COUNTS/APPROACH_*/PUSH_ANGLE/BRAKE_KP/ACTIVE_BRAKE_MAX`, `LINE_OBJ_REV_TILT_MAX`, `OBJ_ARC_*`, `obj_pre_rev_start_ms`, `obj_rev_initialized`, `obj_rev_clear_cnt`, `obj_arc_steer_int`). ⚠️ **Esto RENUMERA los valores de `line_state` que viajan a Qt en `WifiOdomData_t` (los OBJ_* bajan 2; los previos a OBJ no cambian)** — si Qt interpreta el número (Vista 3D), actualizar su mirror. Las comparaciones `>= OBJ_ESPERA_REVERSA` pasaron a `>= OBJ_FRENO_REVERSA` (primer estado OBJ vigente). (2) **Reestructuración**: `ControlStep10ms` (~3300 líneas) quedó como orquestador de ~20 líneas que llama etapas `Ctrl_*` en orden (LeerIMU→VelocidadEncoders→LatchesPared→Odometria→SteeringLazoCerrado→TimingDt→FiltroIMU→CuarentenaLinea→EntradasLinea→OverrideManualLinea→CambiosDeEstado→SetpointDinamico→DeteccionCaida→PIDBalance→MezclaMotores→Telemetria→SalidaMotores); cada case del switch de `line_state` es ahora una función `LineState_*` propia despachada por `Ctrl_MotoresLinea`. Las ex-locales compartidas entre etapas son ahora "variables de ciclo" file-scope con los MISMOS nombres (bloque comentado antes de `Ctrl_LeerIMU`); los `break` internos de cases pasaron a `return`; `late_cycle` quedó como const file-scope documentado como vestigio (siempre 0); local `error` de `ComputeSteeringPID` renombrada a `steer_err` (evita shadowing). Cuerpos de código NO tocados salvo lo descripto. (3) **Verificación mecánica** (sin compilar): llaves/paréntesis balanceados ignorando strings/comentarios, profundidad nunca negativa, todas las funciones nuevas a nivel superior, reindentado por script validado con `diff -w` (solo whitespace), UTF-8 intacto. Backups pre-refactor en el scratchpad de la sesión. **Pendiente: compilar (cero warnings) y probar en el robot TODOS los modos.** | Usuario (goal): "que el código quede lo más legible y sencillo de leer sin romper nada de la lógica" — eligió reestructurar en funciones, borrar los estados muertos ya, y compilar él al final |
| 2026-07-16 | Core/Src/main.c | **Comentarios pedagógicos sobre las fórmulas del control (solo comentarios, cero cambios de lógica/código ejecutable).** El usuario está estudiando el firmware y pidió explicaciones de las fórmulas EN el código (rechazó explícitamente un documento aparte). Agregados en: conversión encoders→m/s (análisis dimensional counts→rps→m/s, convención avance=negativo), filtros EMA (y += β·(x−y), τ≈dt/β), escala del gyro (LSB→°/s, por qué 100 y no 131), ángulo por accel (atan2 de la gravedad, por qué solo vale cuasi-estático), filtro complementario (98/2, τ≈0.49s), tilt lateral (atan2(-ax,√(ay²+az²))), centroide de línea (pesos cuadráticos {±9,±1}, normalización por 9·w_sum), speed_factor cuadrático, PI velocidad→ángulo (cascada: lazo externo pide grados, balance los convierte en PWM), PID de balance (términos P/I/D con unidades, derivada por diferencia finita y sus clamps), soft zone (interpolación lineal 0.35→1), anti-windup (integración condicional), slew-rate del PWM (±8/ciclo), PID de línea (segundo PID: común vs diferencial; EMA antes de derivar), odometría (dead-reckoning, Δθ=ω·dt, regla del trapecio en θ_medio), MotorControl (duty%→CCR: CCR=(ARR+1)·duty/100). NO compilado (el usuario compila) — al ser solo comentarios no cambia el binario. | Usuario: "no entiendo las fórmulas… me gustaría que lo agregues como comentario para lograr comprenderlo sin moverme del código" |
| 2026-07-14 | Core/Src/main.c | **STOP neto de 2s sobre la línea después del giro perpendicular (`LINE_STATE_PERP_SETTLE`, display "PSTOP").** Al reencontrar la línea tras el esquive (saliendo de pared>avanza/gira), el robot llegaba con velocidad, hacía el giro de `PERP_ROTATE` y retomaba FOLLOWING lanzado — "envión que lo desestabiliza" (reportado). Estado nuevo entre el giro y FOLLOWING: `PERP_SETTLE_MS=2000ms` de upright + freno de encoders + **hold de posición** (mismo patrón y constantes `OBJ_PAUSA_POS_*` que el hold de PAUSA_GIRO: ancla de encoders al entrar, 0.06°/count, tope ±1.5°, banda muerta 3 counts, primer ciclo sin corregir) + yaw-assist de gyro como los demás holds. A los 2s pasa a FOLLOWING con los resets de siempre (integral, `line_lost_ms`, `all_black_start_ms` para no re-disparar PERP). Como el giro de PERP ya sigue el sentido del último esquive, el frenado queda automáticamente "dependiendo del lado al que esquivó". Aplica a TODOS los cruces perpendiculares (también los de una intersección normal): 2s de estabilización ahí también, considerado aceptable/beneficioso. NO compilado (el usuario compila). | Usuario: "ni bien ve la línea luego de esquivar, tras pared>avanza o giro, se acelera demasiado — agregá un período de STOP neto de 2 segundos que frene sobre la línea y luego siga sin envión" |
| 2026-07-14 | Core/Src/main.c | **`OBJ_REV_HOLD_ANGLE_MAX` 1.6→2.6°.** El hold de distancia del STOP a veces "intenta corregir pero el ángulo no le da" (reportado): la corrección proporcional saturaba en 1.6° y con fricción alta quedaba empujando sin moverse hasta que el anti-stall (rampa lenta, ~1s) lo destrababa. Techo subido a 2.6°; `ANGLE_MIN` (0.9°) y el anti-stall de respaldo sin cambios. | Usuario: "hay veces que intenta corregir pero no logra porque el ángulo no le da" |
| 2026-07-14 | Core/Src/main.c | **El giro de esquive ya no puede abortar hacia la línea + hold de posición en la pausa post-giro.** (1) **Aborto eliminado**: el ajuste fino del giro (fase 2) abortaba a `LOST_FWD` si el lateral (ADC5/ADC7) nunca vio la pared — pensado para detecciones fantasma (2026-07-10). En el esquive a IZQUIERDA pasaba seguido (reportado: "si ve la línea durante el giro se sale del modo esquive", confirmado por display: la cadena era giro→VUELVE→SIGUE instantáneo → el robot volvía derecho al obstáculo; a derecha no pasaba porque ADC7 sí confirma la pared). Con la entrada por fase STOP el objeto ya está confirmado de sobra por A6/A8 (fantasma casi imposible): ahora el giro SIEMPRE sigue a `PAUSA_GIRO`; si la pared realmente no está, el timeout de pared perdida (5s→reposo) cubre ese caso sin regresar hacia el obstáculo. `obj_rot_p2_seen` eliminado (quedó sin lectores). Esto implementa el "que ignore la línea mientras gira": ningún estado del giro/pausa reacciona a la línea ahora. (2) **Hold de posición en `PAUSA_GIRO`** (pedido: "luego del giro que mantenga posición un poco mejor"): antes upright + freno por velocidad LENTA (solo frena, no devuelve la deriva). Nuevo P por encoders: ancla `obj_pausa_r0/l0` al entrar, corrección `OBJ_PAUSA_POS_KP=0.06°/count` (≈1° por 10 cm), tope ±1.5°, banda muerta 3 counts (~2 cm); primer ciclo sin corregir (el ancla se estampa en el case, que corre después del setpoint). NO compilado (el usuario compila). | Usuario: "si ve la línea durante el giro se sale del modo esquive (a la derecha no pasaba) — que la ignore mientras gira y los primeros segundos de la pared" + "luego del giro que hace, que mantenga posición un poco mejor" |

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
