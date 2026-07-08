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
| Push de odometría (2026-07-06) | `WifiOdomData_t` (cmd `0xDC`, packed: t_ms, x_m, y_m, theta_deg, line_error, line_detected, robot_state, line_state) enviado cada `WIFI_ODOM_PERIOD_MS=500ms` automáticamente en cuanto `f_wifi_connected=1` — **no depende de `ACTIVATE_WIFI_LOG`**, pensado para graficar mapa XY + posición de línea en Qt sin competir por ancho de banda/CPU con la telemetría de control a 10 Hz |

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
- **Última sesión:** 2026-07-06

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
- Detección de objetos en modo línea: ADC 5-8 (largo alcance) con debounce 30 ms → pasa a IDLE automáticamente al detectar obstáculo
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
- El SSID/IP WiFi está hardcodeado en `main.c` (líneas ~244); cambiar manualmente según red
- `USBRxData` tiene el `UNER_PushByte` comentado — los comandos USB desde Qt no se procesan por esa ruta
- Código de debug activo en `ESP01.c` (printfs de estados AT) que genera tráfico USB extra
- **Freeze residual al mover el robot** — Causa raíz identificada (2026-05-18): el DMA del I2C queda en estado "en progreso" indefinidamente sin completar. `mpu_data_ready_for_ctrl` nunca se setea, `ControlStep10ms` nunca corre, `i2c1_tx_busy` queda en 1, display congelado, motores al último PWM. El `while(1)` SIGUE girando (IWDG se patea) pero el sistema es funcionalmente inútil. Fix: watchdog de software en `while(1)` — si pasan >150ms sin dato del MPU se llama `I2C1_Recover()` (9 pulsos SCL + HAL_I2C_Init + I2C_Manager_Init) y se relanza la lectura SIN resetear el MCU. Detectable por "I2C RECOVER\r\n" en USB. **2026-07-01: se sospecha que este freeze estaba agravado (no necesariamente causado) por el viejo sistema de encoders vía EXTI, que compartía prioridad NVIC con el DMA de I2C y podía dejar la transacción I2C pendiente en cola durante ráfagas de pulsos (ver Registro de Cambios 2026-07-01). Con encoders migrados a polling por TIM2 y el DMA de I2C subido a prioridad 0, debería ser mucho menos frecuente — pendiente de confirmar en el robot físico si desaparece del todo.**
- Resolución de velocidad limitada: piso de cuantización de `velocity_est` (calculado en `ControlStep10ms` sobre delta de counts cada 10ms, no sobre el muestreo de 4kHz de `SampleEncoders250us` que solo alimenta el conteo acumulado) ≈ 0.32 m/s con 1 count en una sola rueda por ciclo, ≈ 0.63 m/s si tiquean ambas ruedas. `BRAKE_VEL_DEADBAND=0.35` (2026-07-01) calibrado para filtrar ese piso. No apto para integración de posición ni para reaccionar a velocidades reales por debajo de ese umbral
- **Freeze + reset durante giro de 180° (LOST_ROTATE), reportado repetidamente pese a varios intentos de fix.** Sesión 2026-07-01: se probó primero bajar `LROT_PIVOT`/`LROT_BRAKE` (hipótesis de glitch eléctrico) + corregir prioridad NVIC I2C vs EXTI15_10 — el usuario confirmó que el problema **persistió igual**, descartando la hipótesis de corriente de motor como causa suficiente. El usuario migró el conteo de encoders de EXTI a polling por TIM2 (ver más abajo) sospechando que el bug estaba en el subsistema de encoders. Revisión de código no encontró un bug concreto en el nuevo esquema de polling — arquitectura correcta y con más margen que el EXTI viejo. **Pendiente de validar en el robot físico si el freeze+reset desaparece con el nuevo esquema de encoders.** Si persiste, revisar next: reversión brusca de motores (EMI) con un osciloscopio en VDD durante el pivot, o un logic analyzer en las líneas de encoder para confirmar que no hay rebote/ruido eléctrico real en el sensor.

---

## 📋 Registro de Cambios
> **Instruccion para Claude:** Al finalizar cada sesion, agregar una fila con los cambios realizados (la mas reciente arriba).
> Mantene aca solo las ~10 filas mas recientes: cuando agregues filas nuevas, move las que sobren al TOPE de la tabla de `CHANGELOG.md`. **Nunca borres una fila — siempre movela.**
> Historial completo del proyecto (2026-05-04 en adelante): ver `CHANGELOG.md` en la raiz del proyecto.

| Fecha | Archivo(s) modificado(s) | Cambio realizado | Motivo / Observación |
|-------|--------------------------|------------------|----------------------|
| 2026-07-07 | Core/Src/main.c | **Indicador en pantalla 6 (OBJETO) de la acción del árbitro de wall-following**. El sub-estado del esquive ya se mostraba (`LineStateStr`: ESPER/RETRO/ESQUIV/PAUSA/APRCH/PARED/LIBRE/GIRAP + tiempo transcurrido), pero dentro de los 3 estados de pared no se veía qué acción estaba ejecutando el árbitro. Nueva `ObjWallActionStr()` (AVZ/PIV/REV según `obj_wall_action`); en PARED/LIBRE/GIRAP el título de la pantalla 6 pasa a "PARED>REV" (formato `estado>accion`), en el resto de los estados queda igual que antes. Compilado con GCC de CubeIDE sin errores. | Usuario pidió un indicador en el display del sub-estado del esquivador de objetos; el sub-estado ya estaba — lo que faltaba era la acción del árbitro dentro de los estados de pared |
| 2026-07-07 | Core/Src/main.c | **Restaurada la salida de la reversa por pared por ADC6/ADC8, ahora con umbral 3600** (pedido de sesión anterior que nunca se aplicó: el usuario había pedido subir 3500→3600, pero para entonces el commit `95b43db` ya había reemplazado toda esa condición por salida por encoder — el umbral no existía más). `ObjWallUpdateReverseLatch()` vuelve al esquema del commit `61a81d1`: el latch se suelta cuando `adcAvg[5]` y `adcAvg[7]` (ADC6/ADC8 frontales) leen ≥ `OBJ_WALL_REVERSE_CLEAR_ADC=3600` durante `OBJ_WALL_REVERSE_CLEAR_CNT=4` ciclos consecutivos (40 ms). Eliminados `OBJ_WALL_REVERSE_COUNTS`/`obj_wall_reverse_r0` (salida por 100 counts de encoder derecho). **Se conservan** las mejoras posteriores al commit original: timeout de seguridad `OBJ_WALL_REVERSE_TIMEOUT_MS=4000` (ahora significa "si ADC6/ADC8 nunca despejan en 4s, soltar"), armado del latch restringido a los 3 estados de pared, y liberación al salir de ellos. Compilado con GCC de CubeIDE sin errores. | Usuario preguntó si el cambio 3500→3600 pedido en una sesión anterior se había hecho; al ver que la condición ya no existía, pidió volver a ella con el nuevo umbral |
| 2026-07-07 | Core/Src/main.c | **Fix "en modo pared no le da el ángulo para ir atrás ni el PWM para girar" — autoridad insuficiente en todas las acciones del wall-following** (la reversa inicial OBJ_RETROCESO andaba bien porque tiene 8.5° + excepción de clamp + rampa propia; el modo pared no tenía nada de eso). 4 cambios con el mismo criterio ya validado en el resto del código: (1) `OBJ_WALL_REVERSE_ANGLE` 3.0→**6.0°** — la soft-zone del PID (error<1.5° → autoridad al 15%) hace que reversas de 2-3° no venzan la fricción estática desde parado, exactamente el mismo motivo por el que OBJ_RETROCESO terminó en 8.5°. (2) **Excepción nueva en `sp_limit` (clamp global de setpoint): 7.0° para los 3 estados de pared** — sin esto el límite general de 5.0 recortaba los 6° en silencio (mismo patrón que la excepción de OBJ_RETROCESO=9.0). (3) **Rampa `sp_step_max=0.6°/ciclo` para la reversa por latch en estados de pared** (antes usaba la general de 0.15 → llegar a -6° tardaba 400ms sin empujón inicial; mismo fix que OBJ_RETROCESO 2026-07-03). (4) `OBJ_WALL_PIVOT_POWER` 8→**20** — historial: 5 y 8 insuficientes (2026-05-27), los pivots de LOST_ROTATE necesitaron 14→20→24 (2026-07-05); si sigue débil probar 24. (5) `OBJ_WALL_APPROACH_ANGLE_MAX` 1.5→**2.5°** (1.5 casi no movía, mismo caso que LOST_FWD_ANGLE_MAX 1.0→2.5 del 2026-07-04). Compilado con GCC de CubeIDE sin errores. | Usuario reportó tras probar el árbitro: la reversa inicial anda bien, pero en modo pared no le da el ángulo para retroceder ni el PWM para girar — todas las magnitudes del wall-following estaban por debajo de los mínimos físicos ya descubiertos empíricamente en otros estados |
| 2026-07-07 | Core/Src/main.c | **Árbitro de acción con compromiso mínimo para el wall-following — fix "cambia de decisión tantas veces por segundo que no hace nada"**. Los 3 estados de pared re-evaluaban reversa/pivot/avance/transición **cada ciclo de 10 ms**: con el robot al borde de los umbrales (aun con el filtro+histéresis+debounce de flags), la acción cambiaba varias veces por segundo y ninguna llegaba a producir efecto físico. Nueva función `ObjWallDecideAction()` (corre 1 vez por ciclo junto a `ObjWallUpdateAdc`/`ObjWallUpdateReverseLatch`): elige UNA acción vigente (`obj_wall_action` = AVANZA / PIVOT_CERCA / REVERSA) y **se compromete con ella `OBJ_WALL_ACTION_MIN_MS=450ms`** antes de permitir otra. Excepción: la REVERSA entra y sale al ritmo del latch (que ya tiene su propio compromiso: 100 counts o timeout de 4s) — demorar la entrada arriesga chocar, demorar la salida alarga la reversa. Los 3 estados (setpoint + máquina de estados) ahora consumen `obj_wall_action` en vez de leer los flags crudos. Además, **mínimo de permanencia en estado `OBJ_WALL_STATE_MIN_MS=450ms`**: BORDEAR→LIBRE por "perdí la pared" exige ≥450ms desde la entrada a BORDEAR (`obj_wall_fwd_start_ms`), y GIRO_PARED→BORDEAR por "pared vista" exige ≥450ms desde la entrada al giro (nueva `obj_wall_turn_start_ms`, seteada en LIBRE→GIRO_PARED) — evita rebotar entre estados varias veces por segundo. LIBRE→GIRO ya tenía compromiso natural (100 counts de avance). Tuning: si sigue indeciso subir los 450ms de a 100; si reacciona lento, bajarlos. Compilado con GCC de CubeIDE sin errores ni warnings. | Usuario reportó que tras los fixes del deadlock el robot ya se mueve pero "varía demasiadas veces por segundo y no se decide nunca" — chattering de decisiones sin compromiso temporal |
| 2026-07-07 | Core/Src/main.c | **Fix "el esquive se queda quieto" tras la robustificación del wall-following hecha por el usuario** (filtro IIR de ADC7 + histéresis + debounce + latch de reversa por encoder + arco odométrico, commits `1f71728`..`95b43db`). Diagnóstico: deadlock de tres piezas cooperando. (1) **`balance_hold_active` no excluía los 3 estados de pared** (`OBJ_BORDEAR_PARED`/`OBJ_PARED_LIBRE`/`OBJ_GIRO_PARED`) — mismo patrón de bug ya arreglado en RETROCESO/BUSCAR_PARED/MANUAL: la rama de reversa del latch primero frena a quietud total y después pide −2.2°; con el robot quieto y el setpoint rampeando desde ~0, |error|≤0.70° → hold entra → PID silenciado → el robot no se mueve. (2) **El latch de reversa solo se soltaba con 100 counts del encoder derecho** — exige que el robot SE MUEVA para liberarse; con el PID silenciado (o con 2.2° por debajo de la fricción estática) los counts nunca llegan → latch armado para siempre → quieto indefinidamente. (3) El latch **se armaba en cualquier `line_state`** (corría cada ciclo del modo línea): podía armarse durante el giro de esquive/pausa/FOLLOWING (el sensor lateral pasa cerca del objeto al girar) y entrar ya activo a BORDEAR_PARED. Fixes: 3 estados de pared agregados a la exclusión del hold; latch solo puede armarse dentro de los 3 estados de pared y se suelta al salir de ellos; **nuevo `OBJ_WALL_REVERSE_TIMEOUT_MS=4000`** como colchón (si en 4s no completó los counts, suelta — el robot no logró moverse); `OBJ_WALL_REVERSE_ANGLE` 2.2→**3.0°** (valor previo validado; 2.2 probablemente bajo la fricción estática desde parado — si sigue sin arrancar subir de a 0.5); `OBJ_WALL_PIVOT_POWER` 5→**8** (5 está documentado como insuficiente para rotar en el changelog del 2026-05-27 — con 5, GIRO_PARED tampoco se movía). **Riesgo señalado sin tocar**: el arco odométrico nuevo (`ObjWallComputeArcSteer`) depende del signo de `ODOM_THETA_SIGN`, que sigue **sin validar en el robot físico** — si está invertido, el semicírculo curva HACIA la pared (too_close/reversa permanentes). Verificado compilando `main.c` con la GCC de CubeIDE (sin errores ni warnings). | Usuario reportó que tras sus mejoras de robustez el robot se queda quieto y no logra hacer el esquive; pidió revisar que esté todo bien |
| 2026-07-06 | Core/Src/main.c, Core/Src/UNER.c | **Comandos manuales (adelante/atrás/izquierda/derecha) también funcionan en LINE_FOLLOWING, pero SOLO si no ve la línea**. `UNER.c`: `MOVE_FORWARD/BACKWARD/LEFT/RIGHT/STOP` ahora aceptan `robot_state==3` (LINE_FOLLOWING) además de `==4` (MANUAL_CONTROL) — el filtro real de "solo si no ve la línea" vive en `main.c`, no acá (UNER.c no tiene acceso a `line_detected`). `main.c`: nueva variable `manual_line_override` (calculada una vez por ciclo, justo después de que `line_detected` queda definido): `(robot_state==LINE_FOLLOWING) && !line_detected && (HAL_GetTick()-manual_cmd_last_ms)<250 && (comando activo)`. El chequeo de recencia (<250ms) es necesario porque el decay de `manual_setpoint_cmd`/`manual_steering_cmd` vive dentro de la rama MANUAL (que no corre mientras se sigue la línea normalmente) — sin él, un comando viejo ya soltado hace rato quedaría "congelado" en un valor no-cero y dispararía el override de golpe la próxima vez que se pierda la línea, sin que el usuario esté mandando nada en ese momento. **Reutiliza el control de MANUAL tal cual, sin duplicar el algoritmo**: se extendieron las condiciones de las dos ramas de MANUAL (cálculo de setpoint ~línea 2917, mezcla de motores ~línea 4688) a `|| manual_line_override`, y la condición de la rama normal de LINE_FOLLOWING en esos mismos dos lugares a `&& !manual_line_override` (para que no compitan). **Bugs de clamp silencioso encontrados y corregidos en el camino** (mismo patrón de bug ya visto varias veces en este archivo — un clamp que sigue mirando `robot_state`/`line_state` directamente, sin saber que hay un override activo): (1) `sp_limit` (tope global de ángulo) le daba 5.0° en vez de los 6.0° de MANUAL porque miraba `robot_state==MANUAL_CONTROL` literal. (2) el clamp de `pwm_sat` a ±40 (pensado para `LINE_FOLLOWING`+`FOLLOWING`) se aplicaba durante el override en vez del ±55 de MANUAL, igual que `pwm_limit` (40 en vez de 100). (3) `sp_step_max`/`brake_step_max` (rampas de suavizado) usaban los valores de LINE_FOLLOWING (0.15) en vez de los de MANUAL (0.1). (4) la exclusión de `balance_hold_active` para MANUAL (agregada en una sesión anterior) exigía `robot_state==MANUAL_CONTROL` literal — durante el override no se cumplía y el hold podía volver a silenciar el PID. (5) **el más importante**: `ComputeBrakeSetpointTarget(robot_state)` devuelve `0.0f` de entrada si el estado es `LINE_FOLLOWING` (ver la función) — se pasa explícitamente `ROBOT_STATE_MANUAL_CONTROL` en vez de `robot_state` cuando `manual_line_override` está activo, si no el freno traslacional quedaba completamente anulado durante el override. | Usuario pidió que el seguidor de línea responda a los comandos manuales (adelante/atrás/izquierda/derecha) cuando no está viendo la línea, ignorándolos por completo en cuanto la vuelve a ver |
| 2026-07-06 | Core/Src/main.c, Core/Src/UNER.c | **Segunda vuelta de ajuste de MANUAL: giro a 1/4 de fuerza + corrección de rumbo recto en adelante/atrás**. (1) El giro seguía sintiéndose "demasiado fuerte" pese al suavizado de rampa de la vuelta anterior — la magnitud final (no solo la rampa) era el problema. Reducida a 1/4: `MOVE_LEFT`/`MOVE_RIGHT` en `UNER.c` ahora mandan `∓15.0f`/`±15.0f` (antes `∓60.0f`/`±60.0f`), y `STEER_RATE` (rampa hacia `manual_steering_cmd`) bajado también a 1/4 (`0.25→0.0625`/ciclo). (2) **Adelante/atrás ahora van derecho**: antes, con `manual_steering_cmd=0`, `steering_adjustment` simplemente rampeaba a 0 sin ninguna corrección activa — cualquier asimetría mecánica entre ruedas hacía que el robot se curvara al mover. Se reutilizó tal cual el algoritmo de rumbo recto ya calibrado en la reversa de `OBJ_REVERSE`/wall-following (P sobre `speed_right_rps_s - speed_left_rps_s`, constantes `REV_STRAIGHT_KP/MAX/SLEW`, sin inventar nada nuevo): nueva variable `manual_straight_steer_f` (mismo patrón que `obj_rev_steer_f` pero dedicada a MANUAL, reseteada en entrada a MANUAL y en el bloque de recuperación de caída) que corrige `steering_adjustment` cuando `manual_turning` es falso (`fabsf(manual_steering_cmd) <= 0.5`); al girar, se sincroniza con `steering_adjustment` para que la transición entre "girando" y "derecho" no salte. | Usuario probó el fix anterior: el giro seguía muy fuerte (pidió explícitamente 1/4 de la fuerza actual, "siempre movimientos suaves") y notó que adelante/atrás dobla en vez de ir derecho, pidiendo reutilizar el algoritmo de rumbo recto que ya existe en el código (reversa) en vez de inventar uno nuevo |
| 2026-07-06 | Core/Src/main.c, Core/Src/UNER.c | **Fix "MANUAL no avanza ni retrocede" + control de velocidad + giro suavizado**. Causa raíz real del "no responde" encontrada: `balance_hold_active` (silencia el PID poniendo `balance_pi_scale=0`/`balance_d_scale=0` cuando el error de inclinación es chico) **nunca había tenido excepción para `ROBOT_STATE_MANUAL_CONTROL`** — solo la tenía `LINE_FOLLOWING`. Con un comando de adelante/atrás activo, apenas la inclinación real se acercaba al setpoint pedido (error chico) el hold se activaba y mataba la autoridad del PID justo cuando debía sostener el avance — mismo patrón de bug ya visto y arreglado varias veces en otros estados (`LOST_FWD`/`EDGE_FWD`/`OBJ_RETROCESO`/`OBJ_BUSCAR_PARED`), nunca extendido a MANUAL. Fix: nueva rama `else if (robot_state == ROBOT_STATE_MANUAL_CONTROL && fabsf(manual_setpoint_cmd) > 0.01f) balance_hold_active = 0;` — solo se excluye mientras hay un comando activo, así que el hold sigue sosteniendo el balance estático cuando MANUAL está realmente idle. **Control de velocidad para adelante/atrás** (a pedido, "como en otros modos"): reemplazado el mapeo directo a un ángulo fijo (antes ±4°, con rampa interna carísima de 0.01°/ciclo ≈ 4s para llegar) por un PI de velocidad (mismo patrón que `LINE_VEL_KP`/`LINE_VEL_KP_BRAKE`/`LINE_VEL_KI` del seguidor de línea, extendido a bidireccional): `manual_setpoint_cmd` pasó a ser la velocidad deseada en m/s con signo (antes grados) — `MOVE_FORWARD`/`MOVE_BACKWARD` en `UNER.c` ahora mandan `±1.0f` (antes `±4.0f`). Nuevas constantes locales `MANUAL_SPEED_MAX=1.0` m/s, `MANUAL_ANGLE_MAX=6.0°`, `MANUAL_VEL_KP=6.0` (acelerando), `MANUAL_VEL_KP_BRAKE=10.0` (frenando/revirtiendo), `MANUAL_VEL_KI=2.0`, `MANUAL_VEL_I_MAX=2.0`; nueva integral persistente `manual_vel_integral` (reseteada al entrar a MANUAL y en el bloque de recuperación de caída). **Bug de silencioso descubierto y corregido en el camino**: el clamp global `sp_limit` (por defecto 5.0° para cualquier estado sin excepción propia) habría recortado el nuevo tope de 6° a 5° sin avisar — se agregó excepción `(robot_state == ROBOT_STATE_MANUAL_CONTROL) ? 6.0f` (mismo patrón que la excepción ya existente de `OBJ_RETROCESO`). **Giro suavizado**: `STEER_RATE` (rampa de `steering_adjustment` hacia `manual_steering_cmd` en el bloque de salida de motores) bajado de `1.5` a `0.25` por ciclo — antes llegaba al steering máximo (±60) en ~0.4s (sentido "brusco"/pivot brusco), ahora tarda ~2.4s en llegar y decae igual de suave al soltar (mismo mecanismo de rampa, sin cambios adicionales). | Usuario reportó que en MANUAL los comandos de adelante/atrás llegan pero el robot no logra moverse ("le falta inclinación"), pidió que use control de velocidad (1 m/s máx, 6° máx) como en otros modos, y que el giro (que sí funciona) sea mucho más suave, de a poco hasta soltar |
| 2026-07-06 | Core/Src/main.c | **Eliminado el banco de pruebas del giro de 90° en modo MANUAL** (el ciclo automático que se activaba con el joystick/comandos en reposo, agregado el 2026-07-05). `ROBOT_STATE_MANUAL_CONTROL` ahora ejecuta SIEMPRE el control manual normal (comandos por WiFi/USB) en las dos secciones que lo implementan (cálculo de `base_setpoint_target`/`brake_setpoint_target` en el bloque de setpoint dinámico, y el cálculo de `motorRight/LeftVelocity` en el bloque de salida de motores) — se quitó el `if (joystick_idle) {...} else {...}` de ambas y quedó solo el cuerpo del `else` (ya idéntico al comportamiento real por comandos). Con `manual_setpoint_cmd`/`manual_steering_cmd` en 0 (sin comandos llegando) esa misma rama ya balancea en el lugar sin moverse — no hace falta un caso especial para "reposo". Eliminadas las variables `manual_test_rot_active`/`manual_test_wait_ms` (ya sin uso) y sus resets dispersos (entrada a MANUAL, bloque de recuperación de caída). Las constantes `MROT_*` (locales al bloque eliminado) desaparecieron con él. **Sin tocar**: `obj_rot_*` (compartidas con `LOST_ROTATE`/`EDGE_ROTATE`/`PERP_ROTATE`/`OBJ_GIRO_ESQUIVE`, siguen usándose igual ahí), el registro de comandos UNER (`MOVE_FORWARD/BACKWARD/LEFT/RIGHT/STOP`, `ACTIVATE_MANUAL_CONTROL`) y los comandos `ROTATE_90_*`/`ROTATE_180_*`/`ROTATE_CUSTOM` (ya eran no-op desde la sesión del 2026-07-04, sin cambios). **Verificado**: la cadena completa de comandos manuales (`UNER.c` case `MOVE_*` → `manual_setpoint_cmd`/`manual_steering_cmd`/`manual_cmd_last_ms` vía punteros registrados con `UNER_RegisterManualControl` → consumidos en `main.c` sin cambios de lógica, solo se quitó la rama alternativa de "idle") sigue intacta — el robot se mueve igual que antes por WiFi/USB, solo que ya no hace nada por sí solo cuando no hay comandos. | Usuario pidió eliminar el giro de 90° automático del modo MANUAL, dejándolo solo para mover el robot con comandos por WiFi |
| 2026-07-06 | Core/Inc/UNER.h, Core/Src/UNER.c, Core/Src/main.c, `C:\Microcontroladores\BalancinQT\mainwindow.h`/`mainwindow.cpp` (Qt) | **Push periódico de odometría por WiFi para graficar en Qt (mapa XY + posición de línea)**. Nuevo comando `CMD_WIFI_ODOM_DATA=0xDC` y struct packed `WifiOdomData_t` (t_ms, x_m, y_m, theta_deg, line_error, line_detected, robot_state, line_state) — mirror exacto en `UNER.h` (firmware) y `mainwindow.h` (Qt, mismo patrón `#pragma pack(1)` que ya usa `WifiLogData_t`). `UNER_SendWifiOdomData()` en `UNER.c`, mismo patrón que `UNER_SendWifiLogData` (gateado por `ESP01_StateUDPTCP()==CONNECTED && !ESP01_IsSending()`). En `main.c`: nuevo `#define WIFI_ODOM_PERIOD_MS 500` y bloque de envío en `ControlStep10ms`, gateado únicamente por `f_wifi_connected` (**no** por `ACTIVATE_WIFI_LOG`) — arranca solo con detectar conexión WiFi, sin necesidad de tocar nada desde Qt, y a un ritmo bajo (2 Hz) a propósito para no competir por CPU/ancho de banda con la telemetría de control existente (10 Hz). Nueva variable `line_detected_disp` (file-scope, seteada junto a `line_error_disp` en el bloque de seguidor de línea) para poder leer "¿se ve la línea ahora?" fuera del scope local de `line_detected`. **Lado Qt**: nueva pestaña "Odometría (WiFi)" en `tabWidget_Graficas` (creada 100% por código en el constructor, mismo patrón que las demás pestañas — no se tocó el `.ui`) con un `QChart` que dibuja la trayectoria (x,y) como línea gris, puntos verdes donde se veía la línea y rojos donde no, y un marcador azul grande para la posición actual; autoescala de ejes que solo crece (nunca se achica) con margen de 0.3m; label de texto con X/Y/Theta/línea numéricos. Nuevo método `MainWindow::updateOdomChart()` y case `WIFI_ODOM_DATA` en `decodeData()` (usa `reinterpret_cast<WifiOdomData_t*>`, igual que ya hace `WIFI_LOG_DATA` con `WifiLogData_t`). | Usuario pidió, aprovechando la odometría ya implementada, poder graficar en Qt la posición del robot en un mapa XY y por dónde está la línea, enviado automáticamente por WiFi a un ritmo bajo para no afectar las tareas de control más importantes |

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
- **Push de odometría por WiFi (2026-07-06)**: `CMD_WIFI_ODOM_DATA=0xDC` (struct packed `WifiOdomData_t`, ver "Canal 2 — WiFi UDP") ya tiene mirror y parsing en Qt (`mainwindow.h`/`mainwindow.cpp`) — pestaña "Odometría (WiFi)" con mapa XY. Si se cambia el layout de `WifiOdomData_t` en `UNER.h`, hay que actualizar el mirror en `mainwindow.h` (mismo patrón que `WifiLogData_t`)
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
