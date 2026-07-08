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
| 2026-07-08 | Core/Src/ESP01.c | **Fix inestabilidad WiFi: "conecta perfecto, envía unos segundos, se corta, se reconecta en ciclo, y en algún momento muere del todo"** — cadena de 4 bugs en el driver, todos en el camino de envío: (1) **+IPD pisaba el envío en curso**: al terminar de recibir un paquete de Qt, el parser limpiaba `WAITINGSYMBOL`/`TXCIPSEND`/`SENDINGDATA` — si llegaba en medio de un CIPSEND nuestro, lo abortaba a mitad de camino y el siguiente CIPSEND salía encimado → ERROR → reconexión. Ya no toca los flags de envío. (2) **ERROR mataba el socket**: con `SENDINGDATA` activo, un simple ERROR de CIPSEND (ESP ocupado) ponía `UDPTCPCONNECTED=0` → el chequeo periódico de 5s de `ESP01DOConnection` reabría el socket → el ciclo visible de reconexión. Ahora ERROR aborta solo el envío (flags + buffer TX); los cierres reales siguen llegando por CLOSED/DISCONNECTED. (3) **`esp01TimeoutSending` existía pero NUNCA se armaba**: si un SEND OK se perdía, `SENDINGDATA` quedaba en 1 para siempre → `ESP01_Send` devolvía BUSY eternamente → "se corta totalmente" (solo un paquete entrante lo destrababa, por el bug 1). Ahora se arma en 1s en cada `ESP01_Send`, se cancela en SEND OK/ERROR, y el force-clear limpia TAMBIÉN `TXCIPSEND`/`WAITINGSYMBOL` y descarta el buffer TX (antes dejaba basura que se pegaba al próximo CIPSEND). (4) **Timeout del prompt '>' de 50ms reiniciaba TODA la máquina AT** (`esp01ATSate=ESP01ATAT` → AT/CWMODE/CWJAP/CIPSTART completo con WiFi sano) por un prompt perdido — el '>' solo se detecta con el parser en estado 0, así que se perdía fácil si llegaba a mitad de otro token. Ahora: timeout 50→300ms, y al vencer aborta solo ese envío sin tocar la máquina de estados (si el socket está caído de verdad, CLOSED/DISCONNECTED o DOConnection lo reconectan). Compilado con GCC de CubeIDE sin errores ni warnings. Pendiente de validar en el robot con Qt corriendo. | Usuario reportó WiFi inestable: conecta y envía varios paquetes, a los ~5 segundos deja de enviar y se reconecta, cicla así todo el tiempo, y en algún momento se corta totalmente la conexión |
| 2026-07-08 | Core/Src/main.c | **`OBJ_REV_HOLD_ANGLE` 4.0→2.75°** (corrección de distancia del STOP post-reversa). Historial del valor en el día: 2.0 (no movía) → 2.5 (poco) → 4.0 (muy brusca) → **2.75**. Compilado con GCC de CubeIDE sin errores. | Usuario: la corrección de 4° para mantenerse en el setpoint de ADC quedó muy brusca |
| 2026-07-08 | Core/Src/main.c | **`OBJ_WALL_THRESHOLD` 3750→3600** (umbral "pared visible/perdida" de ADC7 en el wall-following). Con 3750 el robot se alejaba demasiado del objeto antes de declararlo perdido y girar a la izquierda a buscarlo; con 3600 reacciona antes y bordea más pegado. Arrastra automáticamente: `wall_visible` en PARED (transición a LIBRE), la salida de GIRAP (re-ver pared → volver a PARED) y el tick superior de la barra A7 en pantalla 6. Compilado con GCC de CubeIDE sin errores. | Usuario: "quiero que gire a la izquierda cuando hay valores superiores a 3600 mejor, porque se aleja mucho sino" |
| 2026-07-08 | Core/Src/main.c | **`OBJ_REV_CLEAR_ADC` 3600→3500** (setpoint de distancia de la reversa inicial por ADC6/ADC8). Arrastra automáticamente todo lo que referencia esa constante: corte de la reversa, hold de distancia del STOP (banda pasa a ser 3500..3800), condición de estabilidad de 2s previa al giro, y el tick de la barra A6 en pantalla 6. Compilado con GCC de CubeIDE sin errores. | Usuario pidió bajar el setpoint de la reversa a 3500 (queda un poco más cerca del objeto) |
| 2026-07-08 | Core/Src/main.c | **Corrección del STOP a 4° + el giro de esquive espera 2s continuos de distancia estable en 3600**. (1) `OBJ_REV_HOLD_ANGLE` 2.5→**4.0°** (a pedido: 2.5 corregía muy poco; el clamp global de LINE_FOLLOWING es 5.0, entra sin excepción nueva). (2) La transición `OBJ_FRENO_REVERSA`→`OBJ_GIRO_ESQUIVE` ahora exige, además del wait quieto de 3s, que `min(A6,A8)` haya estado **dentro de la banda 3600..3900 durante `OBJ_REV_STABLE_MS=2000ms` CONTINUOS** (nueva `obj_rev_band_enter_ms`: arranca al entrar en banda, se resetea a 0 al salir — mientras el hold de distancia esté corrigiendo, el giro espera). Colchón `OBJ_REV_STOP_TIMEOUT_MS=10000` desde la entrada al STOP: si nunca se estabiliza (objeto sacado, sensor tapado), gira igual. `obj_rev_band_enter_ms` reseteada en la transición RETRO→STOP, en la salida STOP→GIRO y en el bloque de recuperación de caída (junto a `obj_brake_start_ms`). Compilado con GCC de CubeIDE sin errores ni warnings. | Usuario: 2.5° "me parece muy poco para corregir, 4 grados ponele mejor", y pidió que esté viendo por 2 segundos mínimo valores cercanos al setpoint de 3600 antes de rotar |
| 2026-07-08 | Core/Src/main.c | **Display pantalla 6: barra de A6 en lugar de A5 + valor de A8, para diagnosticar la reversa que "se aleja demasiado"**. La barra de arriba ahora muestra `adcAvg[5]` (ADC6, el que corta la reversa inicial) con ticks en 3200 (detección) y 3600 (`OBJ_REV_CLEAR_ADC`); en la fila de abajo, `A8:xxxx` (`adcAvg[7]`) reemplaza a GZ. Motivo diagnóstico: el corte de la reversa exige **A6 Y A8 ≥ 3600 a la vez** — si uno de los dos queda bajo (mal apuntado, viendo otra cosa), la reversa sigue hasta el tope de counts, que es exactamente el síntoma reportado; con ambos a la vista se ve al instante cuál no despeja. Además: `OBJ_REV_MAX_COUNTS` 600→**300** (~25 cm — acota el daño mientras se diagnostica; si la reversa corta siempre por el tope, el culpable es A6 o A8) y `OBJ_REV_HOLD_ANGLE` 2.0→**2.5°** (2.0 estaba al borde de la fricción estática — mínimo empírico conocido de LOST_FWD/APPROACH; si quedaba lejos, la corrección de avance del STOP no llegaba a moverlo y se quedaba lejos). Compilado con GCC de CubeIDE sin errores ni warnings. | Usuario pidió la barra de ADC6 en el display en lugar de ADC5, y reportó que el robot ahora se aleja demasiado en la reversa — sospecha que el corte por ADC no está funcionando |
| 2026-07-08 | Core/Src/main.c | **Reversa inicial (`OBJ_RETROCESO`) corta por ADC6/ADC8 ≥ 3600 + hold de distancia en el STOP** (sobre la base revertida a `c002dce`, que el usuario confirmó que "anda mucho mejor"). (1) `RETRO` ya no corta por 120 counts fijos de encoder (~10 cm, distancia final variable según dónde se detectó el objeto): ahora retrocede hasta que **ADC6 Y ADC8** (`adcAvg[5]`/`adcAvg[7]`, los frontales que miran adelante) lean ≥ `OBJ_REV_CLEAR_ADC=3600` durante `OBJ_REV_CLEAR_CNT=4` ciclos (40 ms) — la distancia final al objeto queda siempre igual. Los counts quedan solo como tope de seguridad (`OBJ_REV_MAX_COUNTS=600` ≈ 50 cm, por si los ADC nunca despejan). Nueva `obj_rev_clear_cnt` (reseteada en el init del estado). (2) **Hold de distancia durante `OBJ_FRENO_REVERSA` (STOP, ~3-4.5 s antes del giro)**: si la inercia lo vuelve a acercar (`min(ADC6,ADC8) < 3600`) corrige hacia atrás `OBJ_REV_HOLD_ANGLE=2°`; si se alejó de más (`> 3600+OBJ_REV_HOLD_BAND=3900`) corrige hacia adelante 2°; dentro de la banda 3600-3900 queda quieto con el freno de encoders de siempre. (3) `FRENO_REVERSA` agregado a la exclusión de `balance_hold_active` (las correcciones de ±2° con error chico eran silenciables por el hold — mismo patrón de bug conocido). Tuning: si la corrección del STOP no mueve el robot, subir `OBJ_REV_HOLD_ANGLE` de a 0.5; si vibra entre atrás/adelante, agrandar `OBJ_REV_HOLD_BAND`. Compilado con GCC de CubeIDE sin errores ni warnings. | Usuario (tras confirmar que la base revertida anda mucho mejor): la reversa post-detección no debe cortar por pulsos de encoder sino por ADC6/8 que miran adelante, "para que sea constante siempre y quede a la misma distancia", y "si se aleja de esos 3600 que corrija" |
| 2026-07-08 | Core/Src/main.c | **REVERSIÓN COMPLETA del wall-following al checkpoint `c002dce` ("checkpoint antes de estabilizar esquive")** — a pedido explícito del usuario: con toda la cadena de robustificación (filtro IIR + histéresis + debounce de ADC7, latch de reversa, árbitro de acciones, arco odométrico, autoridad subida, estabilización post-reversa) el esquive quedó PEOR que antes: giros de 180° constantes, no seguía la pared ni 2 segundos, y EST seguía sin verse. `git checkout c002dce -- Core/Src/main.c` (el trabajo descartado quedó preservado en el commit `9cc6cca` y en el historial `1f71728..0e92b8f` — nada se perdió). **Vuelve el esquema simple que andaba "recontra bien"**: umbrales directos sobre `adcAvg[6]` crudo cada ciclo (`REVERSE_THOLD=600`, `TOO_CLOSE_THOLD=2100`, `THRESHOLD=3750`), reversa de 3.0°, pivot potencia 8, `SPEED_TARGET=1.0`, sin latch (la reversa termina apenas ADC7 ≥ 600), sin árbitro, sin arco, transiciones inmediatas. **Solo se re-portaron 2 cosas sobre la base revertida**: (1) el indicador de acción en pantalla 6 ("PARED>REV", `ObjWallActionStr()` recalculada directo de los umbrales crudos — ya no existe `obj_wall_action`); (2) la exclusión de `balance_hold_active` para los 3 estados de pared (bug real diagnosticado: el hold silenciaba el PID justo cuando la reversa de 3° arranca desde parado). **NO existen más en el código activo**: `OBJ_WALL_REVERSE_CLEAR_ADC` (salida por 3600), `OBJ_WALL_REVERSE_TIMEOUT_MS`, `OBJ_WALL_ACT_*`/`ObjWallDecideAction`, `OBJ_WALL_ACTION_MIN_MS`/`STATE_MIN_MS`, `ObjWallComputeArcSteer`, la fase EST, el tope de pivot de 30°, ni los valores de autoridad subidos (6°/20/2.5°) — las filas de abajo que los describen son historia superseded. Compilado con GCC de CubeIDE sin errores ni warnings. | Usuario: "los giros siguen recontra inestables, hacen 180 todo el tiempo, no sigue la pared ni dos segundos, ¿cómo hacíamos antes? que seguía recontra bien, antes del cambio de la histéresis en los ADC" |
| 2026-07-08 | Core/Src/main.c | **Fix "EST nunca se muestra" + tope de rotación del pivot por cercanía (giros de 180° del árbitro)**. (1) La estabilización nunca llegaba a verse ni a actuar: el armado del latch de reversa usaba `obj_wall_reverse_f` (flag LATERAL, ADC7<750 con histéresis), que sigue en 1 cuando la reversa suelta — los que despejan son los frontales ADC6/8, el lateral no cambió — así que el latch se re-armaba al ciclo siguiente de soltarse y cancelaba la estabilización a los 10ms. Fix: armado normal bloqueado durante la estabilización (`!obj_wall_settle_active`); re-armado durante el hold SOLO si los frontales vuelven a acercarse de verdad (`OBJ_WALL_SETTLE_REARM_ADC=3400`, margen bajo el CLEAR de 3600); y la estabilización solo se entra en la salida NORMAL por frontales despejados (`released_by_clear`), no tras timeout (ahí los frontales siguen cerca y el REARM la cancelaría al instante). (2) **El pivot por cercanía (PIV, potencia 20 + compromiso de 450ms encadenado) no tenía tope de rotación**: con `too_close` sostenido rotaba 180° y perdía la pared. Nuevo tope `OBJ_WALL_PIVOT_MAX_DEG=30°` medido por rumbo de odometría (`\|Δθ\|` desde `obj_wall_pivot_theta0` capturado al entrar al pivot — magnitud, no depende del signo sin validar de `ODOM_THETA_SIGN`): al superarlo, `obj_wall_pivot_capped` fuerza AVANZA (con salida inmediata, exenta del compromiso de 450ms) y el pivot queda bloqueado hasta que `too_close` se suelte al menos una vez. Ojo: si los giros de 180° que ve el usuario fueran en GIRAP (`GIRO_PARED`, que pivotea hasta re-ver la pared y no tiene tope), eso es otro bloque — distinguible por el display (`GIRAP` vs `PARED>PIV`). Compilado con GCC de CubeIDE sin errores ni warnings. | Usuario reportó que EST no aparece nunca en el display (no sabía si la fase corría siquiera — no corría: moría a los 10ms) y que el árbitro hace giros de 180° que pierden la pared |
| 2026-07-08 | Core/Src/main.c | **Fase de estabilización de 3s a la salida de la reversa por pared (`OBJ_WALL_ACT_ESTABILIZA`)** — fix "la salida de reversa es inconsistente: a veces se detiene en 3600 pero después se va un poco adelante (choca) o atrás (pierde la pared)". Al soltarse el latch (ADC6/ADC8 ≥ 3600 o timeout), en vez de volver a avanzar de una, se captura la posición de encoders del punto donde despejó y se sostiene durante `OBJ_WALL_REVERSE_SETTLE_MS=3000ms` con un **hold de posición P puro por encoders**: nueva `ObjWallSettleSetpoint()` (error en metros vía `OBJ_WALL_SETTLE_M_PER_CNT=ENC_VEL_SCALE/ENC_CPR`, deltas positivos=avance como en la odometría; `OBJ_WALL_SETTLE_POS_KP=40°/m`, tope `OBJ_WALL_SETTLE_ANGLE_MAX=2.5°`, banda muerta `OBJ_WALL_SETTLE_POS_DB_M=0.010m` ≈1.5 counts para no vibrar por cuantización). Nueva acción del árbitro `OBJ_WALL_ACT_ESTABILIZA=3` (display "EST"), prioridad REVERSA > ESTABILIZA > PIVOT > AVANZA, exenta del compromiso de 450ms (sigue el timing fijo de su fase, igual que REVERSA); si ADC7 vuelve a < 750 durante el hold, la reversa re-arma y cancela la estabilización. En los 3 estados de pared: rama de setpoint propia (hold de posición, integral de velocidad reseteada), en la máquina de estados cae en la misma rama que la reversa (misma corrección de rumbo recto `REV_STRAIGHT_*`, sin pivots ni transiciones por pared vista/perdida durante los 3s), y comparte la rampa rápida `sp_step_max=0.6°/ciclo` (para frenar la inercia de reversa al pasar de -6° al hold sin tardar medio segundo). En `PARED_LIBRE` además se invalida `obj_wall_clear_initialized` durante reversa/estabilización — `clear_counts` acumula \|deltas\|, la reversa lo inflaba y al salir disparaba el giro a GIRO_PARED de inmediato. Tuning: si oscila en el lugar bajar `SETTLE_POS_KP`; si corrige poco subirlo; los 3000ms son ajustables directo. Compilado con GCC de CubeIDE sin errores ni warnings. | Usuario reportó salida de reversa inconsistente y pidió: detenerse al despejar 3600 y corregir la posición durante los 3 segundos posteriores, para salir estable sin acercarse (choque) ni alejarse (pérdida de la pared) |

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
