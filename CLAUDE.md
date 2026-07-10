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
| Push de odometría (2026-07-06, +ADC de objeto y roll 2026-07-08) | `WifiOdomData_t` (cmd `0xDC`, packed: seq, t_ms, x_m, y_m, theta_deg, line_error, line_detected, robot_state, line_state, adc5, adc6, adc7, adc8, roll_deg) enviado cada `WIFI_ODOM_PERIOD_MS=500ms` automáticamente en cuanto `f_wifi_connected=1` — **no depende de `ACTIVATE_WIFI_LOG`**, pensado para graficar mapa XY + posición de línea + barrera/obstáculo frente al robot + inclinación en la Vista 3D de Qt sin competir por ancho de banda/CPU con la telemetría de control a 10 Hz |

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
- **Última sesión:** 2026-07-10

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
| 2026-07-10 | Core/Src/main.c | **Fix bamboleo, parte 2 (la parte 1 sola mejoró muy poco): deadband de motor adaptativo + freno traslacional por velocidad lenta.** (1) **Deadband cinético/estático**: la compensación fija de +8 PWM está calibrada para vencer la fricción ESTÁTICA (rueda parada), pero durante el vaivén las ruedas ya giran (fricción cinética, menor) — cada cruce del comando por la zona neutra (±2) era una patada de 0→±11 PWM, un relé que sobre una planta inestable sostiene el ciclo límite por sí solo. Ahora: `MOTOR_DEADBAND_KINETIC=4` si la rueda tickeó hace <`WHEEL_MOVING_WINDOW_MS=80ms` (timestamps `wheel_r/l_last_tick_ms` grabados en el bloque de encoders), 8 completo solo con la rueda realmente parada. (2) **Freno anti-deriva por velocidad lenta**: `ComputeBrakeSetpointTarget` usaba `velocity_est_f` (LPF β=0.35, rápido) — meciéndose, la velocidad de ruedas supera el deadband de 0.35 alternando de signo y el freno inclinaba el setpoint ±0.4° con retardo → BOMBEABA la oscilación. Nueva `velocity_est_slow_f` (β=`VEL_SLOW_BETA=0.02`, tau ~0.5s): el vaivén a ~1Hz se atenúa ~3.3× y promedia bajo el deadband (freno quieto), la deriva real sostenida pasa entera con ~0.5-1s de retardo. Reseteada en los 5 lugares donde se resetea `velocity_est_f`. El resto del firmware sigue usando `velocity_est_f` sin cambios. Compilado con GCC de CubeIDE sin errores ni warnings. **Pendiente validar**; si el robot patina al arrancar desde parado, subir `MOTOR_DEADBAND_KINETIC` de a 1; si el freno post-empujón quedó lento, subir `VEL_SLOW_BETA` de a 0.01. | Usuario: tras la parte 1 (hold achicado) "sigue bamboleándose montón / disminuyó un poquito pero muy poco" |
| 2026-07-10 | Core/Src/main.c | **Fix bamboleo en equilibrio: el "punto dulce" (balance hold) era una zona muerta que garantizaba un ciclo límite.** Diagnóstico: con `|error|≤0.70°` y `|gyro|≤4°/s` el hold apagaba P, I y D por completo, y no salía hasta 0.90°/10°/s — pero un péndulo invertido siempre se cae en lazo abierto, así que el robot derivaba casi 1° entero con motores muertos, el PID lo atajaba ya con impulso, cruzaba la vertical, re-entraba al hold del otro lado y se volvía a caer → bamboleo permanente de ±1-2°. Agravantes: `SOFT_ZONE_MIN_SCALE=0.15` dejaba el PID al 15% cerca del equilibrio (comando < `MOTOR_CMD_NEUTRAL=2` → cero par hasta error grande) y el D apagado en hold era justo el término que frenaría la caída incipiente. Fixes: (1) hold achicado: enter 0.70→**0.25°** / 4→**2°/s**, exit 0.90→**0.45°** / 10→**6°/s**; (2) **el D queda vivo dentro del hold** (solo se silencian P/I): parado de verdad D≈0 igual, pero apenas arranca una caída el D la frena ANTES de cruzar el umbral de salida; (3) `SOFT_ZONE_MIN_SCALE` 0.15→**0.35**. El anti-chatter en parado lo sigue dando `MOTOR_CMD_NEUTRAL`. Afecta a todos los modos (mismo PID). Compilado con GCC de CubeIDE sin errores ni warnings. **Pendiente validar en el robot**; si tiembla en parado, bajar `SOFT_ZONE_MIN_SCALE` de a 0.05 o subir `MOTOR_CMD_NEUTRAL` de a 1. Sugerencia extra: probar subir KD desde Qt (0.12 es bajo vs KP=4; probar 0.20-0.30). | Usuario: "está bastante inestable el balance común, se bambolea de un lado al otro sin quedarse quieto, creo que es muy grande el punto dulce" |
| 2026-07-10 | Core/Src/main.c | **Display de balance (PANTALLA 0): reemplazado ER (error) y GZ por KP/KI/KD.** En la columna derecha de `updateDisplay()` (`f_change_display==0`) se sacaron `ER:` (`dynamic_setpoint_f - filtered_roll_deg`) y `GZ:` (gyro Z), y se agregaron `KP:`/`KI:`/`KD:` (`KP_value`/`KI_value`/`KD_value`, mismo formato `FormatSignedFixed(...,3)` + `nbuf+1` que ya usa la pantalla PARAM para las ganancias) en y=12/21/30; `VE:` (velocidad por encoders) bajó a y=39; `CMD:`/`M±xx±xx` (motores) se mantiene sin cambios en y=57. Permite ver las ganancias de balance en tiempo real (útiles al tunearlas desde Qt) sin tener que cambiar a la pantalla PARAM. `gz` sigue usándose igual que antes en el resto del firmware (yaw-assist, odometría) — solo se sacó de este display. Compilado con GCC de CubeIDE sin errores. | Usuario: "quiero que en el display de balance se muestre kp kd y ki, elimina el error y gz" |
| 2026-07-10 | Core/Src/ESP01.c | **Segunda IP hardcodeada eliminada de `ESP01.c` (unificación WiFi, parte 2).** `ESP01.c` tenía su propio `#define SERVER_IP "192.168.1.23"` (línea 18) independiente de `wifiProfiles[]`/`WIFI_PROFILE_ACTIVE` de `main.c` — usado como fallback en `ESP01CIPSTARTRESPONSE` cuando un intento TCP fallaba y se reintentaba por UDP (`else if (tryingTCP)`, línea ~1038). Si algún día se activa ese camino (hoy es código muerto: `tryingTCP` nunca se pone en 1 porque `ESP01_StartTCP` no se llama desde ningún lado), habría reconectado silenciosamente a esta IP vieja en vez de al perfil elegido. Fix: se borran `SERVER_IP`/`SERVER_PORT`/`LOCAL_PORT` y el fallback ahora reusa `esp01RemoteIP`/`esp01RemotePORT`/`esp01LocalPORT` (ya seteados por la llamada real hecha desde `main.c`, la que sí sale de `wifiProfiles[]`) — `ESP01_StartUDP(esp01RemoteIP, atoi(esp01RemotePORT), atoi(esp01LocalPORT))`. Ahora `WIFI_PROFILE_ACTIVE` en `main.c` es el ÚNICO lugar del firmware con una IP/red hardcodeada. Compilado con GCC de CubeIDE sin errores. | Usuario preguntó si `SERVER_IP` de `ESP01.c` también había que cambiarlo al cambiar de red; pidió unificarlo con lo de `main.c` |
| 2026-07-10 | Core/Src/main.c, CLAUDE.md | **Unificación de perfiles WiFi: un solo número para cambiar de red.** Antes había 3 bloques comentados con SSID/password/IP (uno por red) y había que descomentar el correcto y comentar el resto — fácil de dejar mal. Ahora `wifiProfiles[]` es una tabla (struct `WifiProfile_t {ssid, password, ip, label}`) con una fila por red, y `#define WIFI_PROFILE_ACTIVE <n>` (~línea 423) selecciona cuál está activa; `wifiSSID`/`wifiPassword`/`wifiIp` (usados por `ESP01_SetWIFI` y `ESP01_StartUDP`) se asignan desde esa tabla al arranque de `main()` (antes de `CDC_Attach_Rx`), sin cambiar su tipo ni su uso en el resto del código. Perfil activo hoy: `2` (Delco Mendelevich). De paso se corrigió la IP de Delco documentada en CLAUDE.md (`192.168.1.55`→`192.168.1.23`, desactualizada respecto al valor real en el código). Compilado con GCC de CubeIDE sin errores. Para agregar una red nueva: sumar una fila a `wifiProfiles[]` y otra a la tabla de este CLAUDE.md. | Usuario: "unificar la IP que usa el wifi, para que cada vez que cambio de red no tenga que cambiarlo de más de un lugar" |
| 2026-07-09 | Core/Src/main.c | **Precisión de odometría: gating de pivot + bias de gyro aprendido en línea + integración de 2º orden** — fix "cuando lo roto sobre el mismo eje parece que se desplaza". Diagnóstico: 1 count de encoder = ~6.3mm (ENC_CPR=28, muy grueso); girando en el lugar los ticks de cada rueda caen en ciclos de 10ms DISTINTOS con θ cambiando rápido — pasos de ±1 count integrados a rumbos diferentes NO se cancelan → random walk de la posición al rotar. Fixes: (1) **Gating de pivot**: si `|gz| > ODOM_PIVOT_GZ_DPS=40` y el avance neto del ciclo `|dR+dL| ≤ ODOM_PIVOT_MAX_COUNTS=1`, se descarta la traslación de ese ciclo (es ruido de cuantización/timing, no movimiento real). (2) **Bias residual de gz aprendido en línea** (`odom_gz_bias_f`, SOLO para odometría — no toca el PID ni el yaw-assist): con el robot quieto (0 ticks de ambos encoders + gz chico durante 500ms) EMA lenta (`ODOM_GZ_BIAS_ALPHA=0.005`, tau ~2s), clamp ±200 LSB — compensa la deriva térmica del bias hardcodeado, que giraba θ lentamente con el robot parado. (3) **Integración RK2**: la traslación usa el rumbo de MITAD de paso (θ+Δθ/2) — menos sesgo sistemático en arcos. Compilado con GCC de CubeIDE sin errores. Calibraciones pendientes sugeridas (ver conversación): test de 360° para la escala de gz (/100) y tramo recto medido vs odómetro de Qt para `ENC_VEL_SCALE`. | Usuario: al rotar sobre el mismo eje la posición parece desplazarse; pidió mejorar la precisión de la odometría |
| 2026-07-09 | `C:\Microcontroladores\BalancinQT\mainwindow.h`/`mainwindow.cpp`, `odomchartview.h`/`odomchartview.cpp` (solo Qt) | **Mapa de odometría renovado: estética oscura + herramientas CAD**. Estética: fondo oscuro a juego con la Vista 3D (chart #0a0e08, plot #0e140b), grilla sutil verde oscura (sin números — la escala la da la barra nueva), leyenda con labels claros, título reemplazado por tooltip de navegación, cruz del origen (0,0) fija (series `odomOriginH/V`, fuera de la leyenda). Funcionalidades: (1) **Barra de escala** abajo-izquierda (QGraphicsItems, largo "redondo" 5cm..20m que ocupe ≤150px, se recalcula en cada zoom/paneo/autoescala vía `updateOdomScaleBar` desde `repositionOdomAnnotations`). (2) **Botón "Seguir robot"** (toggle): la vista se recentra sola en la posición actual conservando el span del zoom del usuario (reemplaza a la autoescala grow-only mientras está activo). (3) **Herramienta "Medir" estilo CAD** (toggle): el click deja de panear (`OdomChartView::setMeasureMode`, cursor en cruz, señales `hoverAt`/`measureClick`), primer click fija P1, la línea amarilla punteada sigue al cursor con la distancia en vivo, segundo click cierra la medición (texto flotante en el punto medio); un tercer click arranca una nueva. Mapeo viewport→metros en `odomViewPosToValue` (`mapToScene`→`mapFromScene`→`mapToValue`). (4) **Coordenadas del cursor en metros** en vivo (label monoespaciado en la botonera). (5) **Odómetro y velocidad**: `updateOdomChart` ahora recibe `t_ms` del paquete; acumula distancia recorrida (piso de 3mm/muestra contra el ruido de cuantización) y estima velocidad entre paquetes — ambos en el label de info. (6) **Botón "Guardar PNG"** (`grab()` + `QFileDialog`). "Borrar mapa" resetea también odómetro/velocidad/medición. Firmware sin cambios. | Usuario pidió mejorar la odometría y el mapa lo mejor posible, dejarlo muy estético y con funcionalidades que sirvan mucho |
| 2026-07-09 | `C:\Microcontroladores\BalancinQT\robotviewer3d.h`/`robotviewer3d.cpp`, `mainwindow.cpp` (solo Qt) | **Vista 3D mejorada estilo CAD: transiciones animadas, rumbo real, pose suavizada, tinte por inclinación, HUD, ejes y órbita automática**. (1) `setView` ahora anima la cámara (450ms InOutCubic, `goToView`/`m_camAnim`) en vez de saltar — estilo Fusion 360. (2) **El modelo gira con el rumbo real del robot**: nuevo `setPose(pitch, yaw)` — el handler de `0xDC` en `mainwindow.cpp` pasa `theta_deg`; el yaw rota alrededor del Z local del mesh (= vertical del mundo tras la base de -90° en X), orden yaw∘pitch; `setPitch` (paths viejos: log 10Hz, GET_ANGLE) conserva el rumbo. **Si el modelo gira al revés que el robot, negar theta en el handler** (misma incógnita `ODOM_THETA_SIGN`). (3) **Pose interpolada** (`m_poseAnim`, 240ms OutQuad, yaw por camino corto en ±180°): el push de 2 Hz ya no se ve a saltos. (4) **Tinte por inclinación**: material plateado en equilibrio → rojo pleno a ≥20° (lerp de diffuse/ambient) — feedback inmediato de qué tan al límite está sin mirar números. (5) **HUD** en la barra: Roll y Rumbo numéricos (label monoespaciado, se actualiza con la pose mostrada). (6) **Tríada de ejes CAD** en el origen (X rojo, Y verde, Z azul, cuboides finos). (7) **Botón "Órbita auto"** (toggle): turntable de 0.35°/30ms alrededor del centro de vista. Botones con `Qt::NoFocus`; el `QOrbitCameraController` sigue intacto para navegar con el mouse desde cualquier vista. Firmware sin cambios. | Usuario pidió agregar funcionalidades vistosas y útiles al visor 3D, navegación como Fusion 360, interacciones, y mejorar todo lo posible |
| 2026-07-08 | `C:\Microcontroladores\BalancinQT\robotviewer3d.h`/`robotviewer3d.cpp` (solo Qt) | **Vistas predefinidas estilo CAD en la Vista 3D** (pedido: "como en Fusion 360"). Botonera arriba del visor: ISO (45° azimut / ~38° elevación), Frente, Atrás, Izq., Der. y Arriba (planta, con up-vector -Z para mirar derecho hacia abajo). Nuevo `enum class ViewPreset` + `setView(ViewPreset)` públicos: posicionan la cámara a distancia fija (450) del centro de interés (0,80,0) en la dirección de cada vista; el `QOrbitCameraController` sigue funcionando normalmente desde la vista elegida (los botones con `Qt::NoFocus` para no robarle el teclado). Firmware sin cambios. | Usuario pidió vistas seleccionables por defecto (45°, perpendicular) como en Fusion 360 o cualquier CAD para el visor 3D |
| 2026-07-08 | `C:\Microcontroladores\BalancinQT\robotviewer3d.h`/`robotviewer3d.cpp` (solo Qt) | **Fix pivote de la Vista 3D: el modelo ahora rota alrededor de su centro geométrico**. Causa: la traslación de centrado (`-center`) y la rotación de pitch vivían en el mismo `QTransform` de Qt3D, que aplica `T·R·S` — el mesh rotaba alrededor de su ORIGEN (`p' = R·p - c`) y el centro orbitaba ("se inclina mal"). Fix: `m_modelCenter` guardado como pivote y en `setPitch` la traslación de centrado se rota junto con el ángulo (`-R·c`, o sea `p' = R·(p - c)`) — rotación exacta sobre el centro. `m_pitchDeg` cacheado para re-aplicar la pose cuando el bounding volume del mesh termina de cargar. Firmware sin cambios. | Usuario: "el modelo 3D debería rotar en el centro de este, ahora se inclina mal" |

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
