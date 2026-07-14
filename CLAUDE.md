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
- Detección de objetos en modo línea: ADC 5-8 (largo alcance) con debounce de 100 ms → **entra DIRECTO a la fase STOP (`OBJ_FRENO_REVERSA`, 2026-07-14)**: el hold de distancia frena la inercia y lleva/sostiene al robot en la banda 3500..3900 de A6/A8 (con anti-stall tag 9), estabiliza 2s en banda y recién ahí gira. `OBJ_ESPERA_REVERSA` y `OBJ_RETROCESO` quedaron fuera de la cadena activa (código intacto, borrable cuando se valide en el robot)
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
| 2026-07-14 | Core/Src/main.c | **`OBJ_REV_HOLD_ANGLE_MAX` 1.6→2.6°.** El hold de distancia del STOP a veces "intenta corregir pero el ángulo no le da" (reportado): la corrección proporcional saturaba en 1.6° y con fricción alta quedaba empujando sin moverse hasta que el anti-stall (rampa lenta, ~1s) lo destrababa. Techo subido a 2.6°; `ANGLE_MIN` (0.9°) y el anti-stall de respaldo sin cambios. | Usuario: "hay veces que intenta corregir pero no logra porque el ángulo no le da" |
| 2026-07-14 | Core/Src/main.c | **El giro de esquive ya no puede abortar hacia la línea + hold de posición en la pausa post-giro.** (1) **Aborto eliminado**: el ajuste fino del giro (fase 2) abortaba a `LOST_FWD` si el lateral (ADC5/ADC7) nunca vio la pared — pensado para detecciones fantasma (2026-07-10). En el esquive a IZQUIERDA pasaba seguido (reportado: "si ve la línea durante el giro se sale del modo esquive", confirmado por display: la cadena era giro→VUELVE→SIGUE instantáneo → el robot volvía derecho al obstáculo; a derecha no pasaba porque ADC7 sí confirma la pared). Con la entrada por fase STOP el objeto ya está confirmado de sobra por A6/A8 (fantasma casi imposible): ahora el giro SIEMPRE sigue a `PAUSA_GIRO`; si la pared realmente no está, el timeout de pared perdida (5s→reposo) cubre ese caso sin regresar hacia el obstáculo. `obj_rot_p2_seen` eliminado (quedó sin lectores). Esto implementa el "que ignore la línea mientras gira": ningún estado del giro/pausa reacciona a la línea ahora. (2) **Hold de posición en `PAUSA_GIRO`** (pedido: "luego del giro que mantenga posición un poco mejor"): antes upright + freno por velocidad LENTA (solo frena, no devuelve la deriva). Nuevo P por encoders: ancla `obj_pausa_r0/l0` al entrar, corrección `OBJ_PAUSA_POS_KP=0.06°/count` (≈1° por 10 cm), tope ±1.5°, banda muerta 3 counts (~2 cm); primer ciclo sin corregir (el ancla se estampa en el case, que corre después del setpoint). NO compilado (el usuario compila). | Usuario: "si ve la línea durante el giro se sale del modo esquive (a la derecha no pasaba) — que la ignore mientras gira y los primeros segundos de la pared" + "luego del giro que hace, que mantenga posición un poco mejor" |
| 2026-07-14 | Core/Src/main.c, Balancin_Mendelevich.ioc | **ADC clavados en ~4095: causa raíz probable encontrada — tiempo de muestreo de 15 ciclos en los canales de línea.** Investigación pedida ("es por software, antes no pasaba"). Hallazgos: (1) **Los canales de línea (ranks 1-4) muestreaban con `ADC_SAMPLETIME_15CYCLES` (625 ns @ ADC 24 MHz)** mientras los de objeto usaban 144. El capacitor de sample&hold retiene la tensión del canal ANTERIOR del scan — y el rank 1 muestrea justo después del rank 8 (sensor de objeto, reposo ~4095 ≈ 3.3V). Con 625 ns y la impedancia alta del fototransistor del sensor de línea, el capacitor no llega a descargarse al valor real → el canal lee cerca del máximo aunque el sensor esté sano. **Esto explica TODO el cuadro**: por qué se clava en el tope y no en un valor cualquiera, por qué el dedo lo "arregla" (reflexión fuerte → fototransistor saturado → impedancia baja → el capacitor sí se descarga: no era soldadura fría), por qué es indefinido (dura mientras la iluminación del sensor sea marginal; `ADC1_Recover` no puede arreglarlo porque no es un problema de DMA) y por qué "antes no pasaba" (los canales de objeto —reposo 4095— se agregaron al scan después; el usuario intuyó bien con "el duty cycle de cada ADC": es exactamente el sampling time por canal). **Fix: los 8 canales a 144 ciclos** (barrido completo 8×156/24MHz = 52 µs, sobra margen en los 250 µs del trigger de TIM2). (2) El `.ioc` estaba **desincronizado** del código generado (decía 15/112 mezclados donde main.c ya tenía 144 en ranks 5-8) — una regeneración de CubeMX habría degradado también los canales de objeto en silencio; ambos archivos quedaron consistentes en 144. (3) `ADC1_Recover` hacía `__HAL_DMA_ENABLE` manual antes de `HAL_ADC_Start_DMA`: si el stream no llegaba a apagarse, la reconfiguración del HAL se ignora y la DMA sigue a mitad de buffer → TODOS los canales rotados un lugar (un canal de línea lee el 4095 de uno de objeto) — eliminado (peligro latente que el recover periódico de 3s re-sorteaba cada vez). NO compilado (el usuario compila). La cuarentena por software del 2026-07-10 queda como red de seguridad. | Usuario: "sigo teniendo problemas con los ADC que se disparan y quedan lanzados al máximo por tiempo indefinido; necesito saber por qué, es por software, antes no pasaba; puede intervenir el duty cycle de cada ADC — busquemos el problema" |
| 2026-07-14 | Core/Src/main.c | **Wall-following más cauteloso: sin acelerones bordeando la pared.** En pared>avanza (BORDEAR/PARED_LIBRE) el PI de crucero tenía `OBJ_WALL_SPEED_TARGET=1.0 m/s` y tope `OBJ_WALL_FWD_ANGLE=3.5°` — el doble de rápido que el avance VUELVE ya validado (LOST_FWD: 0.40/2.5°); además, desde parado el deadband de velocidad (0.35) hace leer el error completo → arrancaba con el tilt clavado en el tope. Resultado reportado: acelera demasiado y al llegar a la línea siempre se pasa. Bajado a: **target 0.70 m/s** (primero 0.45, "demasiado lento" probado en el robot — 0.70 es el punto medio pedido) y **tope 3.0°** (primero 2.5, el usuario pidió 3 en la misma sesión). Ganancias de freno intactas (KP_BRAKE=22, BRAKE_ANGLE=4°): la sobrevelocidad se sigue castigando fuerte. `BUSCAR_PARED` ya era cauteloso (0.3/1.5°) y `GIRO_PARED` no avanza por sí (upright+pivot) — su "acelerón" era el rearranque del PI al volver a BORDEAR, ahora limitado. NO compilado (el usuario compila). Si ahora bordea muy lento, subir `OBJ_WALL_SPEED_TARGET` de a 0.1. | Usuario: "en pared>giro o pared>avanza se suele acelerar demasiado, al llegar a la línea siempre se pasa — sé más crítico con las aceleraciones fuertes" |
| 2026-07-14 | Core/Src/main.c | **El giro del cruce perpendicular (`PERP_ROTATE`) sigue el sentido del esquive.** Al reencontrar la línea después de bordear el obstáculo (cruce con los 4 ADC en negro), el giro de 90° era SIEMPRE a la derecha (`PROT_DIR=1.0f` fija, convención histórica). Tras un esquive por derecha eso está bien, pero tras uno por IZQUIERDA girar a la derecha devolvía el robot directo al obstáculo. Ahora `PROT_DIR = (float)obj_esquive_dir`: gira hacia el mismo lado del último esquive (derecha→derecha, izquierda→izquierda). Sin esquive previo `obj_esquive_dir` vale +1 (se rearma al entrar al modo línea), así que el cruce perpendicular "normal" conserva la derecha histórica. No hizo falta tocar nada más del bloque: heading y counts ya se comparan en valor absoluto (mismo patrón del espejado del giro de esquive del 2026-07-13). NO compilado (el usuario compila). | Usuario: "si el esquive fue hacia la derecha, cuando vuelvo a ver la línea el giro lo debo hacer hacia la derecha sí o sí (si girara a la izquierda volvería al obstáculo instantáneamente); si el esquive es hacia la izquierda, girar hacia la izquierda también" |
| 2026-07-14 | Core/Src/main.c | **Wall-following: línea ignorada 3s→5s + rumbo recto en la reversa del STOP.** (1) **`OBJ_WALL_LINE_IGNORE_MS` 3000→5000**: en los primeros segundos de pared>avanza (BORDEAR) o pared>gira (GIRO_PARED) el robot a veces veía la línea (la misma que venía siguiendo, todavía a la vista) y abortaba la esquiva a FOLLOWING sin rodear el obstáculo — la ventana compartida de los 3 estados de pared (medida desde la entrada a toda la secuencia, `obj_wall_seq_start_ms`) pasa a 5s. (2) **La reversa hasta el setpoint de ADC salía chueca**: al sacar `OBJ_RETROCESO` de la cadena (cambio de hoy, entrada directa a STOP) se perdió su corrección de rumbo — el hold del STOP retrocedía sin corregir nada. Portado el mismo mecanismo al case de `OBJ_FRENO_REVERSA`: ancla el rumbo de entrada (encoders `obj_rev_r0/l0` al iniciar la fase) y P sobre la diferencia ACUMULADA de counts (`REV_STRAIGHT_KC/MAX/SLEW`, gate por actividad de encoders `REV_STRAIGHT_ACT_MS` — quieto no pivotea), activo durante toda la fase y en ambos sentidos del hold. Reusa las estáticas `obj_rev_*` (libres: RETROCESO está fuera de la cadena). NO compilado (el usuario compila). | Usuario: "por ahí ve la línea en los primeros segundos de pared>avanza o pared>gira y no esquiva el obstáculo — que la esquive los primeros 5 segundos" + "la reversa hasta el setpoint de ADC la suele hacer chueca, no corrige demasiado para hacerla recta" |
| 2026-07-14 | Core/Src/main.c | **Detección de objeto: la fase ESPERA ("ESPER" en display) eliminada de la cadena — la detección entra DIRECTO a la fase STOP (`OBJ_FRENO_REVERSA`).** Motivo final: el balance libre de ESPERA no tiene ancla de posición — llegando con inercia el robot se iba un poco para atrás y **perdía la pared** (los A6/A8 quedaban fuera de rango y la secuencia seguía a distancia equivocada). El hold de distancia del STOP hace todo desde el primer ciclo: `hold_damp` frena la inercia y la corrección proporcional lleva/sostiene la distancia en la banda 3500..3900 de A6/A8; después la ventana de estabilidad (2s en banda + 3s mínimos) y el giro, como siempre. Se agregó **anti-stall (tag 9)** al hold fuera de la banda fina, porque ahora el hold cubre TODA la reversa y su corrección máxima es de solo 1.6° (`OBJ_REV_HOLD_ANGLE_MAX`) — si la reversa queda lenta/corta en el robot, subir ese define. `OBJ_ESPERA_REVERSA` y `OBJ_RETROCESO` quedan fuera de la cadena activa (código intacto, marcados en el enum; borrables cuando el esquema se valide). **Historia del día (3 iteraciones)**: (1) primero se hizo este mismo salto directo a STOP; (2) el usuario reportó "rompiste el giro" y pidió volver a ESPERA(1s)+RETROCESO+STOP (`OBJ_PRE_REVERSE_HOLD_MS` 2000→1000, que queda así aunque el estado esté fuera de la cadena); (3) al ver que aun con 1s el ESPER derivaba hacia atrás y perdía la pared, pidió eliminar ESPERA y entrar directo a STOP — este estado final. Si el giro vuelve a salir mal, pedir detalle de QUÉ hace (¿gira demasiado cerca? ¿no gira?) antes de tocar — la sospecha de la iteración 1 es que la reversa por hold termina a otra distancia que la del RETROCESO. NO compilado (el usuario compila desde esta sesión). | Usuario: "el período que dice en el display ESPERA hace que se vaya un poco para atrás y pierda la pared; necesito que ESPERA no exista, pase directamente a STOP" |
| 2026-07-14 | Core/Src/main.c | **Pantalla ODOMETRIA del OLED: rumbo (y mapa) espejados — corregido.** La flecha de rumbo giraba al revés de la realidad (girando a la derecha apuntaba a la izquierda). El dato `odom_theta_deg` está BIEN (convención estándar: x adelante, y a la IZQUIERDA, θ positivo antihorario) — el bug era solo del dibujo: el mapa mapeaba "+Y odométrico hacia la derecha de la pantalla", espejando todo el plano (yendo derecho no se notaba, de ahí que "la dirección en la que se mueve está bien"). Fix: negado el término Y/senθ en las 3 coordenadas de pantalla (punto de pérdida de línea `lx`, pose `px`, punta de flecha `hx`) — ahora +Y dibuja a la izquierda y la flecha gira para el mismo lado que el robot. Ninguna otra pantalla ni la odometría en sí cambian; el mapa de Qt no se toca (recibe el dato crudo, que era correcto). De paso este síntoma es evidencia de que `ODOM_THETA_SIGN=+1` está bien calibrado (girando a la derecha θ se hace negativo, como pide el comentario de la línea ~797). Compilado con GCC 13.3 de CubeIDE: cero errores, cero warnings. | Usuario: "la odometría del display apunta en la dirección incorrecta: si gira a la derecha apunta hacia el otro lado; solo es la dirección en la que apunta, la dirección en la que se mueve está bien" |
| 2026-07-13 | Core/Src/main.c | **Esquive alternado derecha/izquierda con lateral ADC7/ADC5.** Nuevas estáticas `obj_esquive_dir` (sentido del esquive ACTUAL: +1 derecha, -1 izquierda) y `obj_esquive_next_dir`: en cada detección de objeto la esquiva usa el sentido programado y deja armado el opuesto para la próxima (la primera va a la derecha; al entrar al modo línea se rearma en derecha). **Mismos mecanismos, umbrales y constantes en ambos sentidos** (pedido explícito): solo cambian (1) el sentido del giro de 90° de `OBJ_GIRO_ESQUIVE` (local `rot_dir` multiplica pivot y freno en fases 0/1/2, el heading ya se comparaba en valor absoluto) y (2) el **sensor lateral**: `OBJ_WALL_ADC_IDX` pasó de la constante 6 a `((obj_esquive_dir > 0) ? 6 : 4)` — ADC7 esquivando a derecha (pared a la izquierda, histórico), ADC5 a izquierda (pared a la derecha) — con lo que TODO lo que ya colgaba de ese define (BUSCAR/BORDEAR/LIBRE/GIRO_PARED, latch de reversa, detección de atorado, timeout de pared perdida, ajuste fino del giro) quedó espejado sin tocarlo. Los 4 pivots de pared (3× too_close "alejarse" + búsqueda de GIRO_PARED "acercarse") multiplican `OBJ_WALL_PIVOT_POWER` por el sentido. Display: pantallas 1 y 6 muestran `A7:`/`A5:` según el sentido activo (label y valor dinámicos). La detección de objetos (ADC5-8) no cambia. **Nota Qt**: la pared 3D de la Vista 3D usa `min(adc5,adc6,adc8)` — durante un bordeo IZQUIERDO ADC5 ve la pared de al lado y Qt va a dibujar una barrera fantasma de frente; si molesta, excluir el lateral activo (el firmware ya manda `robot_state`/`line_state` en `WifiOdomData_t`). Compilado con GCC 13.3 de CubeIDE: cero errores, cero warnings. Pendiente validar en el robot (encoders/gyro en giro a izquierda). | Usuario: "necesito que hagamos el esquive en el sentido izquierdo: ahora giramos a la derecha; quiero usar el ADC5, que haga un esquive para la derecha como ahora y el próximo a la izquierda" + "los mismos mecanismos, solamente la rotación 90° hacia el otro lado, y el ADC5 para pared>avanzar y pared>girar como hago con el ADC7" |
| 2026-07-13 | Core/Src/main.c | **Reversa por obstáculo: fin del tambaleo (PI sin equilibrio posible → feedforward) + rumbo recto (gate por actividad de encoders) + "F!" centrado en el display.** (1) **Tambaleo sin poder retroceder**: el PI de velocidad de `OBJ_RETROCESO` no tenía punto de equilibrio — el target (≤0.30 m/s) quedaba DEBAJO del piso de medición (`apply_deadbandf(vel, 0.35)` lee 0 hasta pasar 0.35 m/s reales): medido 0 → empuje máximo + windup; al cruzar el deadband el error saltaba negativo de golpe (KP=28 → 3° por escalón del LPF) → freno activo → quieto → empuje máximo otra vez. Bang-bang permanente = el tambaleo reportado. Reemplazado por **empuje feedforward** `OBJ_REV_PUSH_ANGLE=3.5°` escalado por la aproximación (la fricción la cubre el anti-stall, que sigue igual) + **freno proporcional suave** `OBJ_REV_BRAKE_KP=6°/(m/s)` solo con sobrevelocidad REAL (por encima del deadband), piso `-OBJ_REV_ACTIVE_BRAKE_MAX` como antes. Eliminados el integral (`line_obj_rev_vel_integral` y sus 11 resets), `LINE_OBJ_REV_SPEED_MIN/MAX`, `LINE_OBJ_REV_VEL_KP` y los defines muertos `LINE_OBJ_REV_STEER_GAIN/MAX` (sin uso desde el fix de zigzag). (2) **Va muy torcido**: la corrección de rumbo se apagaba con `velocity_est_f < 0.12` — a velocidad de reversa el LPF de la velocidad cuantizada suele leer menos que eso aunque retroceda, así que la corrección quedaba muda casi toda la reversa. Nuevo gate por **actividad de encoders**: corrige si hubo algún tick en los últimos `REV_STRAIGHT_ACT_MS=250ms` (estáticas `obj_rev_last_counts/obj_rev_last_move_ms`); quieto de verdad el target sigue decayendo a 0 por el slew (el anti-pivoteo del 2026-07-10 se conserva). `REV_STRAIGHT_MIN_VEL` eliminado. (3) **Display**: el "F!" de caído dejó la pila de la derecha del header (se mezclaba con modo/WiFi) y ahora se dibuja centrado; el splash de conexión WiFi también quedó todo centrado (RED/SSID/IP/PC — antes alineado a la izquierda). Helper nuevo `OLED_Str5Centered(y, s)` usado en ambos. Compilado con GCC 13.3 de CubeIDE: cero errores, cero warnings. Pendiente validar en el robot (tuneo: si retrocede muy fuerte bajar `OBJ_REV_PUSH_ANGLE`; si frena de más bajar `OBJ_REV_BRAKE_KP`). | Usuario: "le cuesta bastante ir hacia atrás con el setpoint cuando ve el objeto, se pone a tambalearse sin poder ir hacia atrás; además, si va hacia atrás va muy torcido" + "el FALLEN del display se mezcla con lo otro, que se marque más en el centro" + "que los datos del display cuando aparece la pantalla de wifi aparezcan centrados" |

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
