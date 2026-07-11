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
| 2026-07-10 | Core/Inc/UNER.h, Core/Src/main.c, `C:\Microcontroladores\BalancinQT` (mainwindow.h/.cpp, robotviewer3d.h/.cpp) | **Tercer eje en la Vista 3D (banking lateral) + verificación numérica del apoyo en el piso.** (1) **Firmware**: `WifiOdomData_t` extendido con `lat_deg` (float, al FINAL — mirror Qt actualizado en la misma sesión): inclinación lateral alrededor del eje de avance, accel-only (misma fórmula que el pitch de `calculate_tilt`: atan2(-ax, √(ay²+az²))) con EMA α=0.10 a 100 Hz (`lat_tilt_f`, no participa de ningún control). (2) **Qt**: `setPose(pitch, yaw, lat)` — el modelo ahora se mueve en los TRES ejes (banking = rotación alrededor del X local/eje de avance, orden yaw∘pitch∘lat), interpolado como los otros dos y con `Lat:` en el HUD; `setPitch` conserva rumbo y banking. **Si el banking sale invertido, negar `lat_deg` en el handler de 0xDC** (incógnita de signo hermana de ODOM_THETA_SIGN). (3) **Apoyo en el piso**: el usuario reportó que con ángulo negativo seguía hundiéndose — se verificó la fórmula con un test numérico standalone (mismas matrices T·R·S de Qt3D): el punto más bajo queda clavado en +0.6 para ±45/±90/±120°, con y sin rumbo — la matemática es exacta y simétrica; el hundimiento visto era un **binario viejo corriendo** (la app quedó abierta desde antes del rebuild de las 16:42). Ambos exe recompilados (debug y release, 22:06). | Usuario: "con ángulo negativo se pasa por debajo del suelo siempre" + "me gustaría que también reaccione a movimientos laterales, es el único eje que falta" |
| 2026-07-10 | Core/Src/main.c | **Cuarentena de sensores de línea enganchados (defensa por software; la causa es hardware).** El usuario reporta (recurrente) que un ADC de línea queda clavado en el tope (~4095) y solo baja presionándolo con el dedo — síntoma clásico de **soldadura fría/falso contacto** (la presión restablece el contacto) o emisor IR caído. NO es el bug de software de 2026-07-05 (el promedio móvil se recalcula entero por ciclo desde entonces): si la barra queda clavada, el canal crudo está alto de verdad. Defensa nueva: si un canal de línea (1-4) pasa `LINE_STUCK_MS=8s` continuos por encima de `LINE_STUCK_ADC=3950`, entra en **cuarentena**: contribuye 0 al centroide y a `line_detected`, invalida `all_black` (conservador: sin giros PERP fantasma por un sensor roto), no marca `det` en la franja del display, y su dígito en pantalla 1 se reemplaza por una **X** (diagnóstico visual de qué sensor revisar). Sale de cuarentena solo, apenas vuelve a leer blanco (< LINE_THRESHOLD) — el arreglo con el dedo lo rehabilita al instante. Sin cuarentena, el canal clavado fingía línea permanente de ese lado: arrastraba el centroide, impedía detectar la pérdida de línea y ensuciaba all_black. **Pendiente de hardware**: revisar soldadura/conector del sensor que marque X (el de la izquierda del display = dígito 4 = ADC4/PA4). Compilado con GCC de CubeIDE sin errores ni warnings. | Usuario: "sigo teniendo problemas con el ADC del seguidor de línea que se queda enganchado arriba del todo, el de la izquierda del todo; tengo que ponerle el dedo para que baje" |
| 2026-07-10 | Core/Src/main.c | **Anti-stall generalizado a TODOS los estados que requieren movimiento + destrabe por atascamiento en modo pared.** (1) El anti-stall ad-hoc de LOST_FWD se refactorizó a **`AntiStall_Tick(tag, wants_motion, max_boost)`** (una sola instancia global — los estados son mutuamente excluyentes; `tag` identifica la maniobra y resetea el boost al cambiar de llamador o tras >200ms sin llamadas). Defines `ANTISTALL_*` (VEL_THR 0.05, WAIT 400ms, STEP 0.04°/ciclo, DECAY 0.90, topes FWD 3.0/REV 2.5/MAN 2.0). **Enganchado en**: LOST_FWD (tag 1), EDGE_FWD (2), OBJ_BUSCAR_PARED (3), BORDEAR_PARED avance (4), reversa de pared en los 3 estados (5, compartido — misma maniobra), PARED_LIBRE avance (6), OBJ_RETROCESO (7, empuja hacia atrás solo si el PI pide reversa), MANUAL adelante/atrás (8, en el sentido comandado). FOLLOWING ya tenía su `line_forward_boost` — sin cambios. Ni la inclinación del piso ni la fricción pueden dejar quieto a ningún estado de movimiento. (2) **Destrabe por atascamiento** (pedido: "pared>avanza o pared>pivot atorado contra la pared"): en los 3 estados de pared, ventana de `OBJ_WALL_STUCK_WIN_MS=2500ms` (5s inicial, bajado el mismo día a pedido) — si el movimiento acumulado de encoders no llega a `OBJ_WALL_STUCK_COUNTS=20` (~13 cm), está trabado (empuja sin avanzar o pivot atorado, el anti-stall solo lo apretaría más): dispara el mismo latch de reversa pero con objetivo largo `OBJ_WALL_REV_ESCAPE_COUNTS=60` (~38 cm, timeout doble) para despegarlo de verdad antes de reintentar. La ventana no corre durante la reversa y se resetea al salir de los estados de pared. Compilado con GCC de CubeIDE sin errores ni warnings. | Usuario: "ese anti-stall ponelo en todos los lugares posibles donde se requiera movimiento, que la inclinación del suelo o cualquier objeto no detenga el robot" + "si en pared>avanza o pared>pivot en ~5s no cambia el pulso de encoder, se quedó atorado: reversa bastante para destrabar" |
| 2026-07-10 | Core/Src/main.c | **Aborto de la esquiva si la pared nunca aparece + más sobrepaso en el retorno por odometría.** (1) **Aborto a búsqueda por odometría**: nuevo flag `obj_rot_p2_seen` en el ajuste fino por ADC7 del giro de esquive — si al agotarse el ajuste (tope de counts o timeout) ADC7 NUNCA bajó de 3200 ni un ciclo, la detección fue fantasma o el objeto quedó fuera de alcance: bordear no tiene sentido. En vez de seguir a PAUSA_GIRO→BORDEAR (bordeando una pared inexistente), aborta la esquiva y pasa a `LINE_STATE_LOST_FWD` (el mismo retorno por odometría al punto de pérdida de línea del post-180°), con el reset completo de entrada (line_lost_ms, integrales, steering, vars de secuencia de pared) y 5s de `obj_detect_ignore_until_ms` como el resto de las salidas de pared. Si la vio aunque sea un ciclo (vislumbrada), sigue la secuencia normal. (2) **`LOST_RETURN_OVERSHOOT_M` 0.20→0.35**: con 0.20 (neto ~+0.10 tras el corte de `REACHED_M=0.10`) el robot se plantaba justo sobre el punto donde perdió la línea y no la encontraba ("se queda muy cerca") — entre el error de odometría y que los sensores están adelante, hace falta pasarse más para cruzar la cinta. (3) **Anti-stall del VUELVE (`LOST_FWD`)**: el PI de velocidad satura en P(1.6°)+I(1.0°) → techo 2.5° — si la fricción del momento pide más, quedaba clavado y el robot quieto para siempre. Nuevo boost en rampa (`LOST_FWD_STALL_*`): pidiendo avanzar y sin movimiento de encoders por 400ms, suma 0.04°/ciclo POR ENCIMA del clamp (tope +3°, clamp global 5.0 manda), y decae ×0.90/ciclo apenas se mueve — nunca puede quedarse quieto. Reset junto a cada reset del integral. Compilado con GCC de CubeIDE sin errores ni warnings. | Usuario: "si la pared no aparece en ningún momento que aborte y vuelva a buscar línea por odometría" + "que se pase un poco del punto para volver a encontrar la línea, va justo al punto donde la perdió pero se queda muy cerca" + "en modo VUELVE por odometría se queda quieto porque no le da el ángulo de avance — que nunca se quede quieto y siempre vuelva" |
| 2026-07-10 | Core/Src/main.c | **Giro de esquive verificado por ADC7: fase de ajuste fino si el giro quedó corto.** El giro de 90° de `OBJ_GIRO_ESQUIVE` cortaba solo por encoders/gyro y podía quedar corto (la pared sin entrar en la vista del lateral ADC7). Ahora, al completar la fase de freno: si `ADC7 < OBJ_ROT_ADC_GOOD=3200` → buena detención, sigue a PAUSA_GIRO como siempre; si NO bajó de 3200 → **fase 2 de ajuste fino**: pivot suave a la derecha (mitad del pivot normal) hasta ver `ADC7 < 3200` durante `OBJ_ROT_ADC_GOOD_CYC=3` ciclos seguidos (30ms, anti-glitch), con tope de `OBJ_ROT_ADC_EXTRA_MAX=150` counts extra (~36°) y timeout `OBJ_ROT_ADC_EXTRA_MS=1500ms` por si la pared quedó fuera de alcance — en ese caso sale igual a PAUSA_GIRO. La salida por **overshoot NO entra al ajuste fino** (girar más solo empeoraría). Freno de continuidad en ambas salidas, igual que la fase 1. Estáticas nuevas: `obj_rot_p2_c0/obj_rot_p2_ms/obj_rot_adc_good_cnt`; `obj_rot_phase` ahora puede valer 2 (solo en GIRO_ESQUIVE — LOST/EDGE/PERP_ROTATE no cambian). Nota: las iteraciones 1-4 de la reversa/STOP de hoy quedaron **validadas por el usuario en el robot ("funciona perfecto")**. Compilado con GCC de CubeIDE sin errores ni warnings. | Usuario: "el giro además de los encoders debería usar el ADC7 para saber cuándo detenerse o si se detuvo bien: debajo de 3200 es buena detención, si no pasó por debajo le falta un poco más" |
| 2026-07-10 | Core/Src/main.c | **Reversa/STOP por obstáculo, iteración 4: empuje real hacia atrás + ventana más ancha + reversa de pared por distancia fija.** (1) **Deadband en la medida del PI de reversa** (`OBJ_RETROCESO`): usaba `velocity_est_f` cruda — el piso de cuantización (1 tick lee 0.11 m/s tras el LPF) contra el target achicado por la aproximación (~0.07) daba sobrevelocidad FALSA y el freno activo nuevo inclinaba hacia adelante con el robot casi quieto ("le cuesta ir atrás y a veces avanza"). Ahora `apply_deadbandf(vel, BRAKE_VEL_DEADBAND)`: freno activo solo con velocidad real, y el empuje no se descuenta por ruido. `OBJ_REV_APPROACH_MIN` 0.20→**0.35** (0.20 dejaba la reversa sin fuerza al final). (2) **`OBJ_REV_HOLD_BAND` 300→400**: ventana de quieto/estabilidad pasa de 3500..3800 a **3500..3900** (pedido: "la ventana es muy ambiciosa"). (3) **Reversa de pared (`pared>retroceso`) por counts con latch**: era tilt fijo 3.0° mientras `ADC7 < 600` — 3° muchas veces no vencía la fricción (se quedaba quieto) y al despegarse unos mm ya salía de reversa y volvía a avanzar. Nuevo `obj_wall_rev_latch` (armado en el bloque de encoders, consumido por las ramas de setpoint Y de motores de BORDEAR/LIBRE/GIRO_PARED): al disparar retrocede **`OBJ_WALL_REV_COUNTS`=24 counts (~15 cm) fijos** antes de devolver el control (timeout `OBJ_WALL_REV_TIMEOUT_MS`=3s), y `OBJ_WALL_REVERSE_ANGLE` 3.0→**4.5°** para que arranque de verdad. Compilado con GCC de CubeIDE sin errores ni warnings. | Usuario: "le cuesta bastante ir hacia atrás, ahora avanza un poco", "la ventana 3500-3800 es muy ambiciosa", y "en pared>retroceso se queda quieto — debería ir varios pulsos hacia atrás para alejarse bien y luego esquivar" |
| 2026-07-10 | Core/Src/main.c | **Reversa/STOP por obstáculo, iteración 2: freno ACTIVO en reversa + término D (damping por velocidad) en el hold del STOP.** Con la aproximación desacelerada de la iteración anterior seguía pasándose: dos causas estructurales. (1) **El PI de velocidad de `OBJ_RETROCESO` era unilateral**: `reverse_angle_mag` clampeado a ≥0 — con sobrevelocidad solo podía "soltar el acelerador", nunca inclinar hacia ADELANTE para frenar de verdad; la inercia lo pasaba de largo aunque el target bajara. Ahora el clamp inferior es `-OBJ_REV_ACTIVE_BRAKE_MAX=2.5°`: sobrevelocidad → inclinación de frenado real. (2) **El hold del STOP era P puro sobre distancia**: el robot es un doble integrador (ángulo→aceleración→velocidad→posición) y un P de posición sin D oscila siempre — se pasaba hacia atrás, volvía hacia adelante y chocaba la pared. Nuevo término `hold_damp = OBJ_REV_HOLD_KV(4.0) × velocity_est_f` (deadband 0.15, tope ±2.5°), sumado en TODAS las ramas del hold: inclina en contra del movimiento y frena antes de pasarse en ambos sentidos. Usa la velocidad RÁPIDA de encoders — importante: la rama "en banda" usaba `ComputeBrakeSetpointTarget`, que desde el fix anti-bamboleo de hoy corre con la velocidad LENTA (tau ~0.5s) y acá llegaba tarde justo con la inercia máxima (regresión de hoy mismo, corregida acá). Compilado con GCC de CubeIDE sin errores ni warnings. Tuneo: si frena de más/queda corto en reversa bajar/subir `OBJ_REV_ACTIVE_BRAKE_MAX`; si el STOP queda nervioso bajar `OBJ_REV_HOLD_KV` de a 1. **Iteración 3 (mismo día)**: con el freno activo el robot pasa parte de la reversa casi quieto y el P de rumbo por counts (KC=-1.0) pivoteaba izquierda/derecha alrededor del rumbo inicial sin avanzar — `REV_STRAIGHT_KC` bajado a **-0.4** y la corrección de rumbo ahora corre **solo con |velocity_est_f| > REV_STRAIGHT_MIN_VEL=0.12 m/s** (parado, el target decae a 0 por el slew: no hay rumbo que mantener si no se traslada). | Usuario: "sigue muy inestable cuando va hacia atrás y no llega a frenar, siempre se pasa, teniendo que ir hacia adelante de vuelta y chocándose la pared, está muy brusco" |
| 2026-07-10 | Core/Src/main.c | **Reversa por obstáculo sin zigzag y sin sobrepasar el setpoint de distancia** (3 cambios complementarios; el 1º lo hizo el usuario a mano en esta sesión). (1) *Hold del STOP proporcional* (usuario): reemplazó el bang-bang de ±2.75° fijos por corrección proporcional al error de ADC con zona fina muerta (`OBJ_REV_HOLD_FINE_BAND=60`, ángulo 0.9–1.6° según `OBJ_REV_HOLD_ERR_FOR_MAX=220`) — mejoró bastante pero seguía oscilando. (2) *Fix zigzag de la reversa*: `OBJ_RETROCESO` iba con el PID de línea INVERTIDO mientras veía línea (lo normal, el objeto está sobre la línea; gain 0.70, ±16 PWM) — marcha atrás ese lazo es inestable y era el zigzag reportado; el P "recto" por encoders solo corría sin línea y encima usaba la velocidad instantánea (cuantizada en saltos de 3.57 rps = 1 count/ciclo, puro ruido). Ahora la reversa va SIEMPRE recta: P sobre la diferencia ACUMULADA de counts desde el inicio (`REV_STRAIGHT_KC=-1.0` PWM/count, mismo tope ±8 y slew 2; `rev_dr-rev_dl` ∝ desvío de rumbo, suave) — mantiene el rumbo inicial toda la reversa. `LINE_OBJ_REV_STEER_GAIN/MAX` quedan sin uso (rama eliminada). (3) *Aproximación desacelerada* (pedido del usuario: "ir bajando intensidad o ángulo a medida que llegamos al setpoint"): en los últimos `OBJ_REV_APPROACH_BAND=250` counts de ADC antes del corte (gobernado por el menor de A6/A8, el mismo que decide el corte), la velocidad objetivo se escala linealmente de 1.0 al piso `OBJ_REV_APPROACH_MIN=0.20`, y el tope de inclinación también (piso 0.4 para el arranque) — llega "en puntas de pie" y el hold del STOP arranca casi sin inercia que corregir. Compilado con GCC de CubeIDE sin errores ni warnings. Pendiente validar en el robot. | Usuario: la reversa zigzagueaba; sus cambios al hold mejoraron pero "sigue muy oscilante y le cuesta quedarse quieto en setpoint — deberíamos ir bajando intensidad o ángulo al llegar" |
| 2026-07-10 | Core/Src/main.c | **Splash de conexión WiFi en el display: al obtener IP se muestra por 4s SSID + IP propia + IP destino, pisando cualquier pantalla.** Nuevo `wifi_splash_until_ms` (+`WIFI_SPLASH_MS=4000`): se arma en `appOnESP01ChangeState` al recibir `ESP01_WIFI_NEW_IP` (mismo momento en que arranca el socket UDP — y también en cada reconexión), y al inicio de `updateDisplay()` un bloque previo al switch de pantallas dibuja: header "WIFI", "CONECTADO" centrado (7x10), `RED:` + SSID (truncado a 21 chars, copia manual anti -Wformat-truncation), `IP:` + `ESP01_GetLocalIP()` ("..." si aún NULL) y `PC:` + `wifiIp` del perfil activo. No toca `f_change_display`: al vencer el timer la pantalla que estaba vuelve sola. Compilado con GCC de CubeIDE sin errores ni warnings. | Usuario: 'cuando se conecte correctamente al WiFi, que el display muestre unos segundos el SSID y la IP, sin importar en qué pantalla esté — que se superponga y después vuelva' |
| 2026-07-10 | `C:\Microcontroladores\BalancinQT
obotviewer3d.h`/`.cpp`, `mainwindow.cpp` (solo Qt) | **Obstáculo 3D frente al modelo (por ADC 5/6/8 del push de odometría) + UDP abierto por defecto al iniciar Qt.** Pared roja semitransparente frente al modelo 3D cuando `min(adc5,adc6,adc8) < 3200` (los ADC ya viajaban en `WifiOdomData_t` desde 2026-07-08 — no hizo falta tocar el firmware), a distancia del mapeo crudo ADC→m, en la dirección del rumbo, visible SOLO con roll en [-90°, +30°], más opaca cuanto más cerca. Además, al iniciar la app el UDP se abre solo (mismo camino que el botón 'Abrir UDP', puerto default 30010). Detalle completo en el CLAUDE.md de Qt. Compilado y linkeado completo (Qt 6.5.2/MinGW) sin errores. Firmware sin cambios. | Usuario: '¿los valores del ADC los estoy pasando? para graficar en Qt algún objeto enfrente cuando haya algo cerca, siempre que esté entre -90° y +30°' + 'siempre que inicio el Qt quiero que se abra el UDP por defecto' |
obotviewer3d.h`/`robotviewer3d.cpp` (solo Qt) | **Vista 3D: el modelo ya no atraviesa el piso — se apoya siempre sobre Y=0 en cualquier pose.** Antes la base tenía altura fija (`setTranslation(0,25,0)`) y el modelo rotaba alrededor de su centro: con roll ±90° o volcado, media carrocería quedaba bajo el piso. Ahora `applyModelTransform()` recalcula la altura en cada pose: guarda el bounding box local del mesh (`m_modelMin`/`m_modelMax`, capturados junto con `m_modelCenter` en el callback del `QBoundingVolume`), rota las 8 esquinas del bbox por la pose completa (yaw∘pitch del robot + rotación estática -90°X de la base, con la escala 12 aplicada), toma la Y mínima y traslada la base a `(0, -minY, 0)` — el punto más bajo queda exactamente apoyado en el piso. Cota conservadora (bbox ≥ mesh): en poses intermedias puede flotar apenas, nunca hundirse. La altura fija 25 queda solo como valor inicial hasta que carga el mesh. **Corrección mismo día (bis)**: la composición base·hijo derivada a mano con quaterniones dejaba pasar el piso con ángulos NEGATIVOS (usuario: 'a -90° se va sobrepasando el suelo mal'; con positivos no) — reescrita con las matrices reales de Qt3D (`m_baseTransform->matrix() * m_robotTransform->matrix()` + `M.map(corner)`, corrección delta sobre la traslación actual, exacta por linealidad) + margen de 0.6 unidades para no hacer z-fighting con la grilla. Debug Y release recompilados (antes solo debug — ojo con cuál exe se abre). Compilado con qmake/MinGW (Qt 6.5.2) sin errores ni warnings (`debug/robotviewer3d.o` + moc). Firmware sin cambios. Fila espejo agregada también en el CLAUDE.md de Qt. | Usuario: "cuando el ángulo da ±90° o más, el modelo 3D sobrepasa la línea del piso, físicamente incorrecto y feo — que siempre permanezca por encima, como que se apoye en el piso" |

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
