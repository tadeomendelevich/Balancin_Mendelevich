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
├── ARQUITECTURA.md                  ← diagramas del software: contexto, modulos, UML, las 17 etapas, estados
├── CONFIGURACION_IOC.md            ← el .ioc explicado: reloj, timers, ADC, DMA, NVIC, pines
├── Balancin_Mendelevich.ioc         ← configuración CubeMX (pines, periféricos)
├── Core/
│   ├── Src/
│   │   ├── main.c                   ← loop principal, init, PID, state machine, complementary filter (desde 2026-07-16: ControlCiclo10ms —renombrada 2026-07-20, antes ControlStep10ms— es un orquestador de etapas Ctrl_* + handlers LineState_* por sub-estado)
│   │   ├── MPU6050.c                ← driver IMU MPU-6050 (DMA, fixed-point, calibración/bias)
│   │   ├── ESP01.c                  ← driver WiFi ESP-01 (AT commands, UDP, watchdog)
│   │   ├── UNER.c                   ← protocolo binario UNER (parser RX, encoder TX, 37+ comandos)
│   │   ├── i2c_manager.c            ← gestor I2C no bloqueante con cola (size 8) y DMA
│   │   ├── control_motores.c         ← salida PWM TIM3/TIM4, signos, canales y limites
│   │   ├── boton_usuario.c          ← gestos KEY sin decisiones de modo
│   │   ├── telemetria.c             ← CSV USB, log WiFi y odometria periodica
│   │   ├── comunicacion_usb.c       ← RX hacia UNER, cola TX USB CDC y depuracion
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
│       ├── control_motores.h         ← API de inicializacion y potencia firmada
│       ├── boton_usuario.h          ← eventos de pulsacion, largo y multiclick
│       ├── telemetria.h             ← muestra de solo lectura del control
│       ├── comunicacion_usb.h       ← API comun de transporte y debug USB
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

SoftAP seleccionable por KEY al arrancar o comando 0xDD en IDLE; Station por defecto. Qt puede enviar 0xDD por el enlace Wi-Fi actual o por USB alternativo. El cambio espera al menos 300 ms y a que finalice el envio UDP pendiente (maximo 5 s). Driver AT secuencial, descubrimiento de cliente UDP, plazos reales, parser IPD y cola de ACK. Compilado y testeado en host; pendiente prueba fisica. Ver SOFTAP.md.

El arranque normal es Station; KEY al encender selecciona SoftAP. Red Balancin, WPA2, IP 192.168.4.1. El selector de Qt elige busqueda; Aplicar por Wi-Fi o USB solicita el cambio al robot detenido. Windows cambia de red manualmente. La seleccion dura hasta reiniciar.

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
| Hold de equilibrio (anti-chatter) | `BALANCE_HOLD_*` | Entrada 0.25°/2°s, salida 0.45°/6°s. Anula P y mantiene D con escala suave. I conserva el valor calculado al inicio del ciclo; la integral se multiplica por 0.98 y luego recibe la acumulacion y los limites habituales. |
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
| ADC1 | 8 canales sensores (línea + analógicos) | PA1–PA7, PB0 | DMA circular (DMA2_Stream0), trigger TIM2 TRGO, **480 ciclos/canal** (ADC clock = PCLK2/4 = 24 MHz -> 20,5 us por canal, 164 us la secuencia de 8, sobre una ventana de 250 us). Resolucion 12 bits |
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
- **Última sesión:** 2026-11-24 (fixes de deteccion con robot caido, `PGIRO` trabado y `MOTOR_TEST`; **validado en el robot** — primera prueba fisica desde `V29`)

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
- **11 de los 20 sub-estados de `linea_estado` son inalcanzables (detectado 2026-11-20, sin decidir).** Nada asigna `LOST`, `SEARCHING`, `LOST_ASENTAR`, `EDGE_WAIT` ni `OBJ_BUSCAR_PARED`, y por cascada tampoco corren `LOST_BRAKE/ROTATE/AVANZA` ni `EDGE_ROTATE/ASENTAR/AVANZA`: **la recuperacion de linea perdida (giro 180 y 90 con retorno por odometria, ~700 lineas) no se ejecuta nunca**. Al perder la linea el robot se queda en `FOLLOWING`. Ya era asi en 2026-07-01 (`5206501`), no es regresion de los refactors. Opciones: (a) conectarla agregando la transicion `FOLLOWING -> LOST_BRAKE` en `Ctrl_LecturaYVelocidadLinea` cuando `linea_perdida_ms` pase un umbral — un solo `if`, pero son ~700 lineas que nunca corrieron en el robot; (b) borrarla (recuperable de git); (c) dejarla documentada. Detalle completo y diagramas en `ARQUITECTURA.md` seccion 5.3
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
| 2026-11-24 | Core/Src/main.c, CLAUDE.md, CHANGELOG.md | **El bordeado ya no puede girar en circulos sin fin buscando la pared.** Reporte: si no encuentra la pared "se queda como loco girando". Causa: la rama de busqueda de `LineState_ObjGiroPared` pivotea hacia la pared sin tope propio, y el unico limite (pared perdida 5 s en `Ctrl_LatchesPared`) se reinicia cada vez que el lateral ve la pared. Girando en circulos, en cada vuelta el lateral barre la caja, la ve un instante, el timer vuelve a 0 y ademas pasa a BORDEAR -> la pierde -> PARED_LIBRE -> GIRO_PARED: ciclo infinito. **Primer intento, FALLIDO y revertido**: tope de 150 grados por entrada a `GIRO_PARED` (integrando `|giro_z|`) y tope de 4 entradas por secuencia. **Rompio el esquive**: saltaba a PARADO "de la nada" en pleno bordeado. Dos errores: (a) se supuso que se entra a `GIRO_PARED` una vez por esquina, pero el bordeado es bang-bang — cada vez que el lateral cruza el umbral pasa BORDEAR -> PARED_LIBRE -> GIRO_PARED -> BORDEAR, muchas veces por lado; (b) integrar el giro SIN signo acumula tambien el bamboleo normal del balance. **NO reintentar topes por entrada ni por cantidad de entradas.** **Version final**: `GIRO_PARED` y `PARED_LIBRE` quedan identicos al commit anterior, y en `Ctrl_LatchesPared` (que ya corre en los 3 estados de pared y resetea al salir) se integra el **giro NETO con signo** de toda la secuencia, `obj_pared_giro_neto += giro_z/100 * dt`; si `|giro_neto| > OBJ_PARED_GIRO_NETO_MAX_GRADOS` (450) se rinde. El bamboleo va y viene y se cancela; girar en circulos lo hace crecer sin parar; rodear la caja entera son 360 netos, asi que un esquive normal no llega. Se resetea al salir de los estados de pared. Ademas: `ObjPared_Rendirse()` agrupa la limpieza que ya hacia el timeout de 5 s (comportamiento identico), y `GIVEN_UP` se agrego a la asistencia de rumbo por gyro de `Ctrl_MotoresLinea` para frenar la rotacion que trae si se rinde girando. **`GIVEN_UP` ya era el estado pedido**: balance sin avance, freno por encoders (`Freno_AnguloSegunModo`), sin buscar la linea, vuelve a FOLLOWING al verla, y el override manual funciona porque se activa en cualquier sub-estado de linea sin linea (display: "PARADO"). El timeout de 5 s se conserva para el caso sin giro (pivote que no alcanza a girar). **COMPILADO Y VALIDADO EN EL ROBOT: "anda hermoso"** (tag `V31`). Qt no requiere cambios. | Usuario: "por ahi se queda como loco girando si no encuentra la pared, necesito que no gire como loco nunca, si no encuentra la pared que ponga un limite maximo de velocidad por encoders y quede haciendo balance sin buscar linea hasta que la vuelva a ver o modo manual". En el mismo mensaje: "cuando se pierde de la linea y vuelve por odometria le cuesta mucho el balance" — **pendiente**: el retorno por odometria es inalcanzable (ver ARQUITECTURA.md 5.3); al perder la linea corre `FOLLOWING` sin linea ("SIGUE"), que frena desde el primer ciclo con `Freno_AnguloPorVelocidad` sobre la velocidad RAPIDA y casi siempre en el tope de 4 grados. Se pidio al usuario que diga que muestra el display en ese momento. |
| 2026-11-24 | Core/Src/main.c, CLAUDE.md, CHANGELOG.md | **Bordeado de la caja: la vuelta a la linea exige 2 sensores a la vez.** Reporte: a veces el ruido sacaba al robot del modo esquive mientras rodeaba la caja. Causa: `linea_detectada_cruda = (suma_pesos > 0)`, o sea que alcanza UN sensor sobre `LINEA_UMBRAL_ADC` para arrancar la confirmacion de linea de los 3 estados de pared (`ObjWall_LineConfirmed`, 5 ciclos). **Cambio**: nuevo contador `linea_sensores_activos` (sensores sobre el umbral y fuera de cuarentena en el ciclo actual, calculado en el mismo bucle que los pesos del centroide, cuyos valores no cambian) y nueva constante `OBJ_PARED_LINEA_MIN_SENSORES = 2`. En `ObjWall_LineConfirmed`, la cuenta de 5 ciclos solo puede ARRANCAR en un ciclo con 2 sensores a la vez; una vez arrancada sigue con `linea_detectada` como antes. **Por que no pedir 2 sensores en cada ciclo**: `linea_detectada` tiene antirrebote de liberacion y se sostiene unos ciclos despues de que la cinta pasa; un cruce rapido dura ~2 ciclos bajo los sensores, asi que exigir 2 sensores durante los 5 ciclos haria que nunca confirme la linea real y el robot no volveria a ella. Se decidio NO tocar `linea_detectada_cruda` porque la usan la velocidad deseada, la deteccion de cruce perpendicular y el display. Un sensor en cuarentena no cuenta. Fuera del bordeado no cambia nada. **COMPILADO Y PROBADO EN EL ROBOT: "quedo mejor".** Qt no requiere cambios. | Usuario: "hace que cuando este esquivando la caja, se necesiten DOS ADC al mismo tiempo en umbral para detectar linea, porque hay veces que el ruido hace que salga del modo esquive cuando esta dando vuelta alrededor de la caja". Riesgo a vigilar: si alguna vez no reconoce la linea real al terminar de rodear la caja, bajar a 1 la constante. |
| 2026-11-24 | Core/Src/main.c, CLAUDE.md, CHANGELOG.md | **Pausa post-giro del esquive (`OBJ_PAUSA_GIRO`): agregado termino D de amortiguacion; el robot rodaba adelante/atras despues del giro de 90 y perdia la referencia al objeto.** Causa: el hold de posicion de la pausa era P puro (`OBJ_PAUSA_POSICION_KP`) y el freno era `Freno_AnguloSegunModo`, que usa la velocidad LENTA (tau ~0.5 s) con zona muerta de 0.35 m/s: no ve la deriva lenta que deja el giro, y un P de posicion sin D sobre un doble integrador oscila siempre. Es exactamente el problema que el comentario de `OBJ_DISTANCIA_AMORTIGUACION_KV` ya documenta y que se resolvio en la fase STOP (`OBJ_FRENO_REVERSA`), pero la pausa nunca recibio el mismo arreglo. El giro (`OBJ_GIRO_ESQUIVE`) frena la ROTACION con un contra-pivote, no la TRASLACION (`setpoint_freno_objetivo = 0` durante los giros). **Cambio**: la pausa usa el mismo amortiguador que STOP — `+ KV * deadband(velocidad_est_ema, 0.15)`, mismas constantes `OBJ_DISTANCIA_AMORTIGUACION_*` (4 grados por m/s, tope 2.5) — y, como STOP, pone `setpoint_freno_objetivo = 0` para que el D reemplace al freno general y no se apilen. Signos verificados contra el codigo validado: STOP suma `+KV*v`; `velocidad_est_ema` negativa = avance, asi que inclina para atras al avanzar. Verificado tambien que `velocidad_est_ema = -(der + izq)/2` se anula en un giro en el lugar (los dos encoders cuentan en el mismo sentido: lo prueba la estacion por rueda, validada, que usa esa semisuma como traslacion). **Resultado del D en la pausa: el usuario reporta que "funciona muy bien hacia la derecha"**. Hacia la izquierda aparecio otro problema: el giro **se pasa casi 45 grados** ("casi un giro y medio") con "pulsos raros" al final. **(2) FIX fase 2 del giro**: la fase 2 (ajuste fino por el lateral activo) asume que "el sensor no ve la pared = el giro quedo corto", asi que SOLO puede girar mas en el mismo sentido. Si el lateral no baja de `OBJ_GIRO_ADC_OK` (3200) al terminar el giro, sigue pivoteando, aleja el sensor del objeto, nunca lo ve, y corta recien en `OBJ_GIRO_AJUSTE_EXTRA_MAX` = 150 counts (~36 grados) — coincide con lo que se pasa. Ademas la fase 1 tiene un tope por giroscopio (`abs_heading > 108`) pero la fase 2 NO lo tenia. Cambios: tope de 108 grados tambien en la fase 2 (`overshoot_fase2`, misma formula que la fase 1; `abs_heading` ya se calcula en cada ciclo antes de las fases), y `OBJ_GIRO_AJUSTE_EXTRA_MAX` 150 -> 40 counts (~10 grados: es un retoque, no otro giro). El define solo se usa en la fase 2. **Pendiente la causa raiz** de por que el ADC5 no ve la pared al terminar el giro izquierdo: todos los umbrales de pared (`OBJ_GIRO_ADC_OK` 3200, `OBJ_PARED_UMBRAL` 3600, `MUY_CERCA` 2100, `REVERSA` 600) dicen "ADC7" en su comentario — se calibraron con el sensor del giro derecho y el ADC5 los hereda desde el esquive alternado (2026-07-13) sin calibrar. Prueba propuesta: robot quieto, objeto a la distancia del bordeado a la DERECHA, `GETADCVALUES` (0xA5) desde Qt y mirar la barra 5; repetir a la IZQUIERDA y mirar la barra 7. **Resultado de (2)**: "quedo un poco mejor". **Causa raiz encontrada por el usuario**: en la fase STOP (`OBJ_FRENO_REVERSA`) la reversa NO va derecha — la rueda IZQUIERDA retrocede mas, el robot queda mirando a la izquierda y descentrado, con un poco de zigzag; eso favorece el esquive a la derecha y arruina el de la izquierda (el giro de 90 parte torcido y el lateral apunta lejos del objeto). Ademas a veces se cae durante el giro izquierdo (probable saturacion: pivote de 24% sobre un `pwm_saturado` alto, tope +-60). Prueba en `MOTOR_TEST`: el izquierdo arranca con 3 y el derecho con 2 en AMBOS sentidos, asi que el izquierdo es el mas debil estatico — sin correccion deberia retroceder mas el DERECHO. Que retroceda mas el izquierdo indica que algo lo empuja: lo unico que mete direccion en STOP es la correccion de reversa recta (`ajuste_direccion = -0.4 * (rev_dr - rev_dl)`, solo mientras hay ticks, rampa 2/ciclo, tope 8). Cadena de signos verificada con codigo + la prueba de banco (`motor_*_velocidad` negativo = rueda atras): ajuste negativo -> la izquierda retrocede mas; izquierda atras de mas -> `rev_dr - rev_dl > 0` -> ajuste negativo -> REALIMENTACION POSITIVA. Contradice el historial de julio (con -1.0 "pivoteaba izquierda/derecha alrededor del rumbo inicial", tipico de signo correcto); la mezcla de motores y la formula son identicas en julio (`2ef05f5`, estado borrado `OBJ_RETROCESO`) y hoy. **(3) Experimento con `REVERSA_RECTA_KP_COUNTS` = 0 (solo se usa en STOP)**: sin correccion la reversa se va a la DERECHA (rueda derecha mas rapida) — el desbalance natural, coherente con el izquierdo mas debil. Con -0.4 se iba a la IZQUIERDA. Conclusion: **el signo es correcto** (la correccion empuja en contra de la deriva natural; con signo invertido la habria agrandado hacia la derecha) **pero la ganancia sobraba**: corregia de mas y cruzaba al otro lado, de ahi el zigzag. La cadena de signos que sugeria realimentacion positiva tenia un eslabon mal (el historial de julio tenia razon). **(4) `REVERSA_RECTA_KP_COUNTS` -0.4 -> -0.2.** Resultado: con ganancia mucho menor **se seguia yendo a la izquierda**, y con 0 a la derecha — no es un problema de ganancia (con un error chico, una ganancia chica deberia acercarse a 0). **(5) CAUSA REAL: el ancla de la reversa recta se fijaba al ENTRAR a STOP**, o sea al detectar el objeto, todavia a velocidad de seguidor y girando sobre la linea. La curva recorrida durante el frenado se acumulaba en `rev_dr - rev_dl` como si fuera desvio, y al retroceder la correccion la "deshacia": robot torcido a la izquierda. Al ser un error grande, achicar la ganancia casi no cambiaba nada. **Cambios**: la correccion solo actua despues del frenado (`obj_pre_giro_ms != 0`), y el ancla (`obj_rev_der0/izq0`, con `obj_rev_rumbo_rampeado`, `obj_rev_ultimos_counts` y `obj_rev_ultimo_mov_ms`) se re-estampa en la transicion fase 1 -> fase 2, cuando |v| < 0.15 m/s: en un acercamiento coincide con el cambio de avanzar a retroceder. Verificado que `obj_pre_giro_ms` vuelve a 0 al detectar cada objeto (`Ctrl_LecturaYVelocidadLinea`), en la caida y al salir de STOP, asi que el segundo esquive tambien re-ancla. `REVERSA_RECTA_KP_COUNTS` queda en -0.2: con el ancla correcta el error es solo la deriva real de la reversa y la ganancia hay que reajustarla (derecha -> mas fuerte, -0.3/-0.4; izquierda -> mas debil, -0.1). Resultado: **siguio quedando muy mal**. **(6) Decision del usuario: `REVERSA_RECTA_KP_COUNTS` = 0 (correccion de reversa recta APAGADA en STOP)**, porque sin correccion la reversa se va apenas a la derecha y el esquive quedaba mucho mejor. El re-anclado de (5) se conserva: con ganancia 0 no tiene efecto, pero es el ancla correcta si se vuelve a encender. **Causa no aislada**: queda sin explicar por que cualquier ganancia negativa la tuerce a la izquierda incluso con el ancla corregida. Si se retoma: medir con telemetria `rev_dr - rev_dl` y `ajuste_direccion` durante la reversa, en vez de inferirlo por observacion. **COMPILADO Y PROBADO EN EL ROBOT** (lote final: D en la pausa + tope de la fase 2 + reversa recta apagada). **No se toco el giro** a proposito: tiene historia de muchas iteraciones, y durante un pivote rapido una diferencia de 1 count entre ruedas ya se lee como 0.32 m/s, asi que un D ahi podria meter ruido al giro. Pasos siguientes si no alcanza: (a) amortiguacion tambien durante `OBJ_GIRO_ESQUIVE`; (b) estampar el ancla del hold al EMPEZAR el giro y no al empezar la pausa, para que tambien corrija la deriva acumulada durante el giro. Qt no requiere cambios. | Usuario: "cuando hace el giro de 90 grados derecha o izquierda, cuando frena, siempre se va hacia atras o adelante, no se queda estatico haciendo que se pierda la referencia y distancia al objeto" |
| 2026-11-24 | Core/Src/main.c, CLAUDE.md, CHANGELOG.md | **FIX: el modo linea detectaba objeto y cruce perpendicular con el robot CAIDO, y quedaba trabado en `PGIRO`.** Una sola causa para los dos sintomas reportados. `Ctrl_LecturaYVelocidadLinea` (etapa 9) evalua sus dos transiciones de entrada sin mirar `f_caido`: caido en el piso, los sensores de objeto A5-A8 ven el suelo de cerca (-> `OBJ_FRENO_REVERSA`) o los cuatro de linea ven negro con `accel_asentado` verdadero (un robot caido y quieto esta perfectamente "asentado") (-> `PERPENDICULAR_ROTATE`). Pero caido, `Ctrl_MezclaMotores` sale en su primera linea y ningun `LineState_*` se ejecuta, asi que el estado nunca se completa ni se abandona; y el reset por caida de `Ctrl_DeteccionCaida` corre UNA sola vez (dentro del `if (!f_caido)` que pone `f_caido = 1`), antes de que la etapa 9 del ciclo siguiente lo re-dispare. Resultado: display congelado en `PGIRO` y, al levantar el robot, el handler arrancaba un giro de 90 o una evasion sin nada adelante. Ademas el reset usaba `>= LINE_STATE_OBJ_FRENO_REVERSA` (13) y dejaba afuera a `PERPENDICULAR_ROTATE` (12, el anterior en el enum), asi que una caida EN MEDIO del giro perpendicular tampoco se limpiaba. **Cambios**: (1) `!f_caido` en la condicion de entrada a deteccion de objeto; (2) `!f_caido` en la de cruce perpendicular; (3) el reset por caida pasa a `>= LINE_STATE_PERPENDICULAR_ROTATE` (cubre 12..19). El contador de antirrebote `obj_confirma_cnt` no necesita reset aparte: se recalcula del ADC antes del chequeo y vuelve a 0 con la primera muestra sin objeto. Verificado: diff de codigo = 2 lineas agregadas + 1 umbral; llaves y parentesis del codigo real (sin comentarios) balanceados igual que en HEAD. **(4) FIX MOTOR_TEST**: en `MOTOR_TEST` los motores nunca se movian si el robot no estaba vertical. La rama "sigue caido" de `Ctrl_DeteccionCaida` (etapa 13) pone `motor_*_velocidad = 0` en CADA ciclo mientras `f_caido = 1`, sin mirar el modo, y corre antes de que la etapa 17 aplique el valor que mando Qt por `SETMOTORSPEED` (que igual respondia ACK). Recuperar exige |roll| < 2 grados durante 50 ms, imposible con el robot en la mano o acostado. Ahora ese cero se saltea en `MOTOR_TEST`: ahi los motores los manda Qt, no el PID, asi que el cero por caida no tiene razon de ser. El cero de UNA vez en el instante de caer se conserva (si se vuelca a mitad de prueba, frena; se reanuda reenviando). Workaround sin codigo: sostener el robot vertical un instante (borra `f_caido`) y no inclinarlo mas de ~30 grados. **COMPILADO Y VALIDADO EN EL ROBOT por el usuario ("esta funcionando perfecto ahora")** — es la primera prueba fisica desde el tag `V29` (2026-07-27), asi que valida tambien los 20 commits previos que estaban pendientes de prueba fisica. Qt no requiere cambios. | Usuario: "no deberia detectar objeto en modo linea cuando esta caido, y por ahi se queda trabado en PGIRO". En la misma consulta reporto el motor izquierdo "como bobo para balancearse" tras un tiempo sin usar el robot: NO se toco firmware por eso. Desde el ultimo tag validado en el robot (`V29`, 2026-07-27) hay 20 commits sin prueba fisica, incluido el refactor de `control_motores.c` (ac89e57) que conserva el cableado invertido del motor izquierdo. Prueba de banco en `MOTOR_TEST` (habilitada por el fix 4): el izquierdo arranca en 3 % de PWM y el derecho en 2 %, ambos muy por debajo de la compensacion `MOTOR_*_ZONA_MUERTA = 8` — la friccion queda descartada como causa. No hay PID por motor: el de balance da un PWM comun, la compensacion por motor es a lazo abierto, y `ComputeSteeringPID` (que igualaria las ruedas por encoder) esta apagado (`direccion_pid_habilitado = 0`). Tras flashear este lote el balance funciona bien; no se aislo la causa del sintoma original. Si reaparece, sospechoso siguiente: el encoder izquierdo (la estacion por rueda calcula la rotacion como `deriva_der - deriva_izq`; un encoder muerto convierte cada vaiven en un "giro" a corregir) — probar con `GETSPEED` (0xA4) en `MOTOR_TEST` y con el kill-switch `rueda_mecanismos_habilitados = 0`. |
| 2026-11-20 | CONFIGURACION_IOC.md (nuevo), CLAUDE.md, CHANGELOG.md | **Guia de estudio del `.ioc` con 4 diagramas, y correccion de un dato del ADC que estaba mal documentado. Cero cambio de codigo ni del `.ioc`.** Nuevo `CONFIGURACION_IOC.md`: sintaxis del archivo (el prefijo como dueno del valor, `IPParameters` como indice de lo que realmente se guardo, el escapado con barra), el metodo de las tres vistas (.ioc / CubeMX / codigo generado + RM0383), el arbol de reloj derivado paso a paso (HSE 25 MHz / PLLM 25 = 1 MHz -> x192 = 192 MHz VCO -> /2 = 96 MHz SYSCLK y /4 = 48 MHz para USB) con las cuatro restricciones que lo fuerzan (el USB exige 48 MHz exactos, VCO entre 100 y 432, APB1 tope 50 MHz, Flash 3 wait states a 96 MHz) y el x2 de los timers de APB1 que confunde la cuenta; los cinco timers con la cuenta de su frecuencia; la cadena completa del ADC (TIM2 TRGO -> 8 canales en scan -> DMA2 circular -> mediana de 32); los tres streams de DMA con el por que de NORMAL vs CIRCULAR y byte vs halfword; el NVIC; el mapa de pines; y el checklist de regeneracion. **CORRECCION**: la tabla de perifericos decia que el ADC usa **15 ciclos/canal** — son **480** (`ADC_SAMPLETIME_480CYCLES`, confirmado en el `.ioc` y en las dos lineas de `main.c`). Con el ADC a PCLK2/4 = 24 MHz, eso da 492 ciclos = 20,5 us por canal y **164 us la secuencia de 8, sobre la ventana de 250 us de TIM2 (66%)**; con 15 ciclos serian 9 us. Factor 18: quien planifique agregar canales con el dato viejo se encuentra con conversiones solapadas. Los 480 ciclos son correctos para este hardware (sensores reflectivos de alta impedancia necesitan tiempo de carga del capacitor de muestreo). **HALLAZGOS documentados**: (a) **PA8, PB13, PB14 y PB15 (los encoders) NO estan en el `.ioc`** — se configuran a mano en `USER CODE BEGIN MX_GPIO_Init_2`, asi que la regeneracion no los borra, pero CubeMX los ve LIBRES y podria asignarlos a otro periferico sin marcar conflicto (TIM1_CH1 vive en PA8); se recomienda declararlos `GPIO_Input` + `Locked` como mitigacion. (b) El `.ioc` tiene **todas** las prioridades NVIC en `0:0`, pero las reales se fuerzan en `USER CODE BEGIN 2`, que corre DESPUES de los `MX_*_Init()` y sobrevive la regeneracion — menos fragil de lo que sugeria la advertencia del CLAUDE.md, aunque agregar una interrupcion nueva sin darle prioridad la deja empatada con el DMA del I2C. (c) `RCC.VCOInputMFreq_Value=1562500` es un residuo de cuando `PLLM` valia 16. (d) `StackSize=0x400` (1 KB) explica por que la foto del display vive en RAM estatica y no en la pila. Los dos `.md` nuevos sumados al arbol de archivos. Qt no requiere cambios. | Usuario: "necesito estudiar bien a fondo el .ioc del programa, como hago?" |
| 2026-11-20 | ARQUITECTURA.md (nuevo), CLAUDE.md, CHANGELOG.md | **Documento de arquitectura con 10 diagramas Mermaid, y un hallazgo: 11 de los 20 sub-estados de linea son INALCANZABLES. Cero cambio de codigo.** Nuevo `ARQUITECTURA.md` en la raiz: diagrama de contexto (sensores/actuadores/los dos canales con Qt), diagrama de paquetes (main.c como controlador + las tres capas de modulos, con la inversion de dependencia del driver MPU6050 via punteros de plataforma), UML de clases de los modulos y las structs que cruzan limites (`TelemetriaDatos`, `PantallaDatos`, `UNER_Bindings_t`), diagrama de secuencia de las 17 etapas de `ControlCiclo10ms` con sus dos salidas tempranas, las dos maquinas de estados anidadas, mapa de interrupciones con prioridades NVIC, y la cadena completa del setpoint (base + freno -> rampas separadas -> clamp -> estacion POST-rampa -> PID de balance). Mas una seccion de convenciones de signo, atomicidad de los encoders, piso de cuantizacion y el contrato con Qt. **HALLAZGO**: verificado estaticamente que ninguna de las 45 asignaciones de `linea_estado` escribe `LOST`, `SEARCHING`, `LOST_ASENTAR`, `EDGE_WAIT` ni `OBJ_BUSCAR_PARED`; nadie toma `&linea_estado`, no esta en `UNER_Bindings_t` y ningun otro `.c` la escribe. Por cascada caen tambien `LOST_BRAKE`, `LOST_ROTATE`, `LOST_AVANZA`, `EDGE_ROTATE`, `EDGE_ASENTAR` y `EDGE_AVANZA`: **toda la recuperacion de linea perdida (giro de 180 y de 90 con retorno por odometria) no se ejecuta nunca**, unas 700 lineas. Al perder la linea `linea_estado` se queda en `FOLLOWING`. **No es regresion de los refactors de septiembre**: en `5206501` (2026-07-01) ya nada asignaba `LOST` ni `EDGE_WAIT` desde `FOLLOWING` — la cadena era circular, sin puerta de entrada. Explica por que esas features figuran como "pendiente validar en el robot" desde julio sin validarse nunca. Tambien documentados: `OBJ_PAUSA_GIRO` salta directo a `OBJ_BORDEAR_PARED` salteando `OBJ_BUSCAR_PARED`, y que `LineState_Following()` no tiene ninguna transicion (las dos unicas entradas a sub-estados las decide `Ctrl_LecturaYVelocidadLinea`, y `GIVEN_UP` lo decide `Ctrl_LatchesPared`). Qt no requiere cambios. | Usuario: "podes armarme un diagrama UML y otro diagrama de contexto de TODO el codigo, digamos del MAIN, que funciones usa y todo lo que hay que tener en cuenta". Decision pendiente sobre los 11 estados: conectarlos con un `if` en la etapa 9, borrarlos, o dejarlos documentados. |
| 2026-10-11 | tests/test_esp01_network.c, tests/test_uner_wifi.c, CLAUDE.md | Agregadas simulaciones de comando 0xDD para volver a Station y del ciclo SoftAP -> Station -> SoftAP; verificada entrega de UDP, limpieza del cliente anterior y configuracion de la red nueva. Pasaron pruebas host de ESP01/UNER, motores, USB, telemetria, boton, display y reinicios; firmware Debug/Release compila. | Pedido: verificar el regreso a Router y buscar regresiones. Sin prueba de radio ni robot fisico. |
| 2026-10-01 | Core/Src/main.c, Core/Src/display_oled.c, tests/test_pantalla_regression.py, tests/test_reinicios_regression.py, CLAUDE.md, CHANGELOG.md | Pantalla Wi-Fi limitada a 5 s al arrancar o cambiar de modo. Los avisos de nueva IP/conexion ya no extienden una pantalla visible; cualquier pedido se acota a 5 s. Prueba de 6000 cuadros, temporizador y rollover aprobada; Debug/Release compilan. | Pedido: pantalla Wi-Fi demasiado prolongada. La comparacion historica de main completo no pasa por cambios independientes del freno de linea presentes en el arbol actual. Pendiente prueba fisica. |
| 2026-10-01 | Core/Src/main.c, Core/Src/UNER.c, Core/Inc/UNER.h, tests/test_uner_wifi.c, SOFTAP.md, CLAUDE.md, CHANGELOG.md | Cambio Station/SoftAP por UDP: Qt envia 0xDD por el enlace activo; firmware espera al menos 300 ms y a que termine el envio pendiente antes de reiniciar ESP (tope 5 s). USB sigue disponible. Compilado Debug/Release; pruebas host UNER/ESP01. | Pedido: cambiar desde Qt sin conectar USB cuando el robot usa baterias. Pendiente prueba fisica de ambos sentidos. |
| 2026-09-30 | Core/Src/main.c, .gitignore, tests/test_reinicios_regression.py, CLAUDE.md, CHANGELOG.md | Eliminados obj_pared_perdida_ms (sin lectores) y el envio UART inaccesible condicionado por dato_tx (sin productor distinto de cero), junto con uart_byte_tx. Transporte ESP01 por uart_send_byte intacto. UpdateADC_MovingAverage renombrada ActualizarMedianaADC y balance_escala_pi renombrada balance_escala_p. Comentarios y documentacion del hold ajustados a las operaciones existentes: solo P se anula, I sigue sumandose y su integral se descarga antes de la acumulacion habitual. Ignorada __pycache__/. Prueba del fuente completo adaptada con lista explicita de cambios permitidos; es10ms y codigo generado sin cambios. Debug/Release compilan y enlazan sin errores ni advertencias. main: 6236 a 6224 lineas. | Respaldo previo publicado: 74b1c59 y tag backup/pre-limpieza-2026-09-30. Pedido: todas las mejoras propuestas excepto es10ms. Qt no requiere cambios. Pendiente prueba fisica. |
---

## Decisiones de Diseño
> Registrar el *por qué* de decisiones críticas del firmware.

- **Hardware de motores aislado (2026-09-21):** `control_motores.c` solo traduce dos porcentajes firmados a los canales PWM TIM3/TIM4. La selección de potencia, PID, compensación de zona muerta y siesta permanecen en `Ctrl_SalidaMotores`, porque dependen del estado del robot y de los encoders. Este límite deja el mapeo eléctrico en un módulo pequeño sin trasladar ni duplicar la política de control.
- **Boton y telemetria aislados (2026-09-21):** `boton_usuario.c` solo reconoce gestos; la politica de modos y alertas sigue en main. `telemetria.c` posee temporizadores, secuencia y formatos; recibe una muestra estatica por ciclo sin acceso de escritura al control. CSV admite filas largas completas sin leer fuera del buffer.
- **Transporte USB aislado (2026-09-21):** `comunicacion_usb.c` conserva la cola y el buffer persistente requerido por CDC. UNER mantiene el protocolo compartido USB/WiFi; desde la extraccion siguiente, `telemetria.c` prepara los mensajes a partir de una muestra del controlador. Los consumidores incluyen una sola API en vez de repetir prototipos `extern`.
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
