# Arquitectura del Firmware — Balancín Mendelevich

> Mapa visual del firmware para estudiarlo. Los diagramas están en Mermaid:
> GitHub los renderiza solo, y en VS Code con la extensión "Markdown Preview Mermaid".
> **Fuente de verdad = el código.** Si algo acá no coincide, el código gana.

Índice:
1. [Diagrama de contexto](#1-diagrama-de-contexto) — el firmware y el mundo exterior
2. [Módulos](#2-módulos-diagrama-de-paquetes) — quién depende de quién
3. [UML de módulos](#3-uml-de-módulos-y-estructuras) — APIs y structs
4. [El ciclo de control de 10 ms](#4-el-ciclo-de-control-de-10-ms) — las 17 etapas
5. [Las dos máquinas de estados](#5-las-dos-máquinas-de-estados)
6. [Interrupciones y tiempos](#6-interrupciones-y-tiempos)
7. [La cadena del setpoint](#7-la-cadena-del-setpoint) — cómo se arma el ángulo final
8. [Lo que hay que tener en cuenta](#8-lo-que-hay-que-tener-en-cuenta)

---

## 1. Diagrama de contexto

Qué entra, qué sale y con quién habla el firmware.

```mermaid
flowchart TB
    subgraph EXT_IN["Sensores"]
        MPU["MPU-6050<br/>accel + gyro 6 ejes"]
        ENC["2 encoders<br/>cuadratura 4x"]
        ADC["8 canales ADC<br/>A1-A4 linea / A5-A8 objeto"]
        KEY["Boton KEY<br/>PA0"]
    end

    subgraph FW["FIRMWARE STM32F411 @ 96 MHz"]
        CTRL["Lazo de control 100 Hz<br/>ControlCiclo10ms"]
        BUCLE["while(1)<br/>tareas no criticas"]
    end

    subgraph EXT_OUT["Actuadores y UI"]
        MOT["2 motores DC<br/>PWM ~100 kHz"]
        OLED["OLED SSD1306<br/>8 pantallas"]
        LED["LEDs<br/>PB10 / PC13"]
    end

    QT["App Qt en la PC<br/>BalancinQT"]

    MPU -->|"I2C1 + DMA<br/>14 bytes"| CTRL
    ENC -->|"polling 4 kHz<br/>TIM2"| CTRL
    ADC -->|"DMA circular<br/>trigger TIM2"| BUCLE
    KEY -->|"gestos"| BUCLE

    CTRL -->|"2 porcentajes<br/>firmados"| MOT
    BUCLE --> OLED
    BUCLE --> LED

    CTRL <-->|"USB CDC<br/>protocolo UNER"| QT
    BUCLE <-->|"WiFi UDP<br/>:30000 RX / :30010 TX"| QT

    style FW fill:#e8f0fe,stroke:#4285f4,stroke-width:2px
    style QT fill:#fff4e5,stroke:#f9ab00,stroke-width:2px
```

**Los dos canales con Qt no son lo mismo:**

| | USB CDC | WiFi UDP |
|---|---|---|
| Transporte | USB OTG FS, clase CDC | ESP-01 por USART1, AT commands |
| Protocolo | UNER binario | UNER binario (el mismo) |
| Telemetría | CSV a ~20 Hz | `WifiLogData_t` a ~10 Hz + `WifiOdomData_t` a 2 Hz |
| Siempre disponible | sí | solo si conectó |

**Modo WiFi:** Station por defecto (se conecta a tu router, perfil en `WIFI_PERFIL_ACTIVO`)
o SoftAP (el robot crea la red `Balancin`, IP 192.168.4.1) — se elige con KEY al encender
o con el comando `0xDD` desde Qt con el robot en IDLE.

---

## 2. Módulos (diagrama de paquetes)

`main.c` es el **controlador**: tiene todas las decisiones. Los módulos son **capas finas**
que solo traducen a hardware o formatean datos. Las flechas van en el sentido de la dependencia.

```mermaid
flowchart TB
    MAIN["<b>main.c</b> — 6225 lineas<br/>lazo de control, PID, maquina de estados,<br/>odometria, seguidor de linea, esquive"]

    subgraph CAPA_HW["Capa de hardware"]
        MOT["control_motores.c<br/>76 ln"]
        I2C["i2c_manager.c<br/>197 ln"]
        MPUD["MPU6050.c<br/>204 ln"]
        SSD["ssd1306.c<br/>787 ln"]
    end

    subgraph CAPA_IO["Capa de entrada/salida"]
        BTN["boton_usuario.c<br/>56 ln"]
        TEL["telemetria.c<br/>138 ln"]
        USB["comunicacion_usb.c<br/>206 ln"]
        OLED["display_oled.c<br/>979 ln"]
    end

    subgraph CAPA_NET["Protocolo y red"]
        UNER["UNER.c<br/>1075 ln"]
        ESP["ESP01.c<br/>342 ln"]
    end

    MAIN --> MOT
    MAIN --> BTN
    MAIN --> TEL
    MAIN --> USB
    MAIN --> OLED
    MAIN --> UNER
    MAIN --> ESP
    MAIN --> MPUD
    MAIN --> I2C

    OLED --> SSD
    OLED --> I2C
    MPUD -.->|"callbacks de<br/>plataforma"| MAIN
    TEL --> UNER
    TEL --> USB
    UNER --> USB
    UNER --> ESP
    SSD --> I2C

    style MAIN fill:#e8f0fe,stroke:#4285f4,stroke-width:3px
```

**La regla de diseño que explica todos los límites:** los módulos no toman decisiones de
control. `control_motores.c` recibe dos porcentajes y los convierte a CCR — no sabe qué es
un PID. `telemetria.c` recibe una foto de solo lectura (`TelemetriaDatos`) y la formatea —
no puede escribir nada del control. `display_oled.c` recibe `const PantallaDatos *`.

Esto tiene una consecuencia práctica: **para entender el comportamiento del robot solo
necesitás leer `main.c`.** Los módulos son plomería.

**La flecha punteada** `MPU6050.c -.-> main.c` es la única inversión: el driver del IMU
recibe punteros a funciones (`mpu_writeReg`, `mpu_readReg`, `mpu_readRegDMA`, `mpu_delayMs`,
`mpu_errorCb`) que viven en `main.c`. Es inyección de dependencias para que el driver no
conozca el HAL de ST.

---

## 3. UML de módulos y estructuras

C no tiene clases, así que modelo cada módulo como una "clase" con su API pública y las
structs que cruzan los límites.

```mermaid
classDiagram
    class main {
        +ControlCiclo10ms() void
        +main() int
        -Ctrl_LeerIMU() uint8_t
        -Ctrl_VelocidadEncoders() void
        -Ctrl_LatchesPared() void
        -Ctrl_Odometria() void
        -Ctrl_TimingDt() uint8_t
        -Ctrl_FiltroIMU() void
        -Ctrl_CuarentenaLinea() void
        -Ctrl_LecturaYVelocidadLinea() void
        -Ctrl_CambiosDeEstado() void
        -Ctrl_SetpointDinamico() void
        -Ctrl_DeteccionCaida() void
        -Ctrl_PIDBalance() void
        -Ctrl_MezclaMotores() void
        -Ctrl_Telemetria() void
        -Ctrl_SalidaMotores() void
        -SampleEncoders250us() void
        -LineState_xxx() void
    }

    class ControlMotores {
        +ControlMotores_Init(tim_der, tim_izq) void
        +ControlMotores_Aplicar(der, izq) void
    }

    class Telemetria {
        +Telemetria_Actualizar(datos) void
    }

    class Pantalla {
        +Pantalla_Init(hi2c, busy_flag) void
        +Pantalla_Procesar(reposo, pend, listo) uint8_t
        +Pantalla_Dibujar(datos) void
        +Pantalla_MostrarWifi(duracion_ms) void
        +Pantalla_ForzarActualizacion() void
    }

    class BotonUsuario {
        +BotonUsuario_Init(nivel) void
        +BotonUsuario_Actualizar() BotonUsuarioEventos
    }

    class ComunicacionUSB {
        +USB_Comunicacion_Init() void
        +usb_service_tx() void
        +usb_enqueue_tx(data, len) uint8_t
        +USB_Debug(fmt, ...) void
    }

    class I2CManager {
        +I2C_Manager_Init() void
        +I2C_Manager_Enqueue(req) uint8_t
        +I2C_Manager_EnqueuePriority(req) uint8_t
        +I2C_Manager_Process() void
        +I2C_Manager_IsBusy() uint8_t
        -cola de 8 pedidos
    }

    class UNER {
        +UNER_Init(rx, tx) void
        +UNER_RegisterBindings(bindings) void
        +UNER_PushByte(byte) void
        +UNER_Task() void
        +UNER_SendAllSensors() void
        +UNER_SendWifiLogData(data) void
        +UNER_SendWifiOdomData(data) void
    }

    class TelemetriaDatos {
        <<struct, solo lectura>>
        +uint8_t estado_robot
        +uint8_t linea_estado
        +float roll_filtrado_grados
        +float salida_pid
        +float termino_p, termino_i, termino_d
        +float setpoint_dinamico_final
        +float odom_x_m, odom_y_m, odom_theta_grados
        +uint16_t adc_mediana[8]
        +... 40 campos
    }

    class PantallaDatos {
        <<struct, solo lectura>>
        +uint8_t estado_robot
        +float roll_filtrado_grados
        +uint16_t adc_mediana[8]
        +float KP_value, KD_value, KI_value
        +int8_t obj_esquive_dir
        +... 50 campos
    }

    class UNER_Bindings_t {
        <<struct de punteros>>
        +ESPEJADA EN QT
        +NO RENOMBRAR CAMPOS
    }

    main ..> ControlMotores : usa
    main ..> Telemetria : usa
    main ..> Pantalla : usa
    main ..> BotonUsuario : usa
    main ..> ComunicacionUSB : usa
    main ..> I2CManager : usa
    main ..> UNER : usa
    main ..> TelemetriaDatos : llena cada ciclo
    main ..> PantallaDatos : llena al refrescar
    main ..> UNER_Bindings_t : registra punteros
    Telemetria ..> TelemetriaDatos : consume
    Pantalla ..> PantallaDatos : consume
    UNER ..> UNER_Bindings_t : lee y escribe
```

**El punto más delicado del diagrama:** `UNER_Bindings_t` es una struct de **punteros** a las
variables de control. Qt manda `MODIFY_KP` y UNER escribe directo en `KP_value` a través del
puntero. Sus nombres de campo están **espejados en el proyecto Qt** — renombrar uno rompe la
app sin que el compilador diga nada.

---

## 4. El ciclo de control de 10 ms

`ControlCiclo10ms()` es un **orquestador**: 17 etapas en orden fijo, cada una una función
`Ctrl_*`. No hay lógica suelta, solo la secuencia y dos puertas de salida temprana.

```mermaid
sequenceDiagram
    autonumber
    participant TIM1 as TIM1 100Hz
    participant Bucle as bucle while
    participant Ciclo as ControlCiclo10ms
    participant Mot as control_motores

    TIM1->>Bucle: es10ms = 1
    Note over Bucle: el ISR solo marca el flag,<br/>el trabajo corre en el bucle
    Bucle->>Ciclo: mpu_dato_listo_ctrl?

    Ciclo->>Ciclo: 1. Ctrl_LeerIMU
    Note right of Ciclo: sin dato nuevo -> RETURN
    Ciclo->>Ciclo: 2. Ctrl_VelocidadEncoders
    Ciclo->>Ciclo: 3. Ctrl_LatchesPared
    Ciclo->>Ciclo: 4. Ctrl_Odometria
    Ciclo->>Ciclo: 5. Ctrl_SteeringLazoCerrado
    Ciclo->>Ciclo: 6. Ctrl_TimingDt
    Note right of Ciclo: primer ciclo -> RETURN
    Ciclo->>Ciclo: 7. Ctrl_FiltroIMU
    Ciclo->>Ciclo: 8. Ctrl_CuarentenaLinea
    Ciclo->>Ciclo: 9. Ctrl_LecturaYVelocidadLinea
    Ciclo->>Ciclo: 10. Ctrl_OverrideManualLinea
    Ciclo->>Ciclo: 11. Ctrl_CambiosDeEstado
    Ciclo->>Ciclo: 12. Ctrl_SetpointDinamico
    Ciclo->>Ciclo: 13. Ctrl_DeteccionCaida
    Ciclo->>Ciclo: 14. Ctrl_PIDBalance
    Ciclo->>Ciclo: 15. Ctrl_MezclaMotores
    Ciclo->>Ciclo: 16. Ctrl_Telemetria
    Ciclo->>Ciclo: 17. Ctrl_SalidaMotores
    Ciclo->>Mot: ControlMotores_Aplicar(der, izq)
```

### Qué hace cada etapa

| # | Etapa | Entrada → Salida |
|---|---|---|
| 1 | `Ctrl_LeerIMU` | ¿hay dato nuevo del MPU? + métrica de manipulación |
| 2 | `Ctrl_VelocidadEncoders` | counts acumulados → `velocidad_est_ema`, rps por rueda |
| 3 | `Ctrl_LatchesPared` | ADC lateral → latches de reversa, atorado, timeout |
| 4 | `Ctrl_Odometria` | encoders + gyro Z → pose `x, y, θ` |
| 5 | `Ctrl_SteeringLazoCerrado` | PID de dirección por encoders **(apagado hoy)** |
| 6 | `Ctrl_TimingDt` | `dt` real entre muestras (**solo telemetría**) |
| 7 | `Ctrl_FiltroIMU` | accel + gyro → `roll_filtrado_grados` (filtro complementario) |
| 8 | `Ctrl_CuarentenaLinea` | sensores de línea clavados → los saca de la cuenta |
| 9 | `Ctrl_LecturaYVelocidadLinea` | 4 ADC → centroide `linea_error` + velocidad deseada |
| 10 | `Ctrl_OverrideManualLinea` | comandos manuales mientras no ve la línea |
| 11 | `Ctrl_CambiosDeEstado` | entrar/salir de cada modo → qué resetear |
| 12 | `Ctrl_SetpointDinamico` | **el modo decide cuántos grados inclinarse** |
| 13 | `Ctrl_DeteccionCaida` | histéresis de caída/recuperación |
| 14 | `Ctrl_PIDBalance` | error de ángulo → `pwm_saturado` (PWM común) |
| 15 | `Ctrl_MezclaMotores` | elige rama: línea / manual / balance |
| 16 | `Ctrl_Telemetria` | arma CSV USB + paquetes WiFi |
| 17 | `Ctrl_SalidaMotores` | zona muerta, siesta, "en el aire" → PWM real |

La etapa 15 despacha según el modo:

```mermaid
flowchart LR
    MEZ["Ctrl_MezclaMotores"] --> CAIDO{"f_caido?"}
    CAIDO -->|si| NADA["return<br/>no se toca nada"]
    CAIDO -->|no| MODO{"estado_robot"}
    MODO -->|"LINE_FOLLOWING<br/>sin override"| L["Ctrl_MotoresLinea<br/>+ handler del sub-estado"]
    MODO -->|"MANUAL_CONTROL<br/>u override"| M["Ctrl_MotoresManual"]
    MODO -->|"MOTOR_TEST"| T["nada: lo maneja Qt"]
    MODO -->|resto| B["Ctrl_MotoresBalance<br/>+ estacion por rueda"]
```

---

## 5. Las dos máquinas de estados

Son **dos niveles anidados**: `estado_robot` elige el modo de operación, y dentro del modo
línea, `linea_estado` lleva una segunda máquina de 20 sub-estados.

### 5.1 Modo del robot — `estado_robot`

```mermaid
stateDiagram-v2
    [*] --> IDLE : arranque

    IDLE --> BALANCE_ONLY
    IDLE --> BALANCE_AND_SPEED
    IDLE --> LINE_FOLLOWING
    IDLE --> MANUAL_CONTROL
    IDLE --> MOTOR_TEST

    BALANCE_ONLY --> IDLE
    BALANCE_AND_SPEED --> IDLE
    LINE_FOLLOWING --> IDLE
    MANUAL_CONTROL --> IDLE
    MOTOR_TEST --> IDLE

    note right of IDLE
        Motores cortados.
        Unico modo que acepta
        el cambio de WiFi (0xDD).
    end note

    note right of BALANCE_ONLY
        Solo equilibrio + estacion
        por rueda. Permite la siesta
        de motores.
    end note

    note right of LINE_FOLLOWING
        Contiene la 2da maquina
        de estados completa.
    end note
```

Los cambios de modo vienen de Qt (comandos UNER) o de gestos del botón KEY.
Cualquier modo puede caer a IDLE por caída (`f_caido`) o por corte de velocidad.

### 5.2 Sub-estados del modo línea — `linea_estado`

El `enum` declara 20 sub-estados y el `switch` despachador tiene un `case` para cada uno.
**Pero solo 9 son alcanzables.** Los otros 11 no los escribe nadie, directa ni
indirectamente — ver [5.3](#53-los-11-sub-estados-inalcanzables).

Esta es la máquina **que corre de verdad**:

```mermaid
stateDiagram-v2
    [*] --> FOLLOWING : valor inicial

    FOLLOWING --> OBJ_FRENO_REVERSA : objeto al frente debounce 100 ms
    FOLLOWING --> PERPENDICULAR_ROTATE : 4 ADC negros sin manipulacion
    FOLLOWING --> GIVEN_UP : timeout de pared perdida Ctrl_LatchesPared

    PERPENDICULAR_ROTATE --> FOLLOWING : giro 90 hacia el lado del ultimo esquive

    OBJ_FRENO_REVERSA --> OBJ_GIRO_ESQUIVE : 2 s estable en banda 3500..3900
    OBJ_GIRO_ESQUIVE --> OBJ_PAUSA_GIRO : giro 90 completo
    OBJ_PAUSA_GIRO --> OBJ_BORDEAR_PARED : 2 s de hold de posicion
    OBJ_BORDEAR_PARED --> OBJ_PARED_LIBRE : perdio la pared
    OBJ_PARED_LIBRE --> OBJ_GIRO_PARED : avanzo 100 counts
    OBJ_GIRO_PARED --> OBJ_BORDEAR_PARED : re-vio la pared

    OBJ_FRENO_REVERSA --> FOLLOWING
    OBJ_PAUSA_GIRO --> FOLLOWING
    OBJ_BORDEAR_PARED --> FOLLOWING : vio la linea tras 5 s
    OBJ_PARED_LIBRE --> FOLLOWING
    OBJ_GIRO_PARED --> FOLLOWING
    GIVEN_UP --> FOLLOWING : reaparecio la linea
```

Tres detalles que no se ven en el diagrama:

1. **Las entradas no vienen de los handlers.** `LineState_Following()` **no tiene ninguna
   transición**: la detección de objeto y la de cruce perpendicular las decide
   `Ctrl_LecturaYVelocidadLinea` (etapa 9), y el `GIVEN_UP` lo decide `Ctrl_LatchesPared`
   (etapa 3). Los handlers `LineState_*` solo manejan las transiciones *internas* de cada
   secuencia.
2. **`OBJ_PAUSA_GIRO` salta directo a `OBJ_BORDEAR_PARED`**, saltando `OBJ_BUSCAR_PARED`.
3. **Cualquier estado vuelve a `FOLLOWING`** por caída (`Ctrl_DeteccionCaida`), por cambio de
   modo (`Ctrl_CambiosDeEstado`) o desde las ramas manual/balance de la mezcla de motores.

**El esquive alterna de lado** en cada objeto: `obj_esquive_dir` vale `+1` (gira a la derecha,
bordea con ADC7) o `-1` (espejado: izquierda, ADC5). Los umbrales y mecanismos son los mismos
en ambos sentidos — solo cambian el signo de los pivots y el sensor lateral
(`OBJ_PARED_ADC_INDICE`).

### 5.3 Los 11 sub-estados inalcanzables

> ⚠️ **Hallazgo, pendiente de decisión.** Verificado estáticamente sobre el árbol actual:
> ninguna de las 45 asignaciones de `linea_estado` escribe estos 5 valores, nadie toma la
> dirección de la variable (`&linea_estado` no aparece), no es escribible desde Qt (no está
> en `UNER_Bindings_t`) y ningún otro `.c` la escribe. Por lo tanto son inalcanzables, y con
> ellos cae toda la cadena que dependía de ellos.

**Nunca asignados (5):**

| Estado | Handler que queda sin ejecutarse |
|---|---|
| `LINE_STATE_LOST` | `LineState_Lost()` |
| `LINE_STATE_SEARCHING` | *(cae por fallthrough en el mismo `case`)* |
| `LINE_STATE_LOST_ASENTAR` | `LineState_LostAsentar()` |
| `LINE_STATE_EDGE_WAIT` | `LineState_EdgeWait()` |
| `LINE_STATE_OBJ_BUSCAR_PARED` | `LineState_ObjBuscarPared()` |

**Inalcanzables en cascada (6)** — se asignan, pero solo desde estados inalcanzables:

```mermaid
flowchart LR
    LOST["LOST / SEARCHING<br/><b>nadie lo asigna</b>"] --> LB["LOST_BRAKE"]
    LB --> LR2["LOST_ROTATE"]
    LR2 --> LA["LOST_AVANZA"]
    LS["LOST_ASENTAR<br/><b>nadie lo asigna</b>"] --> LA
    EW["EDGE_WAIT<br/><b>nadie lo asigna</b>"] --> ER["EDGE_ROTATE"]
    ER --> EA["EDGE_ASENTAR"]
    EA --> EAV["EDGE_AVANZA"]
    BP["OBJ_BUSCAR_PARED<br/><b>nadie lo asigna</b>"] --> BORD["OBJ_BORDEAR_PARED<br/><i>este SI es alcanzable<br/>por otro camino</i>"]

    style LOST fill:#fde7e9,stroke:#d93025
    style LS fill:#fde7e9,stroke:#d93025
    style EW fill:#fde7e9,stroke:#d93025
    style BP fill:#fde7e9,stroke:#d93025
    style BORD fill:#e6f4ea,stroke:#34a853
```

**Qué implica en la práctica:** toda la recuperación de línea perdida —el **giro de 180°**
(`LOST_*`) y el **giro de 90° por el borde** (`EDGE_*`), con su retorno por odometría al punto
de pérdida— **no se ejecuta nunca**. Cuando el robot pierde la línea hoy, `linea_estado` se
queda en `FOLLOWING` y lo único que actúa es la rama `else` de `LineState_Following` más el
freno de la etapa 12.

Eso explica por qué varias de esas features figuran en el `CLAUDE.md` como *"pendiente validar
en el robot"* desde julio y nunca se validaron.

**No es una regresión reciente:** revisé `5206501` (2026-07-01) y ya entonces nada asignaba
`LOST` ni `EDGE_WAIT` desde `FOLLOWING` — la cadena era circular (`LOST → LOST_BRAKE → ... →
LOST`, sin puerta de entrada). Viene de antes de los refactors de septiembre.

**Las tres opciones**, para decidir con el robot delante:

1. **Conectarla**: agregar en `Ctrl_LecturaYVelocidadLinea` la transición
   `FOLLOWING → LOST_BRAKE` cuando `linea_perdida_ms` supere un umbral. Es la puerta que falta
   — un solo `if`. Riesgo: son ~700 líneas de código que **nunca corrieron en el robot**.
2. **Borrarla**: ~700 líneas y ~40 defines menos. Recuperable del historial de git.
3. **Dejarla y documentarla**: lo que hace este archivo. Cuesta 0 y no rompe nada.

---

## 6. Interrupciones y tiempos

Cuatro ritmos distintos corriendo a la vez. **Ninguna ISR hace trabajo pesado**: marcan un
flag y el `while(1)` lo procesa.

```mermaid
flowchart TB
    subgraph ISR["Interrupciones — solo marcan flags"]
        T2["TIM2 @ 4 kHz<br/>prio 3"]
        EXTI["EXTI12 MPU_INT<br/>prio 2"]
        DMA["DMA I2C<br/>prio 0"]
        T1["TIM1 @ 100 Hz"]
        U1["USART1 RX<br/>byte a byte"]
        USBI["OTG_FS"]
    end

    subgraph TRABAJO["while(1) — el trabajo real"]
        CICLO["ControlCiclo10ms<br/>100 Hz"]
        T10["bloque es10ms<br/>ADC, UNER, ESP01"]
        DISP["Pantalla_Procesar<br/>60/100 ms"]
        WD["watchdog I2C<br/>>150 ms sin MPU"]
    end

    T2 -->|"SampleEncoders250us<br/>+ trigger ADC"| CICLO
    EXTI -->|"pide lectura DMA<br/>del MPU"| DMA
    DMA -->|"mpu_dato_listo_ctrl"| CICLO
    T1 -->|"es10ms = 1"| T10
    U1 -->|"buffer circular"| T10
    USBI -->|"cola TX / RX"| T10
    CICLO --> DISP
    CICLO -.->|"si se cuelga"| WD
```

| Fuente | Frecuencia | Prio NVIC | Qué hace |
|---|---|---|---|
| **DMA I2C** | — | **0** (la más alta) | completa la lectura del MPU |
| **EXTI12** (MPU_INT) | ~100 Hz | 2 | dato listo → pide lectura DMA |
| **TIM2** | 4 kHz | 3 | muestrea encoders + dispara ADC |
| **TIM1** | 100 Hz | — | marca `es10ms` |
| USART1 RX | por byte | — | recibe del ESP-01 |
| OTG_FS | por paquete | — | USB CDC |

**Por qué el DMA de I2C está en prioridad 0:** antes empataba en 1 con `EXTI15_10` y la
transacción I2C quedaba pendiente en cola durante ráfagas → freeze. Esas prioridades están
forzadas en `USER CODE BEGIN 2` porque **CubeMX las resetea a 0 en cada regeneración**.

**El watchdog de software** es la red de seguridad: si pasan >150 ms sin dato del MPU, llama
`I2C1_Recover()` (9 pulsos de SCL + re-init) **sin resetear el micro**. Se detecta por
`"I2C RECOVER"` en el USB.

---

## 7. La cadena del setpoint

Esto es **el corazón del diseño** y lo que más cuesta ver leyendo el código salteado: el
seguidor de línea, el esquive y el manual **no tocan los motores**. Solo mueven el setpoint
de inclinación, y el PID de balance conserva siempre la autoridad final.

```mermaid
flowchart TB
    subgraph PEDIDO["Etapa 12 — cada modo pide grados"]
        PI["PI de velocidad<br/>del sub-estado"]
        HOLD["hold de posicion<br/>por encoders"]
        AS["AntiStall_Tick<br/>empuje si no arranca"]
        REV["escape de reversa"]
    end

    BASE["setpoint_base_objetivo<br/><i>+ = avanzar</i>"]
    FRENO["setpoint_freno_objetivo<br/><i>siempre contra el movimiento</i>"]

    GUARD["Red de seguridad global<br/>sobrevelocidad > 90% del target<br/><i>solo LOST_AVANZA / EDGE_AVANZA</i>"]

    RAMPA1["rampa propia"]
    RAMPA2["rampa propia<br/><i>mas rapida</i>"]

    SUMA["setpoint_dinamico_final<br/>= base + freno"]
    CLAMP["clamp a SP_LIMIT_DEG<br/>5 grados por defecto"]
    EST["+ Estacion_CorreccionAngulo<br/><i>POST-rampa, slew propio</i>"]

    PID["<b>Etapa 14 — PID de balance</b><br/>error = setpoint - roll_filtrado<br/>unico dueno del PWM"]
    PWM["pwm_saturado<br/>PWM comun"]

    PI --> BASE
    HOLD --> BASE
    AS --> BASE
    REV --> BASE
    BASE --> GUARD
    GUARD --> RAMPA1
    RAMPA1 --> SUMA
    FRENO --> RAMPA2
    RAMPA2 --> SUMA
    SUMA --> CLAMP
    CLAMP --> EST
    EST --> PID
    PID --> PWM

    style PID fill:#e8f0fe,stroke:#4285f4,stroke-width:3px
    style GUARD fill:#fde7e9,stroke:#d93025
```

**Por qué `base` y `freno` van separados:** cada uno tiene su propia rampa, y el freno tiene
que poder entrar **más rápido** que el avance. Si fueran una sola variable, la rampa lenta del
avance frenaría también al freno y llegaría tarde justo cuando más se necesita.

**Por qué la estación entra POST-rampa:** cuando viajaba dentro de `setpoint_base_objetivo`,
la rampa de 0.1°/ciclo + la EMA de velocidad le metían ~150 ms de retraso — la D llegaba en
contrafase y **bombeaba** el vaivén en vez de amortiguarlo. Por eso subir la ganancia siempre
empeoraba. Hoy se aplica al final con su propio slew de 0.4°/ciclo.

**La cascada completa**, de afuera hacia adentro:

```
sensores de linea  ->  PID de linea      ->  ajuste_direccion   (canal DIFERENCIAL)
velocidad deseada  ->  PI de velocidad   ->  grados de setpoint
                                                    |
                                                    v
                       PID de balance    ->  pwm_saturado       (canal COMUN)
                                                    |
                                                    v
                       mezcla: comun +- diferencial ->  2 motores
```

El de línea decide **hacia dónde**, el PI de velocidad decide **cuánto inclinarse**, y el PID
de balance decide **cuánto PWM**. Nadie salta de nivel.

---

## 8. Lo que hay que tener en cuenta

### Convenciones de signo (la fuente de la mitad de las confusiones)

| Variable | Convención |
|---|---|
| `velocidad_est_ema` | **negativo = adelante** |
| `setpoint_base_objetivo` | **positivo = avanzar** |
| `ajuste_direccion` | positivo → rueda derecha más lenta → gira a la derecha |
| `obj_esquive_dir` | `+1` derecha / `-1` izquierda |
| `AntiStall_Tick` | devuelve **siempre positivo**, el signo lo pone quien llama |

### Atomicidad al leer los dos encoders

Las ISR de TIM2 escriben los counts a 4 kHz. Siempre que se leen **las dos ruedas juntas**:

```c
__disable_irq();
int32_t a = encoder_derecho   - ancla_der;
int32_t b = encoder_izquierdo - ancla_izq;
__enable_irq();
float resultado = (a + b) * 0.5f;   // las cuentas AFUERA
```

No es por atomicidad de cada lectura — en Cortex-M4 un `int32_t` alineado se lee en una
instrucción y no se puede partir. Es por **coherencia entre las dos**: si la ISR cae en el
medio, tenés la derecha en `t` y la izquierda en `t+250µs`.

### Piso de cuantización de los encoders

Con `ENCODER_COUNTS_POR_VUELTA = 28` y delta de counts cada 10 ms:

- 1 count en **una** rueda ≈ **0.32 m/s**
- 1 count en **ambas** ≈ 0.63 m/s

Por eso `FRENO_VEL_ZONA_MUERTA = 0.35` — filtra exactamente ese piso. **Nada que necesite
resolución por debajo de 0.32 m/s es confiable**, y por eso la odometría sirve para volver a
un punto pero no para precisión fina.

### El contrato con Qt (romper esto no da error de compilación)

| Qué | Dónde | Consecuencia |
|---|---|---|
| Campos de `UNER_Bindings_t` | `UNER.h` | Qt escribe por puntero → **nunca renombrar** |
| Layout de `WifiLogData_t` | `UNER.h` | hay un mirror packed en `mainwindow.h` |
| Layout de `WifiOdomData_t` | `UNER.h` | ídem |
| Valores del enum `line_state` | `robot_estados.h` | **el orden importa**: viaja como número |
| Puertos UDP 30000 / 30010 | `ESP01_StartUDP` | |
| Rangos de los clamps | `UNER.c` | si Qt permite más, el firmware recorta **en silencio** |

**Renombrar un enum es seguro, reordenarlo no.** Todos los renombres de `FWD`/`SETTLE` que
se hicieron conservaron el orden justamente por esto.

### Al regenerar con CubeMX

1. Nunca tocar fuera de `/* USER CODE BEGIN */` ... `/* USER CODE END */`
2. Hacer **commit antes** de abrir el `.ioc`
3. Después, `git diff` obligatorio: CubeMX borra `EXTI9_5_IRQHandler` y simplifica
   `EXTI15_10_IRQHandler`, y **resetea las prioridades NVIC a 0**

### Mecanismos apagados o degradados

| Qué | Estado |
|---|---|
| `Ctrl_SteeringLazoCerrado` + `ComputeSteeringPID` | `direccion_pid_habilitado` está **hardcodeado en 0**: corre 100 veces por segundo solo para escribir `0.0f`. Feature apagada, no residuo |
| `roll_accel_filtrado` | se envía en el CSV pero **solo se asigna al recuperarse de una caída** → el dato que ves en Qt está congelado |
| `MODIFY_BETA_G/A` (0xBC/0xBD) | vestigiales: los betas son defines fijos, el firmware responde UNKNOWN |
| TIM5 | inicializado pero nunca arrancado (era el re-habilitador de EXTI de encoders) |
| `dt_real` (etapa 6) | **solo telemetría**: el control usa `DT_CTRL_FIJO` siempre |
| 11 de los 20 sub-estados de línea | **inalcanzables** — toda la recuperación de línea perdida (180° y 90°). Ver [5.3](#53-los-11-sub-estados-inalcanzables) |

### Por dónde empezar a leer

```
1. ControlCiclo10ms        las 17 etapas en orden -> el esqueleto
2. Ctrl_PIDBalance         el unico que toca el PWM
3. Ctrl_SetpointDinamico   la mas larga; es un switch gigante por sub-estado
4. Ctrl_LecturaYVelocidadLinea   el centroide, la velocidad deseada, y las
                                 UNICAS dos entradas a sub-estados de linea
5. LineState_ObjFrenoReversa en adelante -> la secuencia de esquive, que es
                                 la unica cadena de sub-estados que corre
```

Dos reglas para no perderse:

- **En `Ctrl_SetpointDinamico`, cada rama solo escribe `setpoint_base_objetivo` y
  `setpoint_freno_objetivo`.** Si una rama no te interesa, salteala entera: no hay efectos
  cruzados.
- **De los 19 handlers `LineState_*`, solo 9 se ejecutan.** Antes de estudiar uno, chequeá
  en [5.3](#53-los-11-sub-estados-inalcanzables) si su estado es alcanzable — te ahorrás
  leer ~700 líneas de código que hoy no corre.
