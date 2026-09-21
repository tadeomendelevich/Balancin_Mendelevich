#ifndef ROBOT_ESTADOS_H
#define ROBOT_ESTADOS_H

/* Estados compartidos por control y pantalla. Mantener los valores del protocolo. */
typedef enum {
    ROBOT_STATE_IDLE = 0,
    ROBOT_STATE_BALANCE_ONLY,
    ROBOT_STATE_BALANCE_AND_SPEED,
    ROBOT_STATE_LINE_FOLLOWING,
    ROBOT_STATE_MANUAL_CONTROL,
    ROBOT_STATE_MOTOR_TEST
} eRobotState;

typedef enum {
    FALL_REASON_NONE = 0,
    FALL_REASON_ANGLE,
    FALL_REASON_UPSIDE_DOWN,
    FALL_REASON_CRITICAL_ZONE,
    FALL_REASON_SPEED
} eFallReason;

// --- Line Search & Loss Control ---
typedef enum {
    LINE_STATE_FOLLOWING = 0,  // Siguiendo línea normalmente
    LINE_STATE_LOST,           // Línea perdida, frenando y buscando
    LINE_STATE_SEARCHING,      // Girando suavemente para buscar
    LINE_STATE_LOST_BRAKE,     // Línea perdida: frena hasta velocidad baja antes de girar
    LINE_STATE_LOST_ROTATE,    // Línea perdida: giro 180° para buscarla
    LINE_STATE_LOST_ASENTAR,    // Post-180°: pausa de estabilización antes de avanzar
    LINE_STATE_LOST_AVANZA,       // Post-180°: avanza hacia adelante hasta encontrar la línea
    LINE_STATE_EDGE_WAIT,      // Línea perdida por un extremo (curva): espera a frenar antes de girar 90°
    LINE_STATE_EDGE_ROTATE,    // Línea perdida por un extremo: gira 90° hacia ese lado
    LINE_STATE_EDGE_ASENTAR,    // Post-90°: pausa de estabilización antes de avanzar
    LINE_STATE_EDGE_AVANZA,       // Post-90°: avanza con velocidad controlada hasta encontrar la línea
    LINE_STATE_GIVEN_UP,       // Ni el giro de 180 ni el de 90 encontraron la línea: reposo total hasta reponerla a mano
    LINE_STATE_PERPENDICULAR_ROTATE,    // Los 4 ADC en negro sin manipulación: cruce perpendicular, gira 90° (sentido del último esquive)
    // NOTA: al eliminar OBJ_ESPERA_REVERSA, OBJ_RETROCESO y OBJ_ARC se
    // RENUMERARON los estados OBJ_* que viajan a Qt en WifiOdomData_t.line_state — si Qt interpreta el valor, actualizar su mirror.
    LINE_STATE_OBJ_FRENO_REVERSA,      // Fase STOP: entra DIRECTO al detectar objeto — frena la inercia y lleva/sostiene la distancia (A6/A8 en banda) hasta estabilizarse → giro
    LINE_STATE_OBJ_GIRO_ESQUIVE,     // Objeto detectado: girando 90° por encoders (sentido según obj_esquive_dir)
    LINE_STATE_OBJ_PAUSA_GIRO,          // Post-rotación: balance estático espera 2s
    LINE_STATE_OBJ_BUSCAR_PARED, // Avanza despacio (2°) hasta encontrar la pared en el lateral activo (ADC7/ADC5)
    LINE_STATE_OBJ_BORDEAR_PARED,      // Wall-following: avanza mientras el lateral activo ve la pared
    LINE_STATE_OBJ_PARED_LIBRE,    // Perdió la pared en OBJ_BORDEAR_PARED: avanza un poco más antes de girar (no chocar la esquina)
    LINE_STATE_OBJ_GIRO_PARED,     // Wall-following: pivot hacia la pared hasta re-verla en el lateral activo
} eLineState;

#endif
