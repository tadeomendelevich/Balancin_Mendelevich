"""Compare button gestures AND main's actions with the pre-extraction firmware.

python tests/test_boton_regression.py --cc <native-gcc>
Requires the Git tag backup/pre-boton-telemetria-2026-09-21.
Only GPIO/time/display are stubbed; the new button module is compiled as-is.
"""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
BASE = "backup/pre-boton-telemetria-2026-09-21"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default="gcc")
    args = parser.parse_args()
    old = subprocess.check_output(["git", "show", BASE + ":Core/Src/main.c"], cwd=ROOT).decode("utf-8")
    current = (ROOT / "Core/Src/main.c").read_text(encoding="utf-8")
    start = old.index("\t      // ── Botón KEY:")
    start = old.index("{", start)
    end = old.index("\n\t  }\n\n      if (wifi_modo_pendiente", start)
    reference = "static void ProcesarBoton(void)\n" + old[start:end]
    start = current.index("static void ProcesarBoton(void)\n{")
    end = current.index("static void ActualizarPantalla(void)\n{", start)
    actual = current[start:end]
    prelude = r'''
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include "robot_estados.h"
#include "boton_usuario.h"
#define KEY_GPIO_Port 0
#define KEY_Pin 0
static uint32_t now, caida_alerta_hasta_ms, obj_ignorar_hasta_ms, refreshes;
static uint8_t pin, estado_robot, f_caido, f_cambiar_pantalla;
static uint8_t HAL_GPIO_ReadPin(int port,int key) { (void)port; (void)key; return pin; }
static uint32_t HAL_GetTick(void) { return now; }
static void Pantalla_ForzarActualizacion(void) { ++refreshes; }
'''
    old_state = r'''
static uint8_t boton_prev, boton_click_cantidad;
static uint32_t boton_ultimo_ms, boton_click_tiempo;
static void reset(uint8_t level) {
    boton_prev=level; boton_click_cantidad=0; boton_ultimo_ms=boton_click_tiempo=0;
}
'''
    new_state = "static void reset(uint8_t level) { BotonUsuario_Init(level); }\n"
    scenarios = r'''
static void step(uint32_t elapsed, uint8_t level) {
    now+=elapsed; pin=level; ProcesarBoton();
    printf("%u,%u,%u,%u,%u,%u\n",now,estado_robot,f_cambiar_pantalla,
           caida_alerta_hasta_ms,obj_ignorar_hasta_ms,refreshes);
}
static void begin(uint8_t level, uint8_t mode, uint32_t time) {
    reset(level); pin=level; estado_robot=mode; now=time;
    f_caido=1; f_cambiar_pantalla=7; caida_alerta_hasta_ms=10000;
    obj_ignorar_hasta_ms=0; refreshes=0;
}
int main(void) {
    /* Every gesture from every robot mode, including five-or-more clicks. */
    for(uint8_t mode=0;mode<=ROBOT_STATE_MOTOR_TEST;++mode) {
        for(unsigned clicks=1;clicks<=7;++clicks) {
            begin(1,mode,1000);
            for(unsigned i=0;i<clicks;++i) { step(10,0); step(30,1); }
            step(400,1); step(1,1);
            assert(refreshes==clicks && caida_alerta_hasta_ms==0);
            assert(f_cambiar_pantalla==7);
            uint8_t expected;
            if(clicks==1) expected=(mode==ROBOT_STATE_IDLE)?ROBOT_STATE_BALANCE_ONLY:ROBOT_STATE_IDLE;
            else if(clicks==2) expected=(mode<=ROBOT_STATE_BALANCE_ONLY)?ROBOT_STATE_LINE_FOLLOWING:ROBOT_STATE_IDLE;
            else {
                uint8_t target=clicks==3?ROBOT_STATE_BALANCE_AND_SPEED:
                               clicks==4?ROBOT_STATE_MANUAL_CONTROL:ROBOT_STATE_MOTOR_TEST;
                expected=mode==target?ROBOT_STATE_IDLE:target;
            }
            assert(estado_robot==expected);
            if(clicks==2 && expected==ROBOT_STATE_LINE_FOLLOWING)
                assert(obj_ignorar_hasta_ms==now+4000U);
        }
    }
    begin(0,ROBOT_STATE_IDLE,10); /* Held at boot selects SoftAP, no mode click. */
    step(1000,0); step(10,1); step(500,1);
    assert(estado_robot==ROBOT_STATE_IDLE && f_cambiar_pantalla==7 && !refreshes);
    begin(1,ROBOT_STATE_IDLE,10);
    step(10,0); step(800,0); assert(f_cambiar_pantalla==7);
    step(1,0); assert(f_cambiar_pantalla==0);
    step(1000,0); step(10,1); step(500,1);
    assert(estado_robot==ROBOT_STATE_IDLE && f_cambiar_pantalla==0);
    begin(1,ROBOT_STATE_IDLE,100);
    step(10,0); step(20,1); step(500,1); assert(estado_robot==ROBOT_STATE_IDLE);
    step(10,0); step(21,1); step(401,1); assert(estado_robot==ROBOT_STATE_BALANCE_ONLY);
    /* A queued short click can resolve while the next press is still held. */
    begin(1,ROBOT_STATE_IDLE,1000);
    step(10,0); step(30,1); step(10,0); step(391,0); step(410,0); step(10,1);
    /* Wrapping tick counter, the original zero sentinel, and mixed traces. */
    for(unsigned pass=0;pass<3;++pass) {
        begin(1,ROBOT_STATE_IDLE,pass==0?UINT32_MAX-100:pass==1?0:1000);
        step(0,0); step(30,1); step(401,1);
        uint32_t random=1234567;
        for(unsigned i=0;i<20000;++i) {
            random=random*1664525U+1013904223U;
            step((random>>16)%1000, (random>>8)&1U);
        }
    }
    return 0;
}
'''
    with tempfile.TemporaryDirectory(prefix="balancin-boton-") as temp:
        temp = Path(temp)
        outputs = []
        env = os.environ.copy()
        env["PATH"] = str(Path(args.cc).resolve().parent) + os.pathsep + env.get("PATH", "")
        for label, state, body in [("before", old_state, reference), ("after", new_state, actual)]:
            source = temp / (label + ".c")
            binary = temp / (label + (".exe" if os.name == "nt" else ""))
            source.write_text(prelude + state + body + scenarios, encoding="utf-8")
            command = [args.cc, "-std=c11", "-Wall", "-Wextra", "-Werror", "-I" + str(ROOT / "Core/Inc"), str(source)]
            if label == "after":
                command.append(str(ROOT / "Core/Src/boton_usuario.c"))
            subprocess.run(command + ["-o", str(binary)], check=True, env=env)
            outputs.append(subprocess.check_output([str(binary)], env=env))
        if outputs[0] != outputs[1]:
            before, after = (output.splitlines() for output in outputs)
            for i, (a, b) in enumerate(zip(before, after)):
                if a != b:
                    raise AssertionError("Button differs at sample {}: {!r} != {!r}".format(i, a, b))
            raise AssertionError("Button output length differs")
        print("PASS: button gestures, all mode actions, alarms, long press, boot hold, boundaries and wrap.")
        print("PASS: {} samples match the pre-extraction firmware.".format(len(outputs[0].splitlines())))


if __name__ == "__main__":
    main()
