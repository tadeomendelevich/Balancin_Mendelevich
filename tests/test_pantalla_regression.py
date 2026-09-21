"""Compare OLED drawing commands with the pre-extraction firmware.

Run: python tests/test_pantalla_regression.py --cc <native-gcc>
Requires Git history containing backup/pre-display-2026-09-21. Compiles the
actual rendering functions from both revisions with drawing/clock/network
stubs, then compares their complete output (not implementation snapshots).
Temporary executables are kept outside the repository. Does not test hardware.
"""
import argparse
import os
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
BASE = "backup/pre-display-2026-09-21"


def between(text, start, end):
    return text[text.index(start):text.index(end, text.index(start))]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default="gcc")
    args = parser.parse_args()
    old = subprocess.check_output(
        ["git", "show", BASE + ":Core/Src/main.c"], cwd=ROOT
    ).decode("utf-8")
    new = (ROOT / "Core/Src/display_oled.c").read_text(encoding="utf-8")
    header = (ROOT / "Core/Inc/display_oled.h").read_text(encoding="utf-8")
    data_type = between(header, "typedef struct {", "} PantallaDatos;") + "} PantallaDatos;\n"
    fields = re.findall(r"^    (\w+) (\w+)(\[[^]]+\])?;", data_type, re.M)
    enums = between(old, "typedef enum {", "/* USER CODE END PTD */")
    new_enums = (ROOT / "Core/Inc/robot_estados.h").read_text(encoding="utf-8")
    assert enums.strip() in new_enums, "Robot/line/fall enum values changed"

    constants = {
        "obj_deteccion_umbral_adc": "OBJ_DETECCION_UMBRAL_ADC",
        "obj_distancia_banda_piso_adc": "OBJ_DISTANCIA_BANDA_PISO_ADC",
        "obj_pared_adc_indice": "OBJ_PARED_ADC_INDICE",
        "obj_pared_umbral": "OBJ_PARED_UMBRAL",
        "obj_pared_reversa_umbral": "OBJ_PARED_REVERSA_UMBRAL",
        "obj_pared_muy_cerca_umbral": "OBJ_PARED_MUY_CERCA_UMBRAL",
    }
    defines = "\n".join(
        re.search(r"^#define " + name + r"\s+[^\n]+", old, re.M)[0]
        for name in constants.values()
    )
    # Both runners use the same actual enum and input data layout.
    prelude = r'''
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#define PANTALLA_ADC_CANTIDAD 8
#define BARRAS_CANTIDAD 8
#define PANTALLA_ANCHO 128
#define PANTALLA_ALTO 64
#define SSD1306_COLOR_WHITE 1
#define SSD1306_COLOR_BLACK 0
#define ESP01_MODE_SOFTAP 2
#define ESP01_WIFI_CONNECTED 1
enum { MOVE_FORWARD=1, MOVE_BACKWARD, MOVE_LEFT, MOVE_RIGHT };
typedef int SSD1306_COLOR_t;
typedef struct { uint8_t FontWidth; } TestFont;
static TestFont Font_5x7={5}, Font_7x10={7};
static uint32_t now, wifi_splash_hasta_ms;
static unsigned scenario;
static const char *wifiSSID="RED PRUEBA MUY LARGA PARA RECORTAR";
static uint32_t HAL_GetTick(void) { return now; }
static int ESP01_GetMode(void) { return scenario%2 ? 2 : 1; }
static int ESP01_StateWIFI(void) { return scenario%3 ? 1 : 0; }
static int ESP01_HasPeer(void) { return scenario%2; }
static const char *ESP01_GetLocalIP(void) { return scenario%3 ? "192.168.4.1" : NULL; }
static const char *ESP01_GetPeerIP(void) { return "192.168.4.100"; }
static uint8_t UNER_GetLastManualCmd(void) { return scenario%5; }
#define SSD1306_Fill(c) printf("fill %d\n",(int)(c))
#define SSD1306_DrawPixel(x,y,c) printf("pixel %d %d %d\n",(int)(x),(int)(y),(int)(c))
#define SSD1306_DrawLine(x,y,a,b,c) printf("line %d %d %d %d %d\n",(int)(x),(int)(y),(int)(a),(int)(b),(int)(c))
#define SSD1306_DrawRectangle(x,y,w,h,c) printf("rect %d %d %d %d %d\n",(int)(x),(int)(y),(int)(w),(int)(h),(int)(c))
#define SSD1306_DrawFilledRectangle(x,y,w,h,c) printf("box %d %d %d %d %d\n",(int)(x),(int)(y),(int)(w),(int)(h),(int)(c))
#define SSD1306_DrawChar5x7(c,x,y) printf("char %d %d %d\n",(int)(c),(int)(x),(int)(y))
#define SSD1306_GotoXY(x,y) printf("goto %d %d\n",(int)(x),(int)(y))
#define SSD1306_Puts(s,f,c) printf("text %s %d %d\n",(s),(int)(f)->FontWidth,(int)(c))
#define SSD1306_RequestUpdate() puts("update")
'''
    old_render = between(old, "static void FormatSignedFixed(", "// Recupera el bus I2C")
    old_clamp = between(old, "static float clampf_local(", "static float apply_deadbandf(")
    new_render = new[new.index("static float clampf_local("):]
    globals_old = "\n".join(
        "static " + typ + " " + name + array + ";"
        for typ, name, array in fields if name not in constants
    )
    # Deterministic varied data, including negative angles, ADC saturation,
    # quarantine, both obstacle sides and saved/lost odometry poses.
    assignments = []
    for index, (typ, name, array) in enumerate(fields):
        if name in constants:
            expression = constants[name].replace("OBJ_PARED_ADC_INDICE", "((d.obj_esquive_dir>0)?6:4)")
        elif array:
            count = 4 if name == "linea_canal_cuarentena" else 8
            assignments.extend(
                "d.{0}[{1}] = (scenario*{2}+{1}*937)%{3};".format(
                    name, i, index + 1, 2 if count == 4 else 4500)
                for i in range(count)
            )
            continue
        elif typ == "float":
            expression = "((int)(scenario%97)-48)*0.17f+{0}*0.03f".format(index)
        else:
            expression = "scenario%2"
        assignments.append("d.{0} = {1};".format(name, expression))
    setup = "\n".join(assignments)
    copy_old = "\n".join(
        ("memcpy({0},d.{0},sizeof({0}));" if array else "{0}=d.{0};").format(name)
        for _, name, array in fields if name not in constants
    )
    driver_start = r'''
int main(void) {
  for (scenario=0; scenario<6000; ++scenario) {
    PantallaDatos d={0};
    now = scenario<5000 ? 1000+scenario*61 : UINT32_MAX-100+(scenario-5000)*61;
'''
    driver_end = r'''
    d.f_cambiar_pantalla=scenario%9;
    d.estado_robot=(scenario/9)%6;
    d.linea_estado=(scenario/54)%21;
    d.f_caido=scenario%13==0;
    d.vel_limite_falla=scenario%26==0;
    d.caida_motivo=(scenario/13)%5;
    d.caida_alerta_hasta_ms=now+(scenario%3 ? 1000 : -1);
    d.obj_esquive_dir=scenario%2 ? -1 : 1;
    d.obj_pared_adc_indice=d.obj_esquive_dir>0 ? 6 : 4;
    d.LINEA_UMBRAL_ADC=3000.0f;
    d.LINE_SPEED_TARGET=2.5f;
    wifi_splash_hasta_ms=scenario%11==0 ? now+1000 : (scenario%7==0 ? now-1 : 0);
    printf("SCENARIO %u\n",scenario);
'''
    with tempfile.TemporaryDirectory(prefix="balancin-display-") as temp:
        temp = Path(temp)
        outputs = []
        env = os.environ.copy()
        compiler_dir = str(Path(args.cc).resolve().parent)
        env["PATH"] = compiler_dir + os.pathsep + env.get("PATH", "")
        for label, rendering, binding, call in [
            ("before", old_clamp + old_render, globals_old, copy_old + "\nupdateDisplay();"),
            ("after", new_render, "", "Pantalla_Dibujar(&d);"),
        ]:
            source = temp / (label + ".c")
            binary = temp / (label + (".exe" if os.name == "nt" else ""))
            source.write_text(prelude + enums + data_type + defines + "\n" + binding
                              + "\n" + rendering + driver_start + setup + driver_end
                              + call + '\nprintf("splash %lu\\n",(unsigned long)wifi_splash_hasta_ms);\n}\n}\n',
                              encoding="utf-8")
            subprocess.run([args.cc, "-std=gnu11", "-O0", str(source), "-lm", "-o", str(binary)],
                           check=True, env=env)
            outputs.append(subprocess.check_output([str(binary)], env=env))
        if outputs[0] != outputs[1]:
            before, after = (output.splitlines() for output in outputs)
            for i, (a, b) in enumerate(zip(before, after)):
                if a != b:
                    raise AssertionError("Drawing differs at output line {}: {!r} != {!r}".format(i, a, b))
            raise AssertionError("Drawing output length differs")
        print("PASS: 6000 frames have identical drawing commands, text and WiFi expiry.")
        print("PASS: robot/line/fall enums unchanged.")

        # Exercise the actual refresh scheduler with pending IMU/DMA flags,
        # including an interrupt that changes a flag during UpdateScreen().
        scheduler = between(new, "void Pantalla_MostrarWifi(", "static float clampf_local(")
        scheduler_source = r'''
#include <stdint.h>
#include <assert.h>
#define DISPLAY_INTERVALO_IDLE_MS 100U
#define DISPLAY_INTERVALO_ACTIVO_MS 60U
static uint32_t now, ultimo_display_ms, wifi_splash_hasta_ms;
static uint8_t busy, done=1, inject, update_calls;
static volatile uint8_t pending, ready;
static struct { uint8_t *busy_flag; } ssd_ctx={&busy};
static uint32_t HAL_GetTick(void) { return now; }
static void SSD1306_UpdateScreen(void) { ++update_calls; if(inject) ready=1; }
static uint8_t SSD1306_IsUpdateDone(void) { return done; }
''' + scheduler + r'''
int main(void) {
    now=99; assert(!Pantalla_Procesar(1,&pending,&ready));
    now=100; assert(Pantalla_Procesar(1,&pending,&ready));
    now=159; assert(!Pantalla_Procesar(0,&pending,&ready));
    now=160; assert(Pantalla_Procesar(0,&pending,&ready));
    now=260; busy=1; assert(!Pantalla_Procesar(0,&pending,&ready));
    busy=0; pending=1; assert(!Pantalla_Procesar(0,&pending,&ready));
    pending=0; ready=1; assert(!Pantalla_Procesar(0,&pending,&ready));
    ready=0; done=0; assert(!Pantalla_Procesar(0,&pending,&ready));
    done=1; inject=1; assert(!Pantalla_Procesar(0,&pending,&ready));
    inject=0; ready=0; assert(Pantalla_Procesar(0,&pending,&ready));
    assert(update_calls==10);
    assert(!Pantalla_Procesar(0,&pending,&ready));
    now=270; Pantalla_ForzarActualizacion();
    assert(Pantalla_Procesar(1,&pending,&ready));
    ultimo_display_ms=UINT32_MAX-20; now=38;
    assert(!Pantalla_Procesar(0,&pending,&ready));
    now=39; assert(Pantalla_Procesar(0,&pending,&ready));
    now=UINT32_MAX-10; Pantalla_MostrarWifi(100);
    assert(wifi_splash_hasta_ms==89);
    return 0;
}
'''
        source = temp / "scheduler.c"
        binary = temp / ("scheduler.exe" if os.name == "nt" else "scheduler")
        source.write_text(scheduler_source, encoding="utf-8")
        subprocess.run([args.cc, "-std=gnu11", str(source), "-o", str(binary)], check=True, env=env)
        subprocess.run([str(binary)], check=True, env=env)
        print("PASS: refresh intervals, IMU/DMA exclusion, forced refresh and tick overflow.")


if __name__ == "__main__":
    main()
