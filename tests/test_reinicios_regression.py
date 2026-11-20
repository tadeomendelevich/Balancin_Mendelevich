"""Compara main completo con el respaldo, expandiendo los reinicios extraidos.

Ejecutar: python tests/test_reinicios_regression.py
No requiere hardware ni ejecutar las decisiones de seguridad del robot.
"""
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]
BASE = "backup/pre-reinicios-2026-09-22"
CALLS = {
    "ReiniciarSetpoints": 7,
    "ReiniciarVelocidad": 5,
    "ReiniciarPIDLinea": 9,
    "ReiniciarEstadoGiro": 5,
    "RegistrarLineaRecuperada": 9,
}


def tokens(source):
    # Conservar literales; ignorar exclusivamente comentarios y espacios.
    pattern = r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|/\*.*?\*/|//[^\n]*|\w+|<<=|>>=|->|\+\+|--|&&|\|\||==|!=|<=|>=|<<|>>|[+*/%&|^\-]=|[^\s]'
    return [m[0] for m in re.finditer(pattern, source, re.S)
            if not m[0].startswith(("//", "/*"))]


def aplicar_limpieza_20260930(source_tokens, perfil_wifi):
    """Admite solo las eliminaciones, renombres y cambios auditados posteriores."""
    replacements = [
        ("uint8_t dato_tx, dato_rx;", "uint8_t dato_rx;"),
        ("static uint8_t uart_byte_tx = 0;", ""),
        ("static uint32_t obj_pared_perdida_ms = 0;", ""),
        ("obj_pared_perdida_ms = 0;", ""),
        ("obj_pared_perdida_ms = HAL_GetTick();", ""),
        ("if (dato_tx && huart1.gState == HAL_UART_STATE_READY) {"
         "uart_byte_tx = dato_tx; dato_tx = 0;"
         "HAL_UART_Transmit_IT(&huart1, &uart_byte_tx, 1); }", ""),
        ("#define WIFI_PERFIL_ACTIVO 1", "#define WIFI_PERFIL_ACTIVO " + perfil_wifi),
        ("Pantalla_MostrarWifi(10000U);", "Pantalla_MostrarWifi(5000U);"),
        ("if (wifi_modo_pendiente && HAL_GetTick() - wifi_cambio_desde_ms >= 300U)",
         "if (wifi_modo_pendiente && HAL_GetTick() - wifi_cambio_desde_ms >= 300U &&"
         " (!UNER_WifiTxPending() || HAL_GetTick() - wifi_cambio_desde_ms >= 5000U))"),
    ]
    result = list(source_tokens)
    for old, new in replacements:
        needle = tokens(old)
        positions = [i for i in range(len(result) - len(needle) + 1)
                     if result[i:i + len(needle)] == needle]
        assert len(positions) == 1, old
        start = positions[0]
        result[start:start + len(needle)] = tokens(new)
    for old, new, count in [("UpdateADC_MovingAverage", "ActualizarMedianaADC", 3),
                            ("balance_escala_pi", "balance_escala_p", 4)]:
        assert result.count(old) == count, old
        result = [new if token == old else token for token in result]
    assert not set(result).intersection({"dato_tx", "uart_byte_tx", "obj_pared_perdida_ms"})
    return result


def verify():
    baseline = subprocess.check_output(
        ["git", "show", BASE + ":Core/Src/main.c"], cwd=ROOT
    ).decode("utf-8")
    # La limpieza posterior unifica estas dos ramas iguales del respaldo.
    duplicate = re.compile(
        r"else if \(\(estado_robot == ROBOT_STATE_BALANCE_AND_SPEED\) \|\|[^\n]*\n"
        r"\s*\(estado_robot == ROBOT_STATE_BALANCE_ONLY\)\) \{([^{}]*)"
        r"\} else \{([^{}]*)\}")
    matches = list(duplicate.finditer(baseline))
    assert len(matches) == 1 and tokens(matches[0][1]) == tokens(matches[0][2])
    baseline = duplicate.sub(lambda m: "else {" + m[2] + "}", baseline)
    current = (ROOT / "Core/Src/main.c").read_text(encoding="utf-8")
    for name, expected in CALLS.items():
        definition = re.compile(r"static void " + name + r"\(void\)\s*\{([^{}]*)\}")
        found = list(definition.finditer(current))
        assert len(found) == 1, name + ": falta o se duplica la definicion"
        body = found[0][1]
        # Sin variables locales ni llamadas: solo asignaciones a las globales.
        assert re.fullmatch(r"(?:\s*\w+\s*=\s*[\w.+ ]+;)+\s*", body), name
        current = definition.sub("", current)
        current, count = re.subn(r"\b" + name + r"\(\);", lambda _: body, current)
        assert count == expected, (name, count, expected)
    perfil = re.search(r"#define\s+WIFI_PERFIL_ACTIVO\s+(\d+)\b", current)
    assert perfil and perfil[1] in {"0", "1", "2", "3"}
    before, after = aplicar_limpieza_20260930(tokens(baseline), perfil[1]), tokens(current)
    if before != after:
        first = next((i for i, (a, b) in enumerate(zip(before, after)) if a != b),
                     min(len(before), len(after)))
        raise AssertionError("Cambio de codigo al expandir los reinicios: token "
                             + str(first) + "\nAntes: " + str(before[first:first + 12])
                             + "\nAhora: " + str(after[first:first + 12]))
    print("PASS: 35 reinicios expandidos; main coincide con el respaldo salvo cambios auditados y perfil Wi-Fi elegido.")


if __name__ == "__main__":
    verify()
