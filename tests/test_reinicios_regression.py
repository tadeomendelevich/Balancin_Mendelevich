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
    before, after = tokens(baseline), tokens(current)
    if before != after:
        first = next((i for i, (a, b) in enumerate(zip(before, after)) if a != b),
                     min(len(before), len(after)))
        raise AssertionError("Cambio de codigo al expandir los reinicios: token "
                             + str(first) + "\nAntes: " + str(before[first:first + 12])
                             + "\nAhora: " + str(after[first:first + 12]))
    print("PASS: 35 reinicios expandidos; todos los tokens de main coinciden con el respaldo.")


if __name__ == "__main__":
    verify()
