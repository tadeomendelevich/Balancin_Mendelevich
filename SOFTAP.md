# Wi-Fi Station y SoftAP

## Uso

- Arranque normal: **Station**, usa el perfil `WIFI_PERFIL_ACTIVO` de `main.c`.
- Mantener **KEY** al encender o resetear la Black Pill: inicia **SoftAP**. Soltar
  KEY después del arranque. Esa liberación no activa motores ni cambia el modo del robot.
- Red directa: **Balancin**, contraseña **Balancin2026**, IP del robot
  **192.168.4.1**. Estos valores están en `Core/Inc/ESP01.h`.
- En Windows conectarse a esa red (la indicación «sin Internet» es normal).
- En Qt elegir **Directo al robot (SoftAP)**. El socket UDP se abre al iniciar Qt;
  si se cerró manualmente, usar **Abrir UDP**. Qt envía descubrimiento cada 2 s.
- Con Qt conectado al robot por Wi-Fi, seleccionar la conexión deseada y pulsar
  **Aplicar modo al robot**. Qt envía la orden por UDP al modo actual; el robot
  debe estar en **IDLE**. También se admite USB si no hay enlace Wi-Fi.
  La ESP se reinicia después de enviar la respuesta. El modo elegido dura hasta
  reiniciar la Black Pill. No se escribe Flash del STM32.
- Al pasar de Station a SoftAP, conectar Windows a **Balancin** después de
  recibir la confirmación. Al volver a Station, reconectar Windows al router.
- Para volver a Station: con Windows todavía conectado a **Balancin**, elegir
  Router en Qt y pulsar **Aplicar modo al robot**. Tras recibir el ACK, conectar
  Windows al router. Otra opción es reiniciar sin KEY.

No hay cambio automático ni modo híbrido: se evita cambiar de red durante una
maniobra. El selector de Qt elige cómo buscar al robot; el botón separado es el
que solicita cambiar su configuración.

## Red y protocolo

STM32 USART1: 115200 8N1, PA9 TX, PA10 RX, PB2 CH_PD. Se conserva el cableado.
ESP escucha UDP 30000; Qt escucha 30010 (configurable en la interfaz).
Station conserva el destino del perfil para compatibilidad con Qt anterior.
SoftAP no transmite telemetría hasta registrar un cliente.

Qt envía el datagrama ASCII exacto `BALANCIN_DISCOVER_V1`. El ESP registra IP y
puerto del remitente mediante `AT+CIPDINFO=1`. Responde
`BALANCIN_V1,<modo>,<IP del robot>`. Los comandos, ACK y telemetría siguen usando
UNER; sus layouts anteriores no cambian. Un cliente válido conserva el destino
durante 10 s desde su último mensaje. Otro cliente puede tomarlo al vencer ese
plazo; no se redirige por paquetes inválidos ni por un comando de motores ajeno.

Qt busca Station mediante broadcast en las interfaces IPv4 activas, y SoftAP
mediante unicast a 192.168.4.1. Si el router bloquea broadcast, se puede escribir
la IP que muestra el OLED en «IP robot». Si hay aislamiento de clientes, debe
deshabilitarse en el router. Permitir UDP entrante para Qt en el firewall de Windows.

Nuevos comandos UNER:

| Comando | Solicitud | Respuesta (después del byte comando) |
|---|---|---|
| `0xDD` SET_WIFI_MODE | 1 byte: 1=Station, 2=SoftAP | ACK `0x0D` o rechazo `0xFF`, seguido del modo solicitado |
| `0xDE` GET_WIFI_STATUS | sin datos | modo, red lista, socket listo, cliente conocido (4 bytes), IP ASCII en 16 bytes con cero final |

El cambio se difiere al menos 300 ms y espera a que termine el envío UDP
pendiente, hasta un máximo de 5 s. Se revalida IDLE antes de aplicarlo.
La UI envía el comando por el enlace Wi-Fi activo, o por USB si solo ese
enlace está disponible. Cambiar el selector de Qt no cambia la red de Windows.

## Driver y compatibilidad AT

- Una transacción AT a la vez: no se interpreta el OK de CIPMUX como éxito de CIPSTART.
- Tarea cooperativa en cada vuelta del main, máximo 8 bytes TX y 256 RX por pasada.
- Plazos con HAL_GetTick, independientes de la frecuencia de llamadas.
- Parser por líneas y longitud binaria +IPD; soporta IP entre comillas o sin ellas,
  paquetes fragmentados y RX durante un envío. Buffer UART de 1024 bytes, con
  detección de desbordamiento; valida checksum/longitud antes de aceptar comandos.
- Si falta prompt o SEND OK se reinicia el módulo para no mezclar comandos AT
  con un payload binario incompleto. Reconexión con la configuración seleccionada.
- SoftAP: WPA2-PSK, canal 6; CIPAP fijo, DHCP habilitado, CWSAP.
- DHCP AP intenta `AT+CWDHCP=1,2` (ESP-AT) y, si recibe ERROR,
  `AT+CWDHCP=0,1` (AT antiguo del ESP8266). Station conserva `1,1`.
- CIPDINFO es obligatorio para SoftAP. Si no está soportado, se informa error
  de configuración por USB y se reintenta; Station permite el destino fijo antiguo.
- Las respuestas de comandos tienen una cola UDP de cuatro tramas; la telemetría
  cede prioridad y puede descartarse si el transporte está ocupado.

Referencia: [comandos Wi-Fi ESP8266](https://docs.espressif.com/projects/esp-at/en/release-v2.2.0.0_esp8266/AT_Command_Set/Wi-Fi_AT_Commands.html)
y [comandos TCP/IP](https://docs.espressif.com/projects/esp-at/en/release-v2.2.0.0_esp8266/AT_Command_Set/TCP-IP_AT_Commands.html).

## Validación y prueba física pendiente

Se compilaron firmware Debug/Release y Qt; hay simulaciones C del UART y tests
de protocolo e interfaz Qt. Esto no identifica el firmware AT grabado en la ESP
ni reemplaza la prueba de radio, alimentación y latencia con el robot real.

1. Flashear Black Pill y comprobar Station con la red habitual: telemetría,
   comandos, desconexión y reconexión. Verificar tiempos de control bajo tráfico.
2. Reiniciar manteniendo KEY, comprobar SSID y dirección en el OLED, conectar
   Windows y Qt directo. Confirmar asignación DHCP y ACK sin activar motores.
3. Cerrar/abrir Qt; probar otra notebook después de 10 s sin cliente anterior.
4. Por Wi-Fi probar Station → SoftAP y SoftAP → Station en IDLE; verificar
   recepción del ACK antes de cambiar Windows de red y rechazo en modo activo.
   Repetir por USB como vía alternativa.
5. Repetir prueba de control con el robot en condiciones habituales de ensayo.

## Respaldo y reversión

En ambos repositorios: tag **backup/pre-softap-2026-09-16**, publicado antes de
modificar el código. En firmware incluye el commit que conserva los cambios
locales previos del usuario. Ese snapshot tenía nombres de campos de telemetría
desalineados, corregidos por esta integración. El commit publicado anterior era
`e6575a3`; Qt estaba en `edf6847`.

Para inspeccionar sin borrar trabajo actual: `git worktree add ../revision-pre-softap
backup/pre-softap-2026-09-16` (comando en una sola línea, en cada repositorio y con
carpetas de destino distintas). Cambiar fuentes no revierte lo ya flasheado:
para volver el robot atrás hay que compilar y grabar la versión elegida.

## Tests host

Con GCC/MinGW en PATH, desde el firmware (salida en un directorio temporal):

```text
gcc -std=c11 -Wall -Wextra -Werror -Itests -ICore/Inc tests/test_ESP01.c -o test_esp.exe
gcc -std=c11 -Wall -Wextra -Werror -Itests -ICore/Inc tests/test_uner_wifi.c -o test_uner.exe
```

Ejecutar ambos binarios. No requieren ni abren hardware. Los tests Qt se describen
en `WIFI_CONNECTION.md` del otro repositorio.
