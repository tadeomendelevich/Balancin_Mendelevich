/* Native test, no hardware:
 * gcc -std=c11 -Wall -Wextra -Werror -Itests -ICore/Inc
 *     tests/test_telemetria.c Core/Src/telemetria.c -o test_telemetria
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "telemetria.h"
#include "UNER.h"
#include "comunicacion_usb.h"

static uint32_t now;
static unsigned cycles, headers, csv_count, wifi_count, odom_count;
static char csv[400];
static uint16_t csv_length;
static WifiLogData_t wifi;
static WifiOdomData_t odom;

uint32_t HAL_GetTick(void) { return now; }
void USB_DebugStr(const char *s) {
    const char *header = "t_ms,dt_us,dt_ctrl_us,accel_roll,roll_accel_filtrado,gyro_y,giro_dps_clampeado,roll_filt,dyn_sp,error,p,i,d,salida_pid,pwm_comando,pwm_saturado,sat,motor_der,motor_izq,pitch,accel_x,accel_y,accel_z,giro_x,giro_y,giro_z\r\n";
    assert(!strcmp(s, header));
    ++headers;
}
void USB_DebugSend(const uint8_t *data, uint16_t length) {
    assert(length < sizeof(csv));
    memcpy(csv, data, length);
    csv[length] = 0;
    csv_length = length;
    ++csv_count;
}
void UNER_SendWifiLogData(WifiLogData_t *data) { wifi = *data; ++wifi_count; }
void UNER_SendWifiOdomData(WifiOdomData_t *data) { odom = *data; ++odom_count; }

static void tick(TelemetriaDatos *data, uint32_t time) {
    TelemetriaDatos before;
    memcpy(&before, data, sizeof(before));
    now = time;
    ++cycles;
    Telemetria_Actualizar(data);
    assert(!memcmp(&before, data, sizeof(before)));
}

int main(void) {
    TelemetriaDatos d = {
        .f_enviar_log_wifi=1, .f_enviar_log_csv=1, .f_wifi_conectado=1,
        .flag_saturacion=1, .linea_detectada_display=1, .estado_robot=3, .linea_estado=17,
        .motor_derecho_velocidad=-12, .motor_izquierdo_velocidad=13,
        .accel_x=-1, .accel_y=2, .accel_z=-3, .giro_x=4, .giro_y=-5, .giro_z=6,
        .roll_filtrado_grados=1.25f, .salida_pid=4.0f,
        .termino_p=1.0f, .termino_i=-2.0f, .termino_d=0.125f,
        .setpoint_dinamico_final=2.5f, .dt_ctrl=0.01f, .dt_real=0.012345f,
        .linea_error=-0.75f, .log_p_line=0.25f, .log_i_line=0.5f,
        .log_d_line=0.75f, .ajuste_direccion=-0.5f,
        .odom_x_m=1.0f, .odom_y_m=-2.0f, .odom_theta_grados=90.0f,
        .linea_error_display=0.625f, .inclinacion_lateral_ema=-3.0f,
        .error=-0.25f, .giro_dps_clampeado=0.5f,
        .accel_angulo_grados=-1.25f, .roll_accel_filtrado=-2.0f,
        .pwm_comando=12.5f, .pwm_saturado=-10.5f, .pitch_grados=0.25f,
        .giro_velocidad_dps=3.0f, .adc_mediana={100,200,300,400,500,600,700,800}
    };
    for (unsigned i=1; i<=10; ++i) {
        tick(&d, i*10);
        assert(csv_count==i/5 && wifi_count==i/10);
        assert(headers==1 && odom_count==0);
        if (i==5) {
            const char *expected = "50,12345,10000,-1250,-2000,3000,500,1250,2500,-250,1000,-2000,125,4000,1250,-1050,1,-12,13,250,-1,2,-3,4,-5,6\r\n";
            assert(!strcmp(csv, expected) && csv_length==strlen(expected));
        }
    }
    const WifiLogData_t expected_wifi = {
        .t_ms=100, .roll_filt=1.25f, .output=4.0f, .p_term=1.0f, .i_term=-2.0f,
        .d_term=0.125f, .mR=-12, .mL=13, .dt_ctrl_us=10000, .dyn_sp=2.5f,
        .line_error=-0.75f, .p_line=0.25f, .i_line=0.5f, .d_line=0.75f,
        .steering_adjustment=-0.5f, .adc1=100, .adc2=200, .adc3=300, .adc4=400
    };
    assert(sizeof(wifi)==64 && !memcmp(&wifi, &expected_wifi, sizeof(wifi)));

    /* Odometry is independent of both log enable flags. */
    d.f_enviar_log_wifi=d.f_enviar_log_csv=0;
    tick(&d, 499); assert(!odom_count);
    tick(&d, 500); assert(odom_count==1);
    WifiOdomData_t expected_odom = {
        .seq=0, .t_ms=500, .x_m=1.0f, .y_m=-2.0f, .theta_deg=90.0f,
        .line_error=0.625f, .line_detected=1, .robot_state=3, .line_state=17,
        .adc5=500, .adc6=600, .adc7=700, .adc8=800, .roll_deg=1.25f, .lat_deg=-3.0f
    };
    assert(sizeof(odom)==41 && !memcmp(&odom, &expected_odom, sizeof(odom)));
    tick(&d, 999); assert(odom_count==1);
    tick(&d, 1000); assert(odom_count==2 && odom.seq==1);
    assert(csv_count==2 && wifi_count==1 && headers==1);

    while(cycles<20) tick(&d, now+10);
    d.f_enviar_log_csv=d.f_enviar_log_wifi=1;
    while(cycles<25) tick(&d, now+10);
    assert(csv_count==3 && headers==1 && wifi_count==1);
    while(cycles<30) tick(&d, now+10);
    assert(csv_count==4 && wifi_count==2);

    /* Previously a >127-byte row caused an out-of-bounds read on transmission. */
    d.f_wifi_conectado=d.f_enviar_log_wifi=0;
    d.roll_filtrado_grados=d.salida_pid=d.termino_p=d.termino_i=d.termino_d=-123.456f;
    d.setpoint_dinamico_final=d.error=d.giro_dps_clampeado=-123.456f;
    d.accel_angulo_grados=d.roll_accel_filtrado=d.pitch_grados=d.giro_velocidad_dps=-123.456f;
    d.pwm_comando=d.pwm_saturado=-123.456f;
    d.accel_x=d.accel_y=d.accel_z=d.giro_x=d.giro_y=d.giro_z=-32768;
    while(cycles<35) tick(&d, UINT32_MAX);
    assert(csv_length>128 && csv_length<320 && csv_length==strlen(csv));
    assert(!strncmp(csv,"4294967295,12345,10000,",strlen("4294967295,12345,10000,")));
    unsigned commas=0;
    for(unsigned i=0;i<csv_length;++i) commas += csv[i]==',';
    assert(commas==25);
    const char *suffix=",-32768,-32768,-32768,-32768,-32768,-32768\r\n";
    assert(!strcmp(csv+csv_length-strlen(suffix),suffix));

    /* Disconnect/reconnect, tick rollover and 16-bit sequence rollover. */
    unsigned previous=odom_count;
    d.f_enviar_log_csv=0;
    tick(&d, UINT32_MAX-200);
    assert(odom_count==previous);
    d.f_wifi_conectado=1;
    tick(&d, UINT32_MAX-200);
    assert(odom_count==previous+1);
    tick(&d, 298); assert(odom_count==previous+1);
    tick(&d, 299); assert(odom_count==previous+2);
    for(unsigned i=0;i<65536;++i) {
        uint16_t expected_sequence=(uint16_t)odom_count;
        unsigned count=odom_count;
        tick(&d, now+500);
        assert(odom_count==count+1 && odom.seq==expected_sequence);
    }
    assert(headers==1 && csv_count==5 && wifi_count==2);
    puts("PASS telemetry: exact CSV/binary fields, cadence, flags, header, long rows, tick/sequence wrap.");
    return 0;
}
