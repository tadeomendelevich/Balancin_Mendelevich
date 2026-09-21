#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "ESP01.h"
#include "usbd_core.h"
#include "UNER.h"
USBD_HandleTypeDef hUsbDeviceFS={USBD_STATE_CONFIGURED};
static uint8_t usb[256], udp[256], modeValue=1, peer=1, busy, idle=1;
static uint16_t usbLen, udpLen;
static unsigned requests;
uint32_t HAL_GetTick(void) { return 100; }
void USB_Debug(const char *s,...) { (void)s; }
void USB_DebugStr(const char *s) { (void)s; }
ESP01_Mode ESP01_GetMode(void) { return (ESP01_Mode)modeValue; }
int ESP01_HasPeer(void) { return peer; }
int ESP01_IsSending(void) { return busy; }
char *ESP01_GetLocalIP(void) { return "192.168.4.1"; }
_eESP01STATUS ESP01_StateWIFI(void) { return ESP01_WIFI_CONNECTED; }
_eESP01STATUS ESP01_StateUDPTCP(void) { return ESP01_UDPTCP_CONNECTED; }
_eESP01STATUS ESP01_Send(uint8_t *p,uint16_t start,uint16_t len,uint16_t size) {
    if(busy) return ESP01_SEND_BUSY;
    for(uint16_t i=0;i<len;++i) udp[i]=p[(start+i)%size];
    udpLen=len; busy=1; return ESP01_SEND_READY;
}
uint8_t usb_enqueue_tx_segments(const uint8_t *a,uint16_t n,const uint8_t *b,uint16_t m) {
    memcpy(usb,a,n); memcpy(usb+n,b,m); usbLen=n+m; return 1;
}
#include "../Core/Src/UNER.c"
static uint8_t request(uint8_t mode) { ++requests; return idle && (mode==1 || mode==2); }
static void commandFrame(uint8_t cmd,const uint8_t *data,unsigned n) {
    uint8_t f[32]={'U','N','E','R',0,':'}; f[4]=(uint8_t)(n+2); f[6]=cmd;
    if(n) memcpy(f+7,data,n);
    uint8_t sum=0; for(unsigned i=0;i<n+7;++i) sum^=f[i]; f[n+7]=sum;
    for(unsigned i=0;i<n+8;++i) UNER_PushByte(f[i]);
    UNER_Task();
}
static void validReply(unsigned size) {
    assert(usbLen==size && usb[4]+6U==size);
    uint8_t sum=0; for(unsigned i=0;i<size;++i) sum^=usb[i]; assert(sum==0);
}
int main(void) {
    uint8_t rxBuf[256]={0},txBuf[256]={0}; _sRx r={.buff=rxBuf}; _sTx t={.buff=txBuf};
    UNER_Init(&r,&t); UNER_Bindings_t b={.request_wifi_mode=request}; UNER_RegisterBindings(&b);
    uint8_t ap=2;
    commandFrame(SET_WIFI_MODE,&ap,1); validReply(10); assert(usb[7]==ACK && usb[8]==2 && requests==1);
    // Busy transport retains the response until the next task iteration.
    idle=0; commandFrame(SET_WIFI_MODE,&ap,1); validReply(10); assert(usb[7]==UNKNOWN && udpReplyCount==1);
    busy=0; UNER_Task(); assert(udpLen==10 && udp[7]==UNKNOWN && udpReplyCount==0);
    unsigned previous=requests; commandFrame(SET_WIFI_MODE,NULL,0); assert(usb[7]==UNKNOWN && requests==previous);
    commandFrame(GET_WIFI_STATUS,NULL,0); validReply(28); assert(usb[7]==1 && usb[10]==1);
    assert(!strcmp((char*)usb+11,"192.168.4.1"));
    modeValue=2; peer=0; busy=0; UNER_Task(); assert(!udpReplyCount);
    WifiLogData_t telemetry={0}; UNER_SendWifiLogData(&telemetry); assert(!udpReplyCount);
    puts("PASS UNER mode command length, ACK/rejection, status layout, busy reply queue, AP without peer");
    return 0;
}
