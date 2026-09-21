// Host integration tests. UART is simulated; no hardware is touched.
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ESP01.h"
static uint32_t tick;
uint32_t HAL_GetTick(void) { return tick; }
void USB_DebugStr(const char *s) { (void)s; }
void UNER_SendAlive(void) {}
#include "../Core/Src/ESP01.c"
static char wire[4096], at[256];
static unsigned wireLen, atLen, expectPayload, sentCount;
static uint8_t sent[256], appRx[512];
static uint16_t appW, appR;
static int legacy, suppressStart, suppressSendOK, suppressPrompt;
static void feed(const char *s) { while(*s) ESP01_WriteRX((uint8_t)*s++); }
static void chpd(uint8_t high) { if(!high) atLen=expectPayload=0; }
static int uart(uint8_t c) {
    if(expectPayload) {
        sent[sentCount++]=c;
        if(!--expectPayload && !suppressSendOK) feed("\r\nSEND OK\r\n");
        return 1;
    }
    assert(atLen<sizeof(at)-1); at[atLen++]=(char)c; at[atLen]=0;
    if(c!='\n') return 1;
    assert(wireLen+atLen<sizeof(wire)); memcpy(wire+wireLen,at,atLen); wireLen+=atLen; wire[wireLen]=0;
    if(!strncmp(at,"AT+CIPSEND=",11)) {
        expectPayload=(unsigned)atoi(at+11); sentCount=0;
        if(!suppressPrompt) feed("OK\r\n> ");
    } else if(!strcmp(at,"AT+CIFSR\r\n")) {
        feed(mode==ESP01_MODE_SOFTAP ? "+CIFSR:APIP,\"192.168.4.1\"\r\nOK\r\n" : "+CIFSR:STAIP,\"192.168.100.20\"\r\nOK\r\n");
    } else if(legacy && !strcmp(at,"AT+CWDHCP=1,2\r\n")) feed("ERROR\r\n");
    else if(!strncmp(at,"AT+CIPSTART=",12) && suppressStart) {}
    else feed("OK\r\n");
    atLen=0; return 1;
}
static void run(unsigned ms) { while(ms--) { ++tick; ESP01_Task(); } }
static void init(int ap) {
    tick=wireLen=atLen=expectPayload=sentCount=appW=appR=0;
    legacy=suppressStart=suppressSendOK=suppressPrompt=0; wire[0]=0;
    _sESP01Handle h={.aDoCHPD=chpd,.aWriteUSARTByte=uart,.bufRX=appRx,.iwRX=&appW,.irRX=&appR,.sizeBufferRX=sizeof(appRx)};
    ESP01_Init(&h);
    if(ap) ESP01_SetSoftAP("Balancin","Balancin2026"); else ESP01_SetWIFI("TestRouter","testpass");
    assert(ESP01_StartUDP(ap ? "192.168.4.1" : "192.168.100.5",30010,30000)==ESP01_UDPTCP_CONNECTING);
}
static void datagram(const char *ip,unsigned port,const uint8_t *data,unsigned len) {
    char hdr[80]; snprintf(hdr,sizeof(hdr),"+IPD,%u,%s,%u:",len,ip,port); feed(hdr);
    for(unsigned i=0;i<len;++i) ESP01_WriteRX(data[i]);
}
static void hello(const char *ip) {
    datagram(ip,30010,(const uint8_t*)ESP01_DISCOVERY,strlen(ESP01_DISCOVERY)); run(100);
}
static void test_station(void) {
    init(0); run(2500); assert(socketUp && networkUp && !peerKnown);
    assert(strstr(wire,"AT+CWMODE=1\r\n") && strstr(wire,"AT+CWJAP=\"TestRouter\""));
    assert(!strstr(wire,"AT+CWSAP"));
    uint8_t data[72]={0}; assert(ESP01_Send(data,0,72,72)==ESP01_SEND_READY);
    run(100); assert(!ESP01_IsSending() && sentCount==72);
    hello("192.168.100.77"); assert(ESP01_HasPeer());
    assert(!strcmp(ESP01_GetPeerIP(),"192.168.100.77"));
    assert(strstr(wire,"\"192.168.100.77\",30010"));
    run(10100); hello("\"192.168.100.123\""); assert(!strcmp(peerIP,"192.168.100.123"));
    puts("PASS station bootstrap, 72-byte telemetry, dynamic destination");
}
static void test_ap(void) {
    init(1); legacy=1; run(2500); assert(socketUp && networkUp && !peerKnown);
    assert(!strcmp(ESP01_GetLocalIP(),"192.168.4.1"));
    assert(strstr(wire,"AT+CWMODE=2") && strstr(wire,"AT+CWDHCP=0,1") && !strstr(wire,"AT+CWJAP"));
    uint8_t data[72]={0}; assert(ESP01_Send(data,0,72,72)==ESP01_UDPTCP_DISCONNECTED);
    hello("192.168.4.2"); assert(peerKnown && sentCount && !memcmp(sent,"BALANCIN_V1,2,192.168.4.1",23));
    hello("192.168.4.3"); assert(!strcmp(peerIP,"192.168.4.2"));
    run(10100); hello("192.168.4.3"); assert(!strcmp(peerIP,"192.168.4.3"));
    feed("WIFI DISCONNECT\r\n"); run(10); assert(socketUp);
    puts("PASS SoftAP, legacy DHCP, discovery, lease and new laptop");
}
static void test_fragmentation(void) {
    init(1); run(2500); hello("192.168.4.2");
    uint8_t f[]={'U','N','E','R',2,':',ALIVE,0};
    for(unsigned i=0;i<sizeof(f)-1;++i) f[7]^=f[i];
    feed("+IPD,8,192.168."); run(100); feed("4.2,30010:"); run(100);
    for(unsigned i=0;i<sizeof(f);++i) { ESP01_WriteRX(f[i]); run(30); }
    assert(appW==sizeof(f) && !memcmp(appRx,f,sizeof(f)));
    f[7]^=1; datagram("192.168.4.2",30010,f,sizeof(f)); run(10); assert(appW==sizeof(f));
    uint8_t large[300]={0}; datagram("192.168.4.2",30010,large,sizeof(large)); run(10); assert(appW==sizeof(f));
    f[7]^=1; datagram("192.168.4.2",30010,f,sizeof(f)); run(10); assert(appW==16);
    appW=510; appR=0; datagram("192.168.4.2",30010,f,sizeof(f)); run(10); assert(appW==510);
    puts("PASS fragmented IPD, invalid checksum, oversized packet, application overflow");
}
static void test_timeouts(void) {
    init(0); suppressStart=1; run(3000); assert(!socketUp);
    run(5000); assert(!socketUp);
    init(0); run(2500); uint8_t data[72]={0};
    suppressPrompt=1; assert(ESP01_Send(data,0,72,72)==ESP01_SEND_READY); run(1200);
    assert(!socketUp && !ESP01_IsSending());
    init(0); run(2500); suppressSendOK=1;
    assert(ESP01_Send(data,0,72,72)==ESP01_SEND_READY); run(2200); assert(!socketUp);
    init(0); run(2500); feed("WIFI DISCONNECT\r\n"); run(1); assert(!socketUp);
    run(2500); assert(socketUp);
    for(unsigned i=0;i<ESP01RXBUFAT+1;++i) ESP01_WriteRX('x');
    run(1); assert(!socketUp);
    puts("PASS no premature socket, send timeouts, reconnect, UART overflow");
}
static void test_interleaving(void) {
    init(1); run(2500); hello("192.168.4.2");
    uint8_t data[72]; memset(data,'>',sizeof(data));
    assert(ESP01_Send(data,0,72,72)==ESP01_SEND_READY);
    datagram("192.168.4.2",30010,(const uint8_t*)ESP01_DISCOVERY,strlen(ESP01_DISCOVERY));
    run(20); assert(!memcmp(sent,data,sizeof(data)));
    run(100); assert(socketUp && peerKnown);
    ESP01_CloseUDPTCP(); run(10000); assert(!socketUp && phase==CLOSED);
    puts("PASS receive while transmitting, binary prompt byte, explicit close");
}
int main(void) {
    test_station(); test_ap(); test_fragmentation(); test_timeouts(); test_interleaving();
    puts("All ESP01 integration simulations passed."); return 0;
}
