/* ESP-01 AT transport: one transaction at a time, bounded cooperative work.
 * STA and SoftAP use the same UNER/UDP payloads. No delays in the control loop.
 */
#include "ESP01.h"
#include "comunicacion_usb.h"
#include "stm32f4xx_hal.h"
#include "UNER.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PEER_LEASE_MS 10000U
#define RX_BUDGET 256U
#define TX_BUDGET 8U
typedef enum { OFF, RESET_LOW, BOOT, CONFIGURE, READY, CLOSED } Phase;
typedef enum { TX_IDLE, TX_COMMAND, TX_PROMPT, TX_PAYLOAD, TX_RESULT } TxPhase;
static _sESP01Handle io;
static OnESP01ChangeState notify;
static ESP01DebugStr debug;
static ESP01_Mode mode;
static Phase phase;
static TxPhase txPhase;
static const char *ssid, *password;
static uint8_t networkUp, socketUp, peerKnown, tcp, metadata, legacyDhcp;
static uint8_t step, waiting, replyOk, replyError, gotPrompt, discoveryReply;
static uint32_t deadline, peerAt, aliveAt;
static char localIP[16], remoteIP[16], peerIP[16];
static uint16_t localPort, remotePort, peerPort;
static volatile uint16_t rxW, rxR;
static volatile uint8_t rxOverflow;
static uint8_t rx[ESP01RXBUFAT], payload[256], packet[256];
static char command[224], line[224], packetIP[16];
static uint16_t commandLen, commandPos, payloadLen, payloadPos, lineLen;
static uint16_t packetLen, packetPos, packetPort;
static uint8_t inPacket, discardLine;
static uint32_t rxAt;

static int expired(uint32_t now, uint32_t when) { return (int32_t)(now-when) >= 0; }
static void event(_eESP01STATUS s) { if (notify) notify(s); }
static void logMessage(const char *s) { if (debug) debug(s); }
static void clearPeer(void) { peerKnown=0; peerIP[0]=0; discoveryReply=0; }
static void resetModule(void) {
    networkUp=socketUp=0; clearPeer(); txPhase=TX_IDLE;
    waiting=replyOk=replyError=gotPrompt=0;
    commandLen=commandPos=payloadLen=payloadPos=lineLen=0;
    inPacket=discardLine=0; rxR=rxW; rxOverflow=0; localIP[0]=0;
    step=0; metadata=1; legacyDhcp=0;
    if (io.aDoCHPD) io.aDoCHPD(0);
    phase=RESET_LOW; deadline=HAL_GetTick()+150U;
    event(ESP01_WIFI_RECONNECTING);
}
static int ipv4(const char *s) {
    unsigned a,b,c,d; char tail;
    return sscanf(s,"%u.%u.%u.%u%c",&a,&b,&c,&d,&tail)==4 &&
           a>0 && a<224 && b<256 && c<256 && d>0 && d<255;
}
static int credentialsValid(const char *s, const char *p, int ap) {
    if (!s || !p || !*s || strlen(s)>32 || strlen(p)>63 || (ap && strlen(p)<8)) return 0;
    return !strpbrk(s,"\r\n\"\\") && !strpbrk(p,"\r\n\"\\");
}
static void queueCommand(const char *s) {
    snprintf(command,sizeof(command),"%s",s);
    commandLen=(uint16_t)strlen(command); commandPos=0;
    replyOk=replyError=gotPrompt=0; waiting=0;
}
static int flushCommand(void) {
    unsigned count=0;
    while (commandPos<commandLen && count++<TX_BUDGET) {
        if (!io.aWriteUSARTByte((uint8_t)command[commandPos])) return 0;
        ++commandPos;
    }
    return commandPos==commandLen;
}
// Strict whole-datagram UNER validation: discovery cannot execute motor commands.
static int validFrame(const uint8_t *p, uint16_t n) {
    if (n<8 || memcmp(p,"UNER",4) || p[5]!=':' || p[4]<2 || n!=(uint16_t)(p[4]+6U)) return 0;
    uint8_t check=0;
    for (uint16_t i=0;i<n;++i) check^=p[i];
    return check==0;
}
static void receivedPacket(void) {
    if (packetLen>sizeof(packet) || !socketUp) return;
    int discover = packetLen==strlen(ESP01_DISCOVERY) &&
                   !memcmp(packet,ESP01_DISCOVERY,packetLen);
    if (!discover && !validFrame(packet,packetLen)) return;
    uint32_t now=HAL_GetTick();
    if (metadata && ipv4(packetIP) && packetPort) {
        int same = peerKnown && !strcmp(peerIP,packetIP) && peerPort==packetPort;
        if (!same && peerKnown && now-peerAt<PEER_LEASE_MS) return;
        // A new peer must explicitly discover or send a valid ALIVE, never a motor command.
        if (!same && !discover && !(packetLen==8 && packet[6]==ALIVE)) return;
        strcpy(peerIP,packetIP); peerPort=packetPort; peerAt=now; peerKnown=1;
    } else if (mode==ESP01_MODE_SOFTAP) return;
    else { // old AT firmware without CIPDINFO: retain the configured STA destination
        strcpy(peerIP,remoteIP); peerPort=remotePort; peerAt=now; peerKnown=1;
    }
    if (discover) { discoveryReply=1; return; }
    if (!io.bufRX || !io.iwRX || !io.sizeBufferRX) return;
    uint16_t w=*io.iwRX;
    if (io.irRX) {
        uint16_t used=(w+io.sizeBufferRX-*io.irRX)%io.sizeBufferRX;
        if (packetLen>=io.sizeBufferRX-used) { logMessage("ESP RX application full\r\n"); return; }
    }
    for (uint16_t i=0;i<packetLen;++i) {
        io.bufRX[w++]=packet[i]; if (w==io.sizeBufferRX) w=0;
    }
    *io.iwRX=w;
}
static int parseIPD(void) {
    // CIPMUX=0: +IPD,len[,remote IP,remote port]:
    unsigned n=0, port=0; char ip[18]={0}; int consumed=0;
    if (sscanf(line,"+IPD,%u,%17[^,],%u:%n",&n,ip,&port,&consumed)==3 &&
        consumed==(int)lineLen && port<=65535) {
        // Accept quoted and unquoted IPv4 (AT revisions differ).
        if (ip[0]=='"') {
            size_t l=strlen(ip);
            if (l<3 || ip[l-1]!='"') return 0;
            memmove(ip,ip+1,l-2); ip[l-2]=0;
        }
        if (strlen(ip)>15 || !ipv4(ip)) return 0;
        strcpy(packetIP,ip); packetPort=(uint16_t)port;
    } else {
        consumed=0;
        if (sscanf(line,"+IPD,%u:%n",&n,&consumed)!=1 || consumed!=(int)lineLen) return 0;
        packetIP[0]=0; packetPort=0;
    }
    if (!n || n>65535) return 0;
    packetLen=(uint16_t)n; packetPos=0; inPacket=1; return 1;
}
static void receivedLine(void) {
    if (!strcmp(line,"OK")) replyOk=1;
    else if (!strcmp(line,"ERROR") || strstr(line,"FAIL") || !strncmp(line,"busy",4)) replyError=1;
    else if (!strcmp(line,"SEND OK") && txPhase==TX_RESULT) {
        txPhase=TX_IDLE; event(ESP01_SEND_OK);
    } else if (!strcmp(line,"WIFI DISCONNECT") || !strcmp(line,"WIFI DISCONNECTED")) {
        if (mode==ESP01_MODE_STATION && phase==READY) resetModule();
    } else if (!strcmp(line,"CLOSED")) {
        if (phase==READY) resetModule();
    } else if (!strcmp(line,"ready")) {
        if (phase==READY) resetModule();
    } else {
        const char *prefix=mode==ESP01_MODE_SOFTAP ? "+CIFSR:APIP,\"" : "+CIFSR:STAIP,\"";
        if (!strncmp(line,prefix,strlen(prefix))) {
            char ip[16]={0};
            if (sscanf(line+strlen(prefix),"%15[^\"]",ip)==1 && ipv4(ip)) strcpy(localIP,ip);
        }
    }
}
static void parseByte(uint8_t c) {
    rxAt=HAL_GetTick();
    if (inPacket) {
        if (packetPos<sizeof(packet)) packet[packetPos]=c;
        if (++packetPos==packetLen) { inPacket=0; receivedPacket(); }
        return;
    }
    if (c=='>' && txPhase==TX_PROMPT) { gotPrompt=1; lineLen=0; return; }
    if (c=='\r') return;
    if (c=='\n') {
        if (!discardLine && lineLen) { line[lineLen]=0; receivedLine(); }
        lineLen=0; discardLine=0; return;
    }
    if (discardLine) return;
    if (lineLen>=sizeof(line)-1) { discardLine=1; lineLen=0; return; }
    // Prompt often has a trailing space; don't prepend it to SEND OK or +IPD.
    if (lineLen==0 && c==' ') return;
    line[lineLen++]=(char)c; line[lineLen]=0;
    if (c==':' && !strncmp(line,"+IPD,",5)) {
        if (!parseIPD()) discardLine=1;
        lineLen=0;
    }
}
static uint32_t commandTimeout(void) { return step==5 && mode==ESP01_MODE_STATION ? 25000U : 4000U; }
static void prepareStep(void) {
    char s[224];
    switch(step) {
    case 0: queueCommand("AT\r\n"); break;
    case 1: queueCommand("ATE0\r\n"); break;
    case 2: queueCommand(mode==ESP01_MODE_SOFTAP ? "AT+CWMODE=2\r\n" : "AT+CWMODE=1\r\n"); break;
    case 3:
        queueCommand(mode==ESP01_MODE_SOFTAP ? "AT+CIPAP=\"" ESP01_AP_IP "\"\r\n" : "AT+CWAUTOCONN=1\r\n"); break;
    case 4:
        queueCommand(mode==ESP01_MODE_STATION ? "AT+CWDHCP=1,1\r\n" :
                     (legacyDhcp ? "AT+CWDHCP=0,1\r\n" : "AT+CWDHCP=1,2\r\n")); break;
    case 5:
        if (mode==ESP01_MODE_SOFTAP) snprintf(s,sizeof(s),"AT+CWSAP=\"%s\",\"%s\",6,3\r\n",ssid,password);
        else snprintf(s,sizeof(s),"AT+CWJAP=\"%s\",\"%s\"\r\n",ssid,password);
        queueCommand(s); break;
    case 6: queueCommand("AT+CIFSR\r\n"); break;
    case 7: queueCommand("AT+CIPMODE=0\r\n"); break;
    case 8: queueCommand("AT+CIPMUX=0\r\n"); break;
    case 9: queueCommand("AT+CIPDINFO=1\r\n"); break;
    case 10:
        if (tcp) snprintf(s,sizeof(s),"AT+CIPSTART=\"TCP\",\"%s\",%u\r\n",remoteIP,remotePort);
        else snprintf(s,sizeof(s),"AT+CIPSTART=\"UDP\",\"%s\",%u,%u,0\r\n",remoteIP,remotePort,localPort);
        queueCommand(s); break;
    default: break;
    }
}
static void configure(uint32_t now) {
    if (!commandLen) prepareStep();
    if (!waiting) {
        if (flushCommand()) { waiting=1; deadline=now+commandTimeout(); }
        return;
    }
    if (replyError) {
        if (step==4 && mode==ESP01_MODE_SOFTAP && !legacyDhcp) {
            legacyDhcp=1; commandLen=0; waiting=replyError=0; return;
        }
        if (step==3 && mode==ESP01_MODE_STATION) replyOk=1; // optional auto-connect
        else if (step==9 && mode==ESP01_MODE_STATION) { metadata=0; replyOk=1; }
        else { logMessage("ESP AT setup error; restarting\r\n"); resetModule(); return; }
    }
    if (replyOk) {
        if (step==6) {
            if (!localIP[0]) { logMessage("ESP missing IP\r\n"); resetModule(); return; }
            networkUp=1; event(ESP01_WIFI_NEW_IP);
        }
        commandLen=0; waiting=replyOk=replyError=0;
        if (++step>10) {
            socketUp=1; phase=READY; aliveAt=now;
            event(ESP01_UDPTCP_CONNECTED);
            logMessage("ESP UDP ready; waiting for Qt\r\n");
        }
    } else if (expired(now,deadline)) { logMessage("ESP AT setup timeout\r\n"); resetModule(); }
}
static void serviceTx(uint32_t now) {
    if (txPhase==TX_IDLE) return;
    if (replyError || expired(now,deadline)) {
        // ESP may still expect binary payload; only a reset safely restores AT framing.
        logMessage("ESP send failed; resynchronizing\r\n");
        event(ESP01_SEND_ERROR); resetModule(); return;
    }
    if (txPhase==TX_COMMAND) {
        if (flushCommand()) { txPhase=TX_PROMPT; deadline=now+1000U; }
    } else if (txPhase==TX_PROMPT && gotPrompt) {
        txPhase=TX_PAYLOAD; deadline=now+2000U;
    } else if (txPhase==TX_PAYLOAD) {
        unsigned count=0;
        while(payloadPos<payloadLen && count++<TX_BUDGET) {
            if (!io.aWriteUSARTByte(payload[payloadPos])) break;
            ++payloadPos;
        }
        if (payloadPos==payloadLen) { txPhase=TX_RESULT; deadline=now+2000U; }
    }
}
void ESP01_Init(_sESP01Handle *h) {
    memset(&io,0,sizeof(io)); if(h) io=*h;
    notify=NULL; debug=NULL; mode=ESP01_MODE_STATION; phase=OFF; txPhase=TX_IDLE;
    rxW=rxR=0; rxOverflow=0; localIP[0]=remoteIP[0]=peerIP[0]=0;
    ssid=password=NULL; networkUp=socketUp=peerKnown=0;
    commandLen=lineLen=0; waiting=inPacket=discardLine=0;
    tcp=0; localPort=30000; remotePort=30010;
}
void ESP01_SetWIFI(const char *s,const char *p) {
    if (!credentialsValid(s,p,0)) { logMessage("Invalid WiFi credentials\r\n"); return; }
    ssid=s; password=p; mode=ESP01_MODE_STATION; tcp=0; resetModule();
}
void ESP01_SetSoftAP(const char *s,const char *p) {
    if (!credentialsValid(s,p,1)) { logMessage("Invalid SoftAP credentials\r\n"); return; }
    ssid=s; password=p; mode=ESP01_MODE_SOFTAP; tcp=0; resetModule();
}
_eESP01STATUS ESP01_StartUDP(const char *ip,uint16_t rp,uint16_t lp) {
    if (!io.aWriteUSARTByte) return ESP01_NOT_INIT;
    if (!ip || !ipv4(ip) || !rp) return ESP01_SEND_ERROR;
    snprintf(remoteIP,sizeof(remoteIP),"%s",ip); remotePort=rp; localPort=lp ? lp : 30000; tcp=0;
    if (phase==READY || phase==CLOSED) resetModule();
    return ESP01_UDPTCP_CONNECTING;
}
_eESP01STATUS ESP01_StartTCP(const char *ip,uint16_t rp,uint16_t lp) {
    _eESP01STATUS result=ESP01_StartUDP(ip,rp,lp); tcp=1; return result;
}
void ESP01_CloseUDPTCP(void) {
    // Hardware off also cancels any partial CIPSEND safely.
    if(io.aDoCHPD) io.aDoCHPD(0);
    phase=CLOSED; txPhase=TX_IDLE; socketUp=networkUp=0; clearPeer();
    event(ESP01_UDPTCP_DISCONNECTED);
}
_eESP01STATUS ESP01_StateWIFI(void) { return networkUp ? ESP01_WIFI_CONNECTED : ESP01_WIFI_DISCONNECTED; }
_eESP01STATUS ESP01_StateUDPTCP(void) { return socketUp ? ESP01_UDPTCP_CONNECTED : ESP01_UDPTCP_DISCONNECTED; }
char *ESP01_GetLocalIP(void) { return networkUp ? localIP : NULL; }
ESP01_Mode ESP01_GetMode(void) { return mode; }
int ESP01_HasPeer(void) { return peerKnown; }
const char *ESP01_GetPeerIP(void) { return peerIP; }
int ESP01_IsHDRRST(void) { return phase==RESET_LOW || phase==BOOT; }
int ESP01_IsSending(void) { return txPhase!=TX_IDLE; }
void ESP01_AttachChangeState(OnESP01ChangeState f) { notify=f; }
void ESP01_AttachDebugStr(ESP01DebugStr f) { debug=f; }
void ESP01_USB_DbgStr(const char *s) { USB_DebugStr(s); }
void ESP01_Timeout10ms(void) { /* deadlines use HAL_GetTick, independent of call frequency */ }
void ESP01_WriteRX(uint8_t b) {
    uint16_t next=(rxW+1U)%ESP01RXBUFAT;
    if(next==rxR) { rxOverflow=1; return; }
    rx[rxW]=b; rxW=next;
}
_eESP01STATUS ESP01_Send(uint8_t *buf,uint16_t offset,uint16_t len,uint16_t ringSize) {
    if (!socketUp || (mode==ESP01_MODE_SOFTAP && !peerKnown)) return ESP01_UDPTCP_DISCONNECTED;
    if (txPhase!=TX_IDLE) return ESP01_SEND_BUSY;
    if (!buf || !len || len>sizeof(payload) || !ringSize || offset>=ringSize || len>ringSize) return ESP01_SEND_ERROR;
    char s[96];
    if (peerKnown && !tcp && metadata)
        snprintf(s,sizeof(s),"AT+CIPSEND=%u,\"%s\",%u\r\n",len,peerIP,peerPort);
    else snprintf(s,sizeof(s),"AT+CIPSEND=%u\r\n",len);
    queueCommand(s);
    for(uint16_t i=0;i<len;++i) payload[i]=buf[(offset+i)%ringSize];
    payloadLen=len; payloadPos=0; txPhase=TX_COMMAND; deadline=HAL_GetTick()+2000U;
    return ESP01_SEND_READY;
}
void ESP01_Task(void) {
    if (phase==OFF || phase==CLOSED || !io.aWriteUSARTByte) return;
    uint32_t now=HAL_GetTick();
    if (phase==RESET_LOW) {
        rxR=rxW;
        if(expired(now,deadline)) { if(io.aDoCHPD) io.aDoCHPD(1); phase=BOOT; deadline=now+1800U; }
        return;
    }
    if (phase==BOOT) {
        rxR=rxW;
        if(expired(now,deadline)) { phase=CONFIGURE; commandLen=0; }
        return;
    }
    if(rxOverflow) { logMessage("ESP RX overflow\r\n"); resetModule(); return; }
    if ((inPacket || lineLen || discardLine) && now-rxAt>1000U) {
        logMessage("ESP partial RX timeout\r\n"); resetModule(); return;
    }
    unsigned count=0;
    while(rxR!=rxW && count++<RX_BUDGET) {
        uint8_t b=rx[rxR]; rxR=(rxR+1U)%ESP01RXBUFAT; parseByte(b);
        if(phase==RESET_LOW) return;
    }
    if(phase==CONFIGURE) { configure(now); return; }
    if(phase!=READY) return;
    if(peerKnown && now-peerAt>=PEER_LEASE_MS) clearPeer();
    serviceTx(now);
    if(phase!=READY || txPhase!=TX_IDLE) return;
    if(discoveryReply) {
        char reply[64];
        int n=snprintf(reply,sizeof(reply),"BALANCIN_V1,%u,%s",(unsigned)mode,localIP);
        if (ESP01_Send((uint8_t*)reply,0,(uint16_t)n,sizeof(reply))==ESP01_SEND_READY) discoveryReply=0;
    } else if(now-aliveAt>=5000U && (mode==ESP01_MODE_STATION || peerKnown)) {
        aliveAt=now; UNER_SendAlive();
    }
}
