/*
 * ESP01.c
 *
 *  Created on: July 31, 2025
 *      Author: Tadeo Mendelevich
 */

#include "ESP01.h"
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "UNER.h"

extern void USB_Debug(const char *fmt, ...);

#define SERVER_IP    		"192.168.1.52"	// Cambiar IP correspondiente al wifi
#define SERVER_PORT  		30010
#define LOCAL_PORT   		30000
#define ALIVE_INTERVAL_FAST_MS 	5000
#define ALIVE_INTERVAL_SLOW_MS 	10000
#define ALIVE_FAST_COUNT 		20

static uint8_t alive_counter = 0;

static enum {
	ESP01ATIDLE,
	ESP01ATAT,
	ESP01ATRESPONSE,
	ESP01ATCIPMUX,
	ESP01ATCWMODE_SET,
	ESP01ATCWMODE_RESPONSE,
	ESP01ATCWDHCP_SET,
	ESP01ATCWDHCP_RESPONSE,
	ESP01ATCWJAP,
	ESP01CWJAPRESPONSE,
	ESP01ATCIFSR,
	ESP01CIFSRRESPONSE,
	ESP01ATCIPCLOSE,
	ESP01ATCIPSTART,
	ESP01CIPSTARTRESPONSE,
	ESP01ATPREPUDP,
	ESP01ATCONNECTED,
	ESP01ATHARDRST0,
	ESP01ATHARDRST1,
	ESP01ATHARDRSTSTOP,
	ESP01ATRECONNECT,
} esp01ATSate = ESP01ATIDLE;

static union{
	struct{
		uint8_t WAITINGSYMBOL: 1;
		uint8_t WIFICONNECTED: 1;
		uint8_t TXCIPSEND: 1;
		uint8_t SENDINGDATA: 1;
		uint8_t HRDRESETON: 1;
		uint8_t ATRESPONSEOK: 1;
		uint8_t UDPTCPCONNECTED: 1;
		uint8_t WAITINGRESPONSE: 1;
	}bit;
	uint8_t byte;
} esp01Flags;

static void ESP01ATDecode();
static void ESP01DOConnection();
static void ESP01_InternalStateChange(_eESP01STATUS state);
static void ESP01_NotifyStateChange(_eESP01STATUS state);
static void ESP01SENDData();
static void ESP01StrToBufTX(const char *str);
static void ESP01ByteToBufTX(uint8_t value);

static uint32_t esp01TimeoutTask = 0;
static uint32_t esp01TimeoutDataRx = 0;
static uint32_t esp01TimeoutTxSymbol = 0;
static uint32_t esp01TimeoutSending = 0;
static OnESP01ChangeState aESP01ChangeState = NULL;
static ESP01DebugStr aDbgStr = NULL;

static const char *esp01SSID = NULL;
static const char *esp01PASSWORD = NULL;
static const char *esp01RemoteIP = NULL;
//static char esp01PROTO[4] = "TCP";
static char esp01PROTO[4] = "UDP";
static char esp01RemotePORT[6] = {0};
static char esp01LocalIP[16] = {0};
static char esp01LocalPORT[6] = {0};

static uint8_t esp01HState = 0;
static uint16_t	esp01nBytes = 0;
static uint8_t	esp01RXATBuf[ESP01RXBUFAT];
static uint8_t	esp01TXATBuf[ESP01TXBUFAT];
static uint16_t	esp01iwRXAT = 0;
static uint16_t	esp01irRXAT = 0;
static uint16_t esp01irTX = 0;
static uint16_t esp01iwTX = 0;

static uint8_t esp01TriesAT = 0;

static _sESP01Handle esp01Handle = {.aDoCHPD = NULL, .aWriteUSARTByte = NULL,
									.bufRX = NULL, .iwRX = NULL, .sizeBufferRX = 0};

const char ATAT[] = "AT\r\n";
const char ATCIPMUX[] = "AT+CIPMUX=1\r\n";
const char ATCWQAP[] = "AT+CWQAP\r\n";
const char ATCWMODE[] = "AT+CWMODE=3\r\n";
const char ATCWJAP[] = "AT+CWJAP=";
const char ATCIFSR[] = "AT+CIFSR\r\n";
const char ATCIPSTART[] = "AT+CIPSTART=";
//const char ATCIPCLOSE[] = "AT+CIPCLOSE\r\n";
const char ATCIPCLOSE[] = "AT+CIPCLOSE=0\r\n";
const char ATCIPSEND[] = "AT+CIPSEND=";

const char respAT[] = "0302AT\r";
const char respATp[] = "0302AT+";
const char respOK[] = "0402OK\r\n";
const char respERROR[] = "0702ERROR\r\n";
const char respWIFIGOTIP[] = "1302WIFI GOT IP\r\n";
const char respWIFICONNECTED[] = "1602WIFI CONNECTED\r\n";
const char respWIFIDISCONNECT[] = "1702WIFI DISCONNECT\r\n";
const char respWIFIDISCONNECTED[] = "1902WIFI DISCONNECTED\r\n";
const char respDISCONNECTED[] = "1402DISCONNECTED\r\n";
const char respSENDOK[] = "0902SEND OK\r\n";
const char respCONNECT[] = "0902CONNECT\r\n";
const char respCLOSED[] = "0802CLOSED\r\n";
const char respCIFSRAPIP[] = "1205+CIFSR:STAIP";
const char respBUSY[] = "0602busy .";
const char respIPD[] = "0410+IPD";
const char respReady[] = "0702ready\r\n";
const char respBUSYP[] = "0602busy p";
const char respBUSYS[] = "0602busy s";
const char respFAIL[]  = "0602FAIL\r\n";
// 	  const char respCIFSRAPIP[] = "1102+CIFSR:APIP";
//    const char respCIFSRAPMAC[] = "1202+CIFSR:APMAC";
//    const char respCIFSRSTAIP[] = "1205+CIFSR:STAIP";
//    const char respCIFSRSTAMAC[] = "1302+CIFSR:STAMAC";

const char *const responses[] = {respAT, respATp, respOK, respERROR, respWIFIGOTIP, respWIFICONNECTED,
								 respWIFIDISCONNECT, respWIFIDISCONNECTED, respDISCONNECTED, respSENDOK, respCONNECT, respCLOSED,
								 respCIFSRAPIP, respBUSY, respIPD, respReady, respBUSYP, respBUSYS, respFAIL, NULL};

static uint8_t indexResponse = 0;
static uint8_t indexResponseChar = 0;

static uint8_t tryingTCP = 0;
static uint8_t waitingForTCPClient = 0;
static uint16_t lastIPDlen = 0;  // guarda la longitud del +IPD

//static uint8_t tcpServerStarted = 0;
//static uint8_t configUDPObtenida = 0; // Unused
static uint8_t udpIniciado = 0;
static uint32_t lastAliveTick = 0;

extern void UNER_SendAlive(void);

//const char _DNSFAIL[] = "DNS FAIL\r";
//const char _ATCIPDNS[] = "AT+CIPDNS_CUR=1,\"208.67.220.220\",\"8.8.8.8\"\r\n";
//const char CIFSRAPIP[] = "+CIFSR:APIP\r";
//const char CIFSRAPMAC[] = "+CIFSR:APMAC\r";
//const char CIFSRSTAIP[] = "+CIFSR:STAIP\r";
//const char CIFSRSTAMAC[] = "+CIFSR:STAMAC\r";

void ESP01_SetWIFI(const char *ssid, const char *password){
	esp01ATSate = ESP01ATIDLE;
	esp01Flags.byte = 0;

	esp01SSID = ssid;
	esp01PASSWORD = password;

	esp01TimeoutTask = 50;
	esp01ATSate = ESP01ATHARDRST0;
	esp01TriesAT = 0;
}


_eESP01STATUS ESP01_StartUDP(const char *RemoteIP, uint16_t RemotePORT, uint16_t LocalPORT){
	char buf[100];
	sprintf(buf,
		">> ESP01_StartUDP params: IP=%s, RemotePort=%u, LocalPort=%u\r\n",
		RemoteIP, (unsigned)RemotePORT, (unsigned)LocalPORT);
	aDbgStr(buf);


	if(esp01Handle.aWriteUSARTByte == NULL)
		return ESP01_NOT_INIT;

	if(LocalPORT == 0)
		LocalPORT = 30000;

	strcpy(esp01PROTO, "UDP");
	udpIniciado      = 0;

	// <<< AQUÍ guardo el puntero a la IP PASADA desde main.c >>>
	esp01RemoteIP = RemoteIP;

	itoa(RemotePORT, esp01RemotePORT, 10);
	itoa(LocalPORT, esp01LocalPORT, 10);

	if(esp01SSID == NULL)
		return ESP01_WIFI_NOT_SETED;

	if(esp01Flags.bit.WIFICONNECTED == 0)
		return ESP01_WIFI_DISCONNECTED;

	esp01ATSate = ESP01ATPREPUDP;

	return ESP01_UDPTCP_CONNECTING;
}

_eESP01STATUS ESP01_StartTCP(const char *RemoteIP, uint16_t RemotePORT, uint16_t LocalPORT){
	if(esp01Handle.aWriteUSARTByte == NULL)
		return ESP01_NOT_INIT;

	if(LocalPORT == 0)
		LocalPORT = 30000;

	strcpy(esp01PROTO, "TCP");

	esp01RemoteIP = RemoteIP;

	itoa(RemotePORT, esp01RemotePORT, 10);
	itoa(LocalPORT, esp01LocalPORT, 10);

	if(esp01SSID == NULL)
		return ESP01_WIFI_NOT_SETED;

	if(esp01Flags.bit.WIFICONNECTED == 0)
		return ESP01_WIFI_DISCONNECTED;

	esp01ATSate = ESP01ATCIPCLOSE;

	return ESP01_UDPTCP_CONNECTING;
}


void ESP01_CloseUDPTCP(){
	if(esp01Handle.aWriteUSARTByte == NULL)
		return;

	esp01ATSate = ESP01ATCIPCLOSE;
}

_eESP01STATUS ESP01_StateWIFI(){
	if(esp01Handle.aWriteUSARTByte == NULL)
		return ESP01_NOT_INIT;

	if(esp01Flags.bit.WIFICONNECTED)
		return ESP01_WIFI_CONNECTED;
	else
		return ESP01_WIFI_DISCONNECTED;
}

char *ESP01_GetLocalIP(){
	if(esp01Flags.bit.WIFICONNECTED &&  esp01LocalIP[0]!='\0')
		return esp01LocalIP;

	return NULL;
}


_eESP01STATUS ESP01_StateUDPTCP(){
	if(esp01Handle.aWriteUSARTByte == NULL)
		return ESP01_NOT_INIT;

	if(esp01Flags.bit.UDPTCPCONNECTED)
		return ESP01_UDPTCP_CONNECTED;
	else
		return ESP01_UDPTCP_DISCONNECTED;
}


void ESP01_WriteRX(uint8_t value){
	if(esp01Handle.bufRX == NULL)
		return;
	esp01RXATBuf[esp01iwRXAT++] = value;
	if(esp01iwRXAT == ESP01RXBUFAT)
		esp01iwRXAT = 0;
}

_eESP01STATUS ESP01_Send(uint8_t *buf, uint16_t irRingBuf, uint16_t length, uint16_t sizeRingBuf){
	if(esp01Handle.aWriteUSARTByte == NULL)
		return ESP01_NOT_INIT;

	if(esp01Flags.bit.UDPTCPCONNECTED == 0)
		return ESP01_UDPTCP_DISCONNECTED;

	if(esp01Flags.bit.SENDINGDATA == 0){
		char strInt[10];
		uint8_t l = 0;

		itoa(length, strInt, 10);
		l = strlen(strInt);
		if(l>4 || l==0)
			return ESP01_SEND_ERROR;

		ESP01StrToBufTX(ATCIPSEND);
		ESP01StrToBufTX(strInt);
		ESP01StrToBufTX("\r\n");


		for(uint16_t i=0; i<length; i++){
			esp01TXATBuf[esp01iwTX++] = buf[irRingBuf++];
			if(esp01iwTX == ESP01TXBUFAT)
				esp01iwTX = 0;
			if(irRingBuf == sizeRingBuf)
				irRingBuf = 0;
		}

		esp01Flags.bit.TXCIPSEND = 1;
		esp01Flags.bit.SENDINGDATA = 1;

		if(aDbgStr != NULL){
			aDbgStr("+&DBGSENDING DATA ");
			aDbgStr(strInt);
			aDbgStr("\n");
		}


		return ESP01_SEND_READY;
	}

	if(aDbgStr != NULL)
		aDbgStr("+&DBGSENDING DATA BUSY\n");

	return ESP01_SEND_BUSY;
}


void ESP01_Init(_sESP01Handle *hESP01){

	memcpy(&esp01Handle, hESP01, sizeof(_sESP01Handle));

	esp01ATSate = ESP01ATIDLE;
	esp01HState = 0;
	esp01irTX = 0;
	esp01iwTX = 0;
	esp01irRXAT = 0;
	esp01iwRXAT = 0;
	esp01Flags.byte = 0;
}


void ESP01_Timeout10ms(){
	if(esp01TimeoutTask)
		esp01TimeoutTask--;

	if(esp01TimeoutDataRx){
		esp01TimeoutDataRx--;
		if(!esp01TimeoutDataRx)
			esp01HState = 0;
	}

	if(esp01TimeoutTxSymbol)
		esp01TimeoutTxSymbol--;

	if(esp01TimeoutSending) {
		esp01TimeoutSending--;
		if(!esp01TimeoutSending && esp01Flags.bit.SENDINGDATA) {
			esp01Flags.bit.SENDINGDATA = 0;
			if(aDbgStr) aDbgStr(">>> TIMEOUT: SENDINGDATA cleared forcedly\r\n");
		}
	}
}

void ESP01_Task(){

	if(esp01irRXAT != esp01iwRXAT)
		ESP01ATDecode();

	if(!esp01TimeoutTask)
		ESP01DOConnection();

	ESP01SENDData();

	// ——— Alive periódico UDP ———
	if (strcmp(esp01PROTO, "UDP")==0 && esp01Flags.bit.UDPTCPCONNECTED) {
	    uint32_t now = HAL_GetTick();
		uint32_t current_interval = (alive_counter < ALIVE_FAST_COUNT) ? ALIVE_INTERVAL_FAST_MS : ALIVE_INTERVAL_SLOW_MS;

	    if ((now - lastAliveTick) >= current_interval) {
	        // sólo enviamos si no estamos ya en medio de un envío
	        // y si no hay bytes +IPD pendientes de procesar
	        if (!esp01Flags.bit.SENDINGDATA
	            && esp01irRXAT == esp01iwRXAT) {
	            lastAliveTick = now;
	            UNER_SendAlive();
				if (alive_counter < 255) { // Evitar desbordamiento
					alive_counter++;
				}
	        }
	    }
	}
}

void ESP01_AttachChangeState(OnESP01ChangeState aOnESP01ChangeState){
	aESP01ChangeState = aOnESP01ChangeState;
}

void ESP01_AttachDebugStr(ESP01DebugStr aDbgStrPtrFun){
	aDbgStr = aDbgStrPtrFun;
}

int ESP01_IsHDRRST(){
	if(esp01ATSate==ESP01ATHARDRST0 || esp01ATSate==ESP01ATHARDRST1 || esp01ATSate==ESP01ATHARDRSTSTOP)
		return 1;
	return 0;
}


/* Private Functions */
static void ESP01ATDecode(){
	uint16_t i;
	uint8_t value;


	static uint16_t recibidos;


	if(esp01ATSate==ESP01ATHARDRST0 || esp01ATSate==ESP01ATHARDRST1 ||
	   esp01ATSate==ESP01ATHARDRSTSTOP){
		esp01irRXAT = esp01iwRXAT;
		return;
	}


	i = esp01iwRXAT;
	esp01TimeoutDataRx = 2;
	while(esp01irRXAT != i){
		value = esp01RXATBuf[esp01irRXAT];

	    // DEBUG: muestro cada byte RX en hex
	    /*if(aDbgStr){
	        char dbg[8];
	        snprintf(dbg, sizeof(dbg), "%02X ", value);
	        aDbgStr(dbg);
	    }*/


		switch(esp01HState){
		case 0:
            indexResponse = 0;
            indexResponseChar = 4;
            while(responses[indexResponse] != NULL){
                if(value == responses[indexResponse][indexResponseChar]){
                    esp01nBytes = (responses[indexResponse][0] - '0');
                    esp01nBytes *= 10;
                    esp01nBytes += (responses[indexResponse][1] - '0');
                    esp01nBytes--;
                    break;
                }
                indexResponse++;
            }
            if(responses[indexResponse] != NULL){
                esp01HState = 1;
                indexResponseChar++;
            }
			else{
				esp01TimeoutDataRx = 0;
				if(esp01Flags.bit.WAITINGSYMBOL){
					if(value == '>'){
						esp01Flags.bit.WAITINGSYMBOL = 0;
						esp01TimeoutTxSymbol = 0;
					}
				}
			}
			break;
		case 1:
            if(value == responses[indexResponse][indexResponseChar]){
                esp01nBytes--;
                if(!esp01nBytes || value=='\r'){
                    esp01HState = (responses[indexResponse][2] - '0');
                    esp01HState *= 10;
                    esp01HState += (responses[indexResponse][3] - '0');
                    break;
                }
            }
            else{
                indexResponse = 0;
                while(responses[indexResponse] != NULL){
                    esp01nBytes = (responses[indexResponse][0] - '0');
                    esp01nBytes *= 10;
                    esp01nBytes += (responses[indexResponse][1] - '0');
                    esp01nBytes -= (indexResponseChar-3);
                    if(esp01nBytes<128 && value==responses[indexResponse][indexResponseChar]){
                        if(esp01nBytes == 0){
                            esp01HState = (responses[indexResponse][2] - '0');
                            esp01HState *= 10;
                            esp01HState += (responses[indexResponse][3] - '0');
                        }
                        break;
                    }
                    indexResponse++;
                }
                if(responses[indexResponse] == NULL){
                    esp01HState = 0;
                    esp01irRXAT--;
                    break;
                }
            }
			indexResponseChar++;
			break;
		case 2:
		    if(value == '\n'){
		        esp01HState = 0;
		        /*if (aDbgStr) {
					char dbgBuf[64];
					sprintf(dbgBuf, "DEBUG: RECIBI OK - estado actual = %d\n", esp01ATSate);
					aDbgStr(dbgBuf);
				}*/
		        switch(indexResponse){
		        case 0://AT
		        case 1:
		            break;
		        case 2: // OK
		            if (   esp01ATSate == ESP01ATRESPONSE
		                || esp01ATSate == ESP01ATCWMODE_RESPONSE
		                || esp01ATSate == ESP01ATCWDHCP_RESPONSE
		                || esp01ATSate == ESP01CWJAPRESPONSE ) {
		                aDbgStr(">>> DEBUG: marcando ATRESPONSEOK = 1\n");
		                esp01TimeoutTask = 0;
		                esp01Flags.bit.ATRESPONSEOK = 1;
		            }
		            else if (esp01ATSate == ESP01CIPSTARTRESPONSE && strcmp(esp01PROTO, "UDP") == 0) {
		                esp01Flags.bit.ATRESPONSEOK = 1;
		                ESP01_NotifyStateChange(ESP01_UDPTCP_CONNECTED);
		            }
		            break;

				case 3://ERROR
					if(esp01Flags.bit.SENDINGDATA){
						esp01Flags.bit.SENDINGDATA = 0;
						esp01Flags.bit.UDPTCPCONNECTED = 0;
						esp01irTX = esp01iwTX;
					}
					break;
				case 4://WIFI GOT IP
					esp01TimeoutTask = 0;
					if(esp01ATSate == ESP01CWJAPRESPONSE)
						esp01Flags.bit.ATRESPONSEOK = 1;
					esp01Flags.bit.WIFICONNECTED = 1;
					ESP01_NotifyStateChange(ESP01_WIFI_CONNECTED);
					break;
				case 5://WIFI CONNECTED
					break;
				case 6://WIFI DISCONNECT
				case 7://WIFI DISCONNECTED
					esp01Flags.bit.UDPTCPCONNECTED = 0;
					esp01Flags.bit.WIFICONNECTED = 0;
					ESP01_NotifyStateChange(ESP01_WIFI_DISCONNECTED);
					ESP01_NotifyStateChange(ESP01_WIFI_RECONNECTING);
					if(esp01ATSate != ESP01CWJAPRESPONSE) {
						esp01ATSate = ESP01ATRECONNECT;
					}
					break;
				case 8://DISCONNECTED
					esp01Flags.bit.UDPTCPCONNECTED = 0;
					break;
				case 9://SEND OK
					esp01Flags.bit.SENDINGDATA = 0;
					ESP01_NotifyStateChange(ESP01_SEND_OK);
					break;
				case 10://CONNECT
					if (waitingForTCPClient) {
						waitingForTCPClient = 0;
						ESP01_USB_DbgStr(">>> Cliente TCP conectado\r\n");
					}
					esp01TimeoutTask = 0;
					esp01Flags.bit.ATRESPONSEOK = 1;
					esp01Flags.bit.UDPTCPCONNECTED = 1;
					ESP01_NotifyStateChange(ESP01_UDPTCP_CONNECTED);
					break;
				case 11://CLOSED
					esp01Flags.bit.UDPTCPCONNECTED = 0;
					break;
				case 13://busy
					esp01Flags.bit.UDPTCPCONNECTED = 0;
					esp01Flags.bit.WIFICONNECTED = 0;
					break;
				case 15://ready
					esp01Flags.bit.UDPTCPCONNECTED = 0;
					esp01Flags.bit.WIFICONNECTED = 0;
					esp01ATSate = ESP01ATHARDRSTSTOP;
					break;
				case 16://busy p
					break;
				case 17://busy s
					break;
				case 18://FAIL
				    {
				        aDbgStr(">>> DEBUG: FAIL received\r\n");
				        if (esp01ATSate == ESP01CWJAPRESPONSE) {
				             esp01ATSate = ESP01ATAT;
				             esp01TimeoutTask = 0;
				        }
				        // Asegurar que se marque como desconectado
				        esp01Flags.bit.WIFICONNECTED = 0;
				        esp01Flags.bit.UDPTCPCONNECTED = 0;
				        ESP01_NotifyStateChange(ESP01_WIFI_DISCONNECTED);
				    }
				    break;
				}
			}
			break;
		case 5://CIFR,STAIP
			if(value == ','){
				esp01HState = 6;
				if(aDbgStr != NULL)
					aDbgStr("+&DBGRESPONSE CIFSR\n");
			}
			else{
				esp01HState = 0;
				esp01irRXAT--;
				if(aDbgStr != NULL)
					aDbgStr("+&DBGERROR CIFSR 5\n");
			}
			break;
		case 6:
			if(value == '\"'){
				esp01HState = 7;
				esp01nBytes = 0;
			}
			break;
		case 7:
			if(value == '\"' || esp01nBytes==16)
				esp01HState = 8;
			else
				esp01LocalIP[esp01nBytes++] = value;
			break;
		case 8:
			if(value == '\n'){
				esp01HState = 0;
				if(esp01nBytes < 16){
					esp01LocalIP[esp01nBytes] = '\0';
					esp01Flags.bit.ATRESPONSEOK = 1;
					esp01TimeoutTask = 0;
				}
				else
					esp01LocalIP[0] = '\0';
				ESP01_NotifyStateChange(ESP01_WIFI_NEW_IP);
			}
			break;
		case 10:  // acabamos de detectar "+IPD"
		    esp01HState   = 11;    // pasamos a parsear longitud (y saltar linkId si existe)
		    esp01nBytes   = 0;     // reiniciar contador de longitud
		    recibidos     = 0;     // reiniciar contador de payload (para debug)
		    break;

		case 11:  // salto opcional de linkId y parseo de longitud hasta ':'
		    if (value == ',') {
		        // si es la primera coma, puede ser el separador linkId→longitud o linkId único
		        // simplemente ignoramos y seguimos aquí
		    }
		    else if (value >= '0' && value <= '9') {
		        // acumulamos dígito de longitud
		        esp01nBytes = esp01nBytes * 10 + (value - '0');
		    }
		    else if (value == ':') {
		        // fin del número de bytes, arrancamos la lectura de payload
		        lastIPDlen  = esp01nBytes;
		        esp01HState = 12;
		    }
		    else {
		        // cualquier otro carácter inesperado nos saca al estado base
		        esp01HState = 0;
		    }
		    break;

		case 12:  // leemos byte a byte el payload
		    // 1) lo metemos en el buffer circular
		    esp01Handle.bufRX[*esp01Handle.iwRX] = value;
		    (*esp01Handle.iwRX)++;
		    if (*esp01Handle.iwRX == esp01Handle.sizeBufferRX)
		        *esp01Handle.iwRX = 0;

		    // 2) Debug al inicio del payload
		    if (recibidos == 0) {
		        USB_Debug(">> +IPD payload begin, len=%u\r\n", esp01nBytes);
		    }
		    recibidos++;

		    // 3) decrementamos contador
		    esp01nBytes--;
		    if (esp01nBytes == 0) {
		        // fin de payload:
		        esp01HState = 0;
		        esp01Flags.bit.WAITINGSYMBOL = 0;
		        esp01Flags.bit.TXCIPSEND     = 0;
		        esp01Flags.bit.SENDINGDATA   = 0;

		        // 4) imprimimos en debug los lastIPDlen bytes recibidos
		        uint16_t start = (*esp01Handle.iwRX + esp01Handle.sizeBufferRX - lastIPDlen)
		                         % esp01Handle.sizeBufferRX;
		        for (uint16_t i = 0; i < lastIPDlen; ++i) {
		            uint8_t c = esp01Handle.bufRX[(start + i) % esp01Handle.sizeBufferRX];
		            char s[2] = { c, '\0' };
		            if (aDbgStr) aDbgStr(s);
		        }
		        if (aDbgStr) aDbgStr("\r\n");

		        // 5) El código para reconfigurar por UDP ha sido eliminado
		        //    para mantener main.c como única fuente de configuración.
		    }
		    break;

		default:
			esp01HState = 0;
			esp01TimeoutDataRx = 0;
		}

		esp01irRXAT++;
		if(esp01irRXAT == ESP01RXBUFAT)
			esp01irRXAT = 0;
	}

}

static void ESP01DOConnection(){
	/*if (aDbgStr) {
		char buf[100];
		sprintf(buf,
			">> DOConnection: state=%d, configUDP=%d, udpIniciado=%d, PROTO=%s\r\n",
			esp01ATSate, configUDPObtenida, udpIniciado, esp01PROTO);
		aDbgStr(buf);
	}*/

	// 1) Si ya tengo la config pendiente y no he lanzado UDP
	/*if (configUDPObtenida && !udpIniciado && strcmp(esp01PROTO, "UDP")==0) {
		// deshabilito el servidor TCP y cierro el socket
		ESP01StrToBufTX("AT+CIPSERVER=0,"); ESP01StrToBufTX(esp01LocalPORT); ESP01StrToBufTX("\r\n");
		ESP01StrToBufTX("AT+CIPCLOSE=0\r\n");

		// apago multiplexado
		ESP01StrToBufTX("AT+CIPMUX=0\r\n");

		// lanzo el socket UDP con la IP ya intacta en esp01RemoteIP
		ESP01StrToBufTX("AT+CIPSTART=\"UDP\",\"");
		ESP01StrToBufTX(esp01RemoteIP);
		ESP01StrToBufTX("\",");
		ESP01StrToBufTX(esp01RemotePORT);
		ESP01StrToBufTX(",");
		ESP01StrToBufTX(esp01LocalPORT);
		ESP01StrToBufTX(",0\r\n");

		udpIniciado      = 1;
		esp01ATSate      = ESP01CIPSTARTRESPONSE;
		esp01TimeoutTask = 3000;
		return;
	}*/


    // 2a) Si antes estábamos en ATCONNECTED pero ahora UDPTCPCONNECTED o WIFICONNECTED cayeron, relanzamos la conexión.
    if (esp01ATSate == ESP01ATCONNECTED && (!esp01Flags.bit.UDPTCPCONNECTED || !esp01Flags.bit.WIFICONNECTED)) {
        aDbgStr(">>> Conexión perdida, reintentando...\r\n");
        esp01ATSate     = ESP01ATRECONNECT;
        esp01TimeoutTask = 0; // Iniciar reconexión inmediatamente
        ESP01_NotifyStateChange(ESP01_WIFI_RECONNECTING);
        return;
    }
    // 2b) Si seguimos vivos, sólo esperamos antes de chequear de nuevo
    if (esp01ATSate == ESP01ATCONNECTED && esp01Flags.bit.UDPTCPCONNECTED) {
        esp01TimeoutTask = 500; // Chequeo cada 5 segundos
        return;
    }


	esp01TimeoutTask = 100;
	switch(esp01ATSate){
	case ESP01ATIDLE:
		esp01TimeoutTask = 0;
		break;
	case ESP01ATRECONNECT:
		if(aDbgStr != NULL) aDbgStr("+&DBGRECONNECTING...\n");
		esp01ATSate = ESP01ATCWJAP; // Intentar reconectar al WiFi
		esp01TimeoutTask = 100; // Dar un tiempo antes de reintentar
		break;
	case ESP01ATHARDRST0:
		esp01Handle.aDoCHPD(0);
		if(aDbgStr != NULL)
			aDbgStr("+&DBGESP01HARDRESET0\n");
		esp01ATSate = ESP01ATHARDRST1;
		break;
	case ESP01ATHARDRST1:
		esp01Handle.aDoCHPD(1);
		if(aDbgStr != NULL)
			aDbgStr("+&DBGESP01HARDRESET1\n");
		esp01ATSate = ESP01ATHARDRSTSTOP;
		esp01TimeoutTask = 500;
		break;
	case ESP01ATHARDRSTSTOP:
		esp01ATSate = ESP01ATAT;
		esp01TriesAT = 0;
		break;
	case ESP01ATAT:
		if(esp01TriesAT){
			esp01TriesAT--;
			if(!esp01TriesAT){
				esp01ATSate = ESP01ATHARDRST0;
				break;
			}
		}
		else
			esp01TriesAT = 4;

		esp01Flags.bit.ATRESPONSEOK = 0;
		aDbgStr(">>> Cambio de estado a ESP01ATAT: lanzando AT\n");
		ESP01StrToBufTX(ATAT);
		if(aDbgStr != NULL)
			aDbgStr("+&DBGESP01AT\n");
		esp01ATSate = ESP01ATRESPONSE;
		break;
	case ESP01ATRESPONSE:
		if(esp01Flags.bit.ATRESPONSEOK)
			esp01ATSate = ESP01ATCWMODE_SET;
		else
			esp01ATSate = ESP01ATAT;
		break;
	case ESP01ATCWMODE_SET:
	    ESP01StrToBufTX("AT+CWMODE=1\r\n");
	    esp01Flags.bit.ATRESPONSEOK = 0;
	    esp01ATSate = ESP01ATCWMODE_RESPONSE;
	    break;

	case ESP01ATCWMODE_RESPONSE:
	    if (esp01Flags.bit.ATRESPONSEOK) {
	        esp01ATSate = ESP01ATCWDHCP_SET;
	    }
	    break;

	case ESP01ATCWDHCP_SET:
	    ESP01StrToBufTX("AT+CWDHCP=1,1\r\n");
	    esp01Flags.bit.ATRESPONSEOK = 0;
	    esp01ATSate = ESP01ATCWDHCP_RESPONSE;
	    break;

	case ESP01ATCWDHCP_RESPONSE:
	    if (esp01Flags.bit.ATRESPONSEOK) {
	        esp01ATSate = ESP01ATCWJAP;
	    }
	    break;

	case ESP01ATCIPMUX:
	    if (strcmp(esp01PROTO, "TCP") == 0) {
	        // multiplicado y server en TCP
	        ESP01StrToBufTX("AT+CIPMUX=1\r\n");
	        ESP01StrToBufTX("AT+CIPSERVER=1,80\r\n");
	        aDbgStr("+&DBGESP01ATCIPMUX+SERVER\n");
	        esp01ATSate = ESP01ATCIFSR;        // sigue con CIFSR en TCP
	    } else {
	        // UDP: multiplex = 0, NO server
	        ESP01StrToBufTX("AT+CIPMUX=0\r\n");
	        aDbgStr("+&DBGESP01ATCIPMUX(single)\n");
	        esp01ATSate = ESP01ATPREPUDP;
	    }
	    break;
	case ESP01ATPREPUDP:
	    ESP01StrToBufTX("AT+CIPSERVER=0\r\n");
	    //ESP01StrToBufTX("AT+CIPCLOSE=0\r\n");
	    ESP01StrToBufTX(ATCIPCLOSE);
	    ESP01StrToBufTX("AT+CIPMUX=0\r\n");
	    ESP01StrToBufTX("AT+CIPSTART=\"UDP\",\"");
	    ESP01StrToBufTX(esp01RemoteIP);
	    ESP01StrToBufTX("\",");
	    ESP01StrToBufTX(esp01RemotePORT);
	    ESP01StrToBufTX(",");
	    ESP01StrToBufTX(esp01LocalPORT);
	    ESP01StrToBufTX(",0\r\n");
	    esp01ATSate      = ESP01CIPSTARTRESPONSE;
	    esp01TimeoutTask = 3000;
	    break;
	case ESP01ATCWJAP:
		if(esp01Flags.bit.WIFICONNECTED){
			esp01ATSate = ESP01ATCIFSR;
			break;
		}
		if(esp01SSID == NULL)
			break;
		ESP01StrToBufTX(ATCWJAP);
		ESP01ByteToBufTX('\"');
		ESP01StrToBufTX(esp01SSID);
		ESP01ByteToBufTX('\"');
		ESP01ByteToBufTX(',');
		ESP01ByteToBufTX('\"');
		ESP01StrToBufTX(esp01PASSWORD);
		ESP01ByteToBufTX('\"');
		ESP01ByteToBufTX('\r');
		ESP01ByteToBufTX('\n');
		if(aDbgStr != NULL)
			aDbgStr("+&DBGESP01ATCWJAP\n");
		esp01Flags.bit.ATRESPONSEOK = 0;
		esp01ATSate = ESP01CWJAPRESPONSE;
		esp01TimeoutTask = 2000;
		break;
	case ESP01CWJAPRESPONSE:
		if(esp01Flags.bit.ATRESPONSEOK){
			esp01ATSate = ESP01ATCIFSR;
			esp01TriesAT = 4;
		}
		else
			esp01ATSate = ESP01ATAT;
		break;
	case ESP01ATCIFSR:
		esp01LocalIP[0] = '\0';
		ESP01StrToBufTX(ATCIFSR);
		if(aDbgStr != NULL)
			aDbgStr("+&DBGESP01CIFSR\n");
		esp01Flags.bit.ATRESPONSEOK = 0;
		esp01ATSate = ESP01CIFSRRESPONSE;
		break;
	case ESP01CIFSRRESPONSE:
		if(esp01Flags.bit.ATRESPONSEOK)
			esp01ATSate = ESP01ATCIPCLOSE;
		else{
			esp01TriesAT--;
			if(esp01TriesAT == 0){
				esp01ATSate = ESP01ATAT;
				break;
			}
			esp01ATSate = ESP01ATCIFSR;
		}
		break;
	case ESP01ATCIPCLOSE:
		if(esp01RemoteIP == NULL)
			break;
		ESP01StrToBufTX(ATCIPCLOSE);
		if(aDbgStr != NULL)
			aDbgStr("+&DBGESP01ATCIPCLOSE\n");
		esp01ATSate = ESP01ATCIPSTART;
		break;
	case ESP01ATCIPSTART:
		ESP01StrToBufTX(ATCIPSTART);
		ESP01ByteToBufTX('\"');
		ESP01StrToBufTX(esp01PROTO);
		ESP01ByteToBufTX('\"');
		ESP01ByteToBufTX(',');
		ESP01ByteToBufTX('\"');
		ESP01StrToBufTX(esp01RemoteIP);
		ESP01ByteToBufTX('\"');
		ESP01ByteToBufTX(',');
		ESP01StrToBufTX(esp01RemotePORT);
		ESP01ByteToBufTX(',');
		ESP01StrToBufTX(esp01LocalPORT);
		ESP01StrToBufTX(",0\r\n");  // <-- ¡añadido ,0!
		if(aDbgStr != NULL)
			aDbgStr("+&DBGESP01ATCIPSTART\n");
		esp01Flags.bit.ATRESPONSEOK = 0;
		esp01Flags.bit.UDPTCPCONNECTED = 0;
		esp01ATSate = ESP01CIPSTARTRESPONSE;
		// Si es UDP, usamos un timeout corto para el workaround
		if (strcmp(esp01PROTO, "UDP") == 0)
		    esp01TimeoutTask = 3000;   // 1 segundo
		else
		    esp01TimeoutTask = 30000;  // 30 s para TCP
		break;
	case ESP01CIPSTARTRESPONSE:
		if (esp01Flags.bit.ATRESPONSEOK) {
			// ya se abrió el socket UDP
			esp01ATSate = ESP01ATCONNECTED;
			esp01Flags.bit.UDPTCPCONNECTED = 1;

			// <<< debug: UDP funcional >>>
			if (strcmp(esp01PROTO, "UDP")==0 && aDbgStr) {
				aDbgStr(">>> DEBUG: AT+CIPSTART OK, UDP socket abierto\r\n");
			}


			ESP01_NotifyStateChange(ESP01_UDPTCP_CONNECTED);
		} else if (tryingTCP) {
			tryingTCP = 0;
			ESP01_USB_DbgStr(">>> TCP CONNECT falló, cambiando a UDP...\r\n");
			ESP01_StartUDP(SERVER_IP, SERVER_PORT, LOCAL_PORT);
		} else {
			esp01ATSate = ESP01ATCIPSTART;  // reintenta sólo el START
			esp01TimeoutTask = 1000;        // y dale otro segundo
		}
		break;

	case ESP01ATCONNECTED:
		// <<< debug: máquina en ESP01ATCONNECTED con UDP >>>
		if (strcmp(esp01PROTO, "UDP")==0 && aDbgStr) {
			aDbgStr(">>> DEBUG: DOConnection en ESP01ATCONNECTED (UDP vivo)\r\n");
		}

		// si perdimos wifi o la conexión UDP, reintenta
		if (!esp01Flags.bit.WIFICONNECTED) {
			esp01ATSate = ESP01ATAT;
		} else if (!esp01Flags.bit.UDPTCPCONNECTED) {
			esp01ATSate = ESP01ATCIPCLOSE;
		} else {
			esp01TimeoutTask = 0;
		}
		break;
	}
}

static void ESP01SENDData(){
	uint8_t value;

	if(esp01Flags.bit.WAITINGSYMBOL){
		if(!esp01TimeoutTxSymbol){
			esp01irTX = esp01iwTX;
			esp01Flags.bit.WAITINGSYMBOL = 0;
			esp01ATSate = ESP01ATAT;
			esp01TimeoutTask = 10;
		}
		return;
	}
	if(esp01irTX != esp01iwTX){
		value = esp01TXATBuf[esp01irTX];
		if(esp01Flags.bit.TXCIPSEND){
			if(value == '>')
				value = '\n';
		}
		if(esp01Handle.aWriteUSARTByte(value)){
			if(esp01Flags.bit.TXCIPSEND){
				if(esp01TXATBuf[esp01irTX] == '>'){
					esp01Flags.bit.TXCIPSEND = 0;
					esp01Flags.bit.WAITINGSYMBOL = 1;
					esp01TimeoutTxSymbol = 5;
				}
			}
			esp01irTX++;
			if(esp01irTX == ESP01TXBUFAT)
				esp01irTX = 0;
		}
	}
}

static void ESP01StrToBufTX(const char *str){
	/*// DEBUG: muestro en consola todo lo que voy a enviar
	    if(aDbgStr){
	        aDbgStr(">> TX: ");
	        aDbgStr(str);
	    }*/

	for(int i=0; str[i]; i++){
		esp01TXATBuf[esp01iwTX++] = str[i];
		if(esp01iwTX == ESP01TXBUFAT)
			esp01iwTX = 0;
	}
}

static void ESP01ByteToBufTX(uint8_t value){
	esp01TXATBuf[esp01iwTX++] = value;
	if(esp01iwTX == ESP01TXBUFAT)
		esp01iwTX = 0;
}

int ESP01_IsSending(void) {
    return esp01Flags.bit.SENDINGDATA;
}

void ESP01_USB_DbgStr(const char *dbgStr) {
    USB_DebugStr(dbgStr);
}

static void ESP01_NotifyStateChange(_eESP01STATUS state) {
    ESP01_InternalStateChange(state);
    if (aESP01ChangeState != NULL) {
        aESP01ChangeState(state);
    }
}

static void ESP01_InternalStateChange(_eESP01STATUS state) {
    switch (state) {
    	case ESP01_WIFI_CONNECTED:
			// Cuando ya estamos en la red, arrancamos UDP (single-connection)
			/*if (!udpIniciado && strcmp(esp01PROTO, "UDP") == 0) {
				// 1) Aseguramos single-connection
				ESP01StrToBufTX("AT+CIPMUX=0\r\n");
				aDbgStr(">>> Forzando single-connection\r\n");

				// 2) Abrimos socket UDP: AT+CIPSTART="UDP","<IP_REMOTA>",<PUERTO_REMOTO>,<PUERTO_LOCAL>,0
				{
					char cmd[64];
					snprintf(cmd, sizeof(cmd),
							 "AT+CIPSTART=\"UDP\",\"%s\",%d,%d,0\r\n",
							 esp01RemoteIP,
							 atoi(esp01RemotePORT),   // puerto remoto
							 LOCAL_PORT               // tu puerto local
					);
					ESP01StrToBufTX(cmd);
				}
				aDbgStr(">>> Iniciando socket UDP\r\n");
				udpIniciado = 1;
			}*/
			break;

        case ESP01_WIFI_NEW_IP:
            // La inicialización de UDP se delega al callback de usuario (appOnESP01ChangeState)
            // para que pueda especificar la IP de destino correcta.
            // if (state == ESP01_WIFI_NEW_IP) {
            //   ESP01_StartUDP(SERVER_IP, 30010, 30000);
            // }
            break;

        case ESP01_UDPTCP_CONNECTED:
            // Una vez conectado el cliente UDP, enviamos UNER_SendAlive
        	if (!udpIniciado && strcmp(esp01PROTO, "UDP")==0) {
				udpIniciado = 1;           // impide re-entradas
				alive_counter = 0; // Reinicia el contador de alives
				lastAliveTick = HAL_GetTick(); // Inicia el primer alive inmediatamente
				ESP01_USB_DbgStr(">>> UDP conectado OK\r\n");
				UNER_SendAlive();
				if (alive_counter < 255) { alive_counter++; }


				ESP01StrToBufTX("AT+CIPSTATUS\r\n");

				// Volvemos a mostrar la configuración
				aDbgStr("=== Parámetros UDP ===\n");
				aDbgStr("SSID: ");       aDbgStr(esp01SSID);    aDbgStr("\n");
				aDbgStr("PASSWORD: ");   aDbgStr(esp01PASSWORD);aDbgStr("\n");
				aDbgStr("REMOTE IP: ");  aDbgStr(esp01RemoteIP); aDbgStr("\n");
				aDbgStr("REMOTE PORT: ");aDbgStr(esp01RemotePORT);aDbgStr("\n");
			}
            break;

        case ESP01_UDPTCP_DISCONNECTED:
        	udpIniciado = 0;  // permite que al reconectar vuelva a entrar en UDPTCP_CONNECTED
            ESP01_USB_DbgStr(">>> UDP desconectado\r\n");
            break;

        default:
            break;
    }
}





/* END Private Functions*/
