/**
  ******************************************************************************
  * @file    ESP01.h
  * @author  Germán E. Hachmann
  * @brief   Header file containing functions prototypes of ESP01 library.
  ******************************************************************************
  * @attention
  *
  *
  * Copyright (c) 2023 HGE.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ESP01_H_
#define ESP01_H_

#include <stdint.h>

/**< Estados ESP01 */
typedef enum{
	ESP01_NOT_INIT = -1,
	ESP01_WIFI_DISCONNECTED,
	ESP01_WIFI_RECONNECTING,
	ESP01_WIFI_NOT_SETED,
	ESP01_WIFI_CONNECTING_WIFI,
	ESP01_WIFI_CONNECTED,
	ESP01_WIFI_NEW_IP,
	ESP01_UDPTCP_DISCONNECTED,
	ESP01_UDPTCP_CONNECTING,
	ESP01_UDPTCP_CONNECTED,
	ESP01_SEND_BUSY,
	ESP01_SEND_READY,
	ESP01_SEND_OK,
	ESP01_SEND_ERROR,
} _eESP01STATUS;

/**< Puntero a función que abstrae el GPIO del microcontrolador  */
typedef void (*DoCHPD)(uint8_t value);

/**< Puntero a función que abstrae el USART del microcontrolador  */
typedef int (*WriteUSARTByte)(uint8_t value);

/**< Puntero a función que se ejecuta cuando   */
typedef void (*OnESP01ChangeState)(_eESP01STATUS esp01State);

/**< Puntero a función que se ejecuta cuando   */
typedef void (*ESP01DebugStr)(const char *dbgStr);


#define ESP01RXBUFAT		128
#define ESP01TXBUFAT		256


/**< Inicializa el driver ESP01 UDP */
typedef struct{
	DoCHPD 				    aDoCHPD;			    /**< Puntero a una función que permite manejar el pin CH_PD del ESP01 */
	WriteUSARTByte		aWriteUSARTByte;	/**< Puntero a una función que escribe un byte en la USART, devuelve 1 si pudo escribir */
	uint8_t 			    *bufRX;				    /**< Puntero al buffer donde se guardarán los datos a recibidos */
	uint16_t			    *iwRX;				    /**< Puntero al índice de escritura del buffer de recepción circular */
	uint16_t			    sizeBufferRX;		  /**< Tamaño en bytes del buffer de recepción*/
} _sESP01Handle;

extern const char *wifiSSID;
extern const char *wifiPassword;
extern const char *wifiIp;

/**
 * @brief ESP01_WIFI Configura y Conecta
 *
 * Conecta a una red wifi especificada.
 * Si ya hay establecida una cominicación se deconecta y conecta a la nueva red wifi.
 * Use ESP01_STWIFI para verificar el estado de la conexión
 *
 * @param [in] ssid: Especifica el nuevo ssid
 * @param [in] password: Especifica el nuevo password
 *
 */
void ESP01_SetWIFI(const char *ssid, const char *password);

/**
 * @brief ESP01_START_UDP Configura y Conecta UDP
 *
 * Comienza una comunicación UDP, siempre que este conectado a WIFI.
 * Si hay una conexión establecida la cierra y se conecta esta nueva IP:PORT
 * Use ESP01_STUDP para verificar el estado de la conexión UDP
 * Si la read WIFI no esta disponible se conecta a esta IP:PORT automáticamante
 * cuando se reestablezaca la conexión WIFI
 *
 * @param [in] RemoteIP: Especifica la IP remota a transmitir
 * @param [in] RemotePORT: Especifica el puerto remoto a transmitir
 *
 */
_eESP01STATUS ESP01_StartUDP(const char *RemoteIP, uint16_t RemotePORT, uint16_t LocalPORT);


/**
 * @brief ESP01_START_TCP Configura y Conecta UDP
 *
 * Comienza una comunicación TCP, siempre que este conectado a WIFI.
 * Si hay una conexión establecida la cierra y se conecta esta nueva IP:PORT
 * Use ESP01_STUDP para verificar el estado de la conexión UDP
 * Si la read WIFI no esta disponible se conecta a esta IP:PORT automáticamante
 * cuando se reestablezaca la conexión WIFI
 *
 * @param [in] RemoteIP: Especifica la IP remota a transmitir
 * @param [in] RemotePORT: Especifica el puerto remoto a transmitir
 *
 */
_eESP01STATUS ESP01_StartTCP(const char *RemoteIP, uint16_t RemotePORT, uint16_t LocalPORT);

/**
 * @brief ESP01_CLOSEUDP Cierra una conexión UDP
 *
 */
void ESP01_CloseUDPTCP();

/**
 * @brief ESP01_STWIFI Devuelve el estado de la conexión WIFI
 *
 */
_eESP01STATUS ESP01_StateWIFI();

/**
 * @brief ESP01_GET_LOCAL_IP Devuelve la IP del ESP01
 *
 * @retVal Devuelve NULL Caundo no tiene IP
 */
char *ESP01_GetLocalIP();

/**
 * @brief ESP01_STWIFI Devuelve el estado de la conexión UDP
 *
 */
_eESP01STATUS ESP01_StateUDPTCP();


/**
 * @brief ESP01_Send
 *
 * Envía los datos guardados en el buffer de transmisión.
 *
 * @param [in] length: longitud del los datos a enviar.
 *
 * @retVal Si pudo transmitir devuelve ESP01_ST_SEND_READY
 */
_eESP01STATUS ESP01_Send(uint8_t *buf, uint16_t irRingBuf, uint16_t length, uint16_t sizeRingBuf);


/**
 * @brief ESP01_Init Inicializa el driver ESP01
 *
 * Esta función debe llamarse en primer lugar para incicializar el driver
 *
 * @param [in] hESP01: Puntero a un manejador para el ESP01
 *
 */
void ESP01_Init(_sESP01Handle *hESP01);


/**
 * @brief ESP01_Timeout10ms Mantiene el timer del driver
 *
 * Esta función debe llamarse cada 10ms para mantener el timer del driver
 *
 */
void ESP01_Timeout10ms();

/**
 * @brief ESP01_TASK Tarea principal del driver
 *
 * Mantiene el estado del ESP01
 * Debe llamarse repetidamente para que el driver pueda verifcar el estado, tranmitir y recibir
 *
 * Ejemplo:
 *
 * while(1){
 * .
 * .
 * .
 * ESP01_TASK();
 * .
 * .
 * .
 * }
 *
 */
void ESP01_Task();

/**
 * @brief ESP01_WRITE_RX Escribe en el buffer de receoción
 *
 * Esta función debe llamarse cada vez que se reciba un bytes por el USART
 *
 * @param [in] value: valor recibido por USART (Enviado por el ESP01)
 *
 */
void ESP01_WriteRX(uint8_t value);

void ESP01_AttachChangeState(OnESP01ChangeState aOnESP01ChangeState);

void ESP01_AttachDebugStr(ESP01DebugStr aDbgStrPtrFun);

int ESP01_IsHDRRST();

int  ESP01_IsSending(void);

void ESP01_USB_DbgStr(const char *dbgStr);

void onESP01StateChange(_eESP01STATUS state);

extern void USB_DebugStr(const char *dbgStr);

#endif /* ESP01_H_ */
