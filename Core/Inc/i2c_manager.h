#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H

#include "main.h"
#include <stdint.h>

extern volatile uint8_t i2c_process_pending;

typedef enum {
    I2C_REQ_NONE = 0,
    I2C_REQ_MEM_READ_DMA,
    I2C_REQ_MASTER_TX_DMA
} I2C_RequestType_t;

typedef void (*I2C_ManagerCallback_t)(void *context, HAL_StatusTypeDef status);

typedef struct {
    I2C_RequestType_t type;
    I2C_HandleTypeDef *hi2c;
    uint16_t devAddr;
    uint16_t memAddr;
    uint16_t memAddSize;
    uint8_t *data;
    uint16_t len;
    I2C_ManagerCallback_t callback;
    void *context;
} I2C_Request_t;

void I2C_Manager_Init(void);
uint8_t I2C_Manager_IsBusy(void);
uint8_t I2C_Manager_Enqueue(const I2C_Request_t *req);
uint8_t I2C_Manager_EnqueuePriority(const I2C_Request_t *req);
void I2C_Manager_Process(void);

void I2C_Manager_OnMemRxCplt(I2C_HandleTypeDef *hi2c);
void I2C_Manager_OnMasterTxCplt(I2C_HandleTypeDef *hi2c);
void I2C_Manager_OnError(I2C_HandleTypeDef *hi2c);
void I2C_Manager_ProcessFromISR(void);  // versión no-recursiva para usar en callbacks
uint8_t I2C_Manager_IsProcessPending(void);

#endif
