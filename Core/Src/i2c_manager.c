#include "i2c_manager.h"
#include <string.h>

#define I2C_MANAGER_QUEUE_SIZE 8

static I2C_Request_t queue[I2C_MANAGER_QUEUE_SIZE];
static volatile uint8_t q_head = 0;
static volatile uint8_t q_tail = 0;
static volatile uint8_t i2c_busy = 0;
static I2C_Request_t current_req;
static volatile uint8_t current_valid = 0;

static uint8_t queue_is_full(void)
{
    return ((uint8_t)((q_head + 1) % I2C_MANAGER_QUEUE_SIZE) == q_tail);
}

static uint8_t queue_is_empty(void)
{
    return (q_head == q_tail);
}

void I2C_Manager_Init(void)
{
    q_head = 0;
    q_tail = 0;
    i2c_busy = 0;
    current_valid = 0;
    memset(&current_req, 0, sizeof(current_req));
}

uint8_t I2C_Manager_IsBusy(void)
{
    return i2c_busy;
}

uint8_t I2C_Manager_Enqueue(const I2C_Request_t *req)
{
    uint8_t ok = 0;

    __disable_irq();
    if (!queue_is_full()) {
        queue[q_head] = *req;
        q_head = (uint8_t)((q_head + 1) % I2C_MANAGER_QUEUE_SIZE);
        ok = 1;
    }
    __enable_irq();

    return ok;
}

uint8_t I2C_Manager_EnqueuePriority(const I2C_Request_t *req)
{
    uint8_t ok = 0;

    __disable_irq();
    uint8_t new_tail = (uint8_t)((q_tail + I2C_MANAGER_QUEUE_SIZE - 1) % I2C_MANAGER_QUEUE_SIZE);
    if (new_tail != q_head) {
        q_tail = new_tail;
        queue[q_tail] = *req;
        ok = 1;
    }
    __enable_irq();

    return ok;
}

void I2C_Manager_Process(void)
{
    HAL_StatusTypeDef st;

    __disable_irq();
    if (i2c_busy || queue_is_empty()) {
        __enable_irq();
        return;
    }

    current_req = queue[q_tail];
    current_valid = 1;
    i2c_busy = 1;
    __enable_irq();

    USB_Debug("I2C start type=%d dev=0x%X len=%d\r\n",
              current_req.type, current_req.devAddr, current_req.len);

    switch (current_req.type) {
    case I2C_REQ_MEM_READ_DMA:
        st = HAL_I2C_Mem_Read_DMA(
            current_req.hi2c,
            current_req.devAddr,
            current_req.memAddr,
            current_req.memAddSize,
            current_req.data,
            current_req.len
        );
        break;

    case I2C_REQ_MASTER_TX_DMA:
        st = HAL_I2C_Master_Transmit_DMA(
            current_req.hi2c,
            current_req.devAddr,
            current_req.data,
            current_req.len
        );
        break;

    default:
        st = HAL_ERROR;
        break;
    }

    if (st != HAL_OK) {
        USB_Debug("I2C start FAIL st=%d\r\n", st);

        __disable_irq();
        i2c_busy = 0;

        if (current_valid && current_req.callback) {
            I2C_ManagerCallback_t cb = current_req.callback;
            void *ctx = current_req.context;
            q_tail = (uint8_t)((q_tail + 1) % I2C_MANAGER_QUEUE_SIZE);
            current_valid = 0;
            __enable_irq();
            cb(ctx, st);
        } else {
            q_tail = (uint8_t)((q_tail + 1) % I2C_MANAGER_QUEUE_SIZE);
            current_valid = 0;
            __enable_irq();
        }

        I2C_Manager_Process();
        return;
    }

    USB_Debug("I2C start OK\r\n");
}

static void complete_current(I2C_HandleTypeDef *hi2c, HAL_StatusTypeDef status)
{
    I2C_ManagerCallback_t cb = NULL;
    void *ctx = NULL;

    __disable_irq();

    if (!current_valid || current_req.hi2c != hi2c) {
        __enable_irq();
        return;
    }

    i2c_busy = 0;
    cb = current_req.callback;
    ctx = current_req.context;

    q_tail = (uint8_t)((q_tail + 1) % I2C_MANAGER_QUEUE_SIZE);
    current_valid = 0;

    __enable_irq();

    if (cb) {
        cb(ctx, status);
    }

    I2C_Manager_Process();
}

void I2C_Manager_OnMemRxCplt(I2C_HandleTypeDef *hi2c)
{
    complete_current(hi2c, HAL_OK);
}

void I2C_Manager_OnMasterTxCplt(I2C_HandleTypeDef *hi2c)
{
    complete_current(hi2c, HAL_OK);
}

void I2C_Manager_OnError(I2C_HandleTypeDef *hi2c)
{
    complete_current(hi2c, HAL_ERROR);
}
