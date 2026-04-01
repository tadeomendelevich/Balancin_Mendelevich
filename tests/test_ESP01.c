#define UNIT_TEST

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "test_framework.h"
#include "stm32f4xx_hal.h"

// Mocks
uint32_t HAL_GetTick(void) { return 0; }
void USB_Debug(const char *fmt, ...) { }
void USB_DebugStr(const char *dbgStr) { }
void UNER_SendAlive(void) { }

// itoa is not standard in C99/C11, so we mock it for the test environment
char* itoa(int value, char* str, int base) {
    if (base == 10) {
        sprintf(str, "%d", value);
    } else {
        // Fallback or handle other bases if necessary
        sprintf(str, "%d", value);
    }
    return str;
}

// Include the source file to test
// We need to disable some warnings that might arise from including a .c file if necessary,
// but for now let's just include it.
// We also need to make sure we don't re-include headers that might conflict,
// but since we are compiling this file alone, it should be fine.
#include "../Core/Src/ESP01.c"

// Test functions
int test_ESP01_Init() {
    _sESP01Handle myHandle;
    uint8_t rxBuffer[128];
    uint16_t iwRx = 0;

    // Initialize the handle with known values
    // Using void* cast to avoid warnings if types don't match exactly in mock context
    myHandle.aDoCHPD = (void*)0x1234;
    myHandle.aWriteUSARTByte = (void*)0x5678;
    myHandle.bufRX = rxBuffer;
    myHandle.iwRX = &iwRx;
    myHandle.sizeBufferRX = 128;

    // Set static variables to dirty state to ensure Init resets them
    esp01ATSate = ESP01ATCONNECTED;
    esp01HState = 99;
    esp01irTX = 10;
    esp01iwTX = 20;
    esp01irRXAT = 30;
    esp01iwRXAT = 40;
    esp01Flags.byte = 0xFF;

    // Call Init
    ESP01_Init(&myHandle);
    ESP01_AttachChangeState(onESP01StateChange);

    // Verify handle copy
    TEST_ASSERT_EQUAL_PTR(myHandle.aDoCHPD, esp01Handle.aDoCHPD);
    TEST_ASSERT_EQUAL_PTR(myHandle.aWriteUSARTByte, esp01Handle.aWriteUSARTByte);
    TEST_ASSERT_EQUAL_PTR(myHandle.bufRX, esp01Handle.bufRX);
    TEST_ASSERT_EQUAL_PTR(myHandle.iwRX, esp01Handle.iwRX);
    TEST_ASSERT_EQUAL_INT(myHandle.sizeBufferRX, esp01Handle.sizeBufferRX);

    // Verify callback attached
    TEST_ASSERT_EQUAL_PTR(onESP01StateChange, aESP01ChangeState);

    // Verify state reset
    TEST_ASSERT_EQUAL_INT(ESP01ATIDLE, esp01ATSate);
    TEST_ASSERT_EQUAL_INT(0, esp01HState);
    TEST_ASSERT_EQUAL_INT(0, esp01irTX);
    TEST_ASSERT_EQUAL_INT(0, esp01iwTX);
    TEST_ASSERT_EQUAL_INT(0, esp01irRXAT);
    TEST_ASSERT_EQUAL_INT(0, esp01iwRXAT);
    TEST_ASSERT_EQUAL_INT(0, esp01Flags.byte);

    return 1; // Pass
}

void onESP01StateChange(_eESP01STATUS state) {
    (void)state;
}

int main() {
    printf("Running ESP01 Tests\n");
    RUN_TEST(test_ESP01_Init);

    printf("Tests run: %d\n", tests_run);
    printf("Tests failed: %d\n", tests_failed);

    return tests_failed;
}
