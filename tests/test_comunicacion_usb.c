/* Native test of the real module, linked separately with CDC/IRQ stubs.
 * gcc -std=c11 -Wall -Wextra -Werror -Itests/usb_stubs -ICore/Inc
 *     tests/test_comunicacion_usb.c Core/Src/comunicacion_usb.c -o test_usb
 * No board or USB device is accessed.
 */
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include "comunicacion_usb.h"
#include "usbd_cdc_if.h"
#include "UNER.h"

static USBD_CDC_HandleTypeDef cdc;
USBD_HandleTypeDef hUsbDeviceFS = {USBD_STATE_CONFIGURED, &cdc};
static void (*rx_callback)(uint8_t *, int);
static uint32_t primask;
static uint8_t forced_result;
static uint8_t *in_flight;
static uint16_t in_flight_length;
static uint8_t wire[8192], received[256];
static size_t wire_length, received_length;
static unsigned transmit_calls;

uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1; }
void __enable_irq(void) { primask = 0; }
void CDC_Attach_Rx(void (*callback)(uint8_t *, int)) { rx_callback = callback; }
void UNER_PushByte(uint8_t byte) {
    assert(received_length < sizeof(received));
    received[received_length++] = byte;
}
uint8_t CDC_Transmit_FS(uint8_t *data, uint16_t length) {
    ++transmit_calls;
    assert(length > 0 && length <= 64);
    if (forced_result != USBD_OK) return forced_result;
    if (cdc.TxState) return USBD_BUSY;
    /* Like CDC: retain the pointer, consume its contents only on completion. */
    in_flight = data;
    in_flight_length = length;
    cdc.TxState = 1;
    return USBD_OK;
}

static void complete_transfer(void) {
    assert(cdc.TxState && in_flight);
    assert(wire_length + in_flight_length <= sizeof(wire));
    memcpy(wire + wire_length, in_flight, in_flight_length);
    wire_length += in_flight_length;
    in_flight = NULL;
    in_flight_length = 0;
    cdc.TxState = 0;
}

static void drain(void) {
    for (unsigned i = 0; i < 100; ++i) {
        if (cdc.TxState) complete_transfer();
        usb_service_tx();
        if (!cdc.TxState) return;
    }
    assert(!"USB queue did not drain");
}

static void expect(const uint8_t *bytes, size_t length) {
    drain();
    assert(wire_length == length);
    assert(memcmp(wire, bytes, length) == 0);
    wire_length = 0;
}

static void test_rx(void) {
    USB_Comunicacion_Init();
    assert(rx_callback);
    uint8_t bytes[] = {'U', 'N', 0, 'E', 'R', 255};
    rx_callback(NULL, 6);
    rx_callback(bytes, 0);
    rx_callback(bytes, -1);
    assert(!received_length);
    rx_callback(bytes, 2);
    rx_callback(bytes + 2, 4);
    assert(received_length == sizeof(bytes));
    assert(!memcmp(received, bytes, sizeof(bytes)));
}

static void test_packets_and_wrap(void) {
    uint8_t first[97], second[79], expected[176];
    for (unsigned round = 0; round < 30; ++round) {
        for (unsigned i = 0; i < sizeof(first); ++i) first[i] = (uint8_t)(i + round);
        for (unsigned i = 0; i < sizeof(second); ++i) second[i] = (uint8_t)(255 - i - round);
        memcpy(expected, first, sizeof(first));
        memcpy(expected + sizeof(first), second, sizeof(second));
        assert(usb_enqueue_tx_segments(first, sizeof(first), second, sizeof(second)));
        assert(!primask);
        /* Caller buffers may be reused while the USB transfer is in progress. */
        memset(first, 0xCC, sizeof(first));
        memset(second, 0xCC, sizeof(second));
        unsigned calls = transmit_calls;
        usb_service_tx();
        usb_service_tx();
        assert(transmit_calls == calls);  /* No overwrite of a busy chunk. */
        expect(expected, sizeof(expected));
    }
}

static void test_capacity_and_irq(void) {
    uint8_t payload[511];
    for (unsigned i = 0; i < sizeof(payload); ++i) payload[i] = (uint8_t)i;
    hUsbDeviceFS.dev_state = 0;
    assert(usb_enqueue_tx_segments(payload, 257, payload + 257, 254));
    assert(!primask);
    assert(!usb_enqueue_tx(payload, 1));
    assert(!usb_enqueue_tx_segments(payload, 1, payload, 1));
    assert(!primask);
    primask = 1;
    assert(!usb_enqueue_tx(payload, 1));
    assert(primask == 1);  /* Full-queue path preserves the caller's IRQ state. */
    primask = 0;
    unsigned calls = transmit_calls;
    usb_service_tx();
    assert(transmit_calls == calls);
    hUsbDeviceFS.dev_state = USBD_STATE_CONFIGURED;
    hUsbDeviceFS.pClassData = NULL;
    usb_service_tx();
    assert(transmit_calls == calls);
    hUsbDeviceFS.pClassData = &cdc;
    expect(payload, sizeof(payload));

    hUsbDeviceFS.dev_state = 0;
    primask = 1;
    assert(usb_enqueue_tx(payload, 3));
    assert(primask == 1);  /* Successful enqueue also preserves IRQ state. */
    primask = 0;
    hUsbDeviceFS.dev_state = USBD_STATE_CONFIGURED;
    expect(payload, 3);

    /* Reject the ENTIRE two-segment message if only its first half fits. */
    hUsbDeviceFS.dev_state = 0;
    assert(usb_enqueue_tx(payload, 500));
    assert(!usb_enqueue_tx_segments(payload, 8, payload, 8));
    hUsbDeviceFS.dev_state = USBD_STATE_CONFIGURED;
    expect(payload, 500);
    assert(usb_enqueue_tx(NULL, 0));
    assert(!usb_enqueue_tx(NULL, 1));
    assert(!usb_enqueue_tx_segments(payload, 1, NULL, 1));
    drain();
    assert(wire_length == 0);
}

static void test_retries(void) {
    const uint8_t payload[] = "Retry without losing bytes";
    forced_result = USBD_BUSY;
    assert(usb_enqueue_tx(payload, sizeof(payload)));
    usb_service_tx();
    assert(!cdc.TxState && !wire_length);
    forced_result = USBD_FAIL;
    usb_service_tx();
    assert(!cdc.TxState && !wire_length);
    forced_result = USBD_OK;
    expect(payload, sizeof(payload));
}

static void test_debug(void) {
    USB_Debug(NULL);
    USB_DebugStr(NULL);
    USB_DebugSend(NULL, 5);
    USB_DebugSend((const uint8_t *)"x", 0);
    assert(!wire_length && !cdc.TxState);
    USB_Debug("%s %c %d %u %X %% %q %s END%", "hola", 'Z', INT_MIN,
              UINT_MAX, 0xDEADBEEFU, (const char *)NULL);
    const char *expected = "hola Z -2147483648 4294967295 DEADBEEF % %q (null) END";
    expect((const uint8_t *)expected, strlen(expected));
    USB_DebugHex(0);
    USB_DebugHex(0xAF);
    USB_DebugStr("\r\n");
    expect((const uint8_t *)"00AF\r\n", 6);
    char long_string[300];
    memset(long_string, 'x', sizeof(long_string));
    long_string[sizeof(long_string) - 1] = 0;
    USB_Debug("%s", long_string);
    expect((const uint8_t *)long_string, 192);
}

int main(void) {
    test_rx();
    test_packets_and_wrap();
    test_capacity_and_irq();
    test_retries();
    test_debug();
    puts("PASS USB: RX, wrap, packets, async lifetime, capacity, IRQ state, BUSY/FAIL retries, debug.");
    return 0;
}
