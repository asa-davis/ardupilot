/*
  simple test of UART interfaces
 */
#include <AP_HAL/AP_HAL.h>
#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif
void setup();
void loop();
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static void setup_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uart->set_stop_bits(1);
    uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    uart->begin(115200);
}

void setup(void)
{
    /*
      start all UARTs at 57600 with default buffer sizes
    */
    hal.scheduler->delay(1000); //Ensure that the uartA can be initialized
    setup_uart(hal.serial(4), "LORD_IMU");
    setup_uart(hal.serial(0), "TERMINAL");
}
static void test_uart(AP_HAL::UARTDriver *uart, AP_HAL::UARTDriver *terminal, const char *name)
{
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    uint8_t pkt[] = {0x75, 0x65, 0x01, 0x02, 0x02, 0x01, 0xE0, 0xC6};
    size_t written = uart->write(pkt, 8);
    terminal -> printf("bytes written: %u\n", written);
    hal.scheduler->delay(10000);
    uint8_t response[8];
    size_t read = uart->read(response, 8);
    terminal -> printf("bytes read: %u\n", read);
    terminal -> printf("Response: ");
    for (int i = 0; i < 8; ++i) {
        terminal -> printf("0x%x ", response[i]);
    }
    terminal -> printf("\n");
}
void loop(void)
{
    test_uart(hal.serial(4), hal.serial(0), "LORD_IMU");
    hal.scheduler->delay(5000);
}
AP_HAL_MAIN();