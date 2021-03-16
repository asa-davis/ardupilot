/*
  simple test of UART interfaces
 */
#include <AP_HAL/AP_HAL.h>
#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif

#include "LORD_Packet.h"

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
static AP_HAL::UARTDriver *imu = hal.serial(4);
static AP_HAL::UARTDriver *console = hal.serial(0);

const static int packetRateHz = 50;
const static uint8_t syncByte1 = 0x75;
const static uint8_t syncByte2 = 0x65;

static void setup_uart(AP_HAL::UARTDriver *uart, const char *name);
static size_t sync();
static LORD_Packet* readLORDPacket(bool skipSyncBytes);

//unused methods
//static void readLORDtoConsole();
//static void sendLORDPingPacket(AP_HAL::UARTDriver *uart, AP_HAL::UARTDriver *console);

void setup() {
    hal.scheduler->delay(1000); //Ensure that the uart can be initialized
    setup_uart(imu, "LORD_IMU");
    setup_uart(console, "TERMINAL");
}

void loop() {
    hal.scheduler->delay(1000/packetRateHz);

    //make sure we are synced
    size_t numBytesSkipped = sync();
    if(numBytesSkipped > 0) {
        console->printf("Synced after skipping %u bytes.\n", numBytesSkipped);
    }

    //read and print the packet
    LORD_Packet* pkt = readLORDPacket(true);
    pkt->print(console);

    //check if checksum is bad
    console->printf("CHECKSUM: 0x%x\n", pkt->correctChecksum);
}

//try to read bytes until we find the two sync bytes in a row (0x75 0x65)
//return the number of bytes skipped
static size_t sync() {
    size_t numBytesSkipped = 0;
    bool synced = false;

    uint8_t currSearchByte = syncByte1;
    uint8_t currByte[1];

    while(!synced) {
        size_t bytesRead = imu -> read(currByte, 1);
        hal.scheduler->delay(1000/packetRateHz);

        //can't read from IMU
        if(bytesRead == 0) {
            console->printf("Can't read from imu!\n");
        }
        //incorrect byte
        else if(currByte[0] != currSearchByte){
            console->printf("Bad byte - 0x%02x != 0x%02x\n", currByte[0], currSearchByte);

            //reset search byte
            currSearchByte = syncByte1;

            numBytesSkipped++;
        }
        //correct byte
        else {
            if(currSearchByte == syncByte1) {
                currSearchByte = syncByte2;
            }
            else {
                synced = true;
            }
        }
    }
    return numBytesSkipped;
}

//reads exactly one packet
static LORD_Packet* readLORDPacket(bool skipSyncBytes) {
    //read header into buffer
    uint8_t headerBuff[4];

    //if we already read sync bytes, we can just read the next two into the header
    if(skipSyncBytes) {
        headerBuff[0] = syncByte1;
        headerBuff[1] = syncByte2;

        uint8_t headerBuff2[2];
        imu -> read(headerBuff2, 2);

        headerBuff[2] = headerBuff2[0];
        headerBuff[3] = headerBuff2[1];
    }
    //otherwise read 4 bytes to header
    else {
        imu -> read(headerBuff, 4);
    }

    //determine payload size
    size_t payloadSize = headerBuff[3];

    //read payload
    uint8_t payloadBuff[payloadSize];
    imu -> read(payloadBuff, payloadSize);

    //read checksum
    uint8_t checksumBuff[2];
    imu -> read(checksumBuff, 2);

    //construct and return packet
    LORD_Packet* pkt = new LORD_Packet(headerBuff, payloadSize, payloadBuff, checksumBuff);
    return pkt;
}

static void setup_uart(AP_HAL::UARTDriver *uart, const char *name) {
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    //uart->set_stop_bits(1);
    //uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    uart->begin(115200);
}

AP_HAL_MAIN();