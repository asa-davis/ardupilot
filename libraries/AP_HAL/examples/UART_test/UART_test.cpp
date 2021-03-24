#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/RingBuffer.h>

#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif

struct LORD_Packet {
    uint8_t header[4];
    uint8_t* payload;
    uint8_t checksum[2];
};

void setup();
void loop();
static void setup_uart(AP_HAL::UARTDriver *uart);
static void readIMU();
static void buildPacket();
static bool validPacket();
static void printCurrPacket();
//static void printAllAvailableBytes();
static void accelGyroData(uint8_t * fieldData, float arr[]);

//HAL variables
const AP_HAL::HAL& hal = AP_HAL::get_HAL();
static AP_HAL::UARTDriver *imu = hal.serial(4);
static AP_HAL::UARTDriver *console = hal.serial(0);

//shared ring buffer
static const uint32_t bufferSize = 1024;
static ByteBuffer buffer{bufferSize};
static uint8_t tempData[bufferSize];    //later, the producer and consumer should each have their own temp buffers

//packet parsing state variables
static struct LORD_Packet currPacket;
enum SearchPhase { sync, payloadSize, payloadAndChecksum };
static SearchPhase currPhase = sync;
static int searchBytes = 1;
//sync bytes phase
static const uint8_t syncByte1 = 0x75;
static const uint8_t syncByte2 = 0x65;
static uint8_t nextSyncByte = syncByte1;

//stats
static uint32_t firstReadTime;
static int numGoodPackets = 0;
static int numBytesSkipped = 0;
static int numBadPackets = 0;
//static bool debugged = false;

void setup() {
    setup_uart(imu);
    setup_uart(console);
    hal.scheduler->delay(1000); //Ensure that the uart can be initialized
}

void loop() {
    //read 1000 packets, then debug the stats
    //if(numGoodPackets < 1000) {
        //collect all available bytes
        readIMU();

        //parse any available bytes
        buildPacket();
    //}
    //else if(!debugged) {
        //console->printf("\nread 1000 packets in %lu milliseconds\n", AP_HAL::millis() - firstReadTime);
        //console->printf("skipped %i bytes\n", numBytesSkipped);
        //console->printf("handled %i bad checksums\n", numBadPackets);
        //debugged = true;
    //}
}

//read all available bytes into ring buffer.
static void readIMU() {
    uint32_t amountRead = imu -> read(tempData, bufferSize);
    buffer.write(tempData, amountRead);
}

//use all available bytes to continue building packets where we left off last loop
static void buildPacket() {
    while(buffer.available() >= searchBytes) {
        switch (currPhase) {
            case sync: {
                bool good = buffer.read_byte(tempData);
                if(!good) break;
                if (tempData[0] == nextSyncByte) {
                    if (nextSyncByte == syncByte2) {
                        nextSyncByte = syncByte1;
                        currPacket.header[0] = 0x75;
                        currPacket.header[1] = 0x65;
                        currPhase = payloadSize;
                        searchBytes = 2;
                    } else {
                        nextSyncByte = syncByte2;
                    }
                } else {
                    numBytesSkipped++;
                    nextSyncByte = syncByte1;
                }
            }
                break;
            case payloadSize: {
                buffer.peekbytes(tempData, searchBytes);
                currPacket.header[2] = tempData[0];
                currPacket.header[3] = tempData[1];
                searchBytes = tempData[1] + 4; //next time we need to peek the second half of the header (which we already peeked) + payload + checksum
                currPhase = payloadAndChecksum;
            }
                break;
            case payloadAndChecksum: {
                buffer.peekbytes(tempData, searchBytes);
                //copy in the payload and checksum, skip second half of header
                for (int i = 2; i < searchBytes - 2; i++) {
                    currPacket.payload[i - 2] = tempData[i];
                }
                currPacket.checksum[0] = tempData[searchBytes - 2];
                currPacket.checksum[1] = tempData[searchBytes - 1];
                //if checksum is good we can move read pointer, otherwise we leave all those bytes and start after the last sync bytes
                if (validPacket()) {
                    printCurrPacket();
                    buffer.read(tempData, searchBytes);
                    numGoodPackets++;
                    if(numGoodPackets == 1)
                        firstReadTime = AP_HAL::millis();
                }
                else {
                    numBadPackets++;
                }
                currPhase = sync;
                searchBytes = 1;
            }
                break;
        }
    }
}

//gets checksum and compares it to curr packet
static bool validPacket() {
    uint8_t checksumByte1 = 0;
    uint8_t checksumByte2 = 0;

    for (int i = 0; i < 4; i++) {
        checksumByte1 += currPacket.header[i];
        checksumByte2 += checksumByte1;
    }

    for (int i = 0; i < currPacket.header[3]; i++) {
        checksumByte1 += currPacket.payload[i];
        checksumByte2 += checksumByte1;
    }

    return (currPacket.checksum[0] == checksumByte1 && currPacket.checksum[1] == checksumByte2);
}


void accelGyroData(uint8_t * fieldData, float arr[]) {
    //vector<float> xyzData;
    uint32_t midx = 0;
    for (uint8_t i = 0; i < 4; i++ ) {
        midx = (midx << 8) + static_cast<uint32_t>(fieldData[i]);
    }
    uint32_t midy = 0;
    for (uint8_t i = 4; i < 8; i++ ) {
        midy = (midy << 8) + static_cast<uint32_t>(fieldData[i]);
    }
    uint32_t midz = 0;
    for (uint8_t i = 8; i < 12; i++ ) {
        midz = (midz << 8) + static_cast<uint32_t>(fieldData[i]);
    }
    arr[0] = ( (* (float *) &midx) ); // Reinterpret_cast does not work for this so
    arr[1] = ( (* (float *) &midy) );
    arr[2] = ( (* (float *) &midz) );
    //xyzData.push_back(* reinterpret_cast<float*>(&midx)); // Works, but its doing the same thing as above, if you do it without address' it doesn't work
    //return xyzData;
}

static void printCurrPacket() {
    /*for(int i = 0; i < 4; i++) {
        console->printf("0x%x ", currPacket.header[i]);
    }
    for(int i = 0; i < currPacket.header[3]; i++) {
        console->printf("0x%x ", currPacket.payload[i]);
    }
    for(int i = 0; i < 2; i++) {
        console->printf("0x%x ", currPacket.checksum[i]); 75 65 80 A3
    }
    console -> printf("\n");*/
    uint8_t payloadLength = currPacket.header[3];
// length descriptor, 12bit payload
    for (uint8_t i = 0; i < payloadLength; i += currPacket.payload[i]) {
        //uint8_t fieldLength = currPacket.payload[i]; //Each field in the payload has its own length property as the first byte of the field
        uint8_t fieldDescriptor = currPacket.payload[i+1]; // Second byte is the field descriptor. Whether it is accel or gyro, etc.
        //ctor<float> xyz; // Maybe call accelGyroData() here instead to reduce reuse of code but could be a waste if we include other data in the field like GPS data
        float retarr[3];
        accelGyroData(currPacket.payload + (i+2),retarr); // This will probably break if the packet is wrong
        switch (fieldDescriptor) {  // Switch to relevant course for accel or gyro or etc. data
            case 4:
                //console->printf("Accel - X: %f,\tY: %f,\tZ: %f\n",retarr[0], retarr[1], retarr[2]);
                break;
            case 5:
                console->printf("Gyro  - X: %f,\tY: %f,\tZ: %f\n",retarr[0], retarr[1], retarr[2]);
                // cout << "Gyro:\nX: " << xyz[0] << "\nY: " << xyz[1] << "\nZ: " << xyz[2] << '\n';
                // firstg = (firstg + 1) % BUFFSIZE; // Before we push on data increment the index so that firstg will equal the most recent data
                // gyroBuffer[firstg] = xyz; // Push vector with gyro xyz onto the buffer
                break;
            default:
                //cout << "Don't want anything but Gyro and Accel\n";
                break;
        }
    }
}

//just prints whatever is in the ring buffer
/*static void printAllAvailableBytes() {
    uint32_t available = buffer.available();
    buffer.read(tempData, available);
    for(int i = 0; i < available; i++) {
        console -> printf("0x%x ", tempData[i]);
    }
    console -> printf("\n");
}*/

static void setup_uart(AP_HAL::UARTDriver *uart) {
    if (uart == nullptr) {
        // that UART doesn't exist on this platform
        return;
    }
    //uart->set_stop_bits(1);
    //uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    uart->begin(115200);
}

AP_HAL_MAIN();