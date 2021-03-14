//
// Created by asa on 3/13/21.
//

#include <AP_HAL/AP_HAL.h>

#include "LORD_Packet.h"

LORD_Packet::LORD_Packet(uint8_t* _header, size_t _payloadSize, uint8_t* _payload, uint8_t* _checksum)
    :header(_header)
    ,payloadSize(_payloadSize)
    ,payload(_payload)
    ,checksum(_checksum)
    {
        calcChecksum()
    }

void LORD_Packet::print(AP_HAL::UARTDriver *console) {
    for(int i = 0; i < 4; i++) {
        console -> printf("0x%x ", header[i]);
    }

    for(int i = 0; i < payloadSize; i++) {
        console -> printf("0x%x ", payload[i]);
    }

    for(int i = 0; i < 2; i++) {
        console -> printf("0x%x ", checksum[i]);
    }
    console -> printf("\n");
}

bool hasCorrectChecksum() {
    return correctChecksum == checksum;
}

//Code adapted from MSCL ByteStream class
static void calcChecksum() {
    uint8_t checksumByte1 = 0;
    uint8_t checksumByte2 = 0;
    uint16_t finalChecksum;

    //loop through header
    for (int i = 0; i < 4; i++) {
        //add the current value to the first checksum byte
        checksumByte1 += header[i];

        //add the current sum of the bytes to checksumByte2
        checksumByte2 += checksumByte1;
    }

    //loop through payload
    for (int i = 0; i < payloadSize; i++) {
        //add the current value to the first checksum byte
        checksumByte1 += payload[i];

        //add the current sum of the bytes to checksumByte2
        checksumByte2 += checksumByte1;
    }

    //get the final checksum from the 2 bytes
    finalChecksum = (static_cast<uint16_t>(checksumByte1) << 8) | static_cast<uint16_t>(checksumByte2);

    correctChecksum =  finalChecksum;
}




