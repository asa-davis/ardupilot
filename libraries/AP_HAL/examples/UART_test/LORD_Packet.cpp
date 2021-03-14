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
    {}

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



