//
// Created by asa on 3/13/21.
//

#ifndef ARDUPILOT_LORD_PACKET_H
#define ARDUPILOT_LORD_PACKET_H

#endif //ARDUPILOT_LORD_PACKET_H

class LORD_Packet {
public:
    uint8_t* header;     //length 4
    size_t payloadSize;
    uint8_t* payload;
    uint8_t* checksum;   //length 2
    uint16_t correctChecksum;

    LORD_Packet(uint8_t* _header, size_t _payloadSize, uint8_t* _payload, uint8_t* _checksum);

    void print(AP_HAL::UARTDriver *console);

private:
    void calcChecksum();
};
