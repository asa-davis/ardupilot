/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/*
  suppport for serial connected AHRS systems
*/

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "AP_ExternalAHRS.h"
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <stdio.h>

#if HAL_EXTERNAL_AHRS_ENABLED

extern const AP_HAL::HAL &hal;
HAL_Semaphore AP_ExternalAHRS::sem;

/*
  header for pre-configured 50Hz data
  assumes the following config for VN-300:
    $VNWRG,75,3,8,35,0003,0F2C,0147,0613*2642
*/
static const uint8_t vn_pkt1_header[] { 0x35, 0x03, 0x00, 0x2c, 0x0f, 0x47, 0x01, 0x13, 0x06 };
#define VN_PKT1_LENGTH 194 // includes header

struct PACKED VN_packet1 {
    uint64_t timeStartup;
    uint64_t timeGPS;
    float uncompAccel[3];
    float uncompAngRate[3];
    float pressure;
    float mag[3];
    float accel[3];
    float gyro[3];
    uint16_t sensSat;
    uint16_t AHRSStatus;
    float ypr[3];
    float quaternion[4];
    float linAccBody[3];
    float yprU[3];
    uint16_t INSStatus;
    double positionLLA[3];
    float velNED[3];
    float posU;
    float velU;
};

// check packet size for 4 groups
static_assert(sizeof(VN_packet1)+2+4*2+2 == VN_PKT1_LENGTH, "incorrect VN_packet1 length");

/*
  header for pre-configured 5Hz data
  assumes the following VN-300 config:
    $VNWRG,76,3,80,4E,0002,0010,20B8,2018*A66B
*/
static const uint8_t vn_pkt2_header[] { 0x4e, 0x02, 0x00, 0x10, 0x00, 0xb8, 0x20, 0x18, 0x20 };
#define VN_PKT2_LENGTH 120 // includes header

struct PACKED VN_packet2 {
    uint64_t timeGPS;
    float temp;
    uint8_t numGPS1Sats;
    uint8_t GPS1Fix;
    double GPS1posLLA[3];
    float GPS1velNED[3];
    float GPS1DOP[7];
    uint8_t numGPS2Sats;
    uint8_t GPS2Fix;
    float GPS2DOP[7];
};

// check packet size for 4 groups
static_assert(sizeof(VN_packet2)+2+4*2+2 == VN_PKT2_LENGTH, "incorrect VN_packet2 length");

AP_ExternalAHRS *AP_ExternalAHRS::_singleton;

// constructor
AP_ExternalAHRS::AP_ExternalAHRS()
{
    AP_Param::setup_object_defaults(this, var_info);
    _singleton = this;
}

#ifndef HAL_EXTERNAL_AHRS_DEFAULT
#define HAL_EXTERNAL_AHRS_DEFAULT 0
#endif


// table of user settable parameters
const AP_Param::GroupInfo AP_ExternalAHRS::var_info[] = {

    // @Param: _TYPE
    // @DisplayName: AHRS type
    // @Description: Type of AHRS device
    // @Values: 0:None,1:VectorNav
    // @User: Standard
    AP_GROUPINFO_FLAGS("_TYPE", 1, AP_ExternalAHRS, devtype, HAL_EXTERNAL_AHRS_DEFAULT, AP_PARAM_FLAG_ENABLE),

    AP_GROUPEND
};


void AP_ExternalAHRS::init(void)
{
    auto &sm = AP::serialmanager();
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    imu = hal.serial(4);
    if (imu == nullptr) {
        AP_HAL::panic("Failed to connect to uart port.");
        test = -3;
        return;
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS::update_thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_HAL::panic("Failed to start ExternalAHRS update thread");
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS initialised");
}

/*
  check the UART for more data
  returns true if the function should be called again straight away
 */
bool AP_ExternalAHRS::check_uart()
{
    return true;
}

void AP_ExternalAHRS::update_thread()
{
    if(!portOpened) {
        portOpened = true;
        imu->begin(115200);
        hal.scheduler->delay(1000);

        //ins_data_message_t ins;
        //ins.accel = Vector3f{6, 6, 6};
        //ins.gyro = Vector3f{6, 6, 6};

        //AP::ins().handle_external(ins);
        //hal.scheduler->delay(1);
    }

    while(true) {
        if(packetReady) {
            packetReady = false;
            ins_data_message_t ins;

            ins.accel = accelNew;
            ins.gyro = gyroNew;

            AP::ins().handle_external(ins);
        }

        readIMU();
        buildPacket();
        hal.scheduler->delay(1);
    }
}

/*
  process packet type 1
 */
void AP_ExternalAHRS::process_packet1(const uint8_t *b)
{

}

/*
  process packet type 2
 */
void AP_ExternalAHRS::process_packet2(const uint8_t *b)
{

}

// get serial port number for the uart
int8_t AP_ExternalAHRS::get_port(void) const
{
    //what is this for? ins checks if it returns > 0...
    //if (!uart) {
        //return -1;
    //}
    //return port_num;
    return 4;
};

// accessors for AP_AHRS
bool AP_ExternalAHRS::healthy(void) const
{
    uint32_t now = AP_HAL::millis();
    return (now - last_pkt1_ms < 40 && now - last_pkt2_ms < 500);
}

bool AP_ExternalAHRS::initialised(void)
{
    return last_pkt1_ms != 0 && last_pkt2_ms != 0;
}

bool AP_ExternalAHRS::get_quaternion(Quaternion &quat)
{
    if (!last_pkt1) {
        return false;
    }
    WITH_SEMAPHORE(sem);
    const struct VN_packet1 &pkt1 = *last_pkt1;
    quat(pkt1.quaternion[3], pkt1.quaternion[0], pkt1.quaternion[1], pkt1.quaternion[2]);
    return true;
}

bool AP_ExternalAHRS::get_origin(Location &loc)
{
    WITH_SEMAPHORE(sem);
    loc = origin;
    return origin_set;
}

bool AP_ExternalAHRS::get_location(Location &loc)
{
    if (!last_pkt2) {
        return false;
    }
    const struct VN_packet2 &pkt2 = *last_pkt2;
    WITH_SEMAPHORE(sem);
    loc = Location{int32_t(pkt2.GPS1posLLA[0] * 1.0e7),
                   int32_t(pkt2.GPS1posLLA[1] * 1.0e7),
                   int32_t(pkt2.GPS1posLLA[2] * 1.0e2),
                   Location::AltFrame::ABSOLUTE};
    return true;
}

Vector2f AP_ExternalAHRS::get_groundspeed_vector()
{
    if (!last_pkt1) {
        return Vector2f{};
    }
    const struct VN_packet1 &pkt1 = *last_pkt1;
    WITH_SEMAPHORE(sem);
    return Vector2f{pkt1.velNED[0], pkt1.velNED[1]};
}

bool AP_ExternalAHRS::get_velocity_NED(Vector3f &vel)
{
    if (!last_pkt1) {
        return false;
    }
    const struct VN_packet1 &pkt1 = *last_pkt1;
    WITH_SEMAPHORE(sem);
    vel.x = pkt1.velNED[0];
    vel.y = pkt1.velNED[1];
    vel.z = pkt1.velNED[2];
    return true;
}

bool AP_ExternalAHRS::get_speed_down(float &speedD)
{
    if (!last_pkt1) {
        return false;
    }
    const struct VN_packet1 &pkt1 = *last_pkt1;
    WITH_SEMAPHORE(sem);
    speedD = pkt1.velNED[2];
    return true;
}

bool AP_ExternalAHRS::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "VectorNav unhealthy");
        return false;
    }
    if (last_pkt2->GPS1Fix < 3) {
        hal.util->snprintf(failure_msg, failure_msg_len, "VectorNav no GPS1 lock");
        return false;
    }
    if (last_pkt2->GPS2Fix < 3) {
        hal.util->snprintf(failure_msg, failure_msg_len, "VectorNav no GPS2 lock");
        return false;
    }
    return true;
}

/*
  get filter status. We don't know the meaning of the status bits yet,
  so assume all OK if we have GPS lock
 */
void AP_ExternalAHRS::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));
    if (last_pkt1 && last_pkt2) {
        status.flags.initalized = 1;
    }
    if (healthy() && last_pkt2) {
        status.flags.attitude = 1;
        status.flags.vert_vel = 1;
        status.flags.vert_pos = 1;

        const struct VN_packet2 &pkt2 = *last_pkt2;
        if (pkt2.GPS1Fix >= 3) {
            status.flags.horiz_vel = 1;
            status.flags.horiz_pos_rel = 1;
            status.flags.horiz_pos_abs = 1;
            status.flags.pred_horiz_pos_rel = 1;
            status.flags.pred_horiz_pos_abs = 1;
            status.flags.using_gps = 1;
        }
    }
}

Vector3f AP_ExternalAHRS::get_gyro(void)
{
    if (!last_pkt1) {
        return Vector3f{};
    }
    const struct VN_packet1 &pkt1 = *last_pkt1;
    WITH_SEMAPHORE(sem);
    return Vector3f(pkt1.gyro[0], pkt1.gyro[1], pkt1.gyro[2]);
}

Vector3f AP_ExternalAHRS::get_accel(void)
{
    if (!last_pkt1) {
        return Vector3f{};
    }
    const struct VN_packet1 &pkt1 = *last_pkt1;
    WITH_SEMAPHORE(sem);
    return Vector3f{pkt1.accel[0], pkt1.accel[1], pkt1.accel[2]};
}

// send an EKF_STATUS message to GCS
void AP_ExternalAHRS::send_status_report(mavlink_channel_t chan) const
{
    if (!last_pkt1) {
        return;
    }
    // prepare flags
    uint16_t flags = 0;
    nav_filter_status filterStatus;
    get_filter_status(filterStatus);
    if (filterStatus.flags.attitude) {
        flags |= EKF_ATTITUDE;
    }
    if (filterStatus.flags.horiz_vel) {
        flags |= EKF_VELOCITY_HORIZ;
    }
    if (filterStatus.flags.vert_vel) {
        flags |= EKF_VELOCITY_VERT;
    }
    if (filterStatus.flags.horiz_pos_rel) {
        flags |= EKF_POS_HORIZ_REL;
    }
    if (filterStatus.flags.horiz_pos_abs) {
        flags |= EKF_POS_HORIZ_ABS;
    }
    if (filterStatus.flags.vert_pos) {
        flags |= EKF_POS_VERT_ABS;
    }
    if (filterStatus.flags.terrain_alt) {
        flags |= EKF_POS_VERT_AGL;
    }
    if (filterStatus.flags.const_pos_mode) {
        flags |= EKF_CONST_POS_MODE;
    }
    if (filterStatus.flags.pred_horiz_pos_rel) {
        flags |= EKF_PRED_POS_HORIZ_REL;
    }
    if (filterStatus.flags.pred_horiz_pos_abs) {
        flags |= EKF_PRED_POS_HORIZ_ABS;
    }
    if (!filterStatus.flags.initalized) {
        flags |= EKF_UNINITIALIZED;
    }

    // send message
    const struct VN_packet1 &pkt1 = *(struct VN_packet1 *)last_pkt1;
    const float vel_gate = 5;
    const float pos_gate = 5;
    const float hgt_gate = 5;
    const float mag_var = 0;
    mavlink_msg_ekf_status_report_send(chan, flags,
                                       pkt1.velU/vel_gate, pkt1.posU/pos_gate, pkt1.posU/hgt_gate,
                                       mag_var, 0, 0);
}

//LORD METHODS


//read all available bytes into ring buffer.
void AP_ExternalAHRS::readIMU() {
    uint32_t amountRead = imu -> read(tempData, bufferSize);
    buffer.write(tempData, amountRead);
}

//use all available bytes to continue building packets where we left off last loop
void AP_ExternalAHRS::buildPacket() {
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
                    getCurrPacket();
                    buffer.read(tempData, searchBytes);
                }
                currPhase = sync;
                searchBytes = 1;
            }
                break;
        }
    }
}

//gets checksum and compares it to curr packet
bool AP_ExternalAHRS::validPacket() {
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


void AP_ExternalAHRS::accelGyroData(uint8_t * fieldData, float arr[]) {
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

void AP_ExternalAHRS::getCurrPacket() {
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
                accelNew = Vector3f{retarr[0], retarr[1], retarr[2]};
                //console->printf("Accel - X: %f,\tY: %f,\tZ: %f\n",retarr[0], retarr[1], retarr[2]);
                break;
            case 5:
                gyroNew = Vector3f{retarr[0], retarr[1], retarr[2]};
                //console->printf("Gyro  - X: %f,\tY: %f,\tZ: %f\n",retarr[0], retarr[1], retarr[2]);
                // cout << "Gyro:\nX: " << xyz[0] << "\nY: " << xyz[1] << "\nZ: " << xyz[2] << '\n';
                // firstg = (firstg + 1) % BUFFSIZE; // Before we push on data increment the index so that firstg will equal the most recent data
                // gyroBuffer[firstg] = xyz; // Push vector with gyro xyz onto the buffer
                break;
            default:
                //cout << "Don't want anything but Gyro and Accel\n";
                break;
        }
    }
    packetReady = true;
}

namespace AP {
AP_ExternalAHRS &externalAHRS()
{
    return *AP_ExternalAHRS::get_singleton();
}

};

#endif  // HAL_EXTERNAL_AHRS_ENABLED

