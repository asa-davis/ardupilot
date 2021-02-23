//
// Created by asa on 2/18/21.
//

#include <utility>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "AP_InertialSensor_LORD.h"

extern const AP_HAL::HAL& hal;

//this will do stuff later
AP_InertialSensor_LORD::AP_InertialSensor_LORD(AP_InertialSensor &imu) : AP_InertialSensor_Backend(imu) {
    hal.console->printf("**LORD** in constructor\n");
}

//this is what gets called from detect_backends in frontend
AP_InertialSensor_Backend *
AP_InertialSensor_LORD::probe(AP_InertialSensor &imu)
{
    auto sensor = new AP_InertialSensor_LORD(imu);
    return sensor;
}

//PUBLIC METHODS
void AP_InertialSensor_LORD::start()
{
    hal.console->printf("**LORD** in start()\n");
}

bool AP_InertialSensor_LORD::update()
{
    hal.console->printf("**LORD** in update()\n");
    return true;
}