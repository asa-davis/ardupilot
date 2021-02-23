//
// Created by asa on 2/18/21.
//

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_LORD : public AP_InertialSensor_Backend {
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu);

    /**
     * Configure the sensors and start reading routine.
     */
    void start() override;
    bool update() override;

private:
    AP_InertialSensor_LORD(AP_InertialSensor &imu);

};
