#pragma once

#include "movement.h"
#include "leg.h"
#include "trajectory.h"

#include "joystick.h"
#include <iostream>

namespace hexapod {

    class HexapodClass {
    public:
        HexapodClass();

        // init

        bool init(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler);

        // Movement API

        const Locations& processMovement(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, int mode, int elapsed, joystruct JS, float AllAngles[18], bool ros_on);
        const Locations& scenarioMovement(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, struct Gesture gesture, float AllAngles[18], bool ros_on);
        const Locations& spinMovement(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, struct Gesture gesture, float AllAngles[18], bool ros_on);

        // scenario
        bool initialized;

    private:
        int mode_;
        int direction_;
        Movement movement_;
        Leg legs_[6];

        // scenario
        int motionindex;

        float RPY_d[3];    //{roll,pitch,yaw} [deg]
        float RPY_init[3]; //{roll,pitch,yaw} initial [deg]
        float RPY_end[3];  //{roll,pitch,yaw} goal [deg]
        float RPY_sub[3];  //{roll,pitch,yaw} goal-initial [deg]

        float XYZ_d[3];    //{x,y,z} [mm]
        float XYZ_init[3]; //{x,y,z} initial [mm]
        float XYZ_end[3];  //{x,y,z} goal [mm]
        float XYZ_sub[3];  //{x,y,z} goal-initial [mm]

        int start_time;
        int Time; //time duration
        int end_time;
        int t; //present time

        bool motion_initialized;
    };

    // extern HexapodClass Hexapod;
}
