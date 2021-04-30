#pragma once

#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0

#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_sdk.h" // Uses Dynamixel SDK library

#include <cmath>

// Control table address
#define ADDR_TORQUE_ENABLE 24     //xl320:24  1byte             // Control table address is different in Dynamixel model
#define ADDR_GOAL_POSITION 30    //xl320:30  2byte
#define ADDR_PRESENT_POSITION 37 //xl320:37  2byte

// Protocol version
#define PROTOCOL_VERSION 2.0 // See which protocol version is used in the Dynamixel

// Default setting
#define BAUDRATE 1000000           //9600, 57600, 115200, 1,000,000
#define DEVICENAME "/dev/ttyUSB0" // Check which port is being used on your controller \
                                  // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE 1  // Value for enabling the torque
#define TORQUE_DISABLE 0 // Value for disabling the torque

namespace hexapod
{

    class Motor
    {
    public:
        Motor(int legIndex, int partIndex);

        float setAngle(float angle);
        float getAngle(void);

        int getMotorNo(int legIndex, int partIndex);

    private:
        int motorID_;
        bool inverse_;
        int range_upper_;
        int range_lower_;
    };

} // namespace hexapod