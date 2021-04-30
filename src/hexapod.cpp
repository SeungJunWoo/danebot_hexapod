
#include "hexapod.h"
#include "motor.h"

#include <wiringPi.h>

#include <iostream>
#include <unistd.h> //usleep
#include "movement.h"

namespace hexapod
{

    HexapodClass Hexapod;

    HexapodClass::HexapodClass() : legs_{{0}, {1}, {2}, {3}, {4}, {5}}
    {
    }

    bool HexapodClass::init(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler)
    {
        bool flag = true;
        for (int i = 0; i < 6; i++)
        {
            for (int k = 0; k < 3; k++)
            {
                int dxl_comm_result = COMM_TX_FAIL;
                uint8_t dxl_error = 0;
                int motorID_ = 3 * i + k + 1;
                dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, motorID_, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS)
                {
                    printf("[%d] Torque on %s\n", motorID_, packetHandler->getTxRxResult(dxl_comm_result));
                    flag = false;
                }
                else if (dxl_error != 0)
                {
                    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
                    flag = false;
                }
                else
                {
                    printf("Dynamixel has been successfully connected \n");
                }
            }
        }
        return flag;
    }

    const Locations &HexapodClass::processMovement(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, int mode, int elapsed, joystruct JS, float AllAngles[18], bool ros_on)
    {
        int direction = joystick_direction(JS);

        if (mode_ != mode)
        {
            mode_ = mode;
            movement_.setMode(mode_);
        }
        else
        {
            if (direction_ != direction)
            {
                movement_.setremain();
            }
        }

        direction_ = direction;

        auto &location = movement_.next(elapsed, direction);

        if (!ros_on)
        {
            dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, 2);
            bool dxl_addparam_result = false;
            for (int i = 0; i < 6; i++)
            {
                float LegAngles[3];
                legs_[i].moveTip(location.get(i), LegAngles);

                for (int k = 0; k < 3; k++)
                {
                    int LegPulse = round(LegAngles[k] * 1023.0 / 300.0) + 512;
                    uint8_t param_goal_position[4];
                    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(LegPulse));
                    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(LegPulse));
                    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(LegPulse));
                    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(LegPulse));

                    // Add Dynamixel goal position value to the Syncwrite storage
                    dxl_addparam_result = groupSyncWrite.addParam(3 * i + k + 1, param_goal_position);
                    if (dxl_addparam_result != true)
                    {
                        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", 3 * i + k + 1);
                    }
                }
            }
            groupSyncWrite.txPacket();
            groupSyncWrite.clearParam();
        }
        else
        {
            for (int i = 0; i < 6; i++)
            {
                float LegAngles[3];
                legs_[i].moveTip(location.get(i), LegAngles);

                for (int k = 0; k < 3; k++)
                {
                    AllAngles[(3*i)+k] = LegAngles[k];
                }
            }
        }

        return location;
    }

    const Locations &HexapodClass::scenarioMovement(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, struct Gesture gesture, float AllAngles[18], bool ros_on)
    { //gesture
        if (initialized != true)
        {
            motionindex = 0;
            motion_initialized = false;
            initialized = true;
        }

        if (!motion_initialized)
        {
            // goal roll, pitch, yaw, x,y,z setting
            RPY_end[0] = gesture.goal_roll_deg[motionindex];
            RPY_end[1] = gesture.goal_pitch_deg[motionindex];
            RPY_end[2] = gesture.goal_yaw_deg[motionindex];
            XYZ_end[0] = gesture.goal_x[motionindex];
            XYZ_end[1] = gesture.goal_y[motionindex];
            XYZ_end[2] = gesture.goal_z[motionindex];

            // initial roll, pitch, yaw setting
            if (motionindex == 0)
            {
                RPY_init[0] = 0;
                RPY_init[1] = 0;
                RPY_init[2] = 0;
                XYZ_init[0] = 0;
                XYZ_init[1] = 0;
                XYZ_init[2] = 0;
            }
            else
            {
                RPY_init[0] = gesture.goal_roll_deg[motionindex - 1]; // initial point(k)=goal point(k-1)
                RPY_init[1] = gesture.goal_pitch_deg[motionindex - 1];
                RPY_init[2] = gesture.goal_yaw_deg[motionindex - 1];
                XYZ_init[0] = gesture.goal_x[motionindex - 1]; 
                XYZ_init[1] = gesture.goal_y[motionindex - 1];
                XYZ_init[2] = gesture.goal_z[motionindex - 1];
            }

            Time = 0; //motion duration

            for (uint8_t i = 0; i < 3; i++) // settng "Time" to the longest duration among three durations.
            {
                RPY_sub[i] = RPY_end[i] - RPY_init[i];
                XYZ_sub[i] = XYZ_end[i] - XYZ_init[i];
                if ((abs(RPY_sub[i]) / (float)gesture.V_ISO[motionindex]) * 1000 > Time)
                {
                    Time = (int)floor(((abs(RPY_sub[i]) / (float)gesture.V_ISO[motionindex]) * 1000) + 0.5);
                }
                if ((abs(XYZ_sub[i]) / (float)gesture.V_ISO[motionindex]) * 1000 > Time)
                {
                    Time = (int)floor(((abs(XYZ_sub[i]) / (float)gesture.V_ISO[motionindex]) * 1000) + 0.5);
                }
            }

            start_time = millis();
            end_time = start_time + Time;

            motion_initialized = true;
        }

        if (Time == 0)
        {
        }
        t = millis() - start_time;

        for (uint8_t i = 0; i < 3; i++) // calculate roll, pitch, yaw _d deg
        {
            RPY_d[i] = (RPY_init[i] + (10.0 * (RPY_sub[i]) / Time / Time / Time * t * t * t) + (-15.0 * (RPY_sub[i]) / Time / Time / Time / Time * t * t * t * t) + (6.0 * (RPY_sub[i]) / Time / Time / Time / Time / Time * t * t * t * t * t));
            XYZ_d[i] = (XYZ_init[i] + (10.0 * (XYZ_sub[i]) / Time / Time / Time * t * t * t) + (-15.0 * (XYZ_sub[i]) / Time / Time / Time / Time * t * t * t * t) + (6.0 * (XYZ_sub[i]) / Time / Time / Time / Time / Time * t * t * t * t * t));
        }

        auto &location = movement_.rotation(RPY_d, XYZ_d);

        if (!ros_on)
        {
            dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, 2);
            bool dxl_addparam_result = false;
            for (int i = 0; i < 6; i++)
            {
                float LegAngles[3];
                legs_[i].moveTip(location.get(i), LegAngles);

                for (int k = 0; k < 3; k++)
                {
                    int LegPulse = round(LegAngles[k] * 1023.0 / 300.0) + 512;
                    uint8_t param_goal_position[4];
                    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(LegPulse));
                    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(LegPulse));
                    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(LegPulse));
                    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(LegPulse));

                    // Add Dynamixel goal position value to the Syncwrite storage
                    dxl_addparam_result = groupSyncWrite.addParam(3 * i + k + 1, param_goal_position);
                    if (dxl_addparam_result != true)
                    {
                        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", 3 * i + k + 1);
                    }
                }
            }
            groupSyncWrite.txPacket();
            groupSyncWrite.clearParam();
        }
        else
        {
            for (int i = 0; i < 6; i++)
            {
                float LegAngles[3];
                legs_[i].moveTip(location.get(i), LegAngles);

                for (int k = 0; k < 3; k++)
                {
                    AllAngles[(3*i)+k] = LegAngles[k];
                }
            }
        }

        if (end_time < ((int)millis() + 5))
        {
            // Change goal position
            if (motionindex == gesture.number_of_traj - 1) //if every small motions are ended
            {
                initialized = false;
                motion_initialized = false;
            }
            else
            {
                motionindex += 1;
                motion_initialized = false;
            }
        }

        return location;
    }


    // const Locations &HexapodClass::spinMovement(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, struct Gesture gesture, float AllAngles[18], bool ros_on)
    // { //gesture
    //     if (initialized != true)
    //     {
    //         motionindex = 0;
    //         motion_initialized = false;
    //         initialized = true;
    //     }

    //     if (!motion_initialized)
    //     {
    //         // goal roll, pitch, yaw, x,y,z setting
    //         RPY_end[0] = gesture.goal_roll_deg[motionindex];
    //         RPY_end[1] = gesture.goal_pitch_deg[motionindex];
    //         RPY_end[2] = gesture.goal_yaw_deg[motionindex];
    //         XYZ_end[0] = gesture.goal_x[motionindex];
    //         XYZ_end[1] = gesture.goal_y[motionindex];
    //         XYZ_end[2] = gesture.goal_z[motionindex];

    //         // initial roll, pitch, yaw setting
    //         if (motionindex == 0)
    //         {
    //             RPY_init[0] = 0;
    //             RPY_init[1] = 0;
    //             RPY_init[2] = 0;
    //             XYZ_init[0] = 0;
    //             XYZ_init[1] = 0;
    //             XYZ_init[2] = 0;
    //         }
    //         else
    //         {
    //             RPY_init[0] = gesture.goal_roll_deg[motionindex - 1]; // initial point(k)=goal point(k-1)
    //             RPY_init[1] = gesture.goal_pitch_deg[motionindex - 1];
    //             RPY_init[2] = gesture.goal_yaw_deg[motionindex - 1];
    //             XYZ_init[0] = gesture.goal_x[motionindex - 1]; 
    //             XYZ_init[1] = gesture.goal_y[motionindex - 1];
    //             XYZ_init[2] = gesture.goal_z[motionindex - 1];
    //         }

    //         Time = 0; //motion duration

    //         for (uint8_t i = 0; i < 3; i++) // settng "Time" to the longest duration among three durations.
    //         {
    //             RPY_sub[i] = RPY_end[i] - RPY_init[i];
    //             XYZ_sub[i] = XYZ_end[i] - XYZ_init[i];
    //             if ((abs(RPY_sub[i]) / (float)gesture.V_ISO) * 1000 > Time)
    //             {
    //                 Time = (int)floor(((abs(RPY_sub[i]) / (float)gesture.V_ISO) * 1000) + 0.5);
    //             }
    //             if ((abs(XYZ_sub[i]) / (float)gesture.V_ISO) * 1000 > Time)
    //             {
    //                 Time = (int)floor(((abs(XYZ_sub[i]) / (float)gesture.V_ISO) * 1000) + 0.5);
    //             }
    //         }

    //         start_time = millis();
    //         end_time = start_time + Time;

    //         motion_initialized = true;
    //     }

    //     if (Time == 0)
    //     {
    //     }
    //     t = millis() - start_time;

    //     for (uint8_t i = 0; i < 3; i++) // calculate roll, pitch, yaw _d deg
    //     {
    //         RPY_d[i] = (RPY_init[i] + (10.0 * (RPY_sub[i]) / Time / Time / Time * t * t * t) + (-15.0 * (RPY_sub[i]) / Time / Time / Time / Time * t * t * t * t) + (6.0 * (RPY_sub[i]) / Time / Time / Time / Time / Time * t * t * t * t * t));
    //         XYZ_d[i] = (XYZ_init[i] + (10.0 * (XYZ_sub[i]) / Time / Time / Time * t * t * t) + (-15.0 * (XYZ_sub[i]) / Time / Time / Time / Time * t * t * t * t) + (6.0 * (XYZ_sub[i]) / Time / Time / Time / Time / Time * t * t * t * t * t));
    //     }

    //     auto &location = movement_.rotation(RPY_d, XYZ_d);

    //     if (!ros_on)
    //     {
    //         dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, 2);
    //         bool dxl_addparam_result = false;
    //         for (int i = 0; i < 6; i++)
    //         {
    //             float LegAngles[3];
    //             legs_[i].moveTip(location.get(i), LegAngles);

    //             for (int k = 0; k < 3; k++)
    //             {
    //                 int LegPulse = round(LegAngles[k] * 1023.0 / 300.0) + 512;
    //                 uint8_t param_goal_position[4];
    //                 param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(LegPulse));
    //                 param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(LegPulse));
    //                 param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(LegPulse));
    //                 param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(LegPulse));

    //                 // Add Dynamixel goal position value to the Syncwrite storage
    //                 dxl_addparam_result = groupSyncWrite.addParam(3 * i + k + 1, param_goal_position);
    //                 if (dxl_addparam_result != true)
    //                 {
    //                     fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", 3 * i + k + 1);
    //                 }
    //             }
    //         }
    //         groupSyncWrite.txPacket();
    //         groupSyncWrite.clearParam();
    //     }
    //     else
    //     {
    //         for (int i = 0; i < 6; i++)
    //         {
    //             float LegAngles[3];
    //             legs_[i].moveTip(location.get(i), LegAngles);

    //             for (int k = 0; k < 3; k++)
    //             {
    //                 AllAngles[(3*i)+k] = LegAngles[k];
    //             }
    //         }
    //     }

    //     if (end_time < ((int)millis() + 5))
    //     {
    //         // Change goal position
    //         if (motionindex == gesture.number_of_traj - 1) //if every small motions are ended
    //         {
    //             initialized = false;
    //             motion_initialized = false;
    //         }
    //         else
    //         {
    //             motionindex += 1;
    //             motion_initialized = false;
    //         }
    //     }

    //     return location;
    // }
}