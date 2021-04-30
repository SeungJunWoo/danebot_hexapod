#include "motor.h"

namespace hexapod
{

    namespace
    {
        int angle2pulse(float angle)
        {
            int pulse;
            pulse = round(angle * 1023.0 / 300.0) + 512;//xl320
            // pulse = round(angle * 4095.0 / 360.0) + 2048; //xh540
            return pulse;
        }

        float pulse2angle(int32_t pulse)
        {
            float angle;
            angle = (float)(pulse - 512) * 300.0 / 1023.0;//xl320
            // angle = (float)(pulse - 2048) * 360.0 / 4095.0; //xh540
            return angle;
        }

        int idAssign(int legIndex, int partIndex)
        {
            switch (legIndex)
            {
            case 0:
                return 1 + partIndex;
            case 1:
                return 4 + partIndex;
            case 2:
                return 7 + partIndex;
            case 3:
                return 10 + partIndex;
            case 4:
                return 13 + partIndex;
            case 5:
                return 16 + partIndex;
            default:
                return 0;
            }
        }
    } // namespace

    Motor::Motor(int legIndex, int partIndex)
    {
        motorID_ = idAssign(legIndex, partIndex);
        inverse_ = partIndex == 1 ? true : false; //conditional operator 0: false,  1: true, 2: false
        if (partIndex==0){
            range_upper_=42;
            range_lower_=-42;
        }
        else if (partIndex==1){
            range_upper_=80;
            range_lower_=-70;
        }
        else{
            range_upper_=0;
            range_lower_=-149;
        }
    }

    float Motor::setAngle(float angle)
    {
        if (angle < range_lower_){
            printf("[motor %d] path angle (%f) is smaller than minimum \n", motorID_, angle);
            angle = range_lower_;
        }
        else if (angle > range_upper_){
            printf("[motor %d] path angle (%f) is bigger than maximum \n", motorID_, angle);
            angle = range_upper_;
        }
        
        if(inverse_){
            angle= -angle;
        }
        
        return angle;
    }

    float Motor::getAngle()
    {
        // dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
        // dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, motorID_, ADDR_PRESENT_POSITION, (uint32_t *)&dxl_present_position, &dxl_error);
        // if (dxl_comm_result != COMM_SUCCESS)
        // {
        //     printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        // }
        // else if (dxl_error != 0)
        // {
        //     printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        // }
        // return pulse2angle(dxl_present_position);
        return 0;
    }

} // namespace hexapod