#pragma once

namespace hexapod { 

    namespace config {
        // all below definition use unit: mm
        const float O_Leg2_J1X = 35.50;
        const float O_Leg1_J1X = 29.00;
        const float O_Leg1_J1Y = 63.00;
        
        const float J1_J2 = 31.00;
        const float J2_J3 = 54.00;
        const float J3_Tip = 75.00;


        // timing setting. unit: ms
        const int movementInterval = 5;
        const int movementSwitchDuration = 150;
    }

}
