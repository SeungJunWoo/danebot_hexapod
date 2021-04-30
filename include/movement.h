#pragma once

#include "base.h"
#include "joystick.h"

namespace hexapod {

    class Movement {
    public:
        Movement();

        void setMode(int newMode);

        void setremain();

        const Locations& next(int elapsed,int direction);

        const Locations& rotation(float RPY[3], float XYZ[3]);

    private:
        int mode_;
        Locations position_;
        int index_;             // index in mode position table
        bool transiting_;       // if still in transiting to new mode
        int remainTime_;
    };

}
