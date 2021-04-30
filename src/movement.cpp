#include "movement.h"
#include "config.h"
#include "trajectory.h"

#include <cstdlib>

using namespace hexapod::config;


namespace hexapod {

    Movement::Movement():
        mode_{0}, transiting_{false}, position_{initial_pose}
    {
    }

    void Movement::setMode(int newMode) {

        mode_ = newMode;

        index_ = 0;// 0,10 for all except rotatez(0,1,2..,19)
        remainTime_ = 200;
    }

    void Movement::setremain(){
        remainTime_ = 25;
    }

    const Locations& Movement::next(int elapsed, int direction) {

        if (elapsed <= 0)
            elapsed = 10;//table.stepDuration;

        if (remainTime_ <= 0) {
            index_ = (index_ + 1)%20;//table.length;
            remainTime_ = 25;//table.stepDuration;
        }
        if (elapsed >= remainTime_)
            elapsed = remainTime_;

        auto ratio = (float)elapsed / remainTime_;

        if (mode_ == 0){
            position_ += (initialStep() - position_)*ratio;
        }
        else if (mode_ == 1){
            position_ += (shiftingStep(direction,index_) - position_)*ratio;
        }
        else if (mode_ == 2){
            position_ += (turningStep(direction,index_) - position_)*ratio;
        }
        else{
            position_ += (onelegStep(index_) - position_)*ratio;
        }

        remainTime_ -= elapsed;

        return position_;
    }

    const Locations& Movement::rotation(float RPY[3], float XYZ[3]) {
        position_ = calMovement(RPY, XYZ);

        return position_;
    }
}
