#pragma once

#include "movement.h"
#include "config.h"
#include <eigen3/Eigen/Core>
#include <math.h>
#include <cmath>
#include "joystick.h"

namespace hexapod {
    extern const Locations initial_pose;
    Locations initialStep();
    Locations shiftingStep(int direction, int index);
    Locations turningStep(int direction, int index);
    Locations climbingStep(int direction, int index);
    Locations onelegStep(int index);
    void shiftingStep_generator(int index);
    void turningStep_generator(int index);
    Locations calMovement(float RPY[3],float XYZ[3]);

    struct Gesture{
        int goal_roll_deg[20];
        int goal_pitch_deg[20];
        int goal_yaw_deg[20];
        int goal_x[20];
        int goal_y[20];
        int goal_z[20];
        int V_ISO[20];
        int number_of_traj;
    };

    extern Gesture Gesture_1;
    extern Gesture Gesture_2;
    extern Gesture Gesture_3;
    extern Gesture Gesture_4;
    extern Gesture Gesture_5;
    extern Gesture Gesture_6;
    extern Gesture Gesture_7;
    extern Gesture Gesture_8;
}
