// #include <Arduino.h>

#include "leg.h"
#include "config.h"
// #include "debug.h"

#include <cmath>

using namespace hexapod::config;

// Angle (CCW)
//          0 : + X-axis
//         90 : + Y-axis
//        180 : - X-axis
//        270 : - Y-axis

namespace hexapod {

    namespace {

        // rotate matrix:
        //  x' = x * cos - y * sin
        //  y' = x * sin + y * cos

        #define SIN45   0.7071
        #define COS45   0.7071

        void rotate0(const Point3D& src, Point3D& dest) {
            dest = src;
        }

        void rotate45(const Point3D& src, Point3D& dest) {
            dest.x_ = src.x_ * COS45 - src.y_ * SIN45;
            dest.y_ = src.x_ * SIN45 + src.y_ * COS45;
            dest.z_ = src.z_;
        }

        void rotate135(const Point3D& src, Point3D& dest) {
            dest.x_ = src.x_ * (-COS45) - src.y_ * SIN45;
            dest.y_ = src.x_ * SIN45 + src.y_ * (-COS45);
            dest.z_ = src.z_;
        }

        void rotate180(const Point3D& src, Point3D& dest) {
            dest.x_ = -src.x_;
            dest.y_ = -src.y_;
            dest.z_ = src.z_;
        }

        void rotate225(const Point3D& src, Point3D& dest) {
            dest.x_ = src.x_ * (-COS45) - src.y_ * (-SIN45);
            dest.y_ = src.x_ * (-SIN45) + src.y_ * (-COS45);
            dest.z_ = src.z_;
        }

        void rotate315(const Point3D& src, Point3D& dest) {
            dest.x_ = src.x_ * COS45 - src.y_ * (-SIN45);
            dest.y_ = src.x_ * (-SIN45) + src.y_ * COS45;
            dest.z_ = src.z_;
        }
    }

    // Public

    Leg::Leg(int legIndex): index_(legIndex) {
        switch(legIndex) {
        case 0: // 45 degree
            mountPosition_ = Point3D(O_Leg1_J1X, O_Leg1_J1Y, 0);
            localConv_ = rotate315;
            worldConv_ = rotate45;
            break;
        case 1: // 0 degree
            mountPosition_ = Point3D(O_Leg2_J1X, 0, 0);
            localConv_ = rotate0;
            worldConv_ = rotate0;
            break;
        case 2: // -45 or 315 degree
            mountPosition_ = Point3D(O_Leg1_J1X, -O_Leg1_J1Y, 0);
            localConv_ = rotate45;
            worldConv_ = rotate315;
            break;
        case 3: // -135 or 225 degree
            mountPosition_ = Point3D(-O_Leg1_J1X, -O_Leg1_J1Y, 0);
            localConv_ = rotate135;
            worldConv_ = rotate225;
            break;
        case 4: // 180 degree
            mountPosition_ = Point3D(-O_Leg2_J1X, 0, 0);
            localConv_ = rotate180;
            worldConv_ = rotate180;
            break;
        case 5: // 135 degree
            mountPosition_ = Point3D(-O_Leg1_J1X, O_Leg1_J1Y, 0);
            localConv_ = rotate225;
            worldConv_ = rotate135;
            break;
        }

        for(int i=0;i<3;i++)
            motors_[i] = new hexapod::Motor(legIndex, i);
    }

    Leg::~Leg() {
        for(int i=0;i<3;i++) {
            delete motors_[i];
            motors_[i] = nullptr;
        }
    }

    void Leg::translateToLocal(const Point3D& world, Point3D& local) {
        localConv_(world - mountPosition_, local);
    }

    void Leg::translateToWorld(const Point3D& local, Point3D& world) {
        worldConv_(local, world);
        world += mountPosition_;
    }

    void Leg::setJointAngle(float angle[3]) {
        Point3D to;
        _forwardKinematics(angle, to);
        moveTipLocal(to);
    }

    void Leg::moveTip(const Point3D& to, float Angles[3]) {
        // if (to == tipPos_)
        //     return;

        Point3D local;
        translateToLocal(to, local);
        // LOG_DEBUG("leg(%d) moveTip(%f,%f,%f)(%f,%f,%f)", index_, to.x_, to.y_, to.z_, local.x_, local.y_, local.z_);
        _move(local, Angles);
        tipPos_ = to;
        tipPosLocal_ = local;
    }

    const Point3D& Leg::getTipPosition(void) {
        return tipPos_;
    }

    void Leg::moveTipLocal(const Point3D& to) {
        // if (to == tipPosLocal_)
        //     return;

        // Point3D world;
        // translateToWorld(to, world);
        // _move(to);
        // tipPos_ = world;
        // tipPosLocal_ = to;
    }

    const Point3D& Leg::getTipPositionLocal(void) {
        return tipPosLocal_;
    }

    

    //
    // Private
    //
    const float pi = std::acos(-1);
    const float hpi = pi/2;

    void Leg::_forwardKinematics(float angle[3], Point3D& out) {
        float radian[3];
        for(int i=0; i<3; i++)
            radian[i] = pi * angle [i] / 180;

        float x = J1_J2 + (std::cos(radian[1]) * J2_J3) + (std::cos(radian[1] + radian[2]) * J3_Tip);

        out.x_ = std::cos(radian[0]) * x;
        out.y_ = std::sin(radian[0]) * x;
        out.z_ = (std::sin(radian[1]) * J2_J3) + (std::sin(radian[1] + radian[2]) * J3_Tip);

    }

    void Leg::_inverseKinematics(const Point3D& to, float angles[3]) {

        float x = to.x_;
        float y = to.y_;

        angles[0] = std::atan2(y, x) * 180 / pi;

        x = std::sqrt((x-J1_J2)*(x-J1_J2) + y*y);
        y = to.z_;
        float ar = std::atan2(y, x);
        float lr2 = x*x + y*y;
        float lr = std::sqrt(lr2);
        float a1 = std::acos((lr2 + J2_J3*J2_J3 - J3_Tip*J3_Tip)/(2*J2_J3*lr));
        float a2 = std::acos((lr2 - J2_J3*J2_J3 + J3_Tip*J3_Tip)/(2*J3_Tip*lr));
        angles[1] = (ar + a1) * 180 / pi;
        angles[2] = - ((a1 + a2)  * 180 / pi);
    }

    void Leg::_move(const Point3D& to, float Angles[3]) {
        float angles[3];
        _inverseKinematics(to, angles);
        for(int i=0; i<3; i++) {
            Angles[i]= motors_[i]->setAngle(angles[i]);
        }
    }

}
