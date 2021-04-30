#pragma once

#include "base.h"
#include "motor.h"

#include <functional>

namespace hexapod {

    class Leg {
    public:
        Leg(int legIndex);
        virtual ~Leg();

        // coordinate system translation
        void translateToLocal(const Point3D& world, Point3D& local);
        void translateToWorld(const Point3D& local, Point3D& world);

        void setJointAngle(float angle[3]);

        // world coordinate system (default)
        void moveTip(const Point3D& to,float Angles[3]);
        const Point3D& getTipPosition(void);

        // local coordinate system version
        void moveTipLocal(const Point3D& to);
        const Point3D& getTipPositionLocal(void);

    private:
        // Local coorinate system
        static void _forwardKinematics(float angle[3], Point3D& out);
        static void _inverseKinematics(const Point3D& to, float angles[3]);
        void _move(const Point3D& to,  float Angles[3]);

    private:
        int index_;
        Motor* motors_[3];
        Point3D mountPosition_;
        std::function<void(const Point3D& src, Point3D& dest)> localConv_;
        std::function<void(const Point3D& src, Point3D& dest)> worldConv_;

        Point3D tipPos_;
        Point3D tipPosLocal_;

    };

}
