#pragma once

#include "CommonStructs.hpp"

namespace simple_flight
{

class IStateEstimator
{
public:
    enum class GroundContactState
    {
        Unknown,
        Airborne,
        Grounded
    };

    // Optional contact information. Kinematics alone cannot distinguish a hover
    // from rest on a surface; estimators without contact sensing use Unknown.
    virtual GroundContactState getGroundContactState() const
    {
        return GroundContactState::Unknown;
    }

    virtual Axis3r getAngles() const = 0;
    virtual Axis3r getAngularVelocity() const = 0;
    virtual Axis3r getPosition() const = 0;
    virtual Axis3r getLinearVelocity() const = 0;
    virtual Axis4r getOrientation() const = 0;
    virtual GeoPoint getGeoPoint() const = 0;

    virtual KinematicsState getKinematicsEstimated() const = 0;

    virtual GeoPoint getHomeGeoPoint() const = 0;

    virtual Axis3r transformToBodyFrame(const Axis3r& world_frame_val) const = 0;

    virtual ~IStateEstimator() = default;
};
}
