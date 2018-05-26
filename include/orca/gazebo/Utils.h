#pragma once
#include <Eigen/Geometry>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Pose3.hh>

inline Eigen::Quaterniond quatFromRPY(double roll,double pitch,double yaw )
{
    return Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) 
        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
}

// Adapted from DARTTypes.hh
inline Eigen::Vector3d convVec3(const ignition::math::Vector3d &_vec3)
{
    return Eigen::Vector3d(_vec3.X(),_vec3.Y(),_vec3.Z());
}

inline ignition::math::Vector3d convVec3(const Eigen::Vector3d &_vec3)
{
    return ignition::math::Vector3d(_vec3[0], _vec3[1], _vec3[2]);
}

inline Eigen::Quaterniond convQuat(const ignition::math::Quaterniond &_quat)
{
    return Eigen::Quaterniond(_quat.W(), _quat.X(), _quat.Y(), _quat.Z());
}

inline ignition::math::Quaterniond convQuat(const Eigen::Quaterniond &_quat)
{
    return ignition::math::Quaterniond(_quat.w(), _quat.x(), _quat.y(), _quat.z());
}

inline Eigen::Affine3d convPose(const ignition::math::Pose3d &_pose)
{
    Eigen::Affine3d res;

    res.translation() = convVec3(_pose.Pos());
    res.linear() = convQuat(_pose.Rot()).toRotationMatrix();

    return res;
}

inline ignition::math::Pose3d convPose(const Eigen::Affine3d &_T)
{
    ignition::math::Pose3d pose;
    pose.Pos() = convVec3(_T.translation());
    pose.Rot() = convQuat(Eigen::Quaterniond(_T.linear()));
    return pose;
}
