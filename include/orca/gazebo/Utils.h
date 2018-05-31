// This file is a part of the ORCA framework.
// Copyright 2017, ISIR / Universite Pierre et Marie Curie (UPMC)
// Copyright 2018, Fuzzy Logic Robotics
// Main contributor(s): Antoine Hoarau, Ryan Lober, and
// Fuzzy Logic Robotics <info@fuzzylogicrobotics.com>
//
// ORCA is a whole-body reactive controller framework for robotics.
//
// This software is governed by the CeCILL-C license under French law and
// abiding by the rules of distribution of free software.  You can  use,
// modify and/ or redistribute the software under the terms of the CeCILL-C
// license as circulated by CEA, CNRS and INRIA at the following URL
// "http://www.cecill.info".
//
// As a counterpart to the access to the source code and  rights to copy,
// modify and redistribute granted by the license, users are provided only
// with a limited warranty  and the software's author,  the holder of the
// economic rights,  and the successive licensors  have only  limited
// liability.
//
// In this respect, the user's attention is drawn to the risks associated
// with loading,  using,  modifying and/or developing or reproducing the
// software by the user in light of its specific status of free software,
// that may mean  that it is complicated to manipulate,  and  that  also
// therefore means  that it is reserved for developers  and  experienced
// professionals having in-depth computer knowledge. Users are therefore
// encouraged to load and test the software's suitability as regards their
// requirements in conditions enabling the security of their systems and/or
// data to be ensured and,  more generally, to use and operate it in the
// same conditions as regards security.
//
// The fact that you are presently reading this means that you have had
// knowledge of the CeCILL-C license and that you accept its terms.

/** @file
 @copyright 2018 Fuzzy Logic Robotics <info@fuzzylogicrobotics.com>
 @author Antoine Hoarau
 @author Ryan Lober
*/

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
