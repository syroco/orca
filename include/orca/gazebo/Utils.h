//|  This file is a part of the ORCA framework.
//|
//|  Copyright 2018, Fuzzy Logic Robotics
//|  Copyright 2017, ISIR / Universite Pierre et Marie Curie (UPMC)
//|
//|  Main contributor(s): Antoine Hoarau, Ryan Lober, and
//|  Fuzzy Logic Robotics <info@fuzzylogicrobotics.com>
//|
//|  ORCA is a whole-body reactive controller framework for robotics.
//|
//|  This software is governed by the CeCILL-C license under French law and
//|  abiding by the rules of distribution of free software.  You can  use,
//|  modify and/ or redistribute the software under the terms of the CeCILL-C
//|  license as circulated by CEA, CNRS and INRIA at the following URL
//|  "http://www.cecill.info".
//|
//|  As a counterpart to the access to the source code and  rights to copy,
//|  modify and redistribute granted by the license, users are provided only
//|  with a limited warranty  and the software's author,  the holder of the
//|  economic rights,  and the successive licensors  have only  limited
//|  liability.
//|
//|  In this respect, the user's attention is drawn to the risks associated
//|  with loading,  using,  modifying and/or developing or reproducing the
//|  software by the user in light of its specific status of free software,
//|  that may mean  that it is complicated to manipulate,  and  that  also
//|  therefore means  that it is reserved for developers  and  experienced
//|  professionals having in-depth computer knowledge. Users are therefore
//|  encouraged to load and test the software's suitability as regards their
//|  requirements in conditions enabling the security of their systems and/or
//|  data to be ensured and,  more generally, to use and operate it in the
//|  same conditions as regards security.
//|
//|  The fact that you are presently reading this means that you have had
//|  knowledge of the CeCILL-C license and that you accept its terms.
#pragma once
#include <Eigen/Geometry>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Pose3.hh>
#include <gazebo/msgs/wrench_stamped.pb.h>
#include <orca/math/Utils.h>

inline Eigen::Matrix<double,6,1>& gazeboMsgToEigen(const ::gazebo::msgs::WrenchStamped& w, Eigen::Matrix<double,6,1>& e_out)
{
    e_out[0] = w.wrench().force().x();
    e_out[1] = w.wrench().force().y();
    e_out[2] = w.wrench().force().z();
    e_out[3] = w.wrench().torque().x();
    e_out[4] = w.wrench().torque().y();
    e_out[5] = w.wrench().torque().z();
    return e_out;
}

inline Eigen::Matrix<double,6,1> gazeboMsgToEigen(const ::gazebo::msgs::WrenchStamped& w)
{
    Eigen::Matrix<double,6,1> e;
    return gazeboMsgToEigen(w, e);
}

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

inline Eigen::Vector3d convVec3(const gazebo::msgs::Vector3d &_vec3)
{
    return Eigen::Vector3d(_vec3.x(),_vec3.y(),_vec3.z());
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


// Allows to use the gazebo command line
#include <stdio.h>
inline std::string custom_exec(const std::string &_cmd)
{
  FILE* pipe = popen(_cmd.c_str(), "r");

  if (!pipe)
    return "ERROR";

  char buffer[128];
  std::string result = "";

  while (!feof(pipe))
  {
    if (fgets(buffer, 128, pipe) != NULL)
      result += buffer;
  }

  pclose(pipe);
  return result;
}


namespace orca
{
namespace gazebo
{

/**
 * \class GazeboWrenchSensor
 * \brief A Wrench sensor w.r.t. a base_link. Wrench is expressed from the world to the body, in the base_link frame.
 *
 * This class is a wrapper to a ForceTorqueSensor or ContactSensor to store frame information.
 * Indeed there is no easy way to get the attributes of the Gazebo sensor.
 */
class GazeboWrenchSensor
{
public:
    GazeboWrenchSensor()
    {}

    virtual ~GazeboWrenchSensor()
    {}

protected:
    GazeboWrenchSensor(const std::string& base_link)
    : base_link_(base_link)
    {}

public:
    /**
     * \brief returns the wrench in the base_link frame
     */
    const Eigen::Matrix<double,6,1>& getWrench() const
    {
        return wrench_;
    }

    /**
     * \brief returns the wrench in the base_link name, from the world to the body
     */
    const std::string& getBaseLinkName()
    {
        return base_link_;
    }

protected:
    void setConnection(::gazebo::event::ConnectionPtr conn_ptr)
    {
        connection_ptr_ = conn_ptr;
    }

    /**
     * \brief sets the wrench in the base_link frame
     * \warning the input wrench should be in the base_link frame, from the world to the body
     */
    void setWrench(const Eigen::Matrix<double,6,1>& wrench)
    {
        wrench_ = wrench;
    }

    Eigen::Matrix<double,6,1>& wrench()
    {
        return wrench_;
    }

private:
    ::gazebo::event::ConnectionPtr connection_ptr_;
    std::string base_link_;
    Eigen::Matrix<double,6,1> wrench_;

}; // GazeboWrenchSensor

class GazeboForceTorqueSensor : public GazeboWrenchSensor
{
public:
    GazeboForceTorqueSensor(const std::string& base_link)
    : GazeboWrenchSensor(base_link)
    {}

    GazeboForceTorqueSensor(::gazebo::physics::LinkPtr link)
    : GazeboWrenchSensor(link->GetName())
    {}

public:
    void connectUpdate(std::function<void (::gazebo::msgs::WrenchStamped)> subscriber)
    {
        if (sensor_ != nullptr)
            setConnection(
                sensor_->ConnectUpdate(subscriber)
            );
    }

    /**
     * \brief sets the ForceTorqueSensorPtr to connect to
     * \warning the ForceTorqueSensorPtr should be with attributes <measure_direction>child_to_parent</measure_direction> <frame>child</frame>
     */
    void setSensor(::gazebo::sensors::ForceTorqueSensorPtr sensor)
    {
        sensor_ = sensor;
    }

    ::gazebo::sensors::ForceTorqueSensorPtr getSensor() const
    {
        return sensor_;
    }

    void setWrenchFrom(const ::gazebo::msgs::WrenchStamped& wrench_stamped)
    {
        gazeboMsgToEigen(wrench_stamped, wrench());
    }

private:
    ::gazebo::sensors::ForceTorqueSensorPtr sensor_ = nullptr;
}; // GazeboForceTorqueSensor

class GazeboContactSensor : public GazeboWrenchSensor
{
public:
    GazeboContactSensor(::gazebo::physics::LinkPtr link, const std::string& collision)
    : GazeboWrenchSensor(link->GetName()), collision_(collision), link_(link)
    {}

public:
    void connectUpdate(std::function<void (void)> subscriber)
    {
        if (sensor_ != nullptr)
            setConnection(
                sensor_->ConnectUpdated(subscriber)
            );
    }

    const std::string& getCollisionName() const
    {
        return collision_;
    }

    /**
     * \brief sets the ContactSensorPtr to get contact messages from
     */
    void setSensor(::gazebo::sensors::ContactSensorPtr sensor)
    {
        sensor_ = sensor;
    }

    ::gazebo::sensors::ContactSensorPtr getSensor() const
    {
        return sensor_;
    }

    ::gazebo::physics::LinkPtr getLink() const
    {
        return link_;
    }

    void setWrenchFrom( void )
    {
        /**
         * \note It appears that JointWrench in the contact sensor are in the link reference frame as of PR #355, as opposed to what's indicated in the doxygen
         * Follow comment here: https://bitbucket.org/osrf/gazebo/commits/16853329fb63d0509c88655cc5a52f9174e87949
         * Doxygen should be
         * https://bitbucket.org/osrf/gazebo/pull-requests/355/bullet-contact-sensor/diff#chg-gazebo/physics/Contact.hh
         * https://bitbucket.org/osrf/gazebo/pull-requests/336/cast-getforcetorque-values-from-world/diff
         * https://bitbucket.org/osrf/gazebo/issues/545/request-change-contact-reference-frame
         *
        */
        if (link_ == nullptr)
            throw std::runtime_error("Link not set.");
        ::gazebo::msgs::Contacts contacts = this->getSensor()->Contacts();
        // "The forces and torques are applied at the CG of perspective links for each collision body, specified in the inertial frame."
        // -> appears to be wrong, forces and torques in the LINK frame w.r.t COG
        Eigen::Matrix<double,6,1> wrench_res_atCoG_inLink = Eigen::Matrix<double,6,1>::Zero();
        auto linkCoGPose = this->getLink()->GetInertial()->Pose(); // the Pose of the CoG in the link Frame
        std::string scoped_collision_name = link_->GetModel()->GetName() + "::" + this->getBaseLinkName() + "::" + this->getCollisionName();
        for (unsigned int i = 0; i < contacts.contact_size(); ++i)
        {
            for (unsigned int j = 0; j < contacts.contact(i).wrench_size(); ++j)
            {
                if (0 == contacts.contact(i).wrench(j).body_1_name().compare(scoped_collision_name))
                {
                    wrench_res_atCoG_inLink.head<3>() += convVec3(contacts.contact(i).wrench(j).body_1_wrench().force());
                    wrench_res_atCoG_inLink.tail<3>() += convVec3(contacts.contact(i).wrench(j).body_1_wrench().torque());
                }
                else if (0 == contacts.contact(i).wrench(j).body_2_name().compare(scoped_collision_name))
                {
                    wrench_res_atCoG_inLink.head<3>() += convVec3(contacts.contact(i).wrench(j).body_2_wrench().force());
                    wrench_res_atCoG_inLink.tail<3>() += convVec3(contacts.contact(i).wrench(j).body_2_wrench().torque());
                }
                else
                    throw std::runtime_error("Mismatching collision bodies names");
            }
        }
        orca::math::transportWrench(wrench_res_atCoG_inLink, convVec3(linkCoGPose.Pos()) , this->wrench());
    }

private:
    ::gazebo::sensors::ContactSensorPtr sensor_ = nullptr;
    ::gazebo::physics::LinkPtr link_ = nullptr;
    std::string collision_;
}; // GazeboContactSensor


} // namespace gazebo
} // namespace orca
