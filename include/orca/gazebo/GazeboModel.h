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

// Gazebo headers
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "orca/gazebo/Utils.h"

namespace orca
{
namespace gazebo
{

class GazeboModel
{
public:
    GazeboModel(const std::string& name)
    : GazeboModel()
    {
        load(name);
    }

    GazeboModel(::gazebo::physics::ModelPtr model)
    : GazeboModel()
    {
        load(model);
    }

    const std::string& getName() const
    {
        assertModelLoaded();
        return name_;
    }

    const std::string& getBaseName()
    {
        assertModelLoaded();
        return base_name_;
    }
    
    void setJointMapping(const std::vector<std::string>& joint_names)
    {
        assertModelLoaded();
        
        if(joint_names.size() != actuated_joint_names_.size())
        {
            std::cerr << "[GazeboModel] Size of provided joint names does not match actual configuration" 
                << "\nCurrent config has " << actuated_joint_names_.size() << " actuated joints."
                << "\nProvided config has " << joint_names.size() << " actuated joints."
                << '\n';
            return;
        }
        
        joints_.clear();
        for(auto joint_name : joint_names)
        {
            auto joint = model_->GetJoint(joint_name);
            if(!joint)
            {
                //print();
                throw std::runtime_error("Joint " + joint_name + " does not exists in model");
            }
            joints_.push_back(joint);
        }
        
        this->actuated_joint_names_ = joint_names;
    }
    
    bool setModelConfiguration(const std::vector<std::string>& joint_names,const std::vector<double>& joint_positions)
    {
        assertModelLoaded();

        if (joint_names.size() != joint_positions.size())
        {
            std::cerr << "[GazeboModel \'" << getName() << "\'] " << "joint_names lenght should be the same as joint_positions : " << joint_names.size() << " vs " << joint_positions.size() << '\n';
            return false;
        }

        auto world = ::gazebo::physics::get_world();
        // make the service call to pause gazebo
        bool is_paused = world->IsPaused();
        if (!is_paused) world->SetPaused(true);

        std::map<std::string, double> joint_position_map;
        for (unsigned int i = 0; i < joint_names.size(); i++)
        {
          joint_position_map[joint_names[i]] = joint_positions[i];
        }

        model_->SetJointPositions(joint_position_map);

        // resume paused state before this call
        world->SetPaused(is_paused);

        return true;
    }

    bool load(const std::string& model_name)
    {
        if(!loadWorld())
            return false;

        #if GAZEBO_MAJOR_VERSION > 8
            auto model = world_->ModelByName( model_name );
        #else
            auto model = world_->GetModel( model_name );
        #endif
        if(! model)
        {
            std::cerr << "[GazeboModel \'" << getName() << "\'] " << "Could not get gazebo model " << model_name << ". Make sure it is loaded in the server" << '\n';
            return false;
        }
        return load(model);
    }

    bool load(::gazebo::physics::ModelPtr model)
    {
        if(!model)
        {
            std::cerr << "Model is invalid" << '\n';
            return false;
        }
        if(!loadWorld())
            return false;

        joints_.clear();
        actuated_joint_names_.clear();
        for(auto joint : model->GetJoints() )
        {
            // Provide feedback to get the internal torque
            joint->SetProvideFeedback(true);
            // Not adding fixed joints
            bool added = false;
            if( true
                #if GAZEBO_MAJOR_VERSION >= 7
                    && joint->GetType() != ::gazebo::physics::Joint::FIXED_JOINT
                #endif
                #if GAZEBO_MAJOR_VERSION > 8
                    && joint->LowerLimit(0u) != joint->UpperLimit(0u)
                    && joint->LowerLimit(0u) != 0
                    && !std::isnan(joint->LowerLimit(0u))
                    && !std::isnan(joint->UpperLimit(0u))
                #else
                    && joint->GetLowerLimit(0u) != joint->GetUpperLimit(0u)
                    && joint->GetLowerLimit(0u).Radian() != 0
                    && !std::isnan(joint->GetLowerLimit(0u).Radian())
                    && !std::isnan(joint->GetUpperLimit(0u).Radian())
                #endif
            )
            {
                joints_.push_back(joint);
                actuated_joint_names_.push_back(joint->GetName());
                added = true;
            }

            std::cout << "[GazeboModel \'" << model->GetName() << "\'] " << (added ? "Adding":"Not adding")
                << (added ? "" : " fixed/invalid") << " joint " << joint->GetName()
                << " type (" << joint->GetType() << ")"
                #if GAZEBO_MAJOR_VERSION > 8
                << " lower limit " << joint->LowerLimit(0u)
                << " upper limit " << joint->UpperLimit(0u)
                #else
                << " lower limit " << joint->GetLowerLimit(0u)
                << " upper limit " << joint->GetUpperLimit(0u)
                #endif
                << '\n';
        }

        if(actuated_joint_names_.size() == 0)
        {
            std::cerr << "[GazeboModel \'" << model->GetName() << "\'] " << "Could not get any actuated joint for model " << model->GetName() << '\n';
            return false;
        }
        

        model_ = model;
        
        links_ = model->GetLinks();
        name_ = model->GetName();
        
        ndof_ = actuated_joint_names_.size();
        current_joint_positions_.setZero(ndof_);
        joint_gravity_torques_.setZero(ndof_);
        current_joint_velocities_.setZero(ndof_);
        current_joint_external_torques_.setZero(ndof_);
        joint_torque_command_.setZero(ndof_);
        current_joint_measured_torques_.setZero(ndof_);

        print();

        return true;
    }

    ::gazebo::sensors::ForceTorqueSensorPtr attachForceTorqueSensorToJoint(const std::string& joint_name,double update_rate = 0)
    {
        assertModelLoaded();
        
    #if GAZEBO_MAJOR_VERSION > 8
        auto joint = model_->GetJoint(joint_name);
        if(!joint)
            throw std::runtime_error("Joint " + joint_name + " does not exists");
        std::stringstream ss;
        ss << "<sdf version='1.4'>";
        ss << "  <sensor name='force_torque' type='force_torque'>";
        ss << "    <update_rate>" << update_rate << "</update_rate>";
        ss << "    <always_on>true</always_on>";
        ss << "    <visualize>true</visualize>";
        ss << "  </sensor>";
        ss << "</sdf>";
        std::string forceTorqueSensorString = ss.str();
        // Create the SDF file to configure the sensor
        sdf::ElementPtr sdf(new sdf::Element);
        sdf::initFile("sensor.sdf", sdf);
        sdf::readString(forceTorqueSensorString, sdf);
        // Create the force torque sensor
        auto mgr = ::gazebo::sensors::SensorManager::Instance();
        std::string sensorName = mgr->CreateSensor(sdf, "default",
            getName() + "::" + joint_name, joint->GetId());
        
        // Make sure the returned sensor name is correct
        auto excepted_name = std::string("default::" + getName() + "::" + joint_name + "::force_torque");
        if(sensorName != excepted_name)
            throw std::runtime_error("Returned sensor name is " + sensorName + " when it should be " + excepted_name);

        // Update the sensor manager so that it can process new sensors.
        mgr->Update();

        // Get a pointer to the force torque sensor
        auto sensor = std::dynamic_pointer_cast<::gazebo::sensors::ForceTorqueSensor>(
                mgr->GetSensor(sensorName));

        // Make sure the above dynamic cast worked.
        if(!sensor) throw std::runtime_error("Could not create force_torque sensor for joint " + joint_name);

        if(!sensor->IsActive()) throw std::runtime_error("Sensor is not active");
        
        std::cout << "[GazeboModel \'" << getName() << "\'] " << "Force torque sensor '" << sensorName << "' successfully created" << '\n';
        return sensor;
#else
        throw std::runtime_error("Adding sensors is only supported for gz version > 8");
#endif
    }

    ::gazebo::sensors::ContactSensorPtr attachContactSensorToLink(const std::string& link_name,double update_rate = 0)
    {
        assertModelLoaded();
        
    #if GAZEBO_MAJOR_VERSION > 8
        auto link = model_->GetLink(link_name);
        if(!link)
            throw std::runtime_error("Link " + link_name + " does not exists");
        std::stringstream ss;
        ss << "<sdf version='1.4'>";
        ss << "  <sensor name='contact' type='contact'>";
        ss << "    <update_rate>" << update_rate << "</update_rate>";
        ss << "    <always_on>true</always_on>";
        ss << "    <visualize>true</visualize>";
        ss << "  </sensor>";
        ss << "</sdf>";
        std::string contactSensorStr = ss.str();
        // Create the SDF file to configure the sensor
        sdf::ElementPtr sdf(new sdf::Element);
        sdf::initFile("sensor.sdf", sdf);
        sdf::readString(contactSensorStr, sdf);
        // Create the force torque sensor
        auto mgr = ::gazebo::sensors::SensorManager::Instance();
        std::string sensorName = mgr->CreateSensor(sdf, "default",
            getName() + "::" + link_name, link->GetId());
        
        // Make sure the returned sensor name is correct
        auto excepted_name = std::string("default::" + getName() + "::" + link_name + "::contact");
        if(sensorName != excepted_name)
            throw std::runtime_error("Returned sensor name is " + sensorName + " when it should be " + excepted_name);

        // Update the sensor manager so that it can process new sensors.
        mgr->Update();

        // Get a pointer to the force torque sensor
        auto sensor = std::dynamic_pointer_cast<::gazebo::sensors::ContactSensor>(
                mgr->GetSensor(sensorName));

        // Make sure the above dynamic cast worked.
        if(!sensor) throw std::runtime_error("Could not create contact sensor for link " + link_name);

        if(!sensor->IsActive()) throw std::runtime_error("Sensor is not active");
        
        std::cout << "[GazeboModel \'" << getName() << "\'] " << "Contact sensor '" << sensorName << "' successfully created" << '\n';
        return sensor;
#else
        throw std::runtime_error("Adding sensors is only supported for gz version > 8");
#endif
    }
    
    const Eigen::Vector3d& getGravity() const
    {
        assertModelLoaded();
        return gravity_vector_;
    }

    const std::vector<std::string>& getActuatedJointNames() const
    {
        assertModelLoaded();
        return actuated_joint_names_;
    }

    const Eigen::Matrix<double,6,1>& getBaseVelocity() const
    {
        assertModelLoaded();
        return current_base_vel_;
    }

    const Eigen::Affine3d& getWorldToBaseTransform() const
    {
        assertModelLoaded();
        return current_world_to_base_;
    }

    const Eigen::VectorXd& getJointPositions() const
    {
        assertModelLoaded();
        return current_joint_positions_;
    }

    const Eigen::VectorXd& getJointVelocities() const
    {
        assertModelLoaded();
        return current_joint_velocities_;
    }

    void setJointGravityTorques(const Eigen::VectorXd& gravity_torques)
    {
        assertModelLoaded();
        if (gravity_torques.size() != joint_gravity_torques_.size())
            throw std::runtime_error("Provided gravity torques do not match the model's dofs");
        joint_gravity_torques_ = gravity_torques;
    }

    const Eigen::VectorXd& getJointExternalTorques() const
    {
        assertModelLoaded();
        return current_joint_external_torques_;
    }

    const Eigen::VectorXd& getJointMeasuredTorques() const
    {
        assertModelLoaded();
        return current_joint_measured_torques_;
    }

    void setJointTorqueCommand(const Eigen::VectorXd& joint_torque_command)
    {
        assertModelLoaded();
        joint_torque_command_ = joint_torque_command;
        brakes_ = false;
    }

    int getNDof() const
    {
        assertModelLoaded();
        return ndof_;
    }

    void printState() const
    {
        if(!model_)
        {
            std::cout << "[GazeboModel] Model is not loaded." << '\n';
            return;
        }
        
        std::cout << "[GazeboModel \'" << getName() << "\'] State :" << '\n';
        std::cout << "- Gravity "                   << getGravity().transpose()                << '\n';
        std::cout << "- Base velocity\n"            << getBaseVelocity().transpose()           << '\n';
        std::cout << "- Tworld->base\n"             << getWorldToBaseTransform().matrix()      << '\n';
        std::cout << "- Joint positions "           << getJointPositions().transpose()         << '\n';
        std::cout << "- Joint velocities "          << getJointVelocities().transpose()        << '\n';
        std::cout << "- Joint external torques "    << getJointExternalTorques().transpose()   << '\n';
        std::cout << "- Joint measured torques "    << getJointMeasuredTorques().transpose()   << '\n';
        std::cout << "- Brakes "    << (brakes_ ? "Enabled" : "Disabled")   << '\n';
    }

    void executeAfterWorldUpdate(std::function<void(uint32_t,double,double)> callback)
    {
        callback_ = callback;
    }

    void setBrakes(bool enable)
    {
        brakes_ = enable;
    }

    double getIterations()
    {
        #if GAZEBO_MAJOR_VERSION > 8
            return world_->Iterations();
        #else
            return world_->GetIterations();
        #endif
    }
    void print() const
    {
        if(!model_)
        {
            std::cout << "[GazeboModel] Model is not loaded." << '\n';
            return;
        }

        std::cerr << "[GazeboModel::" << getName() << "]" << '\n';
        
        std::cout << "  Joints (" << joints_.size() << ")" << '\n';
        for(unsigned int i=0; i < joints_.size() ; ++i)
            std::cout << "     Joint " << i << ": '" << joints_[i]->GetName() << "'" << '\n';
        
        std::cout << "  Actuated joints (" << actuated_joint_names_.size() << ")" << '\n';
        for(unsigned int i=0; i < actuated_joint_names_.size() ; i++)
            std::cout << "     Actuated joint " << i << ": '" << actuated_joint_names_[i] << "'" << '\n';

        std::cout << "  Links (" << links_.size() << ")" << '\n';
        for(unsigned int i=0; i < links_.size() ; i++)
            std::cout << "      Link " << i << ": '" << links_[i]->GetName() << "'" << '\n';
        
        printState();
    }
protected:
    void worldUpdateBegin()
    {
        #if GAZEBO_MAJOR_VERSION > 8
            auto g = world_->Gravity();
        #else
            auto g = world_->GetPhysicsEngine()->GetGravity();
        #endif
        gravity_vector_[0] = g[0];
        gravity_vector_[1] = g[1];
        gravity_vector_[2] = g[2];

        model_->SetEnabled(!brakes_); // Enable the robot when brakes are off

        if(getIterations() > 0 && !brakes_)
        {
            for(int i=0 ; i < ndof_ ; ++i)
                joints_[i]->SetForce(0,joint_torque_command_[i] + joint_gravity_torques_[i]);
        }
        else
        {
            for(int i=0 ; i < ndof_ ; ++i)
                joints_[i]->SetVelocity(0, 0.0);
        }
    }
    void worldUpdateEnd()
    {
        for(int i=0 ; i < ndof_ ; ++i)
        {
            auto joint = joints_[i];
            #if GAZEBO_MAJOR_VERSION > 8
                current_joint_positions_[i] = joint->Position(0);
            #else
                current_joint_positions_[i] = joint->GetAngle(0).Radian();
            #endif
            current_joint_velocities_[i] = joint->GetVelocity(0);
            current_joint_external_torques_[i] = joint->GetForce(0); // WARNING: This is the external estimated force (= force applied to the joint = torque command from user)

            auto w = joint->GetForceTorque(0u);
            #if GAZEBO_MAJOR_VERSION > 8
                auto a = joint->LocalAxis(0u);
            #else
                auto a = joint->GetLocalAxis(0u);
            #endif
            current_joint_measured_torques_[i] = a.Dot(w.body1Torque);
        }

        #if GAZEBO_MAJOR_VERSION > 8
            current_world_to_base_ = convPose(model_->RelativePose());

            current_base_vel_.head(3) = convVec3(model_->RelativeLinearVel());
            current_base_vel_.tail(3) = convVec3(model_->RelativeAngularVel());
        #else
            auto pose = model_->GetRelativePose();
            current_world_to_base_.translation() = Eigen::Vector3d(pose.pos.x,pose.pos.y,pose.pos.z);
            current_world_to_base_.linear() = Eigen::Quaterniond(pose.rot.w,pose.rot.x,pose.rot.y,pose.rot.z).toRotationMatrix();

            auto base_vel_lin = model_->GetRelativeLinearVel();
            auto base_vel_ang = model_->GetRelativeAngularVel();
            current_base_vel_[0] = base_vel_lin.x;
            current_base_vel_[1] = base_vel_lin.y;
            current_base_vel_[2] = base_vel_lin.z;
            current_base_vel_[3] = base_vel_ang.x;
            current_base_vel_[4] = base_vel_ang.y;
            current_base_vel_[5] = base_vel_ang.z;
        #endif
        if(callback_)
        {
            #if GAZEBO_MAJOR_VERSION > 8
                double sim_time = world_->SimTime().Double();
            #else
                double sim_time = world_->GetSimTime().Double();
            #endif
            #if GAZEBO_MAJOR_VERSION > 8
                double dt = world_->Physics()->GetMaxStepSize();
            #else
                double dt = world_->GetPhysicsEngine()->GetMaxStepSize();
            #endif
            callback_(getIterations(),sim_time,dt);
        }
    }
private:
    void assertModelLoaded() const
    {
        if (!model_)
            throw std::runtime_error("[GazeboModel] Model is not loaded");
    }
    GazeboModel()
    {
        current_base_vel_.setZero();
        current_world_to_base_.setIdentity();
        gravity_vector_.setZero();
    }
    bool loadWorld()
    {
        auto world = ::gazebo::physics::get_world();
        if(!world)
        {
            std::cerr << "[GazeboModel::" << getName() << "] " << "Could not load gazebo world" << '\n';
            return false;
        }
        world_ = world;
        world_begin_ = ::gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboModel::worldUpdateBegin,this));
        world_end_   = ::gazebo::event::Events::ConnectWorldUpdateEnd(std::bind(&GazeboModel::worldUpdateEnd,this));
        return true;
    }

    ::gazebo::physics::WorldPtr world_;
    ::gazebo::physics::ModelPtr model_;
    ::gazebo::physics::Joint_V joints_;
    ::gazebo::physics::Link_V links_;
    std::vector<std::string> actuated_joint_names_;
    Eigen::Matrix<double,6,1> current_base_vel_;
    Eigen::Affine3d current_world_to_base_;
    Eigen::VectorXd current_joint_positions_;
    Eigen::VectorXd joint_gravity_torques_;
    Eigen::VectorXd current_joint_velocities_;
    Eigen::VectorXd current_joint_external_torques_;
    Eigen::VectorXd current_joint_measured_torques_;
    Eigen::VectorXd joint_torque_command_;
    std::string name_;
    std::string base_name_;
    Eigen::Vector3d gravity_vector_;
    ::gazebo::event::ConnectionPtr world_begin_;
    ::gazebo::event::ConnectionPtr world_end_;
    int ndof_ = 0;
    bool brakes_ = true;
    std::function<void(uint32_t,double,double)> callback_;
};

} // namespace gazebo
} // namespace orca
