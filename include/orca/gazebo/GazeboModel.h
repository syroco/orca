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
        return name_;
    }

    bool setModelConfiguration(const std::vector<std::string>& joint_names,const std::vector<double>& joint_positions)
    {
        if (!model_)
        {
            std::cerr << "[GazeboModel] Model is not loaded" << '\n';
            return false;
        }

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

        actuated_joint_names_.clear();
        for(auto joint : model->GetJoints() )
        {
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
                actuated_joint_names_.push_back(joint->GetName());
                added = true;
            }

            std::cout << "[GazeboModel \'" << model->GetName() << "\'] " << (added ? "Adding":"Not adding")
                << " joint " << joint->GetName()
                << " type " << joint->GetType()
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

        std::cout << "[GazeboModel \'" << model->GetName() << "\'] " << "Actuated joints" << '\n';
        for(auto n : actuated_joint_names_)
        {
            std::cout << "   - " << n << '\n';
        }

        model_ = model;
        name_ = model->GetName();

        ndof_ = actuated_joint_names_.size();

        current_joint_positions_.setZero(ndof_);
        current_joint_velocities_.setZero(ndof_);
        current_joint_external_torques_.setZero(ndof_);
        joint_torque_command_.setZero(ndof_);

        return true;
    }

    const Eigen::Vector3d& getGravity() const
    {
        return gravity_vector_;
    }

    const std::vector<std::string>& getActuatedJointNames() const
    {
        return actuated_joint_names_;
    }

    const Eigen::Matrix<double,6,1>& getBaseVelocity() const
    {
        return current_base_vel_;
    }

    const Eigen::Affine3d& getWorldToBaseTransform() const
    {
        return current_world_to_base_;
    }

    const Eigen::VectorXd& getJointPositions() const
    {
        return current_joint_positions_;
    }

    const Eigen::VectorXd& getJointVelocities() const
    {
        return current_joint_velocities_;
    }

    const Eigen::VectorXd& getJointExternalTorques() const
    {
        return current_joint_external_torques_;
    }

    void setJointTorqueCommand(const Eigen::VectorXd& joint_torque_command)
    {
        joint_torque_command_ = joint_torque_command;
        for(int i=0 ; i < ndof_ ; ++i)
        {
            auto joint = model_->GetJoint( actuated_joint_names_[i] );
            joint->SetForce(0,joint_torque_command[i]);
        }
    }

    int getNDof() const
    {
        return ndof_;
    }

    void printState() const
    {
        std::cout << "[GazeboModel \'" << getName() << "\'] State :\n" << '\n';
        std::cout << "- Gravity "                   << getGravity().transpose()                << '\n';
        std::cout << "- Base velocity\n"            << getBaseVelocity().transpose()           << '\n';
        std::cout << "- Tworld->base\n"             << getWorldToBaseTransform().matrix()      << '\n';
        std::cout << "- Joint positions "           << getJointPositions().transpose()         << '\n';
        std::cout << "- Joint velocities "          << getJointVelocities().transpose()        << '\n';
        std::cout << "- Joint external torques "    << getJointExternalTorques().transpose()   << '\n';
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
    }
    void worldUpdateEnd()
    {

        for(int i=0 ; i < actuated_joint_names_.size() ; ++i)
        {
            auto joint = model_->GetJoint( actuated_joint_names_[i] );
            #if GAZEBO_MAJOR_VERSION > 8
                current_joint_positions_[i] = joint->Position(0);
            #else
                current_joint_positions_[i] = joint->GetAngle(0).Radian();
            #endif
            current_joint_velocities_[i] = joint->GetVelocity(0);
            current_joint_external_torques_[i] = joint->GetForce(0u); // WARNING: This is the external estimated force
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
    }
private:
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
    std::vector<std::string> actuated_joint_names_;
    Eigen::Matrix<double,6,1> current_base_vel_;
    Eigen::Affine3d current_world_to_base_;
    Eigen::VectorXd current_joint_positions_;
    Eigen::VectorXd current_joint_velocities_;
    Eigen::VectorXd current_joint_external_torques_;
    Eigen::VectorXd joint_torque_command_;
    std::string name_;
    Eigen::Vector3d gravity_vector_;
    ::gazebo::event::ConnectionPtr world_begin_;
    ::gazebo::event::ConnectionPtr world_end_;
    int ndof_ = 0;
};

} // namespace gazebo
} // namespace orca
