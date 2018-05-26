#pragma once

// Gazebo headers
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
// Eigen
#include <Eigen/Geometry>

namespace orca
{
namespace gazebo
{

class GazeboModel
{
public:
    GazeboModel(const std::string& name)
    {
        load(name);
    }

    GazeboModel(::gazebo::physics::ModelPtr model)
    {
        load(model);
    }

    const std::string& getName()
    {
        return name_;
    }

    bool setModelConfiguration(std::vector<std::string> joint_names,std::vector<double> joint_positions)
    {
        if (!model_)
        {
            std::cerr << "[" << getName() << "] " << "Model is not loaded" << '\n';
            return false;
        }

        if (joint_names.size() != joint_positions.size())
        {
            std::cerr << "[" << getName() << "] " << "joint_names lenght should be the same as joint_positions : " << joint_names.size() << " vs " << joint_positions.size() << '\n';
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
        auto world = ::gazebo::physics::get_world();
        if(! world)
        {
            std::cerr << "[" << getName() << "] " << "Could not load gazebo world" << '\n';
            return false;
        }

        #if GAZEBO_MAJOR_VERSION > 8
            model_ = world->ModelByName( model_name );
        #else
            model_ = world->GetModel( model_name );
        #endif
        if(! model_)
        {
            std::cerr << "[" << getName() << "] " << "Could not get gazebo model " << model_name << ". Make sure it is loaded in the server" << '\n';
            return false;
        }
        return load(model_);
    }

    bool load(::gazebo::physics::ModelPtr model)
    {
        if(!model)
        {
            std::cerr << "Model is invalid" << '\n';
            return false;
        }
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

            std::cout << "[" << getName() << "] " << (added ? "Adding":"Not adding")
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
            std::cerr << "[" << getName() << "] " << "Could not get any actuated joint for model " << getName() << '\n';
            return false;
        }

        model_ = model;
        name_ = model->GetName();

        std::cout << "[" << getName() << "] "<< "Physical joints" << std::endl;
        for(auto joint_name : actuated_joint_names_)
        {
            std::cout << "   - " << joint_name << std::endl;
        }

        const int ndof = actuated_joint_names_.size();

        current_joint_positions_.setZero(ndof);
        current_joint_velocities_.setZero(ndof);
        current_joint_torques_.setZero(ndof);
        joint_torque_commands_.setZero(ndof);
    }

private:
    ::gazebo::physics::ModelPtr model_;
    std::vector<std::string> actuated_joint_names_;
    Eigen::Matrix<double,6,1> current_base_vel_;
    Eigen::Affine3d current_world_to_base_;
    Eigen::VectorXd current_joint_positions_;
    Eigen::VectorXd current_joint_velocities_;
    Eigen::VectorXd current_joint_torques_;
    Eigen::VectorXd joint_torque_commands_;
    std::string name_;
};

} // namespace gazebo
} // namespace orca
