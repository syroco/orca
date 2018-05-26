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
        gravity_vector_.setZero();
        load(name);
    }

    GazeboModel(::gazebo::physics::ModelPtr model)
    {
        gravity_vector_.setZero();
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
            std::cerr << "[GazeboModel::" << getName() << "] " << "Model is not loaded" << '\n';
            return false;
        }

        if (joint_names.size() != joint_positions.size())
        {
            std::cerr << "[GazeboModel::" << getName() << "] " << "joint_names lenght should be the same as joint_positions : " << joint_names.size() << " vs " << joint_positions.size() << '\n';
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
            std::cerr << "[GazeboModel::" << getName() << "] " << "Could not get gazebo model " << model_name << ". Make sure it is loaded in the server" << '\n';
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

            std::cout << "[" << model->GetName() << "] " << (added ? "Adding":"Not adding")
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
            std::cerr << "[" << model->GetName() << "] " << "Could not get any actuated joint for model " << model->GetName() << '\n';
            return false;
        }

        std::cout << "[" << getName() << "] "<< "Actuated joints" << '\n';
        for(auto n : actuated_joint_names_)
        {
            std::cout << "   - " << n << '\n';
        }

        model_ = model;
        name_ = model->GetName();

        const int ndof = actuated_joint_names_.size();

        current_joint_positions_.setZero(ndof);
        current_joint_velocities_.setZero(ndof);
        current_joint_torques_.setZero(ndof);
        joint_torque_commands_.setZero(ndof);

        return true;
    }

    const Eigen::Vector3d& getGravity()
    {
        if(!world_)
        {
            std::cerr << "[GazeboModel::" << getName() << "] " << "World is not loaded" << '\n';
            return gravity_vector_;
        }
        #if GAZEBO_MAJOR_VERSION > 8
            auto g = world_->Gravity();
        #else
            auto g = world_->GetPhysicsEngine()->GetGravity();
        #endif
        gravity_vector_[0] = g[0];
        gravity_vector_[1] = g[1];
        gravity_vector_[2] = g[2];
        return gravity_vector_;
    }

private:
    bool loadWorld()
    {
        auto world = ::gazebo::physics::get_world();
        if(!world)
        {
            std::cerr << "[GazeboModel::" << getName() << "] " << "Could not load gazebo world" << '\n';
            return false;
        }
        world_ = world;
        return true;
    }
    ::gazebo::physics::WorldPtr world_;
    ::gazebo::physics::ModelPtr model_;
    std::vector<std::string> actuated_joint_names_;
    Eigen::Matrix<double,6,1> current_base_vel_;
    Eigen::Affine3d current_world_to_base_;
    Eigen::VectorXd current_joint_positions_;
    Eigen::VectorXd current_joint_velocities_;
    Eigen::VectorXd current_joint_torques_;
    Eigen::VectorXd joint_torque_commands_;
    std::string name_;
    Eigen::Vector3d gravity_vector_;
};

} // namespace gazebo
} // namespace orca
