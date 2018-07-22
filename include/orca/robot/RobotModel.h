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

#include "orca/utils/Utils.h"
#include "orca/math/Utils.h"

namespace orca
{
namespace robot
{
/**
 * Struct containing the floating robot state
 * using Eigen data structures.
 */
struct RobotState
{
    void resize(int nrOfInternalDOFs)
    {
        jointPos.resize(nrOfInternalDOFs);
        jointVel.resize(nrOfInternalDOFs);
        reset();
    }

    void reset()
    {
        world_H_base.setIdentity();
        jointPos.setZero();
        baseVel.setZero();
        jointVel.setZero();
        gravity[0] = 0;
        gravity[1] = 0;
        gravity[2] = -9.81;
    }

    Eigen::Matrix4d world_H_base;
    Eigen::VectorXd jointPos;
    Eigen::Matrix<double,6,1> baseVel;
    Eigen::VectorXd jointVel;
    Eigen::Vector3d gravity;
};

struct RobotAcceleration
{
    void resize(int nrOfInternalDOFs)
    {
        jointAcc.resize(nrOfInternalDOFs);
        baseAcc.setZero();
        jointAcc.setZero();
    }

    void random()
    {
        baseAcc.setRandom();
        jointAcc.setRandom();
    }

    void setZero()
    {
        baseAcc.setZero();
        jointAcc.setZero();
    }

    Eigen::Matrix<double,6,1> baseAcc;
    Eigen::VectorXd jointAcc;
};


/**
* @brief The robot model class allow to make kinematics and dynamics computations
* 
*/
class RobotModel
{
public:
    /**
    * @brief The default constructor
    * 
    * @param name You can explicitely give it a name, otherwise it will be extracted from the urdf
    */
    RobotModel(const std::string& name="");
    virtual ~RobotModel();
    /**
    * @brief Returns the name of the model, either set in the constructor, or automatically extracted from the urdf
    * 
    * @return const std::string&
    */
    const std::string& getName() const;
    /**
    * @brief Load the model from an absolute urdf file path
    * 
    * @param modelFile The absolute file path
    * @return bool
    */
    bool loadModelFromFile(const std::string& modelFile);
    /**
    * @brief Load the model from a full urdf string (xacro not supported) 
    * 
    * @param modelString The expanded urdf string
    * @return bool
    */
    bool loadModelFromString(const std::string &modelString);
    /**
    * @brief Get the file url if it was loaded from a file. Empty string otherwise.
    * 
    * @return const std::string&
    */
    const std::string& getUrdfUrl() const;
    /**
    * @brief Get the urdf expanded string used to build the model
    * 
    * @return const std::string&
    */
    const std::string& getUrdfString() const;
    /**
    * @brief Set the complete robot state
    * 
    * @param world_H_base The pose of the robot w.r.t the world frame (identity for fixed-based robots)
    * @param jointPos The current joint positions
    * @param baseVel The base twist (zero for fixed-base robots)
    * @param jointVel The current joint velocities
    * @param gravity The world gravity
    */
    void setRobotState(const Eigen::Matrix4d& world_H_base
                , const Eigen::VectorXd& jointPos
                , const Eigen::Matrix<double,6,1>& baseVel
                , const Eigen::VectorXd& jointVel
                , const Eigen::Vector3d& gravity);
    /**
    * @brief Set the partial robot state
    * 
    * @param jointPos The current joint positions
    * @param jointVel THe current joint velocities
    * @param global_gravity_vector the world gravity
    */
    void setRobotState(const Eigen::VectorXd& jointPos
                , const Eigen::VectorXd& jointVel
                , const Eigen::Vector3d& global_gravity_vector);
    /**
    * @brief Set the partial robot state. Useful for fixed based robots.
    * 
    * @param jointPos The current joint positions
    * @param jointVel The current joint velocities
    */
    void setRobotState(const Eigen::VectorXd& jointPos
                , const Eigen::VectorXd& jointVel);
    /**
    * @brief Print information about the model to cout
    * 
    */
    void print() const;
    /**
    * @brief Set the base frame / the free-floating frame that attaches the robot to the world frame
    * For a humanoid it would be generally somewhere in the hips. For a fixed based robot it would be 
    * the frame between the robot and the table. This frame is also the frame of reference for the
    * jacobians, transforms etc when refFrameName is not specified.
    * 
    * @param fixed_base_or_free_floating_frame p_fixed_base_or_free_floating_frame:...
    */
    void setBaseFrame(const std::string& fixed_base_or_free_floating_frame);
    /**
    * @brief Returns the base/free-floating frame
    * 
    * @return const std::string&
    */
    const std::string& getBaseFrame() const;
    /**
    * @brief Return the global gravity vector
    * 
    * @param global_gravity_vector The global gravity vector (usually 0,0,-9.81)
    */
    void setGravity(const Eigen::Vector3d& global_gravity_vector);
    /**
    * @brief Return the number of actuated joints
    * 
    * @return unsigned int
    */
    unsigned int getNrOfDegreesOfFreedom() const;
    /**
    * @brief Returns the number of DOF + Free floating DOF (6)
    * 
    * @return unsigned int
    */
    unsigned int getConfigurationSpaceDimension() const;
    /**
    * @brief Returns true is the frame exists on the model.
    * 
    * @param frame_name The test frame
    * @return bool
    */
    bool frameExists(const std::string& frame_name) const;

    /**
    * @brief Get the transform (or pose) between a frame to another
    * 
    * @param refFrameName The reference frame
    * @param frameName The frame you want to compute w.r.t the reference frame
    * @return const Eigen::Matrix4d&
    */
    const Eigen::Matrix4d& getRelativeTransform(const std::string& refFrameName, const std::string& frameName);
    /**
    * @brief Get the transform of a frame w.r.t the base frame
    * 
    * @param frameName The frame you want to compute w.r.t the base frame
    * @return const Eigen::Matrix4d&
    */
    const Eigen::Matrix4d& getTransform(const std::string& frameName);

    /**
    * @brief Add an extra frame to a link. This re-creates the whole model, so it should not be used on the control loop.
    * 
    * @param linkName The link from which you want to attach a new frame
    * @param frameName The new frame
    * @param link_H_frame The transform between the link to the frame
    * @return bool
    */
    bool addAdditionalFrameToLink (const std::string &linkName, const std::string &frameName, const Eigen::Matrix4d& link_H_frame);

    const Eigen::Matrix<double,6,1>& getFrameVel(const std::string& frameName);
    const Eigen::Matrix<double,6,1>& getFrameBiasAcc(const std::string& frameName);

    const Eigen::MatrixXd& getFreeFloatingMassMatrix();
    const Eigen::MatrixXd& getMassMatrix();
    const Eigen::VectorXd& getJointPos() const;
    const Eigen::VectorXd& getJointVel() const;
    const Eigen::VectorXd& getMinJointPos();
    const Eigen::VectorXd& getMaxJointPos();
    const Eigen::MatrixXd& getJacobian(const std::string& frameName);
    const Eigen::MatrixXd& getRelativeJacobian(const std::string& refFrameName, const std::string& frameName);
    const Eigen::MatrixXd& getFrameFreeFloatingJacobian(const std::string& frameName);
    const Eigen::VectorXd& generalizedBiasForces();
    const Eigen::VectorXd& getJointGravityTorques();
    const Eigen::VectorXd& getJointCoriolisTorques();
    const Eigen::VectorXd& getJointGravityAndCoriolisTorques();
    unsigned int getNrOfJoints() const;
    std::string getJointName(unsigned int idx) const;
    const std::vector<std::string>& getLinkNames() const;
    const std::vector<std::string>& getFrameNames() const;
    const std::vector<std::string>& getJointNames() const;
    bool isInitialized() const;
    void onRobotInitializedCallback(std::function<void(void)> cb);
protected:
    enum RobotModelType { iDynTree, KDL };
    RobotModelType robot_kinematics_type_ = iDynTree;

    std::function<void(void)> robot_initialized_cb_;
    bool is_initialized_ = false;

    std::string name_;
    std::string urdf_url_;
    std::string urdf_str_;

private:
    template<RobotModelType type = iDynTree> struct RobotModelImpl;
    std::unique_ptr<RobotModelImpl<> > impl_;
};

}
}
