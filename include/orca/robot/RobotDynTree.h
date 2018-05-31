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

#include "orca/utils/Utils.h"
#include "orca/math/Utils.h"

// iDynTree headers
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/Core/EigenHelpers.h>

namespace orca
{
namespace robot
{
/**
 * Struct containing the floating robot state
 * using Eigen data structures.
 */
struct EigenRobotState
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

/**
 * Struct containing the floating robot state
 * using iDynTree data structures.
 * For the semantics of this structures,
 * see KinDynComputation::setRobotState method.
 */
struct iDynTreeRobotState
{
    void resize(int nrOfInternalDOFs)
    {
        jointPos.resize(nrOfInternalDOFs);
        jointVel.resize(nrOfInternalDOFs);
        reset();
    }

    void reset()
    {
        world_H_base.Identity();
        gravity(0) = 0;
        gravity(1) = 0;
        gravity(2) = -9.81;
    }

    void fromEigen(EigenRobotState& eigRobotState)
    {
        iDynTree::fromEigen(world_H_base,eigRobotState.world_H_base);
        iDynTree::toEigen(jointPos) = eigRobotState.jointPos;
        iDynTree::fromEigen(baseVel,eigRobotState.baseVel);
        iDynTree::toEigen(jointVel) = eigRobotState.jointVel;
        iDynTree::toEigen(gravity)  = eigRobotState.gravity;
    }

    iDynTree::Transform world_H_base;
    iDynTree::VectorDynSize jointPos;
    iDynTree::Twist         baseVel;
    iDynTree::VectorDynSize jointVel;
    iDynTree::Vector3       gravity;
};

struct EigenRobotAcceleration
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

struct iDynTreeRobotAcceleration
{
    void resize(int nrOfInternalDOFs)
    {
        jointAcc.resize(nrOfInternalDOFs);
        baseAcc.zero();
        jointAcc.zero();
    }

    void zero()
    {
        baseAcc.zero();
        jointAcc.zero();
    }
    iDynTree::Vector6 baseAcc;
    iDynTree::VectorDynSize jointAcc;
};

struct RobotDataHelper
{
    void resize(const iDynTree::Model & model)
    {
        eigRobotState.resize(model.getNrOfDOFs());
        idynRobotState.resize(model.getNrOfDOFs());
        eigRobotAcc.resize(model.getNrOfDOFs());
        idynRobotAcc.resize(model.getNrOfDOFs());

        idynFFMassMatrix.resize(model);
        eigFFMassMatrix = iDynTree::toEigen(idynFFMassMatrix);
        eigMassMatrix.setZero(model.getNrOfDOFs(),model.getNrOfDOFs());

        generalizedBiasForces.resize(model);
        eigGeneralizedBiasForces.setZero(6 + model.getNrOfDOFs());
        generalizedGravityTorques.resize(model);
        eigJointGravityTorques.setZero(6 + model.getNrOfDOFs());
        extForces.resize(model);

        eigJointGravityTorques.setZero(model.getNrOfDOFs());
        eigJointCoriolisTorques.setZero(model.getNrOfDOFs());
        eigJointGravityAndCoriolisTorques.setZero(model.getNrOfDOFs());

        eigMinJointPos.setZero(model.getNrOfDOFs());
        eigMaxJointPos.setZero(model.getNrOfDOFs());

        eigRobotAcc.setZero();
        idynRobotAcc.zero();
        extForces.zero();

        idynJacobian.resize(6,model.getNrOfDOFs());
        eigJacobian = iDynTree::toEigen(idynJacobian);

        idynFFJacobian.resize(model);
        eigFFJacobian = iDynTree::toEigen(idynFFJacobian);
    }

    EigenRobotAcceleration eigRobotAcc;
    iDynTreeRobotAcceleration idynRobotAcc;
    iDynTree::FreeFloatingMassMatrix idynFFMassMatrix;
    iDynTree::MatrixDynSize idynJacobian;
    Eigen::MatrixXd eigFFJacobian;
    Eigen::MatrixXd eigJacobian;
    Eigen::MatrixXd eigFFMassMatrix;
    Eigen::MatrixXd eigMassMatrix;
    iDynTree::FrameFreeFloatingJacobian idynFFJacobian;
    iDynTree::LinkNetExternalWrenches extForces;
    iDynTree::Vector6 idynFrameBiasAcc;
    iDynTree::Twist idynFrameVel;
    iDynTree::Position idynPramePos;
    iDynTree::FreeFloatingGeneralizedTorques generalizedBiasForces;
    iDynTree::FreeFloatingGeneralizedTorques generalizedGravityTorques;
    Eigen::VectorXd eigGeneralizedBiasForces;
    Eigen::VectorXd eigJointGravityTorques;
    Eigen::VectorXd eigJointCoriolisTorques;
    Eigen::VectorXd eigJointGravityAndCoriolisTorques;
    Eigen::VectorXd eigMinJointPos;
    Eigen::VectorXd eigMaxJointPos;
    EigenRobotState eigRobotState;
    iDynTreeRobotState idynRobotState;
    Eigen::Matrix<double,6,1> eigFrameBiasAcc;
    Eigen::Matrix<double,6,1> eigFrameVel;
    Eigen::Matrix4d eigTransform;
};


class RobotDynTree
{
public:
    RobotDynTree(const std::string& name="");
    const std::string& getName() const;
    bool loadModelFromFile(const std::string& modelFile);
    bool loadModelFromString(const std::string &modelString);
    const std::string& getUrdfUrl() const;
    const std::string& getUrdfString() const;
    void setRobotState(const Eigen::Matrix4d& world_H_base
                , const Eigen::VectorXd& jointPos
                , const Eigen::Matrix<double,6,1>& baseVel
                , const Eigen::VectorXd& jointVel
                , const Eigen::Vector3d& gravity);
    void setRobotState(const Eigen::VectorXd& jointPos
                , const Eigen::VectorXd& jointVel
                , const Eigen::Vector3d& global_gravity_vector);
    void setRobotState(const Eigen::VectorXd& jointPos
                , const Eigen::VectorXd& jointVel);
    void print() const;
    void setBaseFrame(const std::string& fixed_base_or_free_floating_frame);
    const std::string& getBaseFrame() const;
    void setGravity(const Eigen::Vector3d& global_gravity_vector);
    unsigned int getNrOfDegreesOfFreedom() const;
    unsigned int getConfigurationSpaceDimension() const;
    bool frameExists(const std::string& frame_name) const;

    const Eigen::Matrix4d& getRelativeTransform(const std::string& refFrameName, const std::string& frameName);
    const Eigen::Matrix4d& getTransform(const std::string& frameName);

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
    const iDynTree::Model& getRobotModel() const;
    unsigned int getNrOfJoints() const;
    std::string getJointName(unsigned int idx) const;
    const std::vector<std::string>& getLinkNames() const;
    const std::vector<std::string>& getFrameNames() const;
    const std::vector<std::string>& getJointNames() const;
    bool isInitialized() const;
    void onRobotInitializedCallback(std::function<void(void)> cb);
protected:
    std::function<void(void)> robot_initialized_cb_;;
    bool load(const iDynTree::Model& model);
    RobotDataHelper robotData_;
    iDynTree::KinDynComputations kinDynComp_;
    bool is_initialized_ = false;
    std::string base_frame_;
    std::string name_;
    std::string urdf_url_;
    std::string urdf_str_;
    unsigned int ndof_ = 0;
    std::vector<std::string> joint_names_;
    std::vector<std::string> link_names_;
    std::vector<std::string> frame_names_;
// private:
    // class iDynTreeImpl;                     // Forward declaration of the implementation class
    // std::shared_ptr<iDynTreeImpl> robot_impl_;    // PIMPL

};

}
}
