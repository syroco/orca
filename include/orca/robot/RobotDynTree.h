// This file is a part of the orca framework.
// Copyright 2017, ISIR / Universite Pierre et Marie Curie (UPMC)
// Main contributor(s): Antoine Hoarau, hoarau@isir.upmc.fr
//
// This software is a computer program whose purpose is to [describe
// functionalities and technical features of your software].
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

#pragma once

#include <orca/util/Utils.h>
#include <orca/math/Utils.h>
#include <map>
#include <cstdlib>

// Eigen headers
#include <Eigen/Core>

// iDynTree headers
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>

// Helpers function to convert between
// iDynTree datastructures
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
        world_H_base.setIdentity();
        jointPos.resize(nrOfInternalDOFs);
        jointVel.resize(nrOfInternalDOFs);
        jointPos.setZero();
        jointVel.setZero();
        gravity[0] = 0;
        gravity[1] = 0;
        gravity[2] = -9.81;
    }

    void random()
    {
        world_H_base.setIdentity();
        jointPos.setRandom();
        baseVel.setRandom();
        jointVel.setRandom();
        gravity[0] = 0;
        gravity[1] = 0;
        gravity[2] = -9.81;
    }

    void setFixedBase()
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
    }

    void setFixedBase()
    {
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
        eigRobotAcc.resize(model.getNrOfDOFs());
        idynRobotAcc.resize(model.getNrOfDOFs());

        idynMassMatrix.resize(model);
        idynJacobian.resize(6,model.getNrOfDOFs());
        idynJacobianFb.resize(model);
        generalizedBiasForces.resize(model);
        extForces.resize(model);

        eigRobotAcc.setZero();
        idynRobotAcc.zero();
        idynMassMatrix.zero();
        idynJacobian.zero();
        extForces.zero();
    }

    EigenRobotAcceleration eigRobotAcc;
    iDynTreeRobotAcceleration idynRobotAcc;
    iDynTree::FreeFloatingMassMatrix idynMassMatrix;
    iDynTree::MatrixDynSize idynJacobian;
    iDynTree::FrameFreeFloatingJacobian idynJacobianFb;
    iDynTree::LinkNetExternalWrenches extForces;
    iDynTree::Vector6 frameBiasAcc;
    iDynTree::Twist frameVel;
    iDynTree::Position framePos;
    iDynTree::FreeFloatingGeneralizedTorques generalizedBiasForces;
};


struct RobotDynTree
{
public:
    RobotDynTree(const std::string& modelFile="");
    bool loadModelFromFile(const std::string& modelFile );
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
    void print();
    void setBaseFrame(const std::string& fixed_base_or_free_floating_frame);
    const std::string& getBaseFrame() const;
    void setGravity(const Eigen::Vector3d& global_gravity_vector);
    const std::string& getFileURL() const;
    int getNrOfDegreesOfFreedom() const;
    bool frameExists(const std::string& frame_name);

    const std::map<unsigned int, std::pair<double,double> >& getJointPositionLimits();

    const Eigen::Matrix4d& getRelativeTransform(const std::string& refFrameName, const std::string& frameName);
    const Eigen::Matrix<double,6,1>& getFrameVel(const std::string& frameName);
    const Eigen::Matrix<double,6,1>& getFrameBiasAcc(const std::string& frameName);
    const Eigen::MatrixXd& getFreeFloatingMassMatrix();
    const Eigen::VectorXd& getJointPos();
    const Eigen::VectorXd& getJointVel();
    const Eigen::MatrixXd& getRelativeJacobian(const std::string& refFrameName, const std::string& frameName);
    const Eigen::MatrixXd& getFrameFreeFloatingJacobian(const std::string& frameName);
    const Eigen::VectorXd& generalizedBiasForces();
protected:
    RobotDataHelper robotData_;
    iDynTree::KinDynComputations kinDynComp_;
    EigenRobotState eigRobotState_;
    iDynTreeRobotState idynRobotState_;
    bool is_initialized_ = false;
    std::string base_frame_;
    std::string urdf_url_;
    Eigen::Vector3d global_gravity_vector_;
    std::map<unsigned int, std::pair<double,double> > joint_pos_limits_;

};

}
}
