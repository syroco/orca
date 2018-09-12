#include "orca/constraint/DynamicsEquationConstraint.h"

using namespace orca::constraint;
using namespace orca::optim;
using namespace orca::common;

DynamicsEquationConstraint::DynamicsEquationConstraint(const std::string& name)
: EqualityConstraint(name,ControlVariable::X)
{

}

void DynamicsEquationConstraint::onResize()
{
    wrenches_ = this->getProblem()->getWrenches();
    idx_map_ = this->getProblem()->getIndexMap();
    size_map_ = this->getProblem()->getSizeMap();

    ndof_ = this->getRobot()->getNrOfDegreesOfFreedom();
    fulldim_ = this->getRobot()->getConfigurationSpaceDimension();

    int optim_vector_size = size_map_[ControlVariable::X];

    constraintFunction().resize(fulldim_,optim_vector_size);
}

void DynamicsEquationConstraint::onUpdateConstraintFunction(double current_time, double dt)
{
    int acc_idx = idx_map_[ControlVariable::GeneralisedAcceleration];
    int acc_size = size_map_[ControlVariable::GeneralisedAcceleration];
    int fb_wrench_idx = idx_map_[ControlVariable::FloatingBaseWrench];
    int jnt_trq_idx = idx_map_[ControlVariable::JointTorque];
    int wrench_idx = idx_map_[ControlVariable::ExternalWrench];

    constraintFunction().constraintMatrix().setZero();

    // A = [-M St Jt]
    this->constraintMatrix().block(0, acc_idx, acc_size, acc_size) = - this->robot()->getFreeFloatingMassMatrix();

    // St
    this->constraintMatrix().block(6, jnt_trq_idx, ndof_, ndof_).setIdentity();

    // Jt [ sum(wrenches jacobians T) ]
    int i=0;
    for(auto w : wrenches_)
    {
        if(w->isComputing())
            constraintFunction().constraintMatrix().block(0,wrench_idx + i*6 , fulldim_, 6) = w->getJacobianTranspose();
        else
            constraintFunction().constraintMatrix().block(0,wrench_idx + i*6 , fulldim_, 6).setZero();
        i++;
    }

    // b = n(q,qdot)
    // The output is a set of generalized torques (base wrenches + joint torques (Corio-Centrifugal + Gravity))
    this->bound() = this->robot()->generalizedBiasForces();
}

ORCA_REGISTER_CLASS(orca::constraint::DynamicsEquationConstraint)
