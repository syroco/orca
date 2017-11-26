#include <orca/constraint/DynamicsEquationConstraint.h>
#include <orca/optim/OptimisationVector.h>
using namespace orca::constraint;
using namespace orca::optim;
using namespace orca::task;
using namespace orca::common;

DynamicsEquationConstraint::DynamicsEquationConstraint()
: EqualityConstraint(ControlVariable::X)
{

}

void DynamicsEquationConstraint::resize()
{
    MutexLock lock(mutex);
    
    wrenches_ = OptimisationVector().getWrenches();
    idx_map_ = OptimisationVector().getIndexMap();
    size_map_ = OptimisationVector().getSizeMap();
    
    ndof_ = robot().getNrOfDegreesOfFreedom();
    fulldim_ = OptimisationVector().configurationSpaceDimension();
    
    int optim_vector_size = size_map_[ControlVariable::X];

    
    LOG_DEBUG << "[DynamicsEquationConstraint] Resize constraint matrix to " << fulldim_ << "x" << optim_vector_size;
    constraintFunction().resize(fulldim_,optim_vector_size);
    LOG_DEBUG << "[DynamicsEquationConstraint] Done resizing to " << fulldim_ << "x" << optim_vector_size;
}

void DynamicsEquationConstraint::updateConstraintFunction()
{
    int acc_idx = idx_map_[ControlVariable::GeneralisedAcceleration];
    int acc_size = size_map_[ControlVariable::GeneralisedAcceleration];
    int fb_wrench_idx = idx_map_[ControlVariable::FloatingBaseWrench];
    int jnt_trq_idx = idx_map_[ControlVariable::JointSpaceTorque];
    int wrench_idx = idx_map_[ControlVariable::ExternalWrench];
    
    constraintFunction().constraintMatrix().setZero();

    // A = [-M St Jt]
    this->constraintMatrix().block(0, acc_idx, acc_size, acc_size) = - robot().getFreeFloatingMassMatrix();

    // St
    this->constraintMatrix().block(6, jnt_trq_idx, ndof_, ndof_).setIdentity();

    // Jt [ sum(wrenches jacobians T) ]
    int i=0;
    for(auto w : wrenches_)
    {
        MutexLock lock(w->mutex);
        if(w->isActivated())
            constraintFunction().constraintMatrix().block(0,wrench_idx + i*6 , fulldim_, 6) = w->getJacobianTranspose();
        i++;
    }

    // b = n(q,qdot)
    // The output is a set of generalized torques (joint torques + base wrenches)
    this->bound() = robot().generalizedBiasForces();
}
