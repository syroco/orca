#include <orca/constraint/DynamicsEquationConstraint.h>
#include <orca/optim/OptimisationVector.h>
using namespace orca::constraint;
using namespace orca::optim;
using namespace orca::task;

DynamicsEquationConstraint::DynamicsEquationConstraint()
: EqualityConstraint(ControlVariable::X)
{

}

void DynamicsEquationConstraint::resize()
{
    MutexLock lock(mutex);

    const int optim_vector_size = OptimisationVector().getSize(ControlVariable::X);
    const int fulldim = OptimisationVector().ConfigurationSpaceDimension();



    if(optim_vector_size != this->cols())
    {
        LOG_DEBUG << "[DynamicsEquationConstraint] Resize constraint matrix to " << fulldim << "x" << optim_vector_size;

        constraintFunction().resize(fulldim,optim_vector_size);
    }
}

void DynamicsEquationConstraint::computeJacobianTranspose()
{
    return;
    MutexLock lock(mutex);

    const int wrench_idx = OptimisationVector().getIndex(ControlVariable::ExternalWrench);
    const int fulldim = OptimisationVector().ConfigurationSpaceDimension();

    // Jt [ sum(wrenches jacobians T) ]
    int i=0;
    for(auto w : OptimisationVector().getWrenches())
    {
        constraintFunction().constraintMatrix().block(0,wrench_idx + i*6 , fulldim, 6) = w->getJacobianTranspose();
        i++;
    }
}

void DynamicsEquationConstraint::update()
{
    MutexLock lock(mutex);

    const int ndof = robot().getNrOfDegreesOfFreedom();

    const int acc_idx = OptimisationVector().getIndex(ControlVariable::GeneralisedAcceleration);
    const int acc_size = OptimisationVector().getSize(ControlVariable::GeneralisedAcceleration);

    // A = [-M St Jt]
    this->constraintMatrix().block(0, acc_idx, acc_size, acc_size) = - robot().getFreeFloatingMassMatrix();

    const int fb_wrench_idx = OptimisationVector().getIndex(ControlVariable::FloatingBaseWrench);
    const int jnt_trq_idx = OptimisationVector().getIndex(ControlVariable::JointSpaceTorque);

    // St
    this->constraintMatrix().block(0, fb_wrench_idx, 6,6).setZero();
    this->constraintMatrix().block(6, jnt_trq_idx, ndof, ndof).setIdentity();

    const int wrench_idx = OptimisationVector().getIndex(ControlVariable::ExternalWrench);
    const int fulldim = OptimisationVector().ConfigurationSpaceDimension();

    // Jt [ sum(wrenches jacobians T) ]
    int i=0;
    for(auto w : OptimisationVector().getWrenches())
    {
        constraintFunction().constraintMatrix().block(0,wrench_idx + i*6 , fulldim, 6) = w->getJacobianTranspose();
        i++;
    }

    // b = n(q,qdot)
    // The output is a set of generalized torques (joint torques + base wrenches)
    this->bound() = robot().generalizedBiasForces();
}
