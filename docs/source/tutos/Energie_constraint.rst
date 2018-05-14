Energy constraint
-----------------


This tutorial provides an example of addition of a new constraint. The added constraint is related to the robot provisional kinetic energy (which has been recently introduced in https://hal.archives-ouvertes.fr/hal-01614508/en).

This class inherits from GenericConstraint, and the associated **ControleVariable** is **JointSpaceTorque**. Remember the problem is written as a **quadratic problem**:

.. math::

    lb_C \leq Cx \leq ub_C

for which we have to set the bounds and the constraint matrix. Those bounds and constraint are expressed explicitly as a function of joint torque. For this problem, we will create a new class, called **EnergyConstraint**.

.. code-block:: cpp
  
  class EnergyConstraint : public GenericConstraint
  {
    public:
    
     
    void stateInitialize()
     {
         // This function initialize and resize the attributes
            ...
      }
   
    void updateState()
    {
        // This function update all the attributes
        ...
     }   
   
    void updateConstraintFunction()
    {
     ...

     constraintFunction().lowerBound() = ec_min;
     constraintFunction().upperBound() = ec_max;

    }
    void resize()
    {
     ...

     constraintFunction().resize(1,dim); // this resize lb,C and ub, C to zero, lb to -inf, ub to +inf
     constraintFunction().constraintMatrix()= C;
    }
    
    protected:
     // Declare the attributes of this class 
     Eigen::VectorXd  AccelerationDes_;
     bool indice;
     string name_of_cstr ;
     
     Eigen::MatrixXd Masse_, J_, J_transp_, Lambda_ ;
     Eigen::VectorXd Xd_curr_, gravity_Torque_, jdotqdot_;
  };

Now, let's see an complete example :

.. code-block:: cpp

  class EnergyConstraint : public GenericConstraint
  {
    public:
    EnergyConstraint() : GenericConstraint( ControlVariable::JointSpaceTorque )
    {
    }

    
    
    void setAccelerationDes(Eigen::VectorXd Acc)
    {
      AccelerationDes_ = Acc ;
    }

     void stateInitialize()
    {
       const int ndof_ = robot().getNrOfDegreesOfFreedom();
       Masse_.resize(ndof_,ndof_);
       J_.resize(6,ndof_);
       jdotqdot_.resize(6);
       Lambda_.resize(6,6);
       gravity_Torque_.resize(ndof_);
       Xd_curr_.resize(6);
       AccelerationDes_.resize(6);
     }
   
     void updateState()
     {
       const int ndof_ = robot().getNrOfDegreesOfFreedom();

       Masse_ = (robot().getFreeFloatingMassMatrix()).block(6,6,ndof_,ndof_);
       J_ = (robot().getFrameFreeFloatingJacobian("link_7")).block(0,6,6,ndof_);
       Lambda_ = (J_*(Masse_.inverse())*J_.transpose()).inverse();
       Xd_curr_ = robot().getFrameVel("link_7");
       gravity_Torque_ = (robot().generalizedBiasForces()).segment(6,ndof_);
       jdotqdot_ = robot().getFrameBiasAcc("link_7");
     
     }   
    void updateConstraintFunction()
    {
       MutexLock lock(mutex);
       auto world = gazebo::physics::get_world();
       double sim_step_dt = world->GetPhysicsEngine()->GetRealTimeUpdateRate()*1e-6;
       const double n_horizon_steps(1);                          
       double horizon_temps = n_horizon_steps * sim_step_dt ;
    
       ...
       // Compute the current energy of end-effector 
       double ec_curr = 0.5 * Xd_curr.transpose() * Lambda_ * Xd_curr;
       double ec_next(0.);
       ec_next = ec_curr + delta_X.transpose()* Lambda_*(jdotqdot - J*Masse.inverse()*Gravity_torque)[0];
       ...
       Eigen::VectorXd ec_max(1),ec_min(1);
       ec_min << -100 - ec_next ;

       ec_max << ec_lim - ec_next ;

       constraintFunction().lowerBound() = ec_min;
       constraintFunction().upperBound() = ec_max;

     }

    void resize()
     {
       MutexLock lock(mutex);
       const int dim = OptimisationVector().getSize(getControlVariable());

       ...
       ...	
       C = delta_X.transpose() * Lambda_ * J * Masse.inverse();

       constraintFunction().resize(1,dim); // this resize lb,C and ub, C to zero, lb to -inf, ub to +inf
       constraintFunction().constraintMatrix()= C;
       }

  
    protected:
    
      Eigen::Matrix<double,6,1> AccelerationDes_;
   };

The function ``resize ()``  set the new dimension of constraint matrix and bounds with respect to **JointTorque**'s dimension, using ``dim = OptimisationVector().getSize(getControlVariable());``, also, it set the value of constraint matrix.
The function ``setAccelerationDes(Eigen::VectorXd Acc)`` return acceleration desired who is used in the function ``updateConstraintFunction()`` for compute the bounds lb_C et ub_C ,   
In the main() code:

.. code-block:: cpp

  int main(int argc, char** argv)
  {
    ...


    EnergyConstraint energycstr;
    energycstr.setRobotModel(robot);
    Eigen::VectorXd Accelerationdes;
    Accelerationdes.resize(6);
    Accelerationdes = cart_acc_pid.getCommand();
    energycstr.stateInitialize();
    energycstr.updateState();
    energycstr.setAccelerationDes(Accelerationdes);
    energycstr.resize();
    energycstr.updateConstraintFunction();
    ...
    
    energycstr.insertInProblem();
    energycstr.activate();

    ...

     while(i++ <= 3000000)
    {
      ...
      energycstr.insertInProblem();
      energycstr.updateState();

      energycstr.update();

      ...}

      ...

      return 0
    }

You can easily change the type of constraint by changing the dimension of matrix constraint and the constraints.  See the complete code in "orca-pu-constraint.hpp". (https://github.com/ZzhengP/Stage)
