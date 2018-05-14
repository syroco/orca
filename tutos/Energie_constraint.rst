Energy constraint
-----------------


This tutorial give an example to add a new constraint. We will add a constraint to robot's energy (for some security criterion).

This class inherit GenericConstraint, and the **ControleVariable** associate is **JointSpaceTorque**. Remember the problem is written as a **quadratic problem**:

.. math::

    lb_C \leq Cx \leq ub_C

So, we have to set the bounds and the constraint matrix, and those bounds and constraint are expressed explicitly by JointTorque. For this problem, we will create a new class, who is called **EnergyConstraint**.


.. code-block:: cpp
  
  class EnergyConstraint : public GenericConstraint
  {
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
      energycstr.update();

      ...}

      ...

      return 0
    }

You can easily change the type of constraint by changing the dimension of matrix constraint and the constraints.  See the complete code with "orca-pu-gazebo.cc" in examples.
