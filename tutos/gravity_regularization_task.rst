Gravity regularization task
---------------------------

This section give an example to let u know how create a new task. the example here is gravity regularization task, this class is define before main program.

.. code-block:: cpp

  class GravityrRegularisationTask : public RegularisationTask<ControlVariable::GeneralisedTorque>
  {

    void updateAffineFunction()
    {


      Eigen::MatrixXd Massetot  ; 			 // Masstot is including the floating base
      Eigen::VectorXd vel; 		 // Gravity_torque_tot is including the floating base

      vel.setZero(6 + this->robot().getNrOfDegreesOfFreedom()); // initialize a vector of velocity include the floating base.
      vel.tail(this->robot().getNrOfDegreesOfFreedom()) = this->robot().getJointVel();

      Massetot = this->robot().getFreeFloatingMassMatrix();
      // set A and b
      EuclidianNorm().b() =-2* Massetot.inverse() * (this->robot(). generalizedBiasForces() - vel);
      EuclidianNorm().A() = 2*Massetot.inverse();
    }

    };


In orca control section, we said that the tasks are written as **weighted euclidian distance function**:

.. math::

    w_{task}  \lVert \mathbf{E}x + \mathbf{f} \rVert_{W_{norm}}^2


For this class, we not use all optimization vector, but only variables corresponds to Floating base joint torque(6*1) and joint space joint torque(ndof*1).
using ``this->robot().`` allow you to get all robot's parameters, for example, ``this->robot().generalizedBiasForces()`` return force include gravity torque and Coriolis term
see more detail in ``RobotDynTree.cc``.

As the task is expressed by a linear form, we have to specify the term **E** and **f** associate. They are called **A** and **b** here and redefine by:

.. code-block:: cpp

  EuclidianNorm().b() = ...
  EuclidianNorm().A() = ...

Finally, in main() code, we have to declare this class, update it, insert it in the problem and active it, the function ``setWeight()`` represente how much this task is important respect to other task.  :

.. code-block:: cpp

  int main(int argc, char** argv)
  {
    ...


    GravityrRegularisationTask trq_reg_task;
    trq_reg_task.setRobotModel(robot);
    trq_reg_task.setName("gravity_reg_task");
    trq_reg_task.setWeight(1E-4);
    trq_reg_task.update();

    ...
    trq_reg_task.insertInProblem();
    trq_reg_task.activate();

    ...

     while(i++ <= 3000000)
    {
      ...

      trq_reg_task.update();

      ...}

      ...

      return 0
    }

See the complete code with "orca-pu-gazebo.cc" in examples.