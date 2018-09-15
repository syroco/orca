.. _simple_controller:

Simple controller
=========================

.. note:: The source code for this example can be found in ``[orca_root]/examples/basic/01-simple_controller.cc``, or alternatively on github at: https://github.com/syroco/orca/blob/dev/examples/basic/01-simple_controller.cc


Objective
-------------------
In this example we want to show the basics of using ORCA. Here, we create a minimal controller with one task and some common constraints.


Introduction
-------------------

First we need to include the appropriate headers and use the right namespaces. When you are getting started the easiest solution is to use the helper header ``orca.h`` and helper namespace ``orca::all`` which include all the necessary headers and opens up all their namespaces. This helps with reducing the verbosity of the examples here but is not recommended for production builds because it will cause code bloat.


.. code-block:: c++

    #include <orca/orca.h>
    using namespace orca::all;


We then create our ``main()`` function...

.. code-block:: c++

    int main(int argc, char const *argv[])


and parse the command line arguments:

.. code-block:: c++

    if(argc < 2)
    {
        std::cerr << "Usage : " << argv[0] << " /path/to/robot-urdf.urdf (optionally -l debug/info/warning/error)" << "\n";
        return -1;
    }
    std::string urdf_url(argv[1]);

    orca::utils::Logger::parseArgv(argc, argv);


ORCA provides a utility class called ``Logger`` which, as its name implies, helps log output. See the API documentation for more information on logging levels.



Setup
--------------

Now we get to the good stuff. We start by creating a robot model which gives us access to the robot's kinematics and dynamics.

.. code-block:: c++

    auto robot_model = std::make_shared<RobotModel>();
    robot->loadModelFromFile(urdf_url);
    robot->setBaseFrame("base_link");
    robot->setGravity(Eigen::Vector3d(0,0,-9.81));

We first instantiate a ``shared_ptr`` to the class ``RobotModel``. We can pass a robot name, but if we don't, it is extracted from the urdf, which is loaded from a file in ``robot->loadModelFromFile(urdf_url);``. If the URDF is parsed then we need to set the base frame in which all transformations (e.g. end effector pose) are expressed in ``robot->setBaseFrame("base_link");``. Finally we manually set the gravity vector ``robot->setGravity(Eigen::Vector3d(0,0,-9.81));`` (this is optional).

The next step is to set the initial state of the robot. For your convenience, ORCA provides a helper class called ``EigenRobotState`` which stores the whole state of the robot as eigen vectors/matrices.
This class is totally optional, it is just meant to keep consistency for the sizes of all the vectors/matrices.
You can use it to fill data from either a real robot or simulated robot.


.. code-block:: c++

    EigenRobotState eigState;
    eigState.resize(robot->getNrOfDegreesOfFreedom());
    eigState.jointPos.setZero();
    eigState.jointVel.setZero();
    robot->setRobotState(eigState.jointPos,eigState.jointVel);

First we resize all the vectors/matrices to match the robot configuration and set the joint positions and velocities to zero. Initial joint positions are often non-zero but we are lazy and ``setZero()`` is so easy to type. Finally, we set the robot state, ``robot->setRobotState(eigState.jointPos,eigState.jointVel);``. Now the robot is considered 'initialized'.

.. note:: Here we only set :math:`\boldsymbol{q}, \dot{\boldsymbol{q}}` because in this example we are dealing with a fixed base robot.




Creating the Controller
-----------------------------


With the robot created and initialized, we can construct a ``Controller``:

.. code-block:: c++

    // Instanciate an ORCA Controller
    orca::optim::Controller controller(
        "controller"
        ,robot
        ,orca::optim::ResolutionStrategy::OneLevelWeighted
        ,QPSolver::qpOASES
    );

To do so we pass a name, ``"controller"``, the robot model, ``robot``, a ``ResolutionStrategy``, ``orca::optim::ResolutionStrategy::OneLevelWeighted``, and a solver, ``QPSolver::qpOASES``.

.. note:: As of now, the only supported solver is ``qpOASES``, however ``OSQP`` will be integrated in a future release.

.. note:: Other ``ResolutionStrategy`` options include: ``MultiLevelWeighted``, and ``Generalized``. Please be aware that these strategies are not yet officially supported.

If your robot's low level controller takes into account the gravity and coriolis torques already (Like with KUKA LWR) then you can tell the controller to remove these components from the torques computed by the solver. Setting them to false keeps the components in the solution (this is the default behavior).


.. code-block:: c++

    controller.removeGravityTorquesFromSolution(true);
    controller.removeCoriolisTorquesFromSolution(true);





Adding Tasks
--------------------

With the controller created we can now start adding tasks. In this introductory example, we add only a Cartesian acceleration task for the end-effector.

.. code-block:: c++

    auto cart_task = std::make_shared<CartesianTask>("CartTask_EE");
    controller.addTask(cart_task);

A ``shared_ptr`` to a ``CartesianTask`` is created with a unique name, ``CartTask_EE``. The task is then added to the controller to initialize it.

For this task, we want to control ``link_7``,

.. code-block:: c++

    cart_task->setControlFrame("link_7");


And set its desired pose:

.. code-block:: c++

    Eigen::Affine3d cart_pos_ref;
    cart_pos_ref.translation() = Eigen::Vector3d(1.,0.75,0.5); // x,y,z in meters
    cart_pos_ref.linear() = Eigen::Quaterniond::Identity().toRotationMatrix();


We also set the desired cartesian velocity and acceleration to zero.

.. code-block:: c++

    Vector6d cart_vel_ref = Vector6d::Zero();
    Vector6d cart_acc_ref = Vector6d::Zero();


.. note:: Rotation is done with a Matrix3x3 and it can be initialized in a few ways. Note that each of these methods produce equivalent Rotation matrices in this case.

    **Example 1:** create a quaternion from Euler anglers ZYZ convention

    .. code-block:: c++

        Eigen::Quaterniond quat;
        quat = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
             * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
        cart_pos_ref.linear() = quat.toRotationMatrix();

    **Example 2:** create a quaternion from RPY convention

    .. code-block:: c++

        cart_pos_ref.linear() = quatFromRPY(0,0,0).toRotationMatrix();

    **Example 3:** create a quaternion from Kuka Convention

    .. code-block:: c++

        cart_pos_ref.linear() = quatFromKukaConvention(0,0,0).toRotationMatrix();

    **Example 4:** use an Identity quaternion

    .. code-block:: c++

        cart_pos_ref.linear() = Eigen::Quaterniond::Identity().toRotationMatrix();



The desired values are set on the servo controller because ``CartesianTask`` expects a cartesian acceleration, which is computed automatically by the servo controller.

.. code-block:: c++

    cart_task->servoController()->setDesired(cart_pos_ref.matrix(),cart_vel_ref,cart_acc_ref);


Now set the servoing PID

.. code-block:: c++

    Vector6d P;
    P << 1000, 1000, 1000, 10, 10, 10;
    cart_task->servoController()->pid()->setProportionalGain(P);
    Vector6d D;
    D << 100, 100, 100, 1, 1, 1;
    cart_task->servoController()->pid()->setDerivativeGain(D);






Adding Constraints
--------------------

Now we add some constraints. We start with a joint torque constraint for all the actuated DoF. To create it we first get the number of actuated joints from the model.

.. code-block:: c++

    const int ndof = robot->getNrOfDegreesOfFreedom();



The joint torque limit is usually given by the robot manufacturer and included in most robot descriptions, but for now it is not parsed directely from the URDF - so we need to add it manually.

.. code-block:: c++

    auto jnt_trq_cstr = std::make_shared<JointTorqueLimitConstraint>("JointTorqueLimit");
    controller.addConstraint(jnt_trq_cstr);
    Eigen::VectorXd jntTrqMax(ndof);
    jntTrqMax.setConstant(200.0);
    jnt_trq_cstr->setLimits(-jntTrqMax,jntTrqMax);


We first create a ``shared_ptr`` with a unique name, ``auto jnt_trq_cstr = std::make_shared<JointTorqueLimitConstraint>("JointTorqueLimit");`` and add it to the controller ``controller.addConstraint(jnt_trq_cstr);``. We then set the torque limits to :math:`\pm{}200 Nm`.


Contrary to torque limits, joint position limits are automatically extracted from the URDF model. Note that you can set them if you want by simply doing ``jnt_pos_cstr->setLimits(jntPosMin,jntPosMax)``.

.. code-block:: c++

    auto jnt_pos_cstr = std::make_shared<JointPositionLimitConstraint>("JointPositionLimit");
    controller.addConstraint(jnt_pos_cstr);



Joint velocity limits are usually given by the robot manufacturer but like the torque limits, must be added manually for now.

.. code-block:: c++

    auto jnt_vel_cstr = std::make_shared<JointVelocityLimitConstraint>("JointVelocityLimit");
    controller.addConstraint(jnt_vel_cstr);
    Eigen::VectorXd jntVelMax(ndof);
    jntVelMax.setConstant(2.0);
    jnt_vel_cstr->setLimits(-jntVelMax,jntVelMax);


With the tasks anc constraints created and added to the controller, we can begin the control loop.


Control Loop
--------------------

The control loop is where the robot model is updated using the current state information from the real or simulated robot, the control problem is formulated and solved, and the resultant joint torques are sent to the robot actuators. For this example, we simply calculate the joint torques :math:`\boldsymbol{\tau}` at each control time step and do nothing with them. This is because we are not interacting with a real robot or a simulated robot.


.. code-block:: c++

    double dt = 0.001;
    double current_time = 0;

    controller.activateTasksAndConstraints();

    for (; current_time < 2.0; current_time +=dt)
    {
        // Here you can get the data from your robot (API is robot-specific)
        // Something like :
            // eigState.jointPos = myRealRobot.getJointPositions();
            // eigState.jointVel = myRealRobot.getJointVelocities();

        robot->setRobotState(eigState.jointPos,eigState.jointVel);
        controller.update(current_time, dt);
        if(controller.solutionFound())
        {
            const Eigen::VectorXd& trq_cmd = controller.getJointTorqueCommand();

            // Send torques to the REAL robot (API is robot-specific)
            // myRealRobot.set_joint_torques(trq_cmd);
        }
        else
        {
            // WARNING : Optimal solution is NOT found
            // Perform some fallback strategy (see below)
        }
    }

First, since we are manually stepping the time, we initialize the ``current_time`` to zero and the ``dt=0.001``.

The next important step is to activate the tasks and constraints: ``controller.activateTasksAndConstraints();``. This **must** be done before the controller update is called, or else no solution will be found.

Now that the tasks and constraints are activated, we step into the control loop, which increments ``current_time`` from ``0.0`` to ``2.0`` seconds by ``dt``:

.. code-block:: c++

    for (; current_time < 2.0; current_time +=dt)

At the begining of each loop, we must first retrieve the robot's state information so that we can update our robot model being used in the controller. This step depends on the robot-specific API being used and is up to the user to implement.

.. note:: In future examples we demonstrate how to do this with the Gazebo simulator.

After we get the appropriate state information from our robot (in this case, the joint positions and velocities) we update the robot model: ``robot->setRobotState(eigState.jointPos,eigState.jointVel);``.
With the model updated we now update the controller, ``controller.update(current_time, dt);``.
The controller update first updates all of the tasks and constraints, then formulates the optimal control problem, then solves said problem.
If the controller found a solution to the optimal control problem then ``controller.solutionFound()`` will return true and this tells you that you can get that result and use it to control your robot.
Here we extract the optimal control torques, ``const Eigen::VectorXd& trq_cmd = controller.getJointTorqueCommand();`` and then send them to our robot, using robot specific functions.

.. note:: In this example, we extract only the optimal torques, but you of course have access to the full solution:

    .. code-block:: c++

        // The whole optimal solution [AccFb, Acc, Tfb, T, eWrenches]
        const Eigen::VectorXd& full_solution = controller.getSolution();
        // The optimal joint torque command
        const Eigen::VectorXd& trq_cmd = controller.getJointTorqueCommand();
        // The optimal joint acceleration command
        const Eigen::VectorXd& trq_acc = controller.getJointAccelerationCommand();

If the controller fails to find a solution to the problem then ``controller.solutionFound()`` returns ``false``, and you must implement some **fallback** strategy. By fallback, we mean some strategy to be used when we have no idea what torques to send to the robot. A simple but effective strategy, is to simply brake the robot and stop its motion.

.. important:: If the optimal control problem has no solution it is generally because the tasks and constraints are ill-defined and not because no solution exists. For this reason, one can implement fallback strategies which are slightly more intelligent than simply stopping the robot. For example:
    - Compute KKT Solution and send to the robot (solutions without inequality constraints)
    - PID around the current position (to slow to a halt)
    - Switch controllers
    - etc.


Shutting Things Down
-----------------------

Once we are finished using the controller and want to bring everything to a stop, we need to gradually deactivate the tasks and constraints to avoid any erratic behaviors at the end of the motion.
To do so, we start by deactivating the tasks and constraints:

.. code-block:: c++

    controller.deactivateTasksAndConstraints();

We then need to update the controller so the tasks and constraints can slowly ramp down to total deactivation.

.. code-block:: c++

    while(!controller.tasksAndConstraintsDeactivated())
    {
        current_time += dt;
        controller.update(current_time,dt);
    }


Our controller is now deactivated and can be ``deleted`` or destroyed without any issues.

Typically at the end of the execution you would either stop the robot or put it into some robot-specific control mode (position control, gravity compensation, etc.).


Conclusion
--------------

In this example you have seen all of the necessary steps to getting an ORCA controller up and running. In the next examples we will look at more realistic examples where the controller interacts with a robot/simulation.


.. _simple_controller_full_code_listing:

Full Code Listing
---------------------------

.. literalinclude:: ../../../../examples/basic/01-simple_controller.cc
   :language: c++
   :linenos:
