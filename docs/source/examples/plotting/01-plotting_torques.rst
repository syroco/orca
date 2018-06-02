.. _01-plotting_torques:

Using the internal plotting tools
====================================================


.. note:: The source code for this example can be found in ``[orca_root]/examples/plotting/01-plotting_torques.cc``, or alternatively on github at: https://github.com/syroco/orca/blob/dev/examples/plotting/01-plotting_torques.cc

.. code-block:: c++
    :linenos:

    #include <orca/orca.h>
    #include <matplotlibcpp/matplotlibcpp.h>
    using namespace orca::all;

    namespace plt = matplotlibcpp;

    int main(int argc, char const *argv[])
    {
        // Get the urdf file from the command line
        if(argc < 2)
        {
            std::cerr << "Usage : " << argv[0] << " /path/to/robot-urdf.urdf (optionally -l debug/info/warning/error)" << "\n";
            return -1;
        }
        std::string urdf_url(argv[1]);

        // Parse logger level as --log_level (or -l) debug/warning etc
        orca::utils::Logger::parseArgv(argc, argv);

        // Create the kinematic model that is shared by everybody
        auto robot = std::make_shared<RobotDynTree>(); // Here you can pass a robot name
        robot->loadModelFromFile(urdf_url); // If you don't pass a robot name, it is extracted from the urdf
        robot->setBaseFrame("base_link"); // All the transformations (end effector pose for example) will be expressed wrt this base frame
        robot->setGravity(Eigen::Vector3d(0,0,-9.81)); // Sets the world gravity (Optional)

        // This is an helper function to store the whole state of the robot as eigen vectors/matrices
        // This class is totally optional, it is just meant to keep consistency for the sizes of all the vectors/matrices
        // You can use it to fill data from either real robot and simulated robot
        RobotState eigState;
        eigState.resize(robot->getNrOfDegreesOfFreedom()); // resize all the vectors/matrices to match the robot configuration
        // Set the initial state to zero (arbitrary)
        // NOTE : here we only set q,qot because this example asserts we have a fixed base robot
        eigState.jointPos.setZero();
        eigState.jointVel.setZero();
        // Set the first state to the robot
        robot->setRobotState(eigState.jointPos,eigState.jointVel); // Now is the robot is considered 'initialized'

        // Instanciate an ORCA Controller
        orca::optim::Controller controller(
            "controller"
            ,robot
            ,orca::optim::ResolutionStrategy::OneLevelWeighted // MultiLevelWeighted, Generalized
            ,QPSolver::qpOASES
        );

        // Cartesian Task
        auto cart_task = std::make_shared<CartesianTask>("CartTask-EE");
        controller.addTask(cart_task); // Add the task to the controller to initialize it
        // Set the frame you want to control
        cart_task->setControlFrame("link_7"); // We want to control the link_7

        // Set the pose desired for the link_7
        Eigen::Affine3d cart_pos_ref;
        // Translation
        cart_pos_ref.translation() = Eigen::Vector3d(1.,0.75,0.5); // x,y,z in meters
        // Rotation is done with a Matrix3x3
        Eigen::Quaterniond quat;
        // Example 1 : create a quaternion from Euler anglers ZYZ convention
        quat = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
             * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
        // Example 2 : create a quaternion from RPY convention
        cart_pos_ref.linear() = quatFromRPY(0,0,0).toRotationMatrix();
        // Example 3 : create a quaternion from Kuka Convention
        cart_pos_ref.linear() = quatFromKukaConvention(0,0,0).toRotationMatrix();

        // Set the desired cartesian velocity to zero
        Vector6d cart_vel_ref;
        cart_vel_ref.setZero();

        // Set the desired cartesian velocity to zero
        Vector6d cart_acc_ref;
        cart_acc_ref.setZero();

        // Now set the servoing PID
        Vector6d P;
        P << 1000, 1000, 1000, 10, 10, 10;
        cart_task->servoController()->pid()->setProportionalGain(P);
        Vector6d D;
        D << 100, 100, 100, 1, 1, 1;
        cart_task->servoController()->pid()->setDerivativeGain(D);
        // The desired values are set on the servo controller
        // Because cart_task->setDesired expects a cartesian acceleration
        // Which is computed automatically by the servo controller
        cart_task->servoController()->setDesired(cart_pos_ref.matrix(),cart_vel_ref,cart_acc_ref);

        // Get the number of actuated joints
        const int ndof = robot->getNrOfDegreesOfFreedom();

        // Joint torque limit is usually given by the robot manufacturer
        auto jnt_trq_cstr = std::make_shared<JointTorqueLimitConstraint>("JointTorqueLimit");
        controller.addConstraint(jnt_trq_cstr); // Add the constraint to the controller to initialize it
        Eigen::VectorXd jntTrqMax(ndof);
        jntTrqMax.setConstant(200.0);
        jnt_trq_cstr->setLimits(-jntTrqMax,jntTrqMax); // because not read in the URDF for now

        // Joint position limits are automatically extracted from the URDF model
        // Note that you can set them if you want
        // by simply doing jnt_pos_cstr->setLimits(jntPosMin,jntPosMax);
        auto jnt_pos_cstr = std::make_shared<JointPositionLimitConstraint>("JointPositionLimit");
        controller.addConstraint(jnt_pos_cstr); // Add the constraint to the controller to initialize it

        // Joint velocity limits are usually given by the robot manufacturer
        auto jnt_vel_cstr = std::make_shared<JointVelocityLimitConstraint>("JointVelocityLimit");
        controller.addConstraint(jnt_vel_cstr); // Add the constraint to the controller to initialize it
        Eigen::VectorXd jntVelMax(ndof);
        jntVelMax.setConstant(2.0);
        jnt_vel_cstr->setLimits(-jntVelMax,jntVelMax);  // because not read in the URDF for now

        double dt = 0.001;
        double total_time = 1.0;
        double current_time = 0;

        // Shortcut : activate all tasks
        controller.activateTasksAndConstraints();

        // Now you can run the control loop
        std::vector<double> time_log;
        int ncols = std::ceil(total_time/dt);
        Eigen::MatrixXd torqueMat(ndof,ncols);
        torqueMat.setZero();

        for (int count = 0; current_time < total_time; current_time +=dt)
        {
            time_log.push_back(current_time);

            // Here you can get the data from you REAL robot (API might vary)
            // Some thing like :
            //      eigState.jointPos = myRealRobot.getJointPositions();
            //      eigState.jointVel = myRealRobot.getJointVelocities();

            // Now update the internal kinematic model with data from REAL robot
            robot->setRobotState(eigState.jointPos,eigState.jointVel);

            // Step the controller
            if(controller.update(current_time,dt))
            {

                // Get the controller output
                const Eigen::VectorXd& full_solution = controller.getSolution();

                torqueMat.col(count) = controller.getJointTorqueCommand();

                const Eigen::VectorXd& trq_acc = controller.getJointAccelerationCommand();

                // Here you can send the commands to you REAL robot
                // Something like :
                // myRealRobot.setTorqueCommand(trq_cmd);
            }
            else
            {
                // Controller could not get the optimal torque
                // Now you have to save your robot
                // You can get the return code with controller.getReturnCode();
            }

            count++;

            std::cout << "current_time  " << current_time << '\n';
            std::cout << "total_time  " << total_time << '\n';
            std::cout << "time log size  " << time_log.size() << '\n';
            std::cout << "torqueMat.cols " << torqueMat.cols() << '\n';
        }

        // Print the last computed solution (just for fun)
        const Eigen::VectorXd& full_solution = controller.getSolution();
        const Eigen::VectorXd& trq_cmd = controller.getJointTorqueCommand();
        const Eigen::VectorXd& trq_acc = controller.getJointAccelerationCommand();
        LOG_INFO << "Full solution : " << full_solution.transpose();
        LOG_INFO << "Joint Acceleration command : " << trq_acc.transpose();
        LOG_INFO << "Joint Torque command       : " << trq_cmd.transpose();

        // At some point you want to close the controller nicely
        controller.deactivateTasksAndConstraints();
        // Let all the tasks ramp down to zero
        while(!controller.tasksAndConstraintsDeactivated())
        {
            current_time += dt;
            controller.print();
            controller.update(current_time,dt);
        }

        // Plot data
        for (size_t i = 0; i < torqueMat.rows(); i++)
        {
            std::vector<double> trq(time_log.size());
            Eigen::VectorXd::Map(trq.data(),time_log.size()) = torqueMat.row(i);
            plt::plot(time_log,trq);
        }
        plt::show();
        return 0;
    }
