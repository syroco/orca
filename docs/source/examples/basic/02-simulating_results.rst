.. _02-simulating_results:

Simulating the controller performance
====================================================

.. note:: The source code for this example can be found in ``[orca_root]/examples/basic/02-simulating_results.cc``, or alternatively on github at: https://github.com/syroco/orca/blob/dev/examples/basic/02-simulating_results.cc

.. code-block:: c++
    :linenos:


    #include <orca/orca.h>
    using namespace orca::all;



    int main(int argc, char const *argv[])
    {
        if(argc < 2)
        {
            std::cerr << "Usage : " << argv[0] << " /path/to/robot-urdf.urdf (optionally -l debug/info/warning/error)" << "\n";
            return -1;
        }
        std::string urdf_url(argv[1]);

        orca::utils::Logger::parseArgv(argc, argv);

        auto robot = std::make_shared<RobotDynTree>();
        robot->loadModelFromFile(urdf_url);
        robot->setBaseFrame("base_link");
        robot->setGravity(Eigen::Vector3d(0,0,-9.81));
        EigenRobotState eigState;
        eigState.resize(robot->getNrOfDegreesOfFreedom());
        eigState.jointPos.setZero();
        eigState.jointVel.setZero();
        robot->setRobotState(eigState.jointPos,eigState.jointVel);

        orca::optim::Controller controller(
            "controller"
            ,robot
            ,orca::optim::ResolutionStrategy::OneLevelWeighted
            ,QPSolver::qpOASES
        );

        auto cart_task = std::make_shared<CartesianTask>("CartTask-EE");
        controller.addTask(cart_task);
        cart_task->setControlFrame("link_7"); //
        Eigen::Affine3d cart_pos_ref;
        cart_pos_ref.translation() = Eigen::Vector3d(1.,0.75,0.5); // x,y,z in meters
        cart_pos_ref.linear() = Eigen::Quaterniond::Identity().toRotationMatrix();
        Vector6d cart_vel_ref = Vector6d::Zero();
        Vector6d cart_acc_ref = Vector6d::Zero();

        Vector6d P;
        P << 1000, 1000, 1000, 10, 10, 10;
        cart_task->servoController()->pid()->setProportionalGain(P);
        Vector6d D;
        D << 100, 100, 100, 1, 1, 1;
        cart_task->servoController()->pid()->setDerivativeGain(D);

        cart_task->servoController()->setDesired(cart_pos_ref.matrix(),cart_vel_ref,cart_acc_ref);

        const int ndof = robot->getNrOfDegreesOfFreedom();

        auto jnt_trq_cstr = std::make_shared<JointTorqueLimitConstraint>("JointTorqueLimit");
        controller.addConstraint(jnt_trq_cstr);
        Eigen::VectorXd jntTrqMax(ndof);
        jntTrqMax.setConstant(200.0);
        jnt_trq_cstr->setLimits(-jntTrqMax,jntTrqMax);

        auto jnt_pos_cstr = std::make_shared<JointPositionLimitConstraint>("JointPositionLimit");
        controller.addConstraint(jnt_pos_cstr);

        auto jnt_vel_cstr = std::make_shared<JointVelocityLimitConstraint>("JointVelocityLimit");
        controller.addConstraint(jnt_vel_cstr);
        Eigen::VectorXd jntVelMax(ndof);
        jntVelMax.setConstant(2.0);
        jnt_vel_cstr->setLimits(-jntVelMax,jntVelMax);


        controller.activateTasksAndConstraints();
        // for each task, it calls task->activate(), that can call onActivationCallback() if it is set.
        // To set it :
        // task->setOnActivationCallback([&]()
        // {
        //      // Do some initialisation here
        // });
        // Note : you need to set it BEFORE calling
        // controller.activateTasksAndConstraints();





        double dt = 0.001;
        double current_time = 0.0;
        Eigen::VectorXd trq_cmd(ndof);
        Eigen::VectorXd acc_new(ndof);

        controller.update(current_time, dt);

        std::cout << "\n\n\n" << '\n';
        std::cout << "====================================" << '\n';
        std::cout << "Initial State:\n" << cart_task->servoController()->getCurrentCartesianPose() << '\n';
        std::cout << "Desired State:\n" << cart_pos_ref.matrix() << '\n';
        std::cout << "====================================" << '\n';
        std::cout << "\n\n\n" << '\n';
        std::cout << "Begining Simulation..." << '\n';

        for (; current_time < 2.0; current_time +=dt)
        {

            robot->setRobotState(eigState.jointPos,eigState.jointVel);

            // if(current_time % 0.1 == 0.0)
            // {
            //
            // }
            std::cout << "Task position at t = " << current_time << "\t---\t" << cart_task->servoController()->getCurrentCartesianPose().block(0,3,3,1).transpose() << '\n';

            controller.update(current_time, dt);

            if(controller.solutionFound())
            {
                trq_cmd = controller.getJointTorqueCommand();
            }
            else
            {
                std::cout << "[warning] Didn't find a solution, using last valid solution." << '\n';
            }

            acc_new = robot->getMassMatrix().ldlt().solve(trq_cmd - robot->getJointGravityAndCoriolisTorques());

            eigState.jointPos += eigState.jointVel * dt + ((acc_new*dt*dt)/2);
            eigState.jointVel += acc_new * dt;
        }
        std::cout << "Simulation finished." << '\n';
        std::cout << "\n\n\n" << '\n';
        std::cout << "====================================" << '\n';
        std::cout << "Final State:\n" << cart_task->servoController()->getCurrentCartesianPose() << '\n';
        // std::cout << "Position error:\n" << cart_task->servoController()->getCurrentCartesianPose(). - cart_pos_ref.translation() << '\n';




        // All objets will be destroyed here
        return 0;
    }
