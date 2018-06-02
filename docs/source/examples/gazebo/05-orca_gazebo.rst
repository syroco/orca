.. _05-orca_gazebo:

Using Gazebo to simulate an ORCA controller
==================================================

.. note:: The source code for this example can be found in ``[orca_root]/examples/gazebo/05-orca_gazebo.cc``, or alternatively on github at: https://github.com/syroco/orca/blob/dev/examples/gazebo/05-orca_gazebo.cc


.. code-block:: c++
    :linenos:

    #include <orca/orca.h>
    #include <orca/gazebo/GazeboServer.h>
    #include <orca/gazebo/GazeboModel.h>

    using namespace orca::all;
    using namespace orca::gazebo;



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
        RobotState eigState;
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
        cart_pos_ref.translation() = Eigen::Vector3d(0.5,-0.5,0.8); // x,y,z in meters
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

        GazeboServer gzserver(argc,argv);
        auto gzrobot = GazeboModel(gzserver.insertModelFromURDFFile(urdf_url));

        ///////////////////////////////////////
        ///////////////////////////////////////
        ///////////////////////////////////////
        ///////////////////////////////////////

        bool cart_task_activated = false;

        cart_task->onActivatedCallback([&cart_task_activated](){
            std::cout << "CartesianTask activated. Removing gravity compensation and begining motion." << '\n';
            cart_task_activated = true;
        });

        gzrobot.setCallback([&](uint32_t n_iter,double current_time,double dt)
        {
            robot->setRobotState(gzrobot.getWorldToBaseTransform().matrix()
                                ,gzrobot.getJointPositions()
                                ,gzrobot.getBaseVelocity()
                                ,gzrobot.getJointVelocities()
                                ,gzrobot.getGravity()
                            );
            // All tasks need the robot to be initialized during the activation phase
            if(n_iter == 1)
                controller.activateTasksAndConstraints();

            controller.update(current_time, dt);
            if (cart_task_activated)
            {
                if(controller.solutionFound())
                {
                    gzrobot.setJointTorqueCommand( controller.getJointTorqueCommand() );
                }
                else
                {
                    gzrobot.setBrakes(true);
                }
            }
            else
            {
                gzrobot.setJointGravityTorques(robot->getJointGravityTorques());
            }
        });

        std::cout << "Simulation running... (GUI with \'gzclient\')" << "\n";
        gzserver.run();
        return 0;
    }
