.. _02-using_callbacks:

An introduction to the ORCA callback system
====================================================

.. note:: The source code for this example can be found in ``[orca_root]/examples/intermediate/02-using_callbacks.cc``, or alternatively on github at: https://github.com/syroco/orca/blob/dev/examples/intermediate/02-using_callbacks.cc


.. code-block:: c++
    :linenos:


    #include <orca/orca.h>
    #include <chrono>
    using namespace orca::all;

    class TaskMonitor {
    private:
        bool is_activated_ = false;
        bool is_deactivated_ = false;


    public:
        TaskMonitor ()
        {
            std::cout << "TaskMonitor class constructed." << '\n';
        }
        bool isActivated(){return is_activated_;}
        bool isDeactivated(){return is_deactivated_;}

        void onActivation()
        {
            std::cout << "[TaskMonitor] Called 'onActivation' callback." << '\n';
        }

        void onActivated()
        {
            std::cout << "[TaskMonitor] Called 'onActivated' callback." << '\n';
            is_activated_ = true;
        }

        void onUpdateEnd(double current_time, double dt)
        {
            std::cout << "[TaskMonitor] Called 'onUpdateBegin' callback." << '\n';
            std::cout << "  >> current time: " << current_time << '\n';
            std::cout << "  >> dt: " << dt << '\n';
        }

        void onUpdateBegin(double current_time, double dt)
        {
            std::cout << "[TaskMonitor] Called 'onUpdateEnd' callback." << '\n';
            std::cout << "  >> current time: " << current_time << '\n';
            std::cout << "  >> dt: " << dt << '\n';
        }
        void onDeactivation()
        {
            std::cout << "[TaskMonitor] Called 'onDeactivation' callback." << '\n';
        }

        void onDeactivated()
        {
            std::cout << "[TaskMonitor] Called 'onDeactivated' callback." << '\n';
            is_deactivated_ = true;
        }
    };




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

        double dt = 0.1;
        double current_time = 0.0;
        int delay_ms = 500;

        // The good stuff...

        auto task_monitor = std::make_shared<TaskMonitor>();

        cart_task->onActivationCallback(std::bind(&TaskMonitor::onActivation, task_monitor));
        cart_task->onActivatedCallback(std::bind(&TaskMonitor::onActivated, task_monitor));
        cart_task->onComputeBeginCallback(std::bind(&TaskMonitor::onUpdateBegin, task_monitor, std::placeholders::_1, std::placeholders::_2));
        cart_task->onComputeEndCallback(std::bind(&TaskMonitor::onUpdateEnd, task_monitor, std::placeholders::_1, std::placeholders::_2));
        cart_task->onDeactivationCallback(std::bind(&TaskMonitor::onDeactivation, task_monitor));
        cart_task->onDeactivatedCallback(std::bind(&TaskMonitor::onDeactivated, task_monitor));

        std::cout << "[main] Activating tasks and constraints." << '\n';
        controller.activateTasksAndConstraints();
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));

        std::cout << "[main] Starting 'RUN' while loop." << '\n';
        while(!task_monitor->isActivated()) // Run 10 times.
        {
            std::cout << "[main] 'RUN' while loop. Current time: " << current_time << '\n';
            controller.update(current_time, dt);
            current_time +=dt;
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        }
        std::cout << "[main] Exiting 'RUN' while loop." << '\n';

        std::cout << "-----------------\n";

        std::cout << "[main] Deactivating tasks and constraints." << '\n';
        controller.deactivateTasksAndConstraints();
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));

        std::cout << "[main] Starting 'DEACTIVATION' while loop." << '\n';

        while(!task_monitor->isDeactivated())
        {
            std::cout << "[main] 'DEACTIVATION' while loop. Current time: " << current_time << '\n';
            controller.update(current_time, dt);
            current_time += dt;
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        }
        std::cout << "[main] Exiting 'DEACTIVATION' while loop." << '\n';


        std::cout << "[main] Exiting main()." << '\n';
        return 0;
    }
