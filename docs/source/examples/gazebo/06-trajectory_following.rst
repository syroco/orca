.. _06-trajectory_following:

Minimum jerk Cartesian trajectory following
==================================================

.. note:: The source code for this example can be found in ``[orca_root]/examples/gazebo/06-trajectory_following.cc``, or alternatively on github at: https://github.com/syroco/orca/blob/dev/examples/gazebo/06-trajectory_following.cc


.. code-block:: c++
    :linenos:

    #include <orca/orca.h>
    #include <orca/gazebo/GazeboServer.h>
    #include <orca/gazebo/GazeboModel.h>

    using namespace orca::all;
    using namespace orca::gazebo;

    class MinJerkPositionTrajectory {
    private:
        Eigen::Vector3d alpha_, sp_, ep_;
        double duration_ = 0.0;
        double start_time_ = 0.0;
        bool first_call_ = true;
        bool traj_finished_ = false;

    public:
        MinJerkPositionTrajectory (double duration)
        : duration_(duration)
        {
        }

        bool isTrajectoryFinished(){return traj_finished_;}

        void resetTrajectory(const Eigen::Vector3d& start_position, const Eigen::Vector3d& end_position)
        {
            sp_ = start_position;
            ep_ = end_position;
            alpha_ = ep_ - sp_;
            first_call_ = true;
            traj_finished_ = false;
        }

        void getDesired(double current_time, Eigen::Vector3d& p, Eigen::Vector3d& v, Eigen::Vector3d& a)
        {
            if(first_call_)
            {
                start_time_ = current_time;
                first_call_ = false;
            }
            double tau = (current_time - start_time_) / duration_;
            if(tau >= 1.0)
            {
                p = ep_;
                v = Eigen::Vector3d::Zero();
                a = Eigen::Vector3d::Zero();

                traj_finished_ = true;
                return;
            }
            p =                         sp_ + alpha_ * ( 10*pow(tau,3.0) - 15*pow(tau,4.0)  + 6*pow(tau,5.0)   );
            v = Eigen::Vector3d::Zero() + alpha_ * ( 30*pow(tau,2.0) - 60*pow(tau,3.0)  + 30*pow(tau,4.0)  );
            a = Eigen::Vector3d::Zero() + alpha_ * ( 60*pow(tau,1.0) - 180*pow(tau,2.0) + 120*pow(tau,3.0) );
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

        double dt = 0.001;
        double current_time = 0.0;

        GazeboServer gzserver(argc,argv);
        auto gzrobot = GazeboModel(gzserver.insertModelFromURDFFile(urdf_url));

        ///////////////////////////////////////
        ///////////////////////////////////////
        ///////////////////////////////////////
        ///////////////////////////////////////

        MinJerkPositionTrajectory traj(5.0);
        int traj_loops = 0;
        bool exit_control_loop = true;
        Eigen::Vector3d start_position, end_position;


        cart_task->onActivationCallback([](){
            std::cout << "Activating CartesianTask..." << '\n';
        });

        bool cart_task_activated = false;

        cart_task->onActivatedCallback([&](){
            start_position = cart_task->servoController()->getCurrentCartesianPose().block(0,3,3,1);
            end_position = cart_pos_ref.translation();
            traj.resetTrajectory(start_position, end_position);
            std::cout << "CartesianTask activated. Removing gravity compensation and begining motion." << '\n';
            cart_task_activated = true;
        });

        cart_task->onComputeBeginCallback([&](double current_time, double dt){
            if (cart_task->getState() == TaskBase::State::Activated)
            {
                Eigen::Vector3d p, v, a;
                traj.getDesired(current_time, p, v, a);
                cart_pos_ref.translation() = p;
                cart_vel_ref.head(3) = v;
                cart_acc_ref.head(3) = a;
                cart_task->servoController()->setDesired(cart_pos_ref.matrix(),cart_vel_ref,cart_acc_ref);
            }
        });

        cart_task->onComputeEndCallback([&](double current_time, double dt){
            if (cart_task->getState() == TaskBase::State::Activated)
            {
                if (traj.isTrajectoryFinished()  )
                {
                    if (traj_loops < 5)
                    {
                        // flip start and end positions.
                        auto ep = end_position;
                        end_position = start_position;
                        start_position = ep;
                        traj.resetTrajectory(start_position, end_position);
                        std::cout << "Changing trajectory direction." << '\n';
                        ++traj_loops;
                    }
                    else
                    {
                        std::cout << "Trajectory looping finished. Deactivating task and starting gravity compensation." << '\n';
                        cart_task->deactivate();
                    }
                }
            }
        });

        cart_task->onDeactivationCallback([&cart_task_activated](){
            std::cout << "Deactivating task." << '\n';
            cart_task_activated = false;
        });

        cart_task->onDeactivatedCallback([](){
            std::cout << "CartesianTask deactivated. Stopping controller" << '\n';
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
