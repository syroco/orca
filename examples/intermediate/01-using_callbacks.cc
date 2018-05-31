

#include <orca/orca.h>
using namespace orca::all;

class TaskPrinter {
public:
    TaskPrinter ()
    {
        std::cout << "TaskPrinter class constructed." << '\n';
    }
    void onActivation()
    {
        std::cout << "[TaskPrinter Class] Called 'onActivation' callback." << '\n';
    }

    void onActivated()
    {
        std::cout << "[TaskPrinter Class] Called 'onActivated' callback." << '\n';
    }

    void onUpdate(double current_time, double dt)
    {
        std::cout << "[TaskPrinter Class] Called 'onUpdate' callback." << '\n';
        std::cout << ">> current time: " << current_time << '\n';
        std::cout << ">> dt: " << dt << '\n';
    }

    void onDeactivation()
    {
        std::cout << "[TaskPrinter Class] Called 'onDeactivation' callback." << '\n';
    }

    void onDeactivated()
    {
        std::cout << "[TaskPrinter Class] Called 'onDeactivated' callback." << '\n';
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

    double dt = 0.001;
    double current_time = 0.0;


    // The good stuff...

    TaskPrinter task_printer();

    cart_task->onActivationCb(std::bind(&TaskPrinter::onActivation, &task_printer));
    cart_task->onActivatedCb(std::bind(&TaskPrinter::onActivated, &task_printer));
    cart_task->onUpdateCb(std::bind(&TaskPrinter::onUpdate, &task_printer, _1, _2));
    cart_task->onDeactivationCb(std::bind(&TaskPrinter::onDeactivation, &task_printer));
    cart_task->onDeactivatedCb(std::bind(&TaskPrinter::onDeactivated, &task_printer));

    std::cout << "[main] Activating tasks and constraints." << '\n';
    controller.activateTasksAndConstraints();

    std::cout << "[main] Starting 'RUN' while loop." << '\n';
    while(current_time < dt*10.0) // Run 10 times.
    {
        std::cout << "[main] Running in while loop. Current time: " << current_time << '\n';
        controller.update(current_time, dt);
        current_time +=dt;
    }
    std::cout << "[main] Exiting 'RUN' while loop." << '\n';

    std::cout << "\n\n\n" << '\n';

    std::cout << "[main] Deactivating tasks and constraints." << '\n';
    controller.deactivateTasksAndConstraints();

    std::cout << "[main] Starting 'DEACTIVATION' while loop." << '\n';
    while(!controller.tasksAndConstraintsDeactivated())
    {
        std::cout << "[main] Deactivating in while loop. Current time: " << current_time << '\n';
        controller.update(current_time, dt);
        current_time += dt;
    }
    std::cout << "[main] Exiting 'DEACTIVATION' while loop." << '\n';


    std::cout << "[main] Exiting main()." << '\n';
    return 0;
}
