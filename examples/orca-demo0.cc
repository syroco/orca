#include <orca/orca.h>
using namespace orca;
using namespace orca::common;
using namespace orca::optim;
using namespace orca::task;
using namespace orca::constraint;
using namespace orca::robot;
using namespace orca::math;
using namespace orca::util;

int main(int argc, char const *argv[])
{
    // Parse urdf and logger level as --log_level (or -l) debug/warning etc
    orca::util::Logger::parseArgv( argc, argv );

    if(argc < 2)
    {
        std::cerr << "Usage : ./orca-demosimple /path/to/robot-urdf.urdf" << "\n";
        return -1;
    }

    std::string urdf_url(argv[1]);

    auto robot = std::make_shared<RobotDynTree>();
    if(!robot->loadModelFromFile(urdf_url))
        throw std::runtime_error(Formatter() << "Could not load model from urdf file \'" << urdf_url << "\'");

    robot->setBaseFrame("base_link"); // All the transformations will be expressed wrt this base frame
    robot->setGravity(Eigen::Vector3d(0,0,-9.81)); // Sets the world gravity

    // This is an helper function to store the whole state of the robot as eigen vectors/matrices
    // This class is totally optional, it is just meant to keep consistency for the sizes of all the vectors/matrices
    // You can use it to fill data from either real robot and simulated robot
    EigenRobotState eigState;
    eigState.setFixedBase(); // sets world to base to identity and base velocity to zero
    eigState.resize(robot->getNrOfDegreesOfFreedom()); // resize all the vectors/matrices to matche the robot configuration

    // Set the initial state to zero (arbitrary)
    // NOTE : here we only set q,qot because this example asserts we have a fixed base robot
    eigState.jointPos.setZero();
    eigState.jointVel.setZero();
    // Set the first state to the robot
    robot->setRobotState(eigState.jointPos,eigState.jointVel); // Now is the robot is considered 'initialized'

    std::cout << "===== Optimisation problem" << '\n';

    auto problem = std::make_shared<WeightedProblem>();
    problem->setRobotModel(robot);

    std::cout << "===== Cartesian Task creation" << '\n';

    CartesianTask cart_task;
    cart_task.setName("CartTask-EE");
    cart_task.setRobotModel(robot); // Create and Resize the models associated to the task
    cart_task.setProblem(problem);
    cart_task.setControlFrame("link_7"); // We want to control the link_7

    Eigen::Affine3d cart_pos_ref;
    cart_pos_ref.translation() = Eigen::Vector3d(1.,0.75,0.5);
    Eigen::Quaterniond quat;
    quat = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
    cart_pos_ref.linear() = quat.toRotationMatrix();

    Vector6d cart_vel_ref;
    cart_vel_ref.setZero();
    Vector6d cart_acc_ref;
    cart_acc_ref.setZero();

    std::cout << "===== Cartesian Acceleration PID" << '\n';

    CartesianAccelerationPID cart_acc_pid;
    cart_acc_pid.setName("CartPID-EE");
    cart_acc_pid.setRobotModel(robot);
    Vector6d P_gain;
    P_gain << 100, 100, 100, 10, 10, 10;

    Vector6d D_gain;
    D_gain << 10, 10, 10, 1, 1, 1;

    cart_acc_pid.pid().setProportionalGain(P_gain);
    cart_acc_pid.pid().setDerivativeGain(D_gain);
    cart_acc_pid.setControlFrame(cart_task.getControlFrame()); // Of course use the same as the cart task
    cart_acc_pid.setDesired(cart_pos_ref.matrix(),cart_vel_ref,cart_acc_ref);
    cart_acc_pid.update();

    cart_task.setDesired(cart_acc_pid.getCommand());
    cart_task.update(); //The state of the task is now "initialized" (computes for the first time the associated cost). It can now be used.


    std::cout << "===== Dynamics Equation Constraint" << '\n';

    DynamicsEquationConstraint dyn_cstr;
    dyn_cstr.setName("DynamicsEquation");
    dyn_cstr.setRobotModel(robot);
    dyn_cstr.setProblem(problem);
    dyn_cstr.update();

    std::cout << "===== Joint limits" << '\n';

    const int ndof = robot->getNrOfDegreesOfFreedom();

    JointTorqueLimitConstraint jnt_trq_cstr;
    jnt_trq_cstr.setName("JointTorqueLimit");
    Eigen::VectorXd jntTrqMax;
    jntTrqMax.resize(ndof);
    jntTrqMax.setConstant(200.0);
    jntTrqMax<<200,200,200,200,120,120,120;
    jnt_trq_cstr.setRobotModel(robot);
    jnt_trq_cstr.setProblem(problem);
    jnt_trq_cstr.setLimits(-jntTrqMax,jntTrqMax); // because not read in the URDF for now
    jnt_trq_cstr.update();

    JointPositionLimitConstraint jnt_pos_cstr;
    jnt_pos_cstr.setName("JointPositionLimit");
    jnt_pos_cstr.setRobotModel(robot);  // Positions limits are actually read from URDF on loading
    jnt_pos_cstr.setProblem(problem);
    jnt_pos_cstr.update();

    JointVelocityLimitConstraint jnt_vel_cstr;
    Eigen::VectorXd jntVelMax;
    jntVelMax.resize(ndof);
    jntVelMax.setConstant(2.0);
    jnt_vel_cstr.setName("JointVelocityLimit");
    jnt_vel_cstr.setRobotModel(robot);
    jnt_vel_cstr.setProblem(problem);
    jnt_vel_cstr.setLimits(-jntVelMax,jntVelMax);  // because not read in the URDF for now
    jnt_vel_cstr.update();

    std::cout << "==== Regularisation" << '\n';

    RegularisationTask<ControlVariable::X> reg_task; // whole vector
    reg_task.setName("reg_task");
    reg_task.setRobotModel(robot);
    reg_task.setProblem(problem);
    reg_task.EuclidianNorm().setWeight(1E-3);
    reg_task.update();

    problem->setQPSolver( QPSolver::qpOASES );
    problem->print();

    return 0;
}
