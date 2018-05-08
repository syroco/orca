#include <orca/orca.h>
using namespace orca;
using namespace orca::common;
using namespace orca::optim;
using namespace orca::task;
using namespace orca::constraint;
using namespace orca::robot;
using namespace orca::math;
using namespace orca::utils;

int main(int argc, char const *argv[])
{
    // Parse urdf and logger level as --log_level (or -l) debug/warning etc
    orca::utils::Logger::parseArgv( argc, argv );

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
    eigState.setFixedBaseValues(); // sets world to base to identity and base velocity to zero
    eigState.resize(robot->getNrOfDegreesOfFreedom()); // resize all the vectors/matrices to match the robot configuration
    // Set the initial state to zero (arbitrary)
    // NOTE : here we only set q,qot because this example asserts we have a fixed base robot
    eigState.jointPos.setZero();
    eigState.jointVel.setZero();
    // Set the first state to the robot
    robot->setRobotState(eigState.jointPos,eigState.jointVel); // Now is the robot is considered 'initialized'
    robot->isInitialized(); // --> returns true

    // Instanciate and ORCA Controller
    orca::optim::Controller controller(
        robot
        ,orca::optim::ResolutionStrategy::OneLevelWeighted // MultiLevelWeighted, Generalized
        ,QPSolver::qpOASES
    );


    auto cart_task = std::make_shared<CartesianTask>("CartTask-EE");
    cart_task->setControlFrame("link_7"); // We want to control the link_7

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

    auto cart_acc_pid = std::make_shared<CartesianAccelerationPID>("CartPID-EE");
    Vector6d P;
    P << 100, 100, 100, 10, 10, 10;
    cart_acc_pid->pid().setProportionalGain(P);
    Vector6d D;
    D << 0, 10, 10, 1, 1, 1;
    cart_acc_pid->pid().setDerivativeGain(D);
    cart_acc_pid->setDesired(cart_pos_ref.matrix(),cart_vel_ref,cart_acc_ref);

    cart_task->setServoController(cart_acc_pid);
    controller.addTask(cart_task);

    const int ndof = robot->getNrOfDegreesOfFreedom();

    auto jnt_trq_cstr = std::make_shared<JointTorqueLimitConstraint>("JointTorqueLimit");
    controller.addConstraint(jnt_trq_cstr);
    Eigen::VectorXd jntTrqMax(ndof);
    jntTrqMax.setConstant(200.0);
    jnt_trq_cstr->setLimits(-jntTrqMax,jntTrqMax); // because not read in the URDF for now

    auto jnt_pos_cstr = std::make_shared<JointPositionLimitConstraint>("JointPositionLimit");
    controller.addConstraint(jnt_pos_cstr);

    auto jnt_vel_cstr = std::make_shared<JointVelocityLimitConstraint>("JointVelocityLimit");
    controller.addConstraint(jnt_vel_cstr);
    Eigen::VectorXd jntVelMax(ndof);
    jntVelMax.setConstant(2.0);
    jnt_vel_cstr->setLimits(-jntVelMax,jntVelMax);  // because not read in the URDF for now

    controller.update(0,0.001);

    const Eigen::VectorXd& full_solution = controller.getFullSolution();
    const Eigen::VectorXd& trq_cmd = controller.getJointTorqueCommand();
    const Eigen::VectorXd& trq_acc = controller.getJointAccelerationCommand();

    LOG_INFO << "Full solution : " << full_solution.transpose();
    LOG_INFO << "Joint Acceleration command : " << trq_acc.transpose();
    LOG_INFO << "Joint Torque command       : " << trq_cmd.transpose();

    return 0;
}
