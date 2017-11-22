#include <orca/orca.h>
using namespace orca;
using namespace orca::common;
using namespace orca::optim;
using namespace orca::task;
using namespace orca::constraint;
using namespace orca::robot;
using namespace orca::math;
using namespace orca::util;

int main(int argc, char** argv)
{
    orca::util::Logger::setLogLevel( orca::util::LogLevel::debug  );

    if(argc != 2)
    {
        std::cerr << "Usage : ./orca-demo1 /path/to/robot-urdf.urdf" << "\n";
        return -1;
    }

    PosixTimer timer(false);

    std::string urdf_url(argv[1]);

    std::cout << "===== Robot" << '\n';

    auto robot = std::make_shared<RobotDynTree>();
    if(!robot->loadModelFromFile(urdf_url))
    {
        return -1;
    }
    robot->setBaseFrame("link_0");
    robot->setGravity(Eigen::Vector3d(0,0,-9.81));

    EigenRobotState state;
    state.setFixedBase();
    state.resize(robot->getNrOfDegreesOfFreedom());

    robot->setRobotState(state.jointPos,state.jointVel);

    std::cout << "===== Cartesian Task" << '\n';

    CartesianTask cart_task;
    cart_task.setRobotModel(robot);
    cart_task.setControlFrame("link_7");

    Eigen::Affine3d cart_pos_ref;
    cart_pos_ref = Eigen::Translation3d(1.,0.75,0.5);
    cart_pos_ref.translation() = Eigen::Vector3d(1.,0.75,0.5);
    Eigen::Quaterniond quat;
    quat = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
    cart_pos_ref.linear() = quat.toRotationMatrix();
    Eigen::Matrix3d rot;
    rot = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
    cart_pos_ref.linear() = rot;

    Vector6d cart_vel_ref;
    cart_vel_ref.setZero();
    Vector6d cart_acc_ref;
    cart_acc_ref.setZero();

    std::cout << "===== Cartesian Acceleration PID" << '\n';

    CartesianAccelerationPID cart_acc_pid;
    cart_acc_pid.setRobotModel(robot);
    Vector6d P_gain;
    P_gain << 100, 100, 100, 10, 10, 10;

    Vector6d D_gain;
    D_gain << 10, 10, 10, 1, 1, 1;

    cart_acc_pid.pid().setProportionalGain(P_gain);
    cart_acc_pid.pid().setDerivativeGain(D_gain);
    cart_acc_pid.setControlFrame(cart_task.getControlFrame());
    cart_acc_pid.setDesired(cart_pos_ref.matrix(),cart_vel_ref,cart_acc_ref);
    cart_acc_pid.update();

    cart_task.setDesired(cart_acc_pid.getCommand());
    std::cout << "===== Cartesian update()" << '\n';
    cart_task.update();

    std::cout << "===== Contacts" << '\n';

    Contact contact1;
    Contact contact2;
    Contact contact3;
    Contact contact4;

    contact1.setRobotModel(robot);
    contact2.setRobotModel(robot);
    contact3.setRobotModel(robot);
    contact4.setRobotModel(robot);

    contact1.setControlFrame("link_7");
    contact2.setControlFrame("link_2");
    contact3.setControlFrame("link_6");
    contact4.setControlFrame("link_5");

    contact1.update();
    contact2.update();
    contact3.update();
    contact4.update();

    std::cout << "===== Dynamics Equation Constraint" << '\n';
    DynamicsEquationConstraint dynConstr;

    dynConstr.setRobotModel(robot);
    dynConstr.update();

    std::cout << "===== Regularisation" << '\n';

    const int ndof = robot->getNrOfDegreesOfFreedom();

    JointTorqueLimitConstraint jnt_trq_cstr;
    Eigen::VectorXd jntTrqMax;
    jntTrqMax.resize(ndof);
    jntTrqMax.setConstant(200.0);
    jnt_trq_cstr.setRobotModel(robot);
    jnt_trq_cstr.setLimits(-jntTrqMax,jntTrqMax);
    jnt_trq_cstr.update();

    JointPositionLimitConstraint jnt_pos_cstr;
    jnt_pos_cstr.setRobotModel(robot);
    jnt_pos_cstr.update();

    JointVelocityLimitConstraint jnt_vel_cstr;
    Eigen::VectorXd jntVelMax;
    jntVelMax.resize(ndof);
    jntVelMax.setConstant(2.0);
    jnt_vel_cstr.setRobotModel(robot);
    jnt_vel_cstr.setLimits(-jntVelMax,jntVelMax);
    jnt_vel_cstr.update();

    JointAccelerationLimitConstraint jnt_acc_cstr;
    Eigen::VectorXd jntAccMax;
    jntAccMax.resize(ndof);
    jntAccMax.setConstant(4.0);
    jnt_acc_cstr.setRobotModel(robot);
    jnt_acc_cstr.setLimits(-jntAccMax,jntAccMax);
    jnt_acc_cstr.update();


    AccelerationRegularisationTask acc_reg_task;
    RegularisationTask<ControlVariable::X> reg_task;
    TorqueRegularisationTask trq_reg_task;
    WrenchRegularisationTask wrench_reg_task1;
    WrenchRegularisationTask wrench_reg_task2;
    WrenchRegularisationTask wrench_reg_task3;
    WrenchRegularisationTask wrench_reg_task4;

    reg_task.setRobotModel(robot);
    acc_reg_task.setRobotModel(robot);
    trq_reg_task.setRobotModel(robot);
    wrench_reg_task1.setRobotModel(robot);
    wrench_reg_task2.setRobotModel(robot);
    wrench_reg_task3.setRobotModel(robot);
    wrench_reg_task4.setRobotModel(robot);

    reg_task.EuclidianNorm().setWeight(1E-6);
    acc_reg_task.EuclidianNorm().setWeight(1E-6);
    trq_reg_task.EuclidianNorm().setWeight(1E-6);
    wrench_reg_task1.EuclidianNorm().setWeight(1E-6);
    wrench_reg_task2.EuclidianNorm().setWeight(1E-6);
    wrench_reg_task3.EuclidianNorm().setWeight(1E-6);
    wrench_reg_task4.EuclidianNorm().setWeight(1E-6);

    reg_task.update();
    reg_task.insertInProblem();
    acc_reg_task.update();
    trq_reg_task.update();
    wrench_reg_task1.update();
    wrench_reg_task2.update();
    wrench_reg_task3.update();
    wrench_reg_task4.update();

    std::cout << "===== Qp Solver" << '\n';

    WeightedQPSolver qp;
    qp.setPrintLevel(0);

    timer.Reset();
    qp.buildOptimisationProblem();
    std::cout << "qp.buildOptimisationProblem() time: " << std::fixed << timer.Elapsed().count()/1E6 << "ms\n";

    timer.Reset();
    bool success = (qp.solve() == 0);
    std::cout << "qp.solve() time: " << std::fixed << timer.Elapsed().count()/1E6 << "ms\n";
    if(!success)
    {
        std::cout << "FAILED TO SOLVE " << "\n";
        return -2;
    }
    timer.Reset();
    int ntrials=1000;
    int i=ntrials;
    do
    {
        cart_task.insertInProblem();
        acc_reg_task.insertInProblem();
        trq_reg_task.insertInProblem();
        wrench_reg_task1.insertInProblem();
        wrench_reg_task2.insertInProblem();
        wrench_reg_task3.insertInProblem();
        wrench_reg_task4.insertInProblem();

        dynConstr.insertInProblem();
        jnt_trq_cstr.insertInProblem();
        jnt_pos_cstr.insertInProblem();
        jnt_vel_cstr.insertInProblem();
        jnt_acc_cstr.insertInProblem();

        contact1.insertInProblem();
        contact2.insertInProblem();
        contact3.insertInProblem();
        contact4.insertInProblem();
        
        dynConstr.update();

        qp.buildOptimisationProblem();
        qp.solve();
    }while(i--);

    qp.print();

    std::cout << "qp.buildOptimisationProblem() + qp.solve()+ time " << std::fixed << timer.Elapsed().count()/1E6/ntrials << "ms\n";

    if(success)
    {
        std::cout << "Solve succeded : \n" << qp.getPrimalSolution() << '\n';
    }
    else
    {
        std::cerr << "Solve did not succeded" << '\n';
    }
    std::cout << "Done" << "\n";
    return 0;
}
