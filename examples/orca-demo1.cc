#include <orca/orca.h> //this implicitly creates an optimization problem
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

    orca::util::Logger::parseArgv( argc, argv );

    if(argc < 2)
    {
        std::cerr << "Usage : ./orca-demosimple /path/to/robot-urdf.urdf" << "\n";
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
    state.setFixedBase(); // sets world to base to identity and base velocity to zero
    state.resize(robot->getNrOfDegreesOfFreedom());

    state.jointPos.setZero();
    state.jointVel.setZero();
    robot->setRobotState(state.jointPos,state.jointVel);

    std::cout << "===== Cartesian Task creation" << '\n';

    CartesianTask cart_task;
    cart_task.setName("CartTask-link_7");
    cart_task.setRobotModel(robot); // Create and Resize the models associated to the task
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
    cart_acc_pid.setName("CartPID-7");
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
    cart_task.update(); //The state of the task is now "initialized" (computes for the first time the associated cost). It can now be used.

    std::cout << "===== Contacts" << '\n';

    Contact contact1;
    Contact contact2;
    Contact contact3;
    Contact contact4;

    contact1.setName("Contact1");
    contact2.setName("Contact2");
    contact3.setName("Contact3");
    contact4.setName("Contact4");

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

    dynConstr.setName("Dynamics Equation");
    dynConstr.setRobotModel(robot);
    dynConstr.update();

    std::cout << "===== Joint limits" << '\n';

    const int ndof = robot->getNrOfDegreesOfFreedom();

    JointTorqueLimitConstraint jnt_trq_cstr;
    jnt_trq_cstr.setName("JointTorqueLimit");
    Eigen::VectorXd jntTrqMax;
    jntTrqMax.resize(ndof);
    jntTrqMax.setConstant(200.0);
    jntTrqMax<<200,200,200,200,120,120,120;
    jnt_trq_cstr.setRobotModel(robot);
    jnt_trq_cstr.setLimits(-jntTrqMax,jntTrqMax); // because not red in the URDF
    jnt_trq_cstr.update();

    JointPositionLimitConstraint jnt_pos_cstr;
    jnt_pos_cstr.setName("JointPositionLimit");
    jnt_pos_cstr.setRobotModel(robot);  // red in the URDF so no need to set
    jnt_pos_cstr.setJointLimitsFromRobotModel(); //optional, see previous comment
    jnt_pos_cstr.update();

    JointVelocityLimitConstraint jnt_vel_cstr;
    Eigen::VectorXd jntVelMax;
    jntVelMax.resize(ndof);
    jntVelMax.setConstant(2.0);
    jnt_vel_cstr.setName("JointVelocityLimit");
    jnt_vel_cstr.setRobotModel(robot);
    jnt_vel_cstr.setLimits(-jntVelMax,jntVelMax);
    jnt_vel_cstr.update();

    // There is no such thing as an acceleration constraint...

    JointAccelerationLimitConstraint jnt_acc_cstr;
    Eigen::VectorXd jntAccMax;
    jntAccMax.resize(ndof);
    jntAccMax.setConstant(4.0);
    jnt_acc_cstr.setName("JointAccelerationLimit");
    jnt_acc_cstr.setRobotModel(robot);
    jnt_acc_cstr.setLimits(-jntAccMax,jntAccMax);
    jnt_acc_cstr.update();

    std::cout << "==== Regularisation" << '\n';

    RegularisationTask<ControlVariable::X> reg_task; // whole vector

    AccelerationRegularisationTask acc_reg_task; // Only on joint acc
    //RegularisationTask<ControlVariable::GeneralisedJointAcceleration> acc_reg_SAME_task; // equivalent to above task
    TorqueRegularisationTask trq_reg_task;
    WrenchRegularisationTask wrench_reg_task1;
    WrenchRegularisationTask wrench_reg_task2;
    WrenchRegularisationTask wrench_reg_task3;
    WrenchRegularisationTask wrench_reg_task4;

    reg_task.setName("reg_task");
    acc_reg_task.setName("acc_reg_task");
    trq_reg_task.setName("trq_reg_task");
    wrench_reg_task1.setName("wrench_reg_task1");
    wrench_reg_task2.setName("wrench_reg_task2");
    wrench_reg_task3.setName("wrench_reg_task3");
    wrench_reg_task4.setName("wrench_reg_task4");

    reg_task.setRobotModel(robot);
    acc_reg_task.setRobotModel(robot);
    trq_reg_task.setRobotModel(robot);
    wrench_reg_task1.setRobotModel(robot);
    wrench_reg_task2.setRobotModel(robot);
    wrench_reg_task3.setRobotModel(robot);
    wrench_reg_task4.setRobotModel(robot);

    reg_task.EuclidianNorm().setWeight(1E-3);
    acc_reg_task.EuclidianNorm().setWeight(1E-3);
    trq_reg_task.EuclidianNorm().setWeight(1E-3);
    wrench_reg_task1.EuclidianNorm().setWeight(1E-3);
    wrench_reg_task2.EuclidianNorm().setWeight(1E-3);
    wrench_reg_task3.EuclidianNorm().setWeight(1E-3);
    wrench_reg_task4.EuclidianNorm().setWeight(1E-3);

    reg_task.update();
    acc_reg_task.update();
    trq_reg_task.update();
    wrench_reg_task1.update();
    wrench_reg_task2.update();
    wrench_reg_task3.update();
    wrench_reg_task4.update();

    LOG_INFO << "===== Inserting Tasks" << '\n';
    // Insert Tasks/Constraints in Problem
    // Insertion is useful to avoid dynamic allocations due to potentially appearing and disappearing tasks and Constraints
    // Insertion may resize the optim variable and the QP so insertion is time consumming and should be performed only rarely
    // Ideally only at the problem creation time. Inserted tasks can also be removed. This may also imply some resizing.
    // The task and constraints are actually integrated in the QP when their internal state is switched to "activated"
    // Activation does not induce any resizing so is to be preferred while in the loop
    reg_task.insertInProblem(); // This so called "problem" is the one implicitly created when including the header ocra.h
    cart_task.insertInProblem();
    reg_task.insertInProblem();

    dynConstr.insertInProblem();
    jnt_trq_cstr.insertInProblem();
    jnt_pos_cstr.insertInProblem();
    jnt_vel_cstr.insertInProblem();
    jnt_acc_cstr.insertInProblem();
    contact1.insertInProblem();
    contact2.insertInProblem();
    contact3.insertInProblem();
    contact4.insertInProblem();

    std::cout<<"===== Activating tasks and constraints"<<'\n';

    cart_task.activate();
    reg_task.activate();

    dynConstr.activate();
    jnt_trq_cstr.activate();
    jnt_pos_cstr.activate();
    jnt_vel_cstr.activate();
    jnt_acc_cstr.activate();

    // Deactivated by default, just to make clear those four are
    contact1.desactivate();
    contact2.desactivate();
    contact3.desactivate();
    contact4.desactivate();

    std::cout << "===== Qp Solver" << '\n';

    WeightedQPSolver qp;
    qp.resize(); //allocation based on the created optimization problem
    qp.setPrintLevel(0);

    timer.Reset();
    qp.buildOptimisationProblem(); //reading the problem and filling in the matrices
    std::cout << "qp.buildOptimisationProblem() time: " << std::fixed << timer.Elapsed().count()/1E6 << "ms\n";

    timer.Reset();
    bool success = (qp.solve() == 0); // solving the problem
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


        dynConstr.update();

        qp.buildOptimisationProblem();
        qp.solve();
    }while(i--);

    qp.print();

    std::cout << "qp.buildOptimisationProblem() + qp.solve()+ time " << std::fixed << timer.Elapsed().count()/1E6/ntrials << "ms\n";

    if(success)
    {
        std::cout << "Solve succeded : \n" << qp.getPrimalSolution() << '\n'; //Get the solution
    }
    else
    {
        std::cerr << "Solve did not succeded" << '\n';
    }
    std::cout << "Done" << "\n";
    return 0;
}
