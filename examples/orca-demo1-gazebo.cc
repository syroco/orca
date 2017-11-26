#include <orca/orca.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo_client.hh>
#include <sdf/parser_urdf.hh>
#include <fstream>
#include <thread>

using namespace orca::task;
using namespace orca::constraint;
using namespace orca::robot;
using namespace orca::optim;
using namespace orca::math;
using namespace orca::util;
using namespace orca::common;

int main(int argc, char** argv)
{
    orca::util::Logger::setLogLevel( orca::util::LogLevel::debug  );

    if(argc < 2)
    {
        LOG_ERROR << "Usage : ./orca-demo1-gazebo /path/to/lwr.urdf" << "\n";
        LOG_ERROR << "To make it work you need first to set the path to the meshes : (ex path to lwr_description)" << '\n';
        LOG_ERROR << "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/path/to/the/package/containing_meshes" << "\n";
        return -1;
    }

    std::string urdf_url(argv[1]);

    gazebo::printVersion();
    gazebo::setupServer({"--verbose"});
    gazebo::physics::WorldPtr world = gazebo::loadWorld("worlds/empty.world");
    const double sim_step_dt = world->Physics()->GetMaxStepSize();
    std::cout << "sim_step_dt " << sim_step_dt << '\n';

    std::cout << "Inserting model file " << urdf_url << '\n';


    LOG_INFO << "===== URDF To SDF " << '\n';
    sdf::URDF2SDF urdf2sdf;
    TiXmlDocument doc = urdf2sdf.InitModelFile(urdf_url);

    TiXmlPrinter printer;
    printer.SetIndent( "    " );
    doc.Accept( &printer );
    std::string xmltext = printer.CStr();

    world->InsertModelString(xmltext);

    LOG_INFO << "To make it work you need first to set the path to the meshes : (ex path to lwr_description)" << '\n';
    LOG_INFO << "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/path/to/the/package/containing_meshes" << "\n";
    LOG_INFO << "----> otherwise Gazebo won't start !" << "\n";

    for (size_t i = 0; i < 10; i++)
    {
        gazebo::runWorld(world,1);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if(world->ModelCount() == 2)
        {
            break;
        }
    }


    std::cout << "Gazebo contains " << world->ModelCount() << " models" << '\n';
    for(auto m : world->Models())
    {
        std::cout << "  - " << m->GetName() << '\n';
    }

    if(world->ModelCount() != 2)
    {
        std::cerr << "There should be only 2 models, we have " << world->ModelCount() << std::endl;
        return -1;
    }

    gazebo::physics::ModelPtr gz_model = world->ModelByIndex(1);
    std::cout << "Gazebo getting model " << gz_model->GetName() << '\n';
    std::cout << "All Joints : " << gz_model->GetName() << '\n';
    for(auto j : gz_model->GetJoints())
    {
        std::cout << "   - " << j->GetName() << " type : " << j->TypeStr() << " lowerLimit : " << j->GetLowerLimit(0u) << " upperLimit : " << j->GetUpperLimit(0u) <<'\n';
    }

    PosixTimer timer(false);

    LOG_INFO << "===== Robot" << '\n';
    std::shared_ptr<RobotDynTree> robot(new RobotDynTree(urdf_url));
    robot->setBaseFrame("link_0");

    // Build the same with Gazebo joints
    std::vector<std::string> joint_idx;
    for(int i=0 ; i < robot->getNrOfJoints() ; ++i)
    {
        auto joint = gz_model->GetJoint( robot->getJointName(i) );

        if(joint)
        {
            std::cout << "iDynTree adding joint " << i << " name " << robot->getJointName(i) << '\n';
            joint_idx.push_back(robot->getJointName(i) );
        }
        else
        {
            std::cout << "Not adding joint " << robot->getJointName(i) << '\n';
        }

    }

    if(joint_idx.size() != robot->getNrOfDegreesOfFreedom())
    {
        std::cout << "Could not add the " << robot->getNrOfDegreesOfFreedom() << " joints, only " << joint_idx.size() << '\n';
        return -1;
    }

    Eigen::Vector3d gravity_vector;

    gravity_vector[0] = world->Gravity().X();
    gravity_vector[1] = world->Gravity().Y();
    gravity_vector[2] = world->Gravity().Z();

    robot->setGravity(gravity_vector);

    EigenRobotState robot_state;
    robot_state.setFixedBase();
    robot_state.resize(robot->getNrOfDegreesOfFreedom());
    Eigen::VectorXd jointTrq;
    jointTrq.setZero(robot->getNrOfDegreesOfFreedom());

    robot->setRobotState(robot_state.jointPos,robot_state.jointVel);

    LOG_INFO << "===== Cartesian Acceleration PID " << '\n';

    CartesianAccelerationPID cart_acc_pid;
    cart_acc_pid.setRobotModel(robot);

    // Build Set desired command
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

    // set PD gains
    Vector6d P_gain;
    P_gain << 1000, 1000, 1000, 10, 10, 10;

    Vector6d D_gain;
    D_gain << 10, 10, 10, 1, 1, 1;

    cart_acc_pid.pid().setProportionalGain(P_gain);
    cart_acc_pid.pid().setDerivativeGain(D_gain);
    cart_acc_pid.setControlFrame("link_7");
    cart_acc_pid.setDesired(cart_pos_ref.matrix(),cart_vel_ref,cart_acc_ref);
    cart_acc_pid.update();

    LOG_INFO << "===== Cartesian Task " << '\n';

    CartesianTask cart_task;
    cart_task.setRobotModel(robot);
    cart_task.setControlFrame(cart_acc_pid.getControlFrame());
    cart_task.setDesired(cart_acc_pid.getCommand());
    cart_task.update();

    LOG_INFO << "===== Contacts" << '\n';

    Contact contact1;
    Contact contact2;
    Contact contact3;
    Contact contact4;

    contact1.setRobotModel(robot);
    contact2.setRobotModel(robot);
    contact3.setRobotModel(robot);
    contact4.setRobotModel(robot);

    contact1.setControlFrame("link_7");
    contact2.setControlFrame("link_1");
    contact3.setControlFrame("link_6");
    contact4.setControlFrame("link_5");

    contact1.update();
    contact2.update();
    contact3.update();
    contact4.update();

    LOG_INFO << "===== Dynamics Equation Constraint" << '\n';
    DynamicsEquationConstraint dynConstr;
    dynConstr.setRobotModel(robot);

    dynConstr.update();

    LOG_INFO << "===== Regularisation" << '\n';

    const int ndof = robot->getNrOfDegreesOfFreedom();


    const int n_horizon_steps = 15;
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
    jntVelMax.setConstant(20.0);
    jnt_vel_cstr.setRobotModel(robot);
    jnt_vel_cstr.setLimits(-jntVelMax,jntVelMax);
    jnt_vel_cstr.update();

    JointAccelerationLimitConstraint jnt_acc_cstr;
    Eigen::VectorXd jntAccMax;
    jntAccMax.resize(ndof);
    jntAccMax.setConstant(400.0);
    jnt_acc_cstr.setRobotModel(robot);
    jnt_acc_cstr.setLimits(-jntAccMax,jntAccMax);
    jnt_acc_cstr.update();


    RegularisationTask<ControlVariable::X> reg_task;
    reg_task.setRobotModel(robot);
    reg_task.EuclidianNorm().setWeight(1E-4);
    reg_task.update();


    LOG_INFO << "===== Inserting Tasks" << '\n';
    // Insert Tasks/Constraints in Problem
    cart_task.insertInProblem();
    reg_task.insertInProblem();

    dynConstr.insertInProblem();
    jnt_trq_cstr.insertInProblem();
    jnt_pos_cstr.insertInProblem();
    jnt_vel_cstr.insertInProblem();
    jnt_acc_cstr.insertInProblem();
    
    cart_task.activate();
    reg_task.activate();

    dynConstr.activate();
    jnt_trq_cstr.activate();
    jnt_pos_cstr.activate();
    jnt_vel_cstr.activate();
    jnt_acc_cstr.activate();

    reg_task.print();

    // contact1.insertInProblem();
    // contact2.insertInProblem();
    // contact3.insertInProblem();
    // contact4.insertInProblem();

    LOG_INFO << "===== Qp Solver" << '\n';

    WeightedQPSolver qp;
    timer.Reset();
    qp.buildOptimisationProblem();
    std::cout << "qp.buildOptimisationProblem() time: " << std::fixed << timer.Elapsed().count()/1E6 << "ms\n";
    qp.print();
    qp.setPrintLevel(0);

    timer.Reset();
    bool success = (qp.solve() == 0);
    std::cout << "qp.solve() time: " << std::fixed << timer.Elapsed().count()/1E6 << "ms\n";
    if(success)
    {
        std::cout << "Solve succeded : \n" << qp.getPrimalSolution() << '\n';
    }
    else
    {
        std::cerr << "Solve did not succeded" << '\n';
        return -1;
    }

    timer.Reset();

    LOG_WARNING << "Gazebo in Pause. Open the GUI in an other terminal : gzclient --verbose and unpause the simulation" << '\n';
    LOG_WARNING << "Or enter `gz world --pause false`" << '\n';
    gazebo::event::Events::pause.Signal(true);
    int i=0;
    while(i++ <= 100000)
    {
        //std::cout << "Gazebo step ... " << std::endl;
        gazebo::runWorld(world,1);
        //std::cout << "Gazebo step done. Getting robot state : " << std::endl;

        for(int i=0 ; i < joint_idx.size() ; ++i)
        {
            auto joint = gz_model->GetJoint(joint_idx[i]);

            robot_state.jointPos[i] = joint->Position(0);
            robot_state.jointVel[i] = joint->GetVelocity(0);
            jointTrq[i] = joint->GetForce(0u);
        }

        // std::cout << "robot_state.jointPos  " << robot_state.jointPos.transpose() << std::endl;
        // std::cout << "robot_state.jointVel  " << robot_state.jointVel.transpose() << std::endl;
        // std::cout << "            jointTrq  " << jointTrq.transpose() << std::endl;

        robot->setRobotState(robot_state.jointPos,robot_state.jointVel);
        cart_acc_pid.update();
        cart_task.setDesired(cart_acc_pid.getCommand());
        cart_task.update();
        reg_task.update();

        dynConstr.update();
        jnt_trq_cstr.update();
        jnt_pos_cstr.setHorizon( n_horizon_steps * sim_step_dt);
        jnt_vel_cstr.setHorizon( n_horizon_steps * sim_step_dt);
        jnt_pos_cstr.update();
        jnt_vel_cstr.update();
        jnt_acc_cstr.update();

        contact1.update();
        contact2.update();
        contact3.update();
        contact4.update();

        if(i == 2000)
        {
            LOG_WARNING << "Inserting Contact 1";
            contact1.insertInProblem();
            qp.buildOptimisationProblem();
            qp.print();
        }


        qp.buildOptimisationProblem();
        bool solution_found = qp.solve() == 0;

        const Eigen::VectorXd& torque = qp.getPrimalSolution().segment(
                 OptimisationVector().getIndex(ControlVariable::JointSpaceTorque)
                ,OptimisationVector().getSize(ControlVariable::JointSpaceTorque));

        std::cout << i << " qp.solve() torque :  " << torque.transpose() << std::endl;

        for(int i=0 ; i < joint_idx.size() ; ++i)
        {
            auto joint = gz_model->GetJoint(joint_idx[i]);
            joint->SetForce(0,torque[i]);
        }

        if(!solution_found)
        {
            LOG_ERROR << "No Solution found";
            break;
        }
    }

    gazebo::shutdown();
    return 0;
}
