// This file is a part of the ORCA framework.
// Copyright 2017, ISIR / Universite Pierre et Marie Curie (UPMC)
// Copyright 2018, Fuzzy Logic Robotics
// Main contributor(s): Antoine Hoarau, Ryan Lober, and
// Fuzzy Logic Robotics <info@fuzzylogicrobotics.com>
//
// ORCA is a whole-body reactive controller framework for robotics.
//
// This software is governed by the CeCILL-C license under French law and
// abiding by the rules of distribution of free software.  You can  use,
// modify and/ or redistribute the software under the terms of the CeCILL-C
// license as circulated by CEA, CNRS and INRIA at the following URL
// "http://www.cecill.info".
//
// As a counterpart to the access to the source code and  rights to copy,
// modify and redistribute granted by the license, users are provided only
// with a limited warranty  and the software's author,  the holder of the
// economic rights,  and the successive licensors  have only  limited
// liability.
//
// In this respect, the user's attention is drawn to the risks associated
// with loading,  using,  modifying and/or developing or reproducing the
// software by the user in light of its specific status of free software,
// that may mean  that it is complicated to manipulate,  and  that  also
// therefore means  that it is reserved for developers  and  experienced
// professionals having in-depth computer knowledge. Users are therefore
// encouraged to load and test the software's suitability as regards their
// requirements in conditions enabling the security of their systems and/or
// data to be ensured and,  more generally, to use and operate it in the
// same conditions as regards security.
//
// The fact that you are presently reading this means that you have had
// knowledge of the CeCILL-C license and that you accept its terms.

/** @file
 @copyright 2018 Fuzzy Logic Robotics <info@fuzzylogicrobotics.com>
 @author Antoine Hoarau
 @author Ryan Lober
*/

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

    auto robot_model = std::make_shared<RobotModel>();
    robot_model->loadModelFromFile(urdf_url);
    robot_model->setBaseFrame("base_link");
    robot_model->setGravity(Eigen::Vector3d(0,0,-9.81));
    RobotState eigState;
    eigState.resize(robot_model->getNrOfDegreesOfFreedom());
    eigState.jointPos.setZero();
    eigState.jointVel.setZero();
    robot_model->setRobotState(eigState.jointPos,eigState.jointVel);

    orca::optim::Controller controller(
        "controller"
        ,robot_model
        ,orca::optim::ResolutionStrategy::OneLevelWeighted
        ,QPSolverImplType::qpOASES
    );

    auto cart_acc_pid = std::make_shared<CartesianAccelerationPID>("servo_controller");
    Vector6d P;
    P << 1000, 1000, 1000, 10, 10, 10;
    cart_acc_pid->pid()->setProportionalGain(P);
    Vector6d D;
    D << 100, 100, 100, 1, 1, 1;
    cart_acc_pid->pid()->setDerivativeGain(D);
    cart_acc_pid->setControlFrame("link_7");
    Eigen::Affine3d cart_pos_ref;
    cart_pos_ref.translation() = Eigen::Vector3d(0.3,-0.5,0.41); // x,y,z in meters
    cart_pos_ref.linear() = orca::math::quatFromRPY(M_PI,0,0).toRotationMatrix();
    Vector6d cart_vel_ref = Vector6d::Zero();
    Vector6d cart_acc_ref = Vector6d::Zero();
    cart_acc_pid->setDesired(cart_pos_ref.matrix(),cart_vel_ref,cart_acc_ref);

    auto cart_task = controller.addTask<CartesianTask>("CartTask_EE");
    cart_task->setServoController(cart_acc_pid);

    const int ndof = robot_model->getNrOfDegreesOfFreedom();

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
