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
using namespace orca::all;

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

    // The good stuff...

    MinJerkPositionTrajectory traj(5.0);
    int traj_loops = 0;
    bool exit_control_loop = true;
    Eigen::Vector3d start_position, end_position;


    cart_task->onActivationCallback([](){
        std::cout << "Activating CartesianTask..." << '\n';
    });

    cart_task->onActivatedCallback([&](){
        start_position = cart_task->servoController()->getCurrentCartesianPose().block(0,3,3,1);
        end_position = cart_pos_ref.translation();
        traj.resetTrajectory(start_position, end_position);
        std::cout << "CartesianTask activated. Begining trajectory." << '\n';
    });

    cart_task->onComputeBeginCallback([&](double current_time, double dt){
        Eigen::Vector3d p, v, a;
        traj.getDesired(current_time, p, v, a);
        cart_pos_ref.translation() = p;
        cart_vel_ref.head(3) = v;
        cart_acc_ref.head(3) = a;
        cart_task->servoController()->setDesired(cart_pos_ref.matrix(),cart_vel_ref,cart_acc_ref);
    });

    cart_task->onComputeEndCallback([&](double current_time, double dt){
        if (traj.isTrajectoryFinished()  )
        {
            if (traj_loops < 4)
            {
                traj.resetTrajectory(end_position, start_position);
                std::cout << "Changing trajectory direction." << '\n';
                ++traj_loops;
            }
            else
            {
                std::cout << "Trajectory looping finished." << '\n';
                exit_control_loop = true;
            }
        }
    });

    cart_task->onDeactivationCallback([](){
        std::cout << "Deactivating task." << '\n';
    });

    cart_task->onDeactivatedCallback([](){
        std::cout << "CartesianTask deactivated. Stopping controller" << '\n';
    });

    controller.activateTasksAndConstraints();

    // Control loop
    while(traj_loops < 4)
    {
        controller.update(current_time, dt);
        current_time +=dt;
    }
    std::cout << "Out of control loop." << '\n';

    controller.deactivateTasksAndConstraints();


    while(!controller.tasksAndConstraintsDeactivated())
    {
        controller.update(current_time, dt);
        current_time += dt;
    }
    return 0;
}
