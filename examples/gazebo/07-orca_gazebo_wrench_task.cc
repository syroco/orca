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
#include <orca/gazebo/GazeboServer.h>
#include <orca/gazebo/GazeboModel.h>

using namespace orca::all;
using namespace orca::gazebo;



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

    orca::optim::Controller controller(
        "controller"
        ,robot_model
        ,orca::optim::ResolutionStrategy::OneLevelWeighted
        ,QPSolverImplType::qpOASES
    );

    
    auto cart_acc_pid = std::make_shared<CartesianAccelerationPID>("servo_controller");

    cart_acc_pid->pid()->setProportionalGain( { 1000, 1000, 1000, 10, 10, 10 });
    cart_acc_pid->pid()->setDerivativeGain( { 100, 100, 100, 1, 1, 1 });
    
    cart_acc_pid->setControlFrame("link_7");
    // Lets comment the following lines to force the robot to initialise at current pose
    // Eigen::Affine3d cart_pos_ref;
    // cart_pos_ref.translation() = Eigen::Vector3d(0.3,-0.5,0.41); // x,y,z in meters
    // cart_pos_ref.linear() = orca::math::quatFromRPY(M_PI,0,0).toRotationMatrix();
    // Vector6d cart_vel_ref = Vector6d::Zero();
    // Vector6d cart_acc_ref = Vector6d::Zero();
    // cart_acc_pid->setDesired(cart_pos_ref.matrix(),cart_vel_ref,cart_acc_ref);
    
    auto cart_task = controller.addTask<CartesianTask>("CartTask_EE");
    cart_task->setServoController(cart_acc_pid);
    cart_task->setWeight(0.1);
    
    
    auto wrench_task = controller.addTask<WrenchTask>("WrenchTask_Demo");
    wrench_task->setControlFrame("link_7");
    wrench_task->pid()->setProportionalGain({10, 10, 10, 10, 10, 10});
    wrench_task->setDesired({0,0,-10,0,0,0});

    const int ndof = robot_model->getNrOfDegreesOfFreedom();

    auto jnt_trq_cstr = controller.addConstraint<JointTorqueLimitConstraint>("JointTorqueLimit");
    Eigen::VectorXd jntTrqMax(ndof);
    jntTrqMax.setConstant(200.0);
    jnt_trq_cstr->setLimits(-jntTrqMax,jntTrqMax);

    auto jnt_pos_cstr = controller.addConstraint<JointPositionLimitConstraint>("JointPositionLimit");

    auto jnt_vel_cstr = controller.addConstraint<JointVelocityLimitConstraint>("JointVelocityLimit");
    Eigen::VectorXd jntVelMax(ndof);
    jntVelMax.setConstant(2.0);
    jnt_vel_cstr->setLimits(-jntVelMax,jntVelMax);

    GazeboServer gz_server(argc,argv);
    auto gz_model = GazeboModel(gz_server.insertModelFromURDFFile(urdf_url));
    gz_model.setModelConfiguration( { "joint_0", "joint_3","joint_5"} , {1.0,-M_PI/2.,M_PI/2.});


    // Lets decide that the robot is gravity compensated
    // So we need to remove G(q) from the solution
    controller.removeGravityTorquesFromSolution(true);
    gz_model.executeAfterWorldUpdate([&](uint32_t n_iter,double current_time,double dt)
    {
        robot_model->setRobotState(gz_model.getWorldToBaseTransform().matrix()
                            ,gz_model.getJointPositions()
                            ,gz_model.getBaseVelocity()
                            ,gz_model.getJointVelocities()
                            ,gz_model.getGravity()
                        );
        // Compensate the gravity at least
        gz_model.setJointGravityTorques(robot_model->getJointGravityTorques());
        // All tasks need the robot to be initialized during the activation phase
        if(n_iter == 1)
            controller.activateTasksAndConstraints();

        controller.update(current_time, dt);

        if(controller.solutionFound())
        {
            gz_model.setJointTorqueCommand( controller.getJointTorqueCommand() );
        }
        else
        {
            gz_model.setBrakes(true);
        }
    });

    std::cout << "Simulation running... (GUI with \'gzclient\')" << "\n";

    // If you want to pause the simulation before starting it uncomment these lines
    // Note that to unlock it either open 'gzclient' and click on the play button
    // Or open a terminal and type 'gz world -p false'
    //
    std::cout << "Gazebo is paused, open gzclient to unpause it or type 'gz world -p false' in a new terminal" << '\n';
    gazebo::event::Events::pause.Signal(true);

    gz_server.run();
    return 0;
}
