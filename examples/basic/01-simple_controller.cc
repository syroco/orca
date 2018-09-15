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

int main(int argc, char const *argv[])
{
    // Get the urdf file from the command line
    if(argc < 2)
    {
        std::cerr << "Usage : " << argv[0] << " /path/to/robot-urdf.urdf (optionally -l debug/info/warning/error)" << "\n";
        return -1;
    }
    std::string urdf_url(argv[1]);

    //  Parse logger level as --log_level (or -l) debug/warning etc
    orca::utils::Logger::parseArgv(argc, argv);

    // Create the kinematic model that is shared by everybody. Here you can pass a robot name
    auto robot_model = std::make_shared<RobotModel>();

     //  If you don't pass a robot name, it is extracted from the urdf
    robot_model->loadModelFromFile(urdf_url);

    // All the transformations (end effector pose for example) will be expressed wrt this base frame
    robot_model->setBaseFrame("base_link");

    // Sets the world gravity (Optional)
    robot_model->setGravity(Eigen::Vector3d(0,0,-9.81));

    // This is an helper function to store the whole state of the robot as eigen vectors/matrices. This class is totally optional, it is just meant to keep consistency for the sizes of all the vectors/matrices. You can use it to fill data from either real robot and simulated robot.
    RobotState eigState;

    // resize all the vectors/matrices to match the robot configuration
    eigState.resize(robot_model->getNrOfDegreesOfFreedom());

    // Set the initial state to zero (arbitrary). @note: here we only set q,qot because this example asserts we have a fixed base robot
    eigState.jointPos.setZero();
    eigState.jointVel.setZero();

    // Set the first state to the robot
    robot_model->setRobotState(eigState.jointPos,eigState.jointVel);
    // Now is the robot is considered 'initialized'


    // Instanciate an ORCA Controller
    orca::optim::Controller controller(
        "controller"
        ,robot_model
        ,orca::optim::ResolutionStrategy::OneLevelWeighted
        ,QPSolverImplType::qpoases
    );
    // Other ResolutionStrategy options: MultiLevelWeighted, Generalized

    // Cartesian Task
    auto cart_task = controller.addTask<CartesianTask>("CartTask-EE");

    // Set the pose desired for the link_7
    Eigen::Affine3d cart_pos_ref;

    // Setting the translational components.
    cart_pos_ref.translation() = Eigen::Vector3d(1.,0.75,0.5); // x,y,z in meters

    // Rotation is done with a Matrix3x3 and it can be initialized in a few ways. Note that each of these methods produce equivalent Rotation matrices in this case.

    // Example 1 : create a quaternion from Euler anglers ZYZ convention
    Eigen::Quaterniond quat;
    quat = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())
         * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
    cart_pos_ref.linear() = quat.toRotationMatrix();

    // Example 2 : create a quaternion from RPY convention
    cart_pos_ref.linear() = quatFromRPY(0,0,0).toRotationMatrix();

    // Example 3 : create a quaternion from Kuka Convention
    cart_pos_ref.linear() = quatFromKukaConvention(0,0,0).toRotationMatrix();

    // Example 4 : use an Identity quaternion
    cart_pos_ref.linear() = Eigen::Quaterniond::Identity().toRotationMatrix();


    // Set the desired cartesian velocity and acceleration to zero
    Vector6d cart_vel_ref = Vector6d::Zero();
    Vector6d cart_acc_ref = Vector6d::Zero();

    auto cart_acc_pid = std::make_shared<CartesianAccelerationPID>("CartTask-EE-servo_controller");
    // Now set the servoing PID
    Vector6d P;
    P << 1000, 1000, 1000, 10, 10, 10;
    cart_acc_pid->pid()->setProportionalGain(P);
    Vector6d D;
    D << 100, 100, 100, 1, 1, 1;
    cart_acc_pid->pid()->setDerivativeGain(D);

    cart_acc_pid->setControlFrame("link_7");
    // The desired values are set on the servo controller. Because cart_task->setDesired expects a cartesian acceleration. Which is computed automatically by the servo controller
    cart_acc_pid->setDesired(cart_pos_ref.matrix(),cart_vel_ref,cart_acc_ref);
    // Set the servo controller to the cartesian task
    cart_task->setServoController(cart_acc_pid);
    
    // Get the number of actuated joints
    const int ndof = robot_model->getNrOfDegreesOfFreedom();

    // Joint torque limit is usually given by the robot manufacturer
    auto jnt_trq_cstr = controller.addConstraint<JointTorqueLimitConstraint>("JointTorqueLimit");
    Eigen::VectorXd jntTrqMax(ndof);
    jntTrqMax.setConstant(200.0);
    jnt_trq_cstr->setLimits(-jntTrqMax,jntTrqMax);

    // Joint position limits are automatically extracted from the URDF model.
    // Note that you can set them if you want. by simply doing jnt_pos_cstr->setLimits(jntPosMin,jntPosMax).
    auto jnt_pos_cstr = controller.addConstraint<JointPositionLimitConstraint>("JointPositionLimit");

    // Joint velocity limits are usually given by the robot manufacturer
    auto jnt_vel_cstr = controller.addConstraint<JointVelocityLimitConstraint>("JointVelocityLimit");
    Eigen::VectorXd jntVelMax(ndof);
    jntVelMax.setConstant(2.0);
    jnt_vel_cstr->setLimits(-jntVelMax,jntVelMax);


    double dt = 0.5;
    double current_time = 0;

    controller.activateTasksAndConstraints();


    // If your robot's low level controller takes into account the gravity and coriolis torques already (Like with KUKA LWR) then you can tell the controller to remove these components from the torques computed by the solver. Setting them to false keeps the components in the solution (this is the default behavior).
    controller.removeGravityTorquesFromSolution(true);
    controller.removeCoriolisTorquesFromSolution(true);

    // Now you can run the control loop
    for (; current_time < 2.0; current_time +=dt)
    {
        // Here you can get the data from you REAL robot (API is robot-specific)
        // Something like :
            // eigState.jointPos = myRealRobot.getJointPositions();
            // eigState.jointVel = myRealRobot.getJointVelocities();

        // Now update the internal kinematic model with data from the REAL robot
        std::cout << "Setting robot state to : \n" 
            << "Joint Pos : " << eigState.jointPos.transpose() << '\n'
            << "Joint Vel : " << eigState.jointVel.transpose() << '\n';
            
        robot_model->setRobotState(eigState.jointPos,eigState.jointVel);

        // Step the controller + solve the internal optimal problem
        std::cout << "Updating controller..." ;
        controller.update(current_time, dt);
        std::cout << "OK" << '\n';
        
        // Do what you want with the solution
        if(controller.solutionFound())
        {
            // The whole optimal solution [AccFb, Acc, Tfb, T, eWrenches]
            const Eigen::VectorXd& full_solution = controller.getSolution();
            // The optimal joint torque command
            const Eigen::VectorXd& trq_cmd = controller.getJointTorqueCommand();
            // The optimal joint acceleration command
            const Eigen::VectorXd& trq_acc = controller.getJointAccelerationCommand();

            // Send torques to the REAL robot (API is robot-specific)
            //real_tobot->set_joint_torques(trq_cmd);
        }
        else
        {
            // WARNING : Optimal solution is NOT found
            // Switching to a fallback strategy
            // Typical are :
            // - Stop the robot (robot-specific method)
            // - Compute KKT Solution and send to the robot (dangerous)
            // - PID around the current position (dangerous)

            // trq = controller.computeKKTTorques();
            // Send torques to the REAL robot (API is robot-specific)
            // real_tobot->set_joint_torques(trq_cmd);
        }
    }

    // Print the last computed solution (just for fun)
    const Eigen::VectorXd& full_solution = controller.getSolution();
    const Eigen::VectorXd& trq_cmd = controller.getJointTorqueCommand();
    const Eigen::VectorXd& trq_acc = controller.getJointAccelerationCommand();
    std::cout << "Full solution : " << full_solution.transpose() << '\n';
    std::cout << "Joint Acceleration command : " << trq_acc.transpose() << '\n';
    std::cout << "Joint Torque command       : " << trq_cmd.transpose() << '\n';

    // At some point you want to close the controller nicely
    controller.deactivateTasksAndConstraints();


    // Let all the tasks ramp down to zero
    while(!controller.tasksAndConstraintsDeactivated())
    {
        current_time += dt;
        controller.update(current_time,dt);
    }

    // All objets will be destroyed here
    return 0;
}
