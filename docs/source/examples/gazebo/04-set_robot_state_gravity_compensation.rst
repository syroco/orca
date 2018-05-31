.. _04-set_robot_state_gravity_compensation:

Set robot state with gravity compensation
==================================================

.. code-block:: c++
    :linenos:

    #include <orca/orca.h>
    #include <orca/gazebo/GazeboServer.h>
    #include <orca/gazebo/GazeboModel.h>

    using namespace orca::all;
    using namespace orca::gazebo;

    int main(int argc, char** argv)
    {
        // Get the urdf file from the command line
        if(argc < 2)
        {
            std::cerr << "Usage : " << argv[0] << " /path/to/robot-urdf.urdf" << "\n";
            return -1;
        }
        std::string urdf_url(argv[1]);

        // Instanciate the gazebo server with de dedfault empty world
        GazeboServer gzserver(argc,argv);
        // This is equivalent to GazeboServer gz("worlds/empty.world")
        // Insert a model onto the server and create the GazeboModel from the return value
        // You can also set the initial pose, and override the name in the URDF
        auto gzrobot = GazeboModel(gzserver.insertModelFromURDFFile(urdf_url));

        // Create an ORCA robot
        auto robot_kinematics = std::make_shared<RobotDynTree>();
        robot_kinematics->loadModelFromFile(urdf_url);
        robot_kinematics->print();

        // Set the gazebo model init pose
        // auto joint_names = robot_kinematics->getJointNames();
        // std::vector<double> init_joint_positions(robot_kinematics->getNrOfDegreesOfFreedom(),0);

        // gzrobot.setModelConfiguration(joint_names,init_joint_positions);
        // or like this
        // gzrobot.setModelConfiguration({"joint_2","joint_5"},{1.5,0.0});

        // Update the robot on at every iteration
        gzrobot.setCallback([&](uint32_t n_iter,double current_time,double dt)
        {
            robot_kinematics->setRobotState(gzrobot.getWorldToBaseTransform().matrix()
                                ,gzrobot.getJointPositions()
                                ,gzrobot.getBaseVelocity()
                                ,gzrobot.getJointVelocities()
                                ,gzrobot.getGravity()
                            );
            gzrobot.setJointGravityTorques(robot_kinematics->getJointGravityTorques());
        });

        // Run the main simulation loop.
        // This is a blocking call that runs the simulation steps
        // It can be stopped by CTRL+C
        // You can optionally add a callback that happends after WorldUpdateEnd
        std::cout << "Simulation running... (GUI with \'gzclient\')" << "\n";
        gzserver.run();
        return 0;
    }
