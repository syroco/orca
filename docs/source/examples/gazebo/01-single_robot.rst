.. _01-single_robot:

Simulating a single robot
=========================

.. note:: The source code for this example can be found in ``[orca_root]/examples/gazebo/01-single_robot.cc``, or alternatively on github at: https://github.com/syroco/orca/blob/dev/examples/gazebo/01-single_robot.cc


.. code-block:: c++
    :linenos:

    #include <orca/gazebo/GazeboServer.h>
    #include <orca/gazebo/GazeboModel.h>

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
        // This is equivalent to GazeboServer gz("worlds/empty.world")
        GazeboServer s;
        // Insert a model onto the server and create the GazeboModel from the return value
        // You can also set the initial pose, and override the name in the URDF
        auto m = GazeboModel(s.insertModelFromURDFFile(urdf_url));

        // This is how you can get the full state of the robot
        std::cout << "Model \'" << m.getName() << "\' State :\n" << '\n';
        std::cout << "- Gravity "                   << m.getGravity().transpose()                << '\n';
        std::cout << "- Base velocity\n"            << m.getBaseVelocity().transpose()           << '\n';
        std::cout << "- Tworld->base\n"             << m.getWorldToBaseTransform().matrix()      << '\n';
        std::cout << "- Joint positions "           << m.getJointPositions().transpose()         << '\n';
        std::cout << "- Joint velocities "          << m.getJointVelocities().transpose()        << '\n';
        std::cout << "- Joint external torques "    << m.getJointExternalTorques().transpose()   << '\n';
        std::cout << "- Joint measured torques "    << m.getJointMeasuredTorques().transpose()   << '\n';

        // You can optionally register a callback that will be called
        // after every WorldUpdateEnd, so the internal gazebo model is updated
        // and you can get the full state (q,qdot,Tworld->base, etc)
        m.setCallback([&](uint32_t n_iter,double current_time,double dt)
        {
            std::cout << "[" << m.getName() << "]" << '\n'
                << "- iteration    " << n_iter << '\n'
                << "- current time " << current_time << '\n'
                << "- dt           " << dt << '\n';
            // Example : get the minimal state
            const Eigen::VectorXd& q = m.getJointPositions();
            const Eigen::VectorXd& qdot = m.getJointVelocities();

            std::cout << "ExtTrq " << m.getJointExternalTorques().transpose() << '\n';
            std::cout << "MeaTrq " << m.getJointMeasuredTorques().transpose() << '\n';
        });

        // Run the main simulation loop.
        // This is a blocking call that runs the simulation steps
        // It can be stopped by CTRL+C
        // You can optionally add a callback that happends after WorldUpdateEnd
        s.run();
        return 0;
    }
