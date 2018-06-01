.. _02-multi_robot:

Simulating multiple robots
===========================

.. note:: The source code for this example can be found in ``[orca_root]/examples/gazebo/02-multi_robot.cc``, or alternatively on github at: https://github.com/syroco/orca/blob/dev/examples/gazebo/02-multi_robot.cc


.. code-block:: c++
    :linenos:

    #include <orca/gazebo/GazeboServer.h>
    #include <orca/gazebo/GazeboModel.h>

    using namespace orca::gazebo;
    using namespace Eigen;

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
        GazeboServer gz_server;

        // Insert a model onto the server and create the GazeboModel from the return value
        // You can also set the initial pose, and override the name in the URDF
        auto gz_model_one = GazeboModel(gz_server.insertModelFromURDFFile(urdf_url
            ,Vector3d(-2,0,0)
            ,quatFromRPY(0,0,0)
            ,"one"));

        // Insert a second model with a different pose and a different name
        auto gz_model_two = GazeboModel(gz_server.insertModelFromURDFFile(urdf_url
            ,Vector3d(2,0,0)
            ,quatFromRPY(0,0,0)
            ,"two"));

        // You can optionally register a callback for each GazeboModel so you can do individual updates on it
        // The function is called after every WorldUpdateEnd, so the internal gazebo model is updated
        // and you can get the full state (q,qdot,Tworld->base, etc)
        gz_model_two.setCallback([&](uint32_t n_iter,double current_time,double dt)
        {
            std::cout << "gz_model_two \'" << gz_model_two.getName() << "\' callback " << '\n'
                << "- iteration    " << n_iter << '\n'
                << "- current time " << current_time << '\n'
                << "- dt           " << dt << '\n';
            // Example : get the joint positions
            // gz_model_two.getJointPositions()
        });

        // Run the main simulation loop.
        // This is a blocking call that runs the simulation steps
        // It can be stopped by CTRL+C
        // You can optionally add a callback that happends after WorldUpdateEnd
        gz_server.run([&](uint32_t n_iter,double current_time,double dt)
        {
            std::cout << "GazeboServer callback " << '\n'
                << "- iteration    " << n_iter << '\n'
                << "- current time " << current_time << '\n'
                << "- dt           " << dt << '\n';
        });
        return 0;
    }
