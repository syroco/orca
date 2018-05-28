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
    // This is equivalent to GazeboServer gz("worlds/empty.world")
    GazeboServer gzserver;
    // Insert a model onto the server and create the GazeboModel from the return value
    // You can also set the initial pose, and override the name in the URDF
    auto gzrobot = GazeboModel(gzserver.insertModelFromURDFFile(urdf_url));

    // Create an ORCA robot
    auto robot = std::make_shared<RobotDynTree>();
    robot->loadModelFromFile(urdf_url);
    robot->print();

    // Update the robot on at every iteration
    gzrobot.setCallback([&](uint32_t n_iter,double current_time,double dt)
    {
        robot->setRobotState(gzrobot.getWorldToBaseTransform().matrix()
                            ,gzrobot.getJointPositions()
                            ,gzrobot.getBaseVelocity()
                            ,gzrobot.getJointVelocities()
                            ,gzrobot.getGravity()
                        );
    });

    // Run the main simulation loop.
    // This is a blocking call that runs the simulation steps
    // It can be stopped by CTRL+C
    // You can optionally add a callback that happends after WorldUpdateEnd
    gzserver.run();
    return 0;
}
