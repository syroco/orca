#include <orca/gazebo/GazeboServer.h>
#include <orca/gazebo/GazeboModel.h>

using namespace orca::gazebo;

int main(int argc, char** argv)
{
    if(argc < 2)
    {
        std::cerr << "Usage : " << argv[0] << " /path/to/robot-urdf.urdf" << "\n";
        return -1;
    }
    std::string urdf_url(argv[1]);

    GazeboServer gz("worlds/empty.world");

    auto gz_model_one = GazeboModel(gz.insertModelFromURDFFile(urdf_url,"one"));
    auto gz_model_two = GazeboModel(gz.insertModelFromURDFFile(urdf_url,"two"));
    
    gz.stepOnce();
    
    gz_model_one.printState();
    gz_model_two.printState();

    return 0;
}
