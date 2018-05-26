#include <orca/gazebo/GazeboServer.h>
#include <orca/gazebo/GazeboModel.h>

using namespace orca::gazebo;
using namespace Eigen;

int main(int argc, char** argv)
{
    if(argc < 2)
    {
        std::cerr << "Usage : " << argv[0] << " /path/to/robot-urdf.urdf" << "\n";
        return -1;
    }
    std::string urdf_url(argv[1]);

    GazeboServer gz("worlds/empty.world");

    auto gz_model_one = GazeboModel(gz.insertModelFromURDFFile(urdf_url
        ,Vector3d(-2,0,0)
        ,quatFromRPY(0,0,0)
        ,"one"));

    auto gz_model_two = GazeboModel(gz.insertModelFromURDFFile(urdf_url
        ,Vector3d(2,0,0)
        ,quatFromRPY(0,0,0)
        ,"two"));

    gz_model_two.setCallback([&](uint32_t n_iter,double current_time,double dt)
    {
        std::cout << "gz_model_two \'" << gz_model_two.getName() << "\' callback " << '\n'
            << "- iteration    " << n_iter << '\n'
            << "- current time " << current_time << '\n'
            << "- dt           " << dt << '\n';
    });
    
    
    gz.run([&](uint32_t n_iter,double current_time,double dt)
    {
        std::cout << "GazeboServer callback " << '\n'
            << "- iteration    " << n_iter << '\n'
            << "- current time " << current_time << '\n'
            << "- dt           " << dt << '\n';
    });

    return 0;
}
