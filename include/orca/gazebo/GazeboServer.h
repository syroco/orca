#pragma once
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/SensorsIface.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <sdf/parser_urdf.hh>
#include <fstream>
#include <thread>
#include <Eigen/Dense>
#include "orca/gazebo/GazeboModel.h"

#if GAZEBO_MAJOR_VERSION < 6
struct g_vectorStringDup
{
  char *operator()(const std::string &_s)
  {
    return strdup(_s.c_str());
  }
};
namespace gazebo
{
    bool setupServer(const std::vector<std::string> &_args)
    {
      std::vector<char *> pointers(_args.size());
      std::transform(_args.begin(), _args.end(), pointers.begin(),
                     g_vectorStringDup());
      pointers.push_back(0);
      bool result = ::gazebo::setupServer(_args.size(), &pointers[0]);

      // Deallocate memory for the command line arguments alloocated with strdup.
      for (size_t i = 0; i < pointers.size(); ++i)
        free(pointers.at(i));

      return result;
    }
}
#endif

namespace orca
{
namespace gazebo
{

class GazeboServer
{
public:
    GazeboServer(bool load_default_values=true
        ,const std::string& world_name = "worlds/empty.world"
        ,std::vector<std::string> server_options = {"--verbose"})
    {
        ::gazebo::printVersion();
        if(load_default_values)
            load(world_name,server_options);
        world_begin_ =  ::gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboServer::worldUpdateBegin,this));
        world_end_ = ::gazebo::event::Events::ConnectWorldUpdateEnd(std::bind(&GazeboServer::worldUpdateEnd,this));
    }

    bool load(const std::string& world_name = "worlds/empty.world",std::vector<std::string> server_options = {"--verbose"})
    {
        if(!::gazebo::setupServer(server_options))
        {
            std::cerr << "[GazeboServer] Could not setup server" << '\n';
            return false;
        }
        this->loadWorld(world_name);
        return static_cast<bool>(world_);
    }

    ::gazebo::physics::WorldPtr loadWorld(const std::string& world_name)
    {
        auto world = ::gazebo::loadWorld(world_name);
        if(!world)
        {
            std::cerr << "[GazeboServer] Could not load world with world file " << world_name << '\n';
            return 0;
        }
        world_ = world;
        return world_;
    }

    virtual ~GazeboServer()
    {
        ::gazebo::shutdown();
    }

    double getDt()
    {
        if(!world_)
        {
            std::cerr << "[GazeboServer] World is not loaded" << '\n';
            return 0;
        }
        #if GAZEBO_MAJOR_VERSION > 8
            dt_ = world_->Physics()->GetMaxStepSize();
        #else
            dt_ = world_->GetPhysicsEngine()->GetMaxStepSize();
        #endif
        return dt_;
    }

    ::gazebo::physics::ModelPtr insertModelFromURDFFile(const std::string& urdf_url)
    {
        TiXmlDocument doc(urdf_url);
        doc.LoadFile();
        return insertModelFromTinyXML(&doc);
    }

    ::gazebo::physics::ModelPtr insertModelFromURDFString(const std::string& urdf_str)
    {
        TiXmlDocument doc;
        doc.Parse(urdf_str.c_str());
        return insertModelFromTinyXML(&doc);
    }

    int getModelCount()
    {
        #if GAZEBO_MAJOR_VERSION > 8
            return world_->ModelCount();
        #else
            return world_->GetModelCount();
        #endif
    }

    std::vector<std::string> getModelNames()
    {
        std::vector<std::string> names;

        #if GAZEBO_MAJOR_VERSION > 8
            for(auto model : world_->Models())
                names.push_back(model->GetName());
        #else
            for(auto model : world_->GetModels())
                names.push_back(model->GetName());
        #endif

        return names;
    }

    bool modelExists(const std::string& model_name)
    {
        #if GAZEBO_MAJOR_VERSION > 8
            auto model = world_->ModelByName( model_name );
        #else
            auto model = world_->GetModel( model_name );
        #endif
        return static_cast<bool>(model);
    }

    const Eigen::Vector3d& getGravity()
    {
        if(!world_)
        {
            std::cerr << "[GazeboServer] World is not loaded" << '\n';
            return gravity_vector_;
        }
        #if GAZEBO_MAJOR_VERSION > 8
            auto g = world_->Gravity();
        #else
            auto g = world_->GetPhysicsEngine()->GetGravity();
        #endif
        gravity_vector_[0] = g[0];
        gravity_vector_[1] = g[1];
        gravity_vector_[2] = g[2];
        return gravity_vector_;
    }

    std::vector<std::string> getModelJointNames(const std::string& model_name)
    {
        std::vector<std::string> joint_names;
        auto model = getModelByName(model_name);
        if(!model)
            return joint_names;
        for(auto j : model->GetJoints())
            joint_names.push_back(j->GetName());
        return joint_names;
    }

    ::gazebo::physics::Joint_V getJointsFromNames(const std::string& model_name,const std::vector<std::string>& joint_names)
    {
        ::gazebo::physics::Joint_V jv;
        auto model = getModelByName(model_name);
        if(!model)
            return jv;
        for(auto n : joint_names)
        {
            auto joint = model->GetJoint(n);
            if(joint)
                jv.push_back(joint);
        }
        return jv;
    }

    ::gazebo::physics::ModelPtr getModelByName(const std::string& model_name)
    {
        #if GAZEBO_MAJOR_VERSION > 8
            return world_->ModelByName(model_name);
        #else
            return world_->GetModel(model_name);
        #endif
    }
    ::gazebo::physics::WorldPtr getWorld()
    {
        return world_;
    }
private:
    ::gazebo::physics::ModelPtr insertModelFromTinyXML(TiXmlDocument* doc)
    {
        if(!doc)
        {
            std::cerr << "[GazeboServer] Document provided is null" << '\n';
            return 0;
        }
        TiXmlElement* robotElement = doc->FirstChildElement("robot");
        if(!robotElement)
        {
            std::cerr << "[GazeboServer] Could not get the <robot> tag in the URDF " << '\n';
            return 0;
        }

        std::string robot_name;
        if(!getRobotNameFromTinyXML(robotElement,robot_name))
            return 0;

        std::cout << "[GazeboServer] Trying to insert model \'" << robot_name << "\'" << std::endl;

        TiXmlPrinter printer;
        printer.SetIndent( "    " );
        doc->Accept( &printer );
        std::string xmltext = printer.CStr();

        world_->InsertModelString(xmltext);

        std::atomic<bool> do_exit(false);
        auto th = std::thread([&]()
        {
            int i=40;
            while(--i)
            {
                if(do_exit)
                    return;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            std::cerr << "[GazeboServer] \x1B[33m\n\nTo make it work you need first to set the path to the **repo/package** containing the meshes\e[0m" << std::endl;
            std::cerr << "[GazeboServer] \x1B[33mexport GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/path/to/the/package_above\e[0m" << std::endl;
            std::cerr << "[GazeboServer] \x1B[33mGazebo won't be able to load the URDF if it cannot load the model meshes\n\n\e[0m" << std::endl;
        });

        int max_trials = 10;
        ::gazebo::physics::ModelPtr model;
        for (size_t i = 0; i < max_trials; i++)
        {
            std::cout << "[GazeboServer] Runing the world (" << i+1 << "/" << max_trials << ") ... " << std::endl;

            ::gazebo::runWorld(world_,1);

            std::cout << "[GazeboServer] Now verifying if the model is correctly loaded..." << std::endl;

            model = getModelByName(robot_name);
            if(model)
            {
                do_exit = true;
                if(th.joinable())
                    th.join();
                std::cout << "[GazeboServer] Model " << robot_name << " successfully loaded" << std::endl;
                countExistingSensors();
                return model;
            }
            std::cout << "[GazeboServer] Model not yet loaded" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        return 0;
    }

    //     std::shared_ptr<RobotDynTree> robot(new RobotDynTree(urdf_url));
    //     robot->setBaseFrame("base_link");
    //
    //     // Build the same with Gazebo joints
    //     std::vector<std::string> joint_idx;
    //     for(int i=0 ; i < robot->getNrOfJoints() ; ++i)
    //     {
    //         auto joint = gz_model->GetJoint( robot->getJointName(i) );
    //
    //         if(joint)
    //         {
    //             std::cout << "[GazeboServer] iDynTree adding joint " << i << " name " << robot->getJointName(i) << '\n';
    //             joint_idx.push_back(robot->getJointName(i) );
    //         }
    //         else
    //         {
    //             std::cout << "[GazeboServer] Not adding joint " << robot->getJointName(i) << '\n';
    //         }
    //
    //     }
    //
    //     if(joint_idx.size() != robot->getNrOfDegreesOfFreedom())
    //     {
    //         std::cout << "[GazeboServer] Could not add the " << robot->getNrOfDegreesOfFreedom() << " joints, only " << joint_idx.size() << '\n';
    //         return -1;
    //     }

    bool getRobotNameFromTinyXML(TiXmlElement* robotElement, std::string& robot_name)
    {
        if(robotElement)
        {
            if (robotElement->Attribute("name"))
            {
                robot_name = robotElement->Attribute("name");
            }
            else
            {
                std::cerr << "[GazeboServer] Could not get the robot tag in the URDF " << '\n';
                return false;
            }
        }
        else
        {
            std::cerr << "[GazeboServer] Provided robot element is invalid" << '\n';
            return false;
        }

        if (robot_name.empty())
        {
            std::cerr << "[GazeboServer] Robot name is empty" << '\n';
            return false;
        }
        return true;
    }
    void worldUpdateBegin()
    {
        int tmp_sensor_count = 0;
        #if GAZEBO_MAJOR_VERSION > 8
            for(auto model : world_->Models())
                tmp_sensor_count += model->GetSensorCount();
        #else
            for(auto model : world_->GetModels())
                tmp_sensor_count += model->GetSensorCount();
        #endif
        do{
            if(tmp_sensor_count > n_sensors_)
            {
                if (!::gazebo::sensors::load())
                {
                    std::cerr << "[GazeboServer] Unable to load sensors\n";
                    break;
                }
                if (!::gazebo::sensors::init())
                {
                    std::cerr << "[GazeboServer] Unable to initialize sensors\n";
                    break;
                }
                ::gazebo::sensors::run_once(true);
                ::gazebo::sensors::run_threads();
                n_sensors_ = tmp_sensor_count;
            }else{
                // NOTE: same number, we do nothing, less it means we removed a model
                n_sensors_ = tmp_sensor_count;
            }
        }while(false);

        if(n_sensors_ > 0)
        {
            ::gazebo::sensors::run_once();
        }
    }

    void worldUpdateEnd()
    {

    }

    void countExistingSensors()
    {
        n_sensors_ = 0;
        #if GAZEBO_MAJOR_VERSION > 8
        for(auto model : world_->Models())
            n_sensors_ += model->GetSensorCount();
        #else
        for(auto model : world_->GetModels())
            n_sensors_ += model->GetSensorCount();
        #endif
    }

    double dt_ = 0.001;
    ::gazebo::physics::WorldPtr world_;
    ::gazebo::event::ConnectionPtr world_begin_;
    ::gazebo::event::ConnectionPtr world_end_;
    Eigen::Vector3d gravity_vector_;
    int n_sensors_ = 0;
};

} // namespace gazebo
} // namespace orca
