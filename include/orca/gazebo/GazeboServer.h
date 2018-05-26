#pragma once
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/SensorsIface.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <sdf/parser_urdf.hh>
#include <sdf/parser.hh>
#include <sdf/sdf.hh>
#include <fstream>
#include <thread>
#include "orca/gazebo/Utils.h"

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
        world_begin_ =  ::gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboServer::worldUpdateBegin,this));
        world_end_ = ::gazebo::event::Events::ConnectWorldUpdateEnd(std::bind(&GazeboServer::worldUpdateEnd,this));
        return world_;
    }

    virtual ~GazeboServer()
    {
        ::gazebo::shutdown();
    }

    double getDt()
    {
        assertWorldLoaded();
        #if GAZEBO_MAJOR_VERSION > 8
            return world_->Physics()->GetMaxStepSize();
        #else
            return world_->GetPhysicsEngine()->GetMaxStepSize();
        #endif
    }

    void runOnce()
    {
        assertWorldLoaded();
        ::gazebo::runWorld(world_, 1);
    }

    void run(std::function<void(uint32_t,double,double)> callback=0)
    {
        assertWorldLoaded();
        if(callback)
            registerCallback(callback);
        ::gazebo::runWorld(world_, 0);
    }
    ::gazebo::physics::ModelPtr insertModelFromURDFFile(const std::string& urdf_url
        , Eigen::Vector3d init_pos = Eigen::Vector3d::Zero()
        , Eigen::Quaterniond init_rot = Eigen::Quaterniond::Identity()
        , std::string model_name="")
    {
        TiXmlDocument doc(urdf_url);
        doc.LoadFile();
        return insertModelFromTinyXML(&doc,init_pos,init_rot,model_name);
    }

    ::gazebo::physics::ModelPtr insertModelFromURDFString(const std::string& urdf_str
        , Eigen::Vector3d init_pos = Eigen::Vector3d::Zero()
        , Eigen::Quaterniond init_rot = Eigen::Quaterniond::Identity()
        , std::string model_name="")
    {
        TiXmlDocument doc;
        doc.Parse(urdf_str.c_str());
        return insertModelFromTinyXML(&doc,init_pos,init_rot,model_name);
    }

    int getModelCount()
    {
        assertWorldLoaded();
        #if GAZEBO_MAJOR_VERSION > 8
            return world_->ModelCount();
        #else
            return world_->GetModelCount();
        #endif
    }

    std::vector<std::string> getModelNames()
    {
        assertWorldLoaded();
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
        assertWorldLoaded();
        #if GAZEBO_MAJOR_VERSION > 8
            auto model = world_->ModelByName( model_name );
        #else
            auto model = world_->GetModel( model_name );
        #endif
        return static_cast<bool>(model);
    }

    const Eigen::Vector3d& getGravity()
    {
        assertWorldLoaded();
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
        assertWorldLoaded();
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
    
    void registerCallback(std::function<void(uint32_t,double,double)> callback)
    {
        callback_ = callback;
    }
private:
    void assertWorldLoaded()
    {
        if(!world_)
            throw std::runtime_error("[GazeboServer] World is not loaded");
    }
    ::gazebo::physics::ModelPtr insertModelFromTinyXML(TiXmlDocument* doc
        , Eigen::Vector3d init_pos = Eigen::Vector3d::Zero()
        , Eigen::Quaterniond init_rot = Eigen::Quaterniond::Identity()
        , std::string model_name="")
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


        if(model_name.empty())
        {
            // Extract model name from URDF
            if(!getRobotNameFromTinyXML(robotElement,model_name))
                return 0;
        }
        else
        {
            // Set the robot name to user specified name
            if(!setRobotNameToTinyXML(robotElement,model_name))
                return 0;
        }

        std::cout << "[GazeboServer] Trying to insert model \'" << model_name << "\'" << std::endl;

        sdf::URDF2SDF urdf_to_sdf;
        TiXmlDocument sdf_xml = urdf_to_sdf.InitModelDoc(doc);
        // NOTE : from parser_urdf.cc
        //          URDF is compatible with version 1.4. The automatic conversion script
        //          will up-convert URDF to SDF.
        //          sdf->SetAttribute("version", "1.4");
        // I'm setting it to the current sdf version to avoid warnings
        sdf_xml.RootElement()->SetAttribute("version",sdf::SDF::Version());
        
        TiXmlPrinter printer;
        printer.SetIndent( "    " );
        sdf_xml.Accept( &printer );
        std::string xml_str = printer.CStr();
        sdf::SDF _sdf;
        _sdf.SetFromString(xml_str);
        
        // Set the initial position
        ignition::math::Pose3d init_pose(convVec3(init_pos),convQuat(init_rot));
        // WARNING: These elemts should always exist, so i'm not checking is they are null
        _sdf.Root()->GetElement("model")->GetElement("pose")->Set<ignition::math::Pose3d>(init_pose);
        
        //std::cout << "[GazeboServer] Converted to sdf :\n" << _sdf.ToString() << '\n';
        
        
        world_->InsertModelSDF(_sdf);

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

            model = getModelByName(model_name);
            if(model)
            {
                do_exit = true;
                if(th.joinable())
                    th.join();
                std::cout << "[GazeboServer] Model " << model_name << " successfully loaded" << std::endl;
                countExistingSensors();
                return model;
            }
            std::cout << "[GazeboServer] Model not yet loaded" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        return 0;
    }

    bool getRobotNameFromTinyXML(TiXmlElement* robotElement, std::string& model_name)
    {
        if(robotElement)
        {
            if (robotElement->Attribute("name"))
            {
                model_name = robotElement->Attribute("name");
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

        if (model_name.empty())
        {
            std::cerr << "[GazeboServer] Robot name is empty" << '\n';
            return false;
        }
        return true;
    }

    bool setRobotNameToTinyXML(TiXmlElement* robotElement, std::string& model_name)
    {
        std::string dummy;
        if(!getRobotNameFromTinyXML(robotElement,dummy))
            return false;

        robotElement->RemoveAttribute("name");
        robotElement->SetAttribute("name", model_name);
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
        if(callback_)
            callback_(world_->Iterations(),
                      getSimTime(),
                      getDt());
    }

    double getSimTime()
    {
        #if GAZEBO_MAJOR_VERSION > 8
            return world_->SimTime().Double();
        #else
            return world_->GetSimTime().Double();
        #endif
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

    std::function<void(uint32_t,double,double)> callback_;
    ::gazebo::physics::WorldPtr world_;
    ::gazebo::event::ConnectionPtr world_begin_;
    ::gazebo::event::ConnectionPtr world_end_;
    Eigen::Vector3d gravity_vector_;
    int n_sensors_ = 0;
};

} // namespace gazebo
} // namespace orca
