//|  This file is a part of the ORCA framework.
//|
//|  Copyright 2018, Fuzzy Logic Robotics
//|  Copyright 2017, ISIR / Universite Pierre et Marie Curie (UPMC)
//|
//|  Main contributor(s): Antoine Hoarau, Ryan Lober, and
//|  Fuzzy Logic Robotics <info@fuzzylogicrobotics.com>
//|
//|  ORCA is a whole-body reactive controller framework for robotics.
//|
//|  This software is governed by the CeCILL-C license under French law and
//|  abiding by the rules of distribution of free software.  You can  use,
//|  modify and/ or redistribute the software under the terms of the CeCILL-C
//|  license as circulated by CEA, CNRS and INRIA at the following URL
//|  "http://www.cecill.info".
//|
//|  As a counterpart to the access to the source code and  rights to copy,
//|  modify and redistribute granted by the license, users are provided only
//|  with a limited warranty  and the software's author,  the holder of the
//|  economic rights,  and the successive licensors  have only  limited
//|  liability.
//|
//|  In this respect, the user's attention is drawn to the risks associated
//|  with loading,  using,  modifying and/or developing or reproducing the
//|  software by the user in light of its specific status of free software,
//|  that may mean  that it is complicated to manipulate,  and  that  also
//|  therefore means  that it is reserved for developers  and  experienced
//|  professionals having in-depth computer knowledge. Users are therefore
//|  encouraged to load and test the software's suitability as regards their
//|  requirements in conditions enabling the security of their systems and/or
//|  data to be ensured and,  more generally, to use and operate it in the
//|  same conditions as regards security.
//|
//|  The fact that you are presently reading this means that you have had
//|  knowledge of the CeCILL-C license and that you accept its terms.

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
    inline bool setupServer(const std::vector<std::string> &_args)
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
    GazeboServer()
    {
        std::cout <<"\x1B[32m[[--- Gazebo Server ---]]\033[0m"<< '\n';
        load();
    }
    GazeboServer(int argc, char * argv[])
    {
        std::vector<std::string> v;
        for (int i = 0; i < argc; i++)
            v.push_back(argv[i]);
        load(v);
    }
    GazeboServer(int argc, char const * argv[])
    {
        std::vector<std::string> v;
        for (int i = 0; i < argc; i++)
            v.push_back(argv[i]);
        load(v);
    }
    GazeboServer(std::vector<std::string> server_options,const std::string& world_name = "worlds/empty.world")
    {
        load(server_options,world_name);
    }
    GazeboServer(const std::string& world_name,std::vector<std::string> server_options = {"--verbose"})
    {
        load(server_options,world_name);
    }

    bool load(std::vector<std::string> server_options = {"--verbose"},const std::string& world_name = "worlds/empty.world")
    {
        std::vector<std::string> plugins;
        for (size_t i = 0; i < server_options.size() - 1; i++)
        {
            std::string op(server_options[i]);
            std::string next_op(server_options[i+1]);

            if (op.find("verbose") != std::string::npos)
            {
                ::gazebo::printVersion();
                ::gazebo::common::Console::SetQuiet(false);
            }
            if (op.find("-s") != std::string::npos || op.find("--server-plugin") != std::string::npos)
            {
                plugins.push_back(next_op);
                i++;
            }
        }

        std::cout <<"\x1B[32m[[--- Gazebo setup ---]]\033[0m"<< '\n';
        if(plugins.size())
        {
            std::cout <<"\x1B[32m Loading plugins\033[0m"<< '\n';
            for(auto plugin : plugins)
            {
                ::gazebo::addPlugin(plugin);
                std::cout <<"\x1B[32m   - " << plugin << "\033[0m"<< '\n';
            }
        }

        if(!::gazebo::setupServer(server_options))
        {
            std::cerr << "[GazeboServer] Could not setup server" << '\n';
            return false;
        }
        return bool(this->loadWorld(world_name));
    }

    ::gazebo::physics::WorldPtr loadWorld(const std::string& world_name)
    {
        auto world = ::gazebo::loadWorld(world_name);
        if(!world)
        {
            std::cerr << "[GazeboServer] Could not load world with world file " << world_name << '\n';
            return nullptr;
        }
        world_ = world;
        world_begin_ =  ::gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&GazeboServer::worldUpdateBegin,this));
        world_end_ = ::gazebo::event::Events::ConnectWorldUpdateEnd(std::bind(&GazeboServer::worldUpdateEnd,this));
        #if GAZEBO_MAJOR_VERSION > 8
            ::gazebo::sensors::SensorManager::Instance()->Init();
            ::gazebo::sensors::SensorManager::Instance()->RunThreads();
        #endif
        return world_;
    }
    
    bool enablePhysicsEngine(bool enable)
    {
        assertWorldLoaded();
    #if GAZEBO_MAJOR_VERSION > 8
        world_->SetPhysicsEnabled(enable);
    #else
        world_->EnablePhysicsEngine(enable);
    #endif
        return true;
    }
    
    bool fileExists(const std::string& file_path)
    {
        return std::ifstream(file_path).good();
    }
    
    std::string fileToString(const std::string& file_path)
    {
        std::string out;
        std::ifstream ifs(file_path);
        out.assign((std::istreambuf_iterator<char>(ifs)),
                (std::istreambuf_iterator<char>()));
        return out;
    }
    
    bool spawnModel(const std::string& instanceName,
            const std::string& modelName)
    {
    //     assertWorldLoaded();
    // 
    //     //check if file exists
    //     const string path = ::gazebo::common::SystemPaths::Instance()->FindFileURI(
    //             modelName);
    //     if (path.empty()) {
    //         std::cout << "\x1B[32m[[--- Model " << modelName
    //                 << " couldn't be found ---]]\033[0m" << std::endl;
    //         return false;
    //     }
    // 
    //     std::string model_str;
    //     std::string model_path;
    // 
    //     std::vector<std::string> files_to_try = {
    //         path + "/model.urdf")
    //         , path + "/model.sdf")
    //         , path + "/" + instanceName + ".urdf")
    //         , path + "/" + instanceName + ".sdf")
    //     }
    // 
    //     for(auto f : files_to_try)
    //     {
    //         if(fileExists(f)) 
    //         {
    //             model_str = fileToString(f);
    //             model_path = f;
    //         }
    //     }
    // 
    //     if (model_str.empty()) {
    //         std::cout << "\x1B[32m[[--- Model file " << model_path
    //                 << " is empty ! ---]]\033[0m" << std::endl;
    //         return false;
    //     }
    // 
    //     TiXmlDocument gazebo_model_xml;
    //     gazebo_model_xml.Parse(model_xml.c_str());
    // 
    //     TiXmlElement* name_urdf = gazebo_model_xml.FirstChildElement("robot");
    //     TiXmlElement* name_sdf = gazebo_model_xml.FirstChildElement("model");
    // 
    //     if(name_urdf)
    //     {
    //         if (nameElement->Attribute("name") != NULL) {
    //             // removing old model name
    //             nameElement->RemoveAttribute("name");
    //         }
    //         // replace with user specified name
    //         nameElement->SetAttribute("name", instanceName);
    //     }
    // 
    //     if (!nameElement) {
    //         cout << "it's not an urdf check for sdf" << endl;
    //         nameElement = gazebo_model_xml.FirstChildElement("model");
    //         if (!nameElement) {
    //             std::cout
    //                     << "\x1B[31m[[--- Can't be parsed: No <model> or <robot> tag found! ---]]\033[0m"
    //                     << std::endl;
    //             return false;
    //         } else {
    //             // handle sdf
    //             sdf::SDF root;
    //             root.SetFromString(model_xml);
    // #if GAZEBO_MAJOR_VERSION >= 6
    //               sdf::ElementPtr nameElementSDF = root.Root()->GetElement("model");
    // #else
    //               sdf::ElementPtr nameElementSDF = root.root->GetElement("model");
    // #endif
    //             nameElementSDF->GetAttribute("name")->SetFromString(instanceName);
    //         }
    //     } else {
    //         // handle urdf
    // 
    //         if (nameElement->Attribute("name") != NULL) {
    //             // removing old model name
    //             nameElement->RemoveAttribute("name");
    //         }
    //         // replace with user specified name
    //         nameElement->SetAttribute("name", instanceName);
    //     }
    // 
    // //	world->InsertModelFile(modelName);
    //     TiXmlPrinter printer;
    //     printer.SetIndent("    ");
    //     gazebo_model_xml.Accept(&printer);
    // 
    //     world_->InsertModelString(printer.CStr());
    // 
    //     gazebo::common::Time timeout((double) timeoutSec);
    // 
    //     auto modelDeployTimer(new gazebo::common::Timer());
    // 
    //     modelDeployTimer->Start();
    //     while (modelDeployTimer->GetRunning()) {
    //         if (modelDeployTimer->GetElapsed() > timeout) {
    //             gzerr
    //                     << "SpawnModel: Model pushed to spawn queue, but spawn service timed out waiting for model to appear in simulation under the name "
    //                     << instanceName << endl;
    //             modelDeployTimer->Stop();
    //             return false;
    //         }
    // 
    //         {
    // #if GAZEBO_MAJOR_VERSION > 8
    //             auto model = world_->ModelByName(instanceName);
    // #else
    //             auto model = world_->GetModel(instanceName);
    // #endif
    //             if (model){
    //                 modelDeployTimer->Stop();
    //                 break;
    //             }
    //         }
    //         std::this_thread::sleep_for(std::chrono::milliseconds(500));
    //     }

        return false;
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

    void run()
    {
        assertWorldLoaded();
        ::gazebo::runWorld(world_, 0);
    }

    void shutdown()
    {
        ::gazebo::event::Events::sigInt.Signal();
        std::cout <<"\x1B[32m[[--- Stoping Simulation ---]]\033[0m"<< '\n';
        if(world_)
            world_->Fini();
        std::cout <<"\x1B[32m[[--- Gazebo Shutdown... ---]]\033[0m"<< '\n';
        //NOTE: This crashes as gazebo is running is a thread
        ::gazebo::shutdown();
    }

    ::gazebo::physics::ModelPtr insertModelFromURDFFile(const std::string& urdf_url
        , Eigen::Vector3d init_pos = Eigen::Vector3d::Zero()
        , Eigen::Quaterniond init_rot = Eigen::Quaterniond::Identity()
        , std::string model_name="")
    {
        std::string cmd = "gz sdf -p " + urdf_url;
        return insertModelFromSDFString( custom_exec(cmd) );
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

    void executeAfterWorldUpdate(std::function<void(uint32_t,double,double)> callback)
    {
        callback_ = callback;
    }

    virtual ~GazeboServer()
    {

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
            return nullptr;
        }
        TiXmlElement* robotElement = doc->FirstChildElement("robot");
        if(!robotElement)
        {

            TiXmlPrinter printer;
            printer.SetIndent( "    " );
            doc->Accept( &printer );
            std::cerr << printer.CStr() << '\n';
            std::cerr << "[GazeboServer] Could not get the <robot> tag in the URDF " << '\n';
            return nullptr;
        }


        if(model_name.empty())
        {
            // Extract model name from URDF
            if(!getRobotNameFromTinyXML(robotElement,model_name))
                return nullptr;
        }
        else
        {
            // Set the robot name to user specified name
            if(!setRobotNameToTinyXML(robotElement,model_name))
                return nullptr;
        }

        std::cout << "[GazeboServer] Trying to insert model \'" << model_name << "\'" << std::endl;
        
        sdf::URDF2SDF urdf_to_sdf;
        TiXmlDocument sdf_xml = urdf_to_sdf.InitModelDoc(doc);
        // NOTE : from parser_urdf.cc
        //          URDF is compatible with version 1.4. The automatic conversion script
        //          will up-convert URDF to SDF.
        //          sdf->SetAttribute("version", "1.4");
        // I'm setting it to the current sdf version to avoid warnings
        // NOTE 2 : Settings the version to 1.6 (kinetic+16.04) disallow the fixed joints
        // to have any <origin rpy="" part, it is ignored for some reason
        // FIXME : Find out how to convert URDF to sdf 1.6 (or ar least current version)
        //sdf_xml.RootElement()->SetAttribute("version",sdf::SDF::Version());

        TiXmlPrinter printer;
        printer.SetIndent( "    " );
        sdf_xml.Accept( &printer );

        return insertModelFromSDFString(printer.CStr(),init_pos,init_rot,model_name);
    }

    ::gazebo::physics::ModelPtr insertModelFromSDFString(std::string sdf_string
        , Eigen::Vector3d init_pos = Eigen::Vector3d::Zero()
        , Eigen::Quaterniond init_rot = Eigen::Quaterniond::Identity()
        , std::string model_name="")
    {
        sdf::SDF _sdf;
        _sdf.SetFromString(sdf_string);

        // Set the initial position
        ignition::math::Pose3d init_pose(convVec3(init_pos),convQuat(init_rot));
        // WARNING: These elemts should always exist, so i'm not checking is they are null
        _sdf.Root()->GetElement("model")->GetElement("pose")->Set<ignition::math::Pose3d>(init_pose);

        if(model_name.empty())
        {
            model_name = _sdf.Root()->GetElement("model")->GetAttribute("name")->GetAsString();
        }
        
        std::cout << "[GazeboServer] Converted to sdf :\n" << _sdf.ToString() << '\n';

        world_->InsertModelSDF(_sdf);

        std::atomic<bool> do_exit(false);
        auto th = std::thread([&]()
        {
            int i=10;
            while(--i)
            {
                if(do_exit)
                    return;
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }

            std::cerr << "[GazeboServer] \x1B[33m\n\nYou are seeing this message because gazebo does not know where to find the meshes for the \'" << model_name << "\' model\n\e[0m" << std::endl
                << "\x1B[33mTo make it work, you need to append the path to the dir containing the meshes to the GAZEBO_MODEL_PATH env variable\n, one level above the package.xml. Example :\n\e[0m" << std::endl
                << "\x1B[33mIf the meshes for the model \'" << model_name << "\' are located at /path/to/" << model_name << "_description\n\e[0m" << std::endl
                << "\x1B[33mThen append \'/path/to/\' to GAZEBO_MODEL_PATH with export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/path/to/\n\e[0m" << std::endl
                << "\x1B[33mIf you are using ROS (and rospack find  " << model_name << "_description works), just add this to your ~/.bashrc :\n\nexport GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$ROS_PACKAGE_PATH\n\e[0m" << std::endl
                << "\x1B[33mIf you are NOT using ROS, add also :\n\nsource /usr/share/gazebo/setup.sh\n\e[0m" << std::endl;
        });

        int max_trials = 10;
        for (size_t i = 0; i < max_trials; i++)
        {
            std::cout << "[GazeboServer] Runing the world (" << i+1 << "/" << max_trials << ") ... " << std::endl;

            ::gazebo::runWorld(world_,1);

            std::cout << "[GazeboServer] Now verifying if the model is correctly loaded..." << std::endl;
                        
            ::gazebo::physics::ModelPtr model = getModelByName(model_name);
            if(model)
            {
                do_exit = true;
                if(th.joinable())
                    th.join();
                std::cout << "[GazeboServer] Model '" << model_name << "' successfully loaded" << std::endl;
                countExistingSensors();
                return model;
            }
            std::cout << "[GazeboServer] Model '" << model_name << "' yet loaded" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        do_exit = true;
        if(th.joinable())
            th.join();
        return nullptr;
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
        #if GAZEBO_MAJOR_VERSION > 8
            ::gazebo::sensors::SensorManager::Instance()->Update();
        #else
            int tmp_sensor_count = 0;
            for(auto model : world_->GetModels())
                tmp_sensor_count += model->GetSensorCount();

            // FIXME: Find out a way to be notified when a sensor is replaced
            // Here we only check the number of sensors 
            if(tmp_sensor_count > n_sensors_)
            {
                std::cerr << "[GazeboServer] Loading " << tmp_sensor_count - n_sensors_ << " sensors\n";
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
            } else {
                // NOTE: same number, we do nothing, less it means we removed a model
                n_sensors_ = tmp_sensor_count;
            }
            
            if(n_sensors_ > 0)
                ::gazebo::sensors::run_once();
        #endif
    }

    void worldUpdateEnd()
    {
        if(callback_)
            callback_(getIterations(),getSimTime(),getDt());
    }

    double getIterations()
    {
        #if GAZEBO_MAJOR_VERSION > 8
            return world_->Iterations();
        #else
            return world_->GetIterations();
        #endif
    }

    double getSimTime()
    {
        assertWorldLoaded();
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
