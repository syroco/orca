#pragma once
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo_client.hh>
#include <sdf/parser_urdf.hh>
#include <fstream>
#include <thread>
#include "orca/robot/RobotDynTree.h"

namespace orca
{
namespace gazebo
{

class GazeboServer
{
public:
    GazeboServer(const std::string& world_name = "worlds/empty.world")
    {
        ::gazebo::printVersion();
        ::gazebo::setupServer({"--verbose"});
        world_ = ::gazebo::loadWorld(world_name);
    }
    double getDt()
    {
        #if GAZEBO_MAJOR_VERSION > 8
        dt_ = world_->Physics()->GetMaxStepSize();
        #else
        dt_ = world_->GetPhysicsEngine()->GetMaxStepSize();
        #endif
        return dt_;
    }
    bool insertModelFromURDFFile(const std::string& urdf_url)
    {
        TiXmlDocument doc(urdf_url);
        doc.LoadFile();
        return insertModelFromTinyXML(&doc);
    }
    bool insertModelFromURDFString(const std::string& urdf_str)
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
    
    virtual ~GazeboServer()
    {
        ::gazebo::shutdown();
    }
private:
    bool insertModelFromTinyXML(TiXmlDocument* doc)
    {
        if(!doc)
        {
            std::cerr << "Document provided is null" << '\n';
            return false;
        }
        TiXmlElement* robotElement = doc->FirstChildElement("robot");
        if(!robotElement)
        {
            std::cerr << "Could not get the <robot> tag in the URDF " << '\n';
            return false;
        }
        
        std::string robot_name;
        if(!getRobotNameFromTinyXML(robotElement,robot_name))
            return false;
        
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

            std::cerr << "\x1B[33m\n\nTo make it work you need first to set the path to the **repo/package** containing the meshes\e[0m" << std::endl;
            std::cerr << "\x1B[33mexport GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/path/to/the/package_above\e[0m" << std::endl;
            std::cerr << "\x1B[33mGazebo won't be able to load the URDF if it cannot load the model meshes\n\n\e[0m" << std::endl;
        });

        int max_trials = 10;
        for (size_t i = 0; i < max_trials; i++)
        {
            std::cout << "Runing the world (" << i+1 << "/" << max_trials << ") ... " << std::endl;
            
            ::gazebo::runWorld(world_,1);
            
            std::cout << "Now verifying if the model is correctly loaded..." << std::endl;
            
            #if GAZEBO_MAJOR_VERSION > 8
            auto model = world_->ModelByName(robot_name);
            #else
            auto model = world_->GetModel(robot_name);
            #endif
            if(model)
            {
                do_exit = true;
                if(th.joinable())
                    th.join();
                std::cout << "Model " << robot_name << " successfully loaded" << std::endl;
                return true;
            }
            std::cout << "Model not yet loaded" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        return false;
    }
    
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
                std::cerr << "Could not get the robot tag in the URDF " << '\n';
                return false;
            }
        }
        else
        {
            std::cerr << "Provided robot element is invalid" << '\n';
            return false;
        }
        
        if (robot_name.empty())
        {
            std::cerr << "Robot name is empty" << '\n';
            return false;
        }
        return true;
    }

    double dt_ = 0.001;
    ::gazebo::physics::WorldPtr world_;
    ::gazebo::physics::ModelPtr model_;
};

} // namespace gazebo
} // namespace orca
