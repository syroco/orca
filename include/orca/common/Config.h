#pragma once
#include <orca/utils/Utils.h>
#include <orca/common/Parameter.h>

namespace orca
{
    namespace common
    {
        /**
        * @brief Represents a set of parameters that can be loaded from a YAML file
        * 
        */
        class Config
        {
        public:
            using Ptr  = std::shared_ptr<Config>;
    
            Config(const std::string& config_name)
            : name_(config_name)
            {
                LOG_DEBUG << "[" << getName() << "] " << "Starting new config";
            }

            /**
            * @brief Returns the name of the config. Usually the name of the task owning the config.
            */
            const std::string& getName() const
            {
                return name_;
            }
            /**
            * @brief Returns true if all params added with @addParameter have been set
            *
            * @return true is all the required parameters are loaded properly
            */
            void addParameter(const std::string& param_name,ParameterBase* param
                    , ParamPolicy policy = ParamPolicy::Required
                    , std::function<void()> on_loading_success = 0
                    , std::function<void()> on_loading_failed = 0)
            {
                if(param_name.empty())
                    utils::orca_throw(utils::Formatter() << "Cannot have an empty parameter name !");
                
                if(!param)
                    utils::orca_throw(utils::Formatter() << "Param is null !");
                
                if(utils::key_exists(parameters_,param_name))
                {
                    LOG_ERROR << "[" << getName() << "] " << "Parameter " << param_name << " already declared !";
                    return;
                }
                param->setName(param_name);
                param->setRequired(policy == ParamPolicy::Required);
                param->onLoadingSuccess(on_loading_success);
                param->onLoadingFailed(on_loading_failed);
                parameters_[param_name] = param;
            }

            /**
            * @brief Returns a param via its name.
            * 
            * @param param_name The name of the param (might not exist)
            * @return orca::common::ParameterBase* The param pointer, nullptr if if does not exists
            */
            ParameterBase* getParameter(const std::string& param_name)
            {
                if(!utils::key_exists(parameters_,param_name))
                {
                    LOG_ERROR << "[" << getName() << "] " << "Parameter " << param_name << " does not exists !";
                    return nullptr;
                }
                return parameters_[param_name];
            }

            void printParameters() const
            {
                if(parameters_.empty())
                {
                    LOG_INFO << "[" << getName() << "] " << "No parameters declared !";
                    return;
                }
                std::stringstream ss;
                ss << "[" << getName() << "] Parameters :\n";
                for(auto p : parameters_)
                {
                    ss << " * " << p.second->getName()
                        << "\n      - is required " << std::boolalpha << p.second->isRequired() 
                        << "\n      - is set " << std::boolalpha << p.second->isSet() << '\n'; 
                }
                LOG_INFO << ss.str();
            }

            bool configureFromFile(const std::string& yaml_url)
            {
                YAML::Node config = YAML::LoadFile(yaml_url);
                
                YAML::Emitter out;
                out << config;
                
                return configureFromString(out.c_str());
            }

            bool configureFromString(const std::string& yaml_str)
            {
                LOG_INFO << "[" << getName() << "] Starting configuring from file";
                if(parameters_.empty())
                {
                    LOG_ERROR << "[" << getName() << "] " << "No parameters declared with addParam!";
                    return false;
                }
                // This can throw an exception
                // Is it good to keep it ?
                YAML::Node config;
                try {
                    config = YAML::Load(yaml_str);
                } catch(std::exception& e) {
                    utils::orca_throw(utils::Formatter() << e.what() << "\nYaml file does not seem to be valid.");
                }
                
                if(!config)
                {
                    return false;
                }
                if(!(config.IsMap() || config.IsSequence()))
                {
                    return false;
                }
                
                auto to_string = [](const YAML::Node& n) -> std::string 
                        { YAML::Emitter out; out << n; return out.c_str(); };

                std::cout << "Configuring from config " << to_string(config) << '\n';
                
                for(auto c : config)
                {
                    std::cout << "Analysing subconfig " << to_string(c.first) << '\n';
                    
                    auto param_name = c.first.as<std::string>();
                    // Special case for the 'type' param
                    if(param_name == "type")
                    {
                        continue;
                    }
                    
                    if(!utils::key_exists(parameters_,param_name))
                    {
                        std::stringstream ss;
                        ss << "[" << getName() << "] " 
                            << "Parameter \"" << param_name << "\" not declared but present in the yaml file\n"
                            << "Did you forget to addParameter() in the component constructor ?\n"
                            << "Declared parameters : \n";
                        for(auto p : parameters_)
                        {
                            ss << " - " << p.second->getName() << '\n';
                        }
                        LOG_WARNING << ss.str();
                        //return false;
                    }
                    else
                    {
                        auto param = parameters_[param_name];
                        if(param->loadFromString(to_string(c.second)))
                        {
                            LOG_INFO << "[" << getName() << "] " << "Parameter \"" << param_name << "\" --> " << c.second;
                        }
                        else
                        {
                            LOG_ERROR << "[" << getName() << "] " << "Could not load \"" << param_name << "\"";
                        }
                    }
                }
                bool is_configured = isConfigured();
                if(!is_configured)
                {
                    printParameters();
                    std::stringstream ss;

                    for(auto p : parameters_)
                    {
                        LOG_WARNING_IF(p.second->isRequired() && ! p.second->isSet()) << "[" << getName() << "] "
                                << "Required parameter \"" << p.second->getName() 
                                << "\" is not set";
                    }
                    LOG_WARNING << "[" << getName() << "] " << "Configuring failed";
                }
                
                LOG_INFO_IF(is_configured) << "[" << getName() << "] " << "Sucessfully configured";
                
                return is_configured;
            }
            
            bool isConfigured() const
            {
                bool ok = true;
                for(auto p : parameters_)
                {
                    if(p.second->isRequired())
                    {
                        ok &= p.second->isSet();
                    }
                }
                return ok;
            }

        private:
            std::string name_;
            std::map<std::string, ParameterBase* > parameters_;
        };
    } // namespace common
} // namespace orca
