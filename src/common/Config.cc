#include <orca/common/Config.h>

using namespace orca::common;
using namespace orca::utils;

Config::Config(const std::string& config_name)
: OrcaObject(config_name)
{}

Config::~Config()
{
    while(!parameters_to_delete_.empty())
    {
        delete parameters_to_delete_.front();
        parameters_to_delete_.pop_front();
    }
}


void Config::addParameter(const std::string& param_name,ParameterBase* param
        , ParamPolicy policy /*= ParamPolicy::Required*/
        , std::function<void()> on_loading_success /*= 0*/)
{
    if(param_name.empty())
        orca_throw(Formatter() << "Cannot have an empty parameter name !");
    
    if(!param)
        orca_throw(Formatter() << "Param is null !");
    
    if(key_exists(parameters_,param_name))
    {
        LOG_ERROR << "[" << getName() << "] " << "Parameter " << param_name << " already declared !";
        return;
    }
    param->setName(param_name);
    param->setRequired(policy == ParamPolicy::Required);
    param->onSuccess(on_loading_success);
    parameters_[param_name] = param;
}


ParameterBase* Config::getParameter(const std::string& param_name)
{
    if(!key_exists(parameters_,param_name))
    {
        LOG_ERROR << "[" << getName() << "] " << "Parameter " << param_name << " does not exists !";
        return nullptr;
    }
    return parameters_[param_name];
}

void Config::print() const
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

std::string Config::fileToString(const std::string& yaml_url)
{
    YAML::Node config;
    try {
        config = YAML::LoadFile(yaml_url);
    } catch(std::exception& e) {
        orca_throw(Formatter() << e.what() << "\nYaml url [" << yaml_url <<"] does not seem to be valid.");
    }

    YAML::Emitter out;
    out << config;
    
    return out.c_str();
}

bool Config::loadFromFile(const std::string& yaml_url)
{
    return loadFromString(fileToString(yaml_url));
}

void Config::onSuccess(std::function<void()> f)
{
    on_success_ = f;
}

bool Config::loadFromString(const std::string& yaml_str)
{
    if(config_loaded_)
    {
        LOG_ERROR << "[" << getName() << "] " << "Config is already loaded";
        return true;
    }
    if(yaml_str.empty())
    {
        LOG_ERROR << "[" << getName() << "] " << "Provided config is empty";
        return false;
    }
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
        orca_throw(Formatter() << e.what() << "\nYaml file does not seem to be valid, usually bad formatting.\n\n" << yaml_str);
    }
    // For now we pass the node value to the config. Maybe add the key also to check if it corresponds to
    // the config name ?
//                 auto config_name = config.first.as<std::string>();
//                 if(config_name != this->getName())
//                 {
//                     orca_throw(Formatter() << "YAML Node name (" << config_name << ") does not match with config name (" << name_ ")");
//                 }
    
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
        // Check if key exists, send a warning otherwise
        if(!key_exists(parameters_,param_name))
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
    
    // Check if all the required parameters are present in the config file
    bool all_present = true;
    for(auto p : parameters_)
    {
        if(p.second->isRequired())
        {
            if(!config[p.first])
            {
                if(p.second->isSet())
                {
                    LOG_WARNING << "[" << getName() << "] " << "Required parameter '" << p.first 
                        << "' is set but not present in config file.\n"
                        << "Declare the parameter to remove this warning.";
                    p.second->print();
                }
                else
                {
                    LOG_WARNING << "[" << getName() << "] " << "Required parameter '" << p.first << "' is not present in config file";
                    all_present = false;
                }
            }
        }
    }
    
    // All required parameters should be in the config file
    if(!all_present)
    {
        LOG_ERROR << "[" << getName() << "] " << "Configuring failed";
        return false;
    }
    
    // Check if all required parameters are set (can be from file or calling specific methods)
    bool all_set = areAllRequiredParametersSet();
    if(!all_set)
    {
        for(auto p : parameters_)
        {
            LOG_WARNING_IF(p.second->isRequired() && ! p.second->isSet()) << "[" << getName() << "] "
                    << "Required parameter '" << p.second->getName() 
                    << "' is not set yet. It has to be set before any update.";
        }
    }
    
    LOG_INFO << "[" << getName() << "] " << "Sucessfully configured";

    if(on_success_)
        on_success_();

    config_loaded_ = true;
    return true;
}

bool Config::areAllRequiredParametersSet() const
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

const Config::ParamMap& Config::getAllParameters() const
{
    return parameters_;
}
