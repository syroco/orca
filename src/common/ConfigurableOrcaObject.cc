#include <orca/common/ConfigurableOrcaObject.h>

using namespace orca::common;

ConfigurableOrcaObject::ConfigurableOrcaObject(const std::string& name)
: OrcaObject(name)
, config_(std::make_shared<Config>(name))
{
    
}

void ConfigurableOrcaObject::addParameter(const std::string& param_name,ParameterBase* param
                    , ParamPolicy policy /*= ParamPolicy::Required*/
                    , std::function<void()> on_loading_success /*= 0*/
                    , std::function<void()> on_loading_failed /*= 0*/)
{
    config_->addParameter(param_name,param,policy,on_loading_success,on_loading_failed);
}

ParameterBase* ConfigurableOrcaObject::getParameter(const std::string& param_name)
{
    return config_->getParameter(param_name);
}

void ConfigurableOrcaObject::printConfig() const
{
    config_->print();
}

bool ConfigurableOrcaObject::configureFromFile(const std::string& yaml_url)
{
    return config_->loadFromFile(yaml_url);
}

bool ConfigurableOrcaObject::configureFromString(const std::string& yaml_str)
{
    return config_->loadFromString(yaml_str);
}

bool ConfigurableOrcaObject::isConfigured() const
{
    return config_->areAllRequiredParametersSet();
}
