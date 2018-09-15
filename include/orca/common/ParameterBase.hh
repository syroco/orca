#pragma once

#include <orca/utils/Utils.h>

namespace orca
{
namespace common
{
/**
* @brief The ParamPolicy defines if an error should be thrown when trying to do anything with the task
* before all #Required parameters are set at least once.
*/
enum class ParamPolicy
{
    Required = 0
    , Optional
};
/**
* @brief ParameterBase is the public interface to any parameter
*/
class ParameterBase
{
public:
    using Ptr = std::shared_ptr<ParameterBase>;
    
    virtual ~ParameterBase() {}
    
    /**
    * @brief Load the parameter from a YAML string
    * It calls a callback on success
    */
    bool loadFromString(const std::string& s)
    {
        try 
        {
            if(onLoadFromString(s))
            {
                if(on_success_)
                    on_success_();
                return true;
            }
        } catch(std::exception& e) {
            utils::orca_throw(e.what());
        }

        return false;
    }
    virtual void print() const = 0;
    virtual bool isSet() const = 0;
    const std::string& getName() const { return name_; }
    void setName(const std::string& name) { name_ = name; }
    void setRequired(bool is_required) { is_required_ = is_required; }
    bool isRequired() const { return is_required_; }
    /**
    * @brief The callback called if #loadFromString is successful
    */
    void onSuccess(std::function<void(void)> f)
    {
        on_success_ = f;
    }
protected:
    virtual bool onLoadFromString(const std::string& s) = 0;
private:
    std::string name_;
    bool is_required_ = false;
    std::function<void(void)> on_success_;
};

} // namespace common
} // namespace orca