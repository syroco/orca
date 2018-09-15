#pragma once
#include <orca/utils/Utils.h>
#include <orca/common/OrcaObject.h>
#include <orca/common/Parameter.h>

namespace orca
{
    namespace common
    {
        /**
        * @brief Represents a set of parameters that can be loaded from a YAML file
        * 
        */
        class Config : public OrcaObject
        {
        public:
            using Ptr  = std::shared_ptr<Config>;
            using ParamMap = std::map<std::string, ParameterBase* >;
            
            Config(const std::string& config_name);

            /**
            * @brief Returns true if all params added with @addParameter have been set
            *
            * @return true is all the required parameters are loaded properly
            */
            void addParameter(const std::string& param_name,ParameterBase* param
                    , ParamPolicy policy = ParamPolicy::Required
                    , std::function<void()> on_loading_success = 0);
            
            // TODO: Figure out if we can move the Parameter class inside
            // the config, so you can have :
            // struct MyTask{ my_param; }
            // MyTask() { addParameter("my_param",my_param); } --> init with ref
            //
            // template<class T>
            // void addParameter(const std::string& param_name,T& param
            //         , ParamPolicy policy = ParamPolicy::Required
            //         , std::function<void()> on_loading_success = 0)
            // {
            //     this->addParameter(param_name,new Parameter<T>(param),policy,on_loading_success);
            // }

            /**
            * @brief Returns a param via its name.
            * 
            * @param param_name The name of the param (might not exist)
            * @return orca::common::ParameterBase* The param pointer, nullptr if if does not exists
            */
            ParameterBase* getParameter(const std::string& param_name);
            /**
            * @brief Print all parameters to std::cout
            */
            void print() const;

            bool loadFromFile(const std::string& yaml_url);

            bool loadFromString(const std::string& yaml_str);
            
            bool areAllRequiredParametersSet() const;

            const ParamMap& getAllParameters() const;
            
            void onSuccess(std::function<void()> f);
        private:
            std::string fileToString(const std::string& yaml_url);
            ParamMap parameters_;
            std::function<void()> on_success_;
        };
    } // namespace common
} // namespace orca
