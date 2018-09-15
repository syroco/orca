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
            virtual ~Config();
            /**
            * @brief Returns true if all params added with @addParameter have been set
            *
            * @return true is all the required parameters are loaded properly
            */
            void addParameter(const std::string& param_name,ParameterBase* param
                    , ParamPolicy policy = ParamPolicy::Required
                    , std::function<void()> on_loading_success = 0);

            template<class T>
            void addParameter(const std::string& param_name,T& param
                    , ParamPolicy policy = ParamPolicy::Required
                    , std::function<void()> on_loading_success = 0)
            {
                parameters_to_delete_.push_back(new Parameter<T>(param));
                this->addParameter(param_name,parameters_to_delete_.back(),policy,on_loading_success);
            }

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
            /**
            * @brief Configure the task from YAML/JSON file. It must contain all the required parameters.
            *
            * @return true is all the required parameters are loaded properly
            */
            bool loadFromFile(const std::string& yaml_url);
            /**
            * @brief Configure the task from YAML/JSON string. It must contain all the required parameters.
            *
            * @return true is all the required parameters are loaded properly
            */
            bool loadFromString(const std::string& yaml_str);
            
            bool areAllRequiredParametersSet() const;

            const ParamMap& getAllParameters() const;
            
            void onSuccess(std::function<void()> f);
        private:
            std::string fileToString(const std::string& yaml_url);
            ParamMap parameters_;
            std::list<ParameterBase*> parameters_to_delete_;
            std::function<void()> on_success_;
            bool config_loaded_ = false;
        };
    } // namespace common
} // namespace orca
