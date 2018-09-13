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
            using ParamMap = std::map<std::string, ParameterBase* >;
            
            Config(const std::string& config_name);

            /**
            * @brief Returns the name of the config. Usually the name of the task owning the config.
            */
            const std::string& getName() const;
            /**
            * @brief Returns true if all params added with @addParameter have been set
            *
            * @return true is all the required parameters are loaded properly
            */
            void addParameter(const std::string& param_name,ParameterBase* param
                    , ParamPolicy policy = ParamPolicy::Required
                    , std::function<void()> on_loading_success = 0
                    , std::function<void()> on_loading_failed = 0);

            /**
            * @brief Returns a param via its name.
            * 
            * @param param_name The name of the param (might not exist)
            * @return orca::common::ParameterBase* The param pointer, nullptr if if does not exists
            */
            ParameterBase* getParameter(const std::string& param_name);

            void print() const;

            bool loadFromFile(const std::string& yaml_url);

            bool loadFromString(const std::string& yaml_str);
            
            bool areAllRequiredParametersSet() const;

            const ParamMap& getAllParameters() const;
        private:
            std::string name_;
            ParamMap parameters_;
        };
    } // namespace common
} // namespace orca
