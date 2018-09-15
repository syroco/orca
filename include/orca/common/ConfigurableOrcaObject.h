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

#include <orca/common/OrcaObject.h>
#include <orca/common/Config.h>

namespace orca
{
namespace common
{
    class ConfigurableOrcaObject : public OrcaObject
    {
    public:
        using Ptr = std::shared_ptr<ConfigurableOrcaObject>;
        
        virtual ~ConfigurableOrcaObject() {}
        /**
        * @brief Creates a config with the name provided.
        */
        ConfigurableOrcaObject(const std::string& name);
        /**
        * @brief Configure the task from YAML/JSON string. It must contain all the required parameters.
        *
        * @return true is all the required parameters are loaded properly.
        */
        bool configureFromString(const std::string& yaml_str);
        /**
        * @brief Configure the task from YAML/JSON file. It must contain all the required parameters.
        *
        * @return true is all the required parameters are loaded properly.
        */
        bool configureFromFile(const std::string& yaml_url);
        /**
        * @brief Returns true if all params added with @addParameter have been set.
        *
        * @return true is all the required parameters are loaded properly.
        */
        bool isConfigured() const;
        /**
        * @brief Add a parameter in the task. Required parameters need to be set before updating.
        */
        void addParameter(const std::string& param_name,ParameterBase* param
                    , ParamPolicy policy = ParamPolicy::Required
                    , std::function<void()> on_loading_success = 0);
        /**
        * @brief Same as #addParameter, but via reference.
        */
        template<class T>
        void addParameter(const std::string& param_name,T& param
                , ParamPolicy policy = ParamPolicy::Required
                , std::function<void()> on_loading_success = 0)
        {
            config_->addParameter(param_name,param,policy,on_loading_success);
        }
        /**
        * @brief Returns a parameter based on his name. nullptr if it does not exists.
        */
        ParameterBase* getParameter(const std::string& param_name);
        /**
        * @brief Output config to std::cout.
        */
        void printConfig() const;
        /**
        * @brief Get the reference to the config
        */
        Config::Ptr config();
    private:
        Config::Ptr config_;
    };
} // namespace common
} // namespace orca
