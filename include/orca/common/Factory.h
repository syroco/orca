#pragma once

#include <memory>
#include <map>
#include <functional>
#include <orca/utils/Utils.h>
#include <orca/common/ConfigurableOrcaObject.h>

namespace orca
{
    namespace common
    {
        class Factory
        {
        public:
            static Factory* Instance()
            {
                static Factory factory;
                return &factory;
            }
            std::shared_ptr<ConfigurableOrcaObject> createPtr(const std::string& instance_name,const std::string& class_name)
            {
                if(instance_name.empty())
                {
                    std::cout << "[Factory] Instance name is empty !" << '\n';
                    return nullptr;
                }
                
                if(!utils::key_exists(m_,class_name))
                {
                    std::cout << "[Factory] Class name  '" << class_name << "' is not present in the factory." <<  '\n';
                    printAvailableClasses();
                    std::cout << "[Factory] Did you forget to add ORCA_REGISTER_CLASS(" << class_name << ") at the end to the class cpp file ?" <<  '\n';
                    return nullptr;
                }
                auto c = m_[class_name](instance_name);
                if(!c)
                {
                    printAvailableClasses();
                    return nullptr;
                }
                std::cout << "[Factory] '" << instance_name << "' of type " << class_name << " successfully created" << '\n';
                return c;
            }
            bool registerClass(const std::string& class_name,std::function<std::shared_ptr<ConfigurableOrcaObject>(const std::string&)> f)
            {
                if(utils::key_exists(m_,class_name))
                    return false;
                std::cout << "[Factory] " << "Successfully registered class " << class_name << '\n';
                m_[class_name] = f;
                return true;
            }
            void printAvailableClasses()
            {
                if(m_.empty())
                {
                    std::cout << "[Factory] Factory is empty !" << '\n';
                    return;
                }
                std::cout << "[Factory] Available classes : " << '\n';
                for(auto c : m_)
                {
                    std::cout << " - " << c.first << '\n';
                }
            }
        private:
            std::map<std::string,std::function<std::shared_ptr<ConfigurableOrcaObject>(const std::string&)> > m_;
        };
        
    } // namespace common
} // namespace orca


#define ORCA_REGISTER_CLASS(CLASSNAME,...) \
namespace { \
    bool ok##__VA_ARGS__ = ::orca::common::Factory::Instance()->registerClass(#CLASSNAME,[](const std::string& name) { return std::make_shared<CLASSNAME>(name); }); \
};
