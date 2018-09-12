#pragma once

#include <memory>
#include <map>
#include <functional>
#include <orca/utils/Utils.h>
#include <orca/common/TaskBase.h>

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
            std::shared_ptr<TaskBase> createPtr(const std::string& instance_name,const std::string& class_name)
            {
                if(instance_name.empty())
                {
                    std::cout << "[Factory] Instance name is empty !" << '\n';
                    return nullptr;
                }
                
                std::cout << "[Factory] Instanciating " << instance_name << " class of type " << class_name << '\n';
                if(!utils::key_exists(m_,class_name))
                {
                    printAvailableClasses();
                    return nullptr;
                }
                auto c = m_[class_name](instance_name);
                if(!c)
                {
                    printAvailableClasses();
                    return nullptr;
                }
                std::cout << "[Factory] " << instance_name << " of type " << class_name << " successfully created" << '\n';
                return c;
            }
            bool registerClass(const std::string& class_name,std::function<std::shared_ptr<TaskBase>(const std::string&)> f)
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
            std::map<std::string,std::function<std::shared_ptr<TaskBase>(const std::string&)> > m_;
        };
        
    } // namespace common
} // namespace orca


#define ORCA_REGISTER_CLASS(CLASSNAME) \
namespace { \
    bool ok = ::orca::common::Factory::Instance()->registerClass(#CLASSNAME,[](const std::string& name) { return std::make_shared<CLASSNAME>(name); }); \
};
