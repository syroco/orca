#pragma once

#include <memory>
namespace orca
{
    namespace common
    {
        class TaskBase;
    }
}

#define ORCA_REGISTER_CLASS(CLASSNAME) \
    orca::common::Factory()->registerClass(#CLASSNAME,[](){ return std::make_shared<CLASSNAME>() });

namespace orca
{
    namespace common
    {
        class SimpleFactory
        {
            TaskBase::Ptr createPtr(const std::string& class_name)
            {
                return m_[class_name]();
            }
            void registerClass(const std::string& class_name,std::function<TaskBase::Ptr(void)> f)
            {
                m_[class_name] = f;
            }
        private:
            std::map<std::string,std::function<TaskBase::Ptr(void)> > m_;
        };
        
        static SimpleFactory* Factory()
        {
            static SimpleFactory factory;
            return &factory;
        }
    }
}

