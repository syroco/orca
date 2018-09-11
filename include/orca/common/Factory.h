#pragma once

#include <memory>
#include <map>
#include <functional>

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
        public:
            std::shared_ptr<TaskBase> createPtr(const std::string& class_name)
            {
                return m_[class_name]();
            }
            void registerClass(const std::string& class_name,std::function<std::shared_ptr<TaskBase>(void)> f)
            {
                m_[class_name] = f;
            }
        private:
            std::map<std::string,std::function<std::shared_ptr<TaskBase>(void)> > m_;
        };
        
        static SimpleFactory* Factory()
        {
            static SimpleFactory factory;
            return &factory;
        }
    }
}

