#pragma once

#include <memory>
#include <map>
#include <functional>
#include <orca/common/TaskBase.h>

// namespace orca
// {
//     namespace common
//     {
//         class TaskBase;
//     }
// }

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


// #define ORCA_REGISTER_CLASS(CLASSNAME) \
//     orca::common::Factory()->registerClass(#CLASSNAME,[]() -> std::shared_ptr<TaskBase> { return std::make_shared<CLASSNAME>(); });
//     
#define ORCA_REGISTER_CLASS(CLASSNAME) \
namespace { \
    double a = 1.0; \
};
