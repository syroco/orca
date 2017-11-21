#pragma once 

#include <orca/rtt_orca/common/RttTaskCommon.h>

namespace rtt_orca
{
namespace task
{
    struct RttGenericTask: public common::RttTaskCommon
    {
        RttGenericTask(RTT::TaskContext* owner,orca::task::GenericTask* comm,const std::string& name)
        : common::RttTaskCommon(owner,comm,name)
        {
            owner->addOperation("setWeight",&orca::task::GenericTask::setWeight,comm,RTT::OwnThread);
            owner->addOperation("getWeight",&orca::task::GenericTask::getWeight,comm,RTT::OwnThread);
            owner->addOperation("insertInProblem",&orca::task::GenericTask::insertInProblem,comm,RTT::OwnThread);
            owner->addOperation("removeFromProblem",&orca::task::GenericTask::removeFromProblem,comm,RTT::OwnThread);
        }
    };
} // namespace task
} // namespace rtt_orca