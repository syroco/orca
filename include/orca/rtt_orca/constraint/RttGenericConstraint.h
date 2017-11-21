#pragma once 

#include <orca/rtt_orca/common/RttTaskCommon.h>

namespace rtt_orca
{
namespace constraint
{
    struct RttGenericConstraint: public common::RttTaskCommon
    {
        RttGenericConstraint(RTT::TaskContext* owner,orca::constraint::GenericConstraint* comm,const std::string& name)
        : common::RttTaskCommon(owner,comm,name)
        {
            owner->addOperation("desactivate",&orca::constraint::GenericConstraint::desactivate,comm,RTT::OwnThread);
            owner->addOperation("activate",&orca::constraint::GenericConstraint::activate,comm,RTT::OwnThread);
            owner->addOperation("isActivated",&orca::constraint::GenericConstraint::isActivated,comm,RTT::OwnThread);
            owner->addOperation("insertInProblem",&orca::constraint::GenericConstraint::insertInProblem,comm,RTT::OwnThread);
            owner->addOperation("removeFromProblem",&orca::constraint::GenericConstraint::removeFromProblem,comm,RTT::OwnThread);
        }
    };
} // namespace constraint
} // namespace rtt_orca
