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

        }
    };
} // namespace constraint
} // namespace rtt_orca
