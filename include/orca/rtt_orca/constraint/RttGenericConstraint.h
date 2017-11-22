#pragma once 

#include <orca/rtt_orca/common/RttTaskCommon.h>
//#include <orca/constraint/ConstraintData.h>

namespace rtt_orca
{
namespace constraint
{
    struct RttGenericConstraint: public common::RttTaskCommon
    {
        RttGenericConstraint(RTT::TaskContext* owner,orca::constraint::GenericConstraint* comm,const std::string& name)
        : common::RttTaskCommon(owner,comm,name)
        {
            //this->provides("output")->addPort("constraintData",port_constraint_);
        }
        //RTT::OutputPort<orca::constraint::ConstraintData> port_constraint_;
        //orca::constraint::ConstraintData constraint_data_;
    };
} // namespace constraint
} // namespace rtt_orca
