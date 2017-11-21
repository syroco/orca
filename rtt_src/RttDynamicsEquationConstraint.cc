#include <orca/rtt_orca/constraint/RttGenericConstraint.h>

namespace rtt_orca
{
namespace constraint
{
    class RttDynamicsEquationConstraint
    : public orca::constraint::DynamicsEquationConstraint
    , public constraint::RttGenericConstraint
    {
    public:
        RttDynamicsEquationConstraint(const std::string& name)
        : constraint::RttGenericConstraint(this,this,name)
        {
        }

        void updateHook()
        {
            this->updateRobotModel();
            orca::constraint::DynamicsEquationConstraint::update();
        }
    };
}
}

ORO_CREATE_COMPONENT(rtt_orca::constraint::RttDynamicsEquationConstraint)
