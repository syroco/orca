#include <orca/rtt_orca/constraint/RttGenericConstraint.h>
#include <orca/common/WrenchData.h>

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
            // int i = ports.size();
            // ports.push_back(new RTT::InputPort<double>());
            // this->addPort("port" + std::to_string(i++),*ports[i]);
        }

        void updateHook()
        {
            if(!this->updateRobotModel())
                return;
            // 
            // int i=0;
            // for(auto &port_wrench : port_wrenches_)
            // {
            //     std::list<orca::common::WrenchData>::iterator it = std::next(wrenches_jac_t_.begin(), i);
            //     if(port_wrench.readNewest(*it) == RTT::NoData)
            //     {
            //         (*it).rows = 0;
            //         (*it).cols = 0;
            //     }
            //     i++;
            // }
            // orca::constraint::DynamicsEquationConstraint::updateJacobianTranspose(wrenches_jac_t_);
            orca::constraint::DynamicsEquationConstraint::update();
            
            // Fill the message to be sent
            // constraint_data_.name = orca::constraint::DynamicsEquationConstraint::getName();
            // constraint_data_.control_var = this->getControlVariable();
            // constraint_data_.rows = this->rows();
            // constraint_data_.cols = this->cols();
            // constraint_data_.constraint_matrix = this->getConstraintMatrix();
            // constraint_data_.lower_bound = this->getLowerBound();
            // constraint_data_.upper_bound = this->getUpperBound();
            
            //port_constraint_.write(constraint_data_);
        }
        // RTT::OperationCaller<int(void)> effectors;
        // std::list<RTT::InputPort<orca::common::WrenchData> > port_wrenches_;
        // std::list<orca::common::WrenchData > wrenches_jac_t_;
        
    };
}
}

ORO_CREATE_COMPONENT(rtt_orca::constraint::RttDynamicsEquationConstraint)
