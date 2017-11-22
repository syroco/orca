#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/Operation.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/Property.hpp>
#include <rtt/Service.hpp>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/scripting/Scripting.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>

#include <orca/optim/OptimisationVector.h>
#include <orca/optim/ControlVariable.h>
#include <orca/optim/QPSolver.h>
#include <orca/optim/WeightedQPSolver.h>

namespace rtt_orca
{
namespace optim
{
    template<class QP>
    class RttQPSolver: public RTT::TaskContext
    {
    public:
        RttQPSolver(const std::string& name)
        : RTT::TaskContext(name)
        {
            this->addOperation("setPrintLevel",&orca::optim::QPSolver::setPrintLevel,&qp_,RTT::OwnThread);
            this->addOperation("getPrimalSolution",&orca::optim::QPSolver::getPrimalSolution,&qp_,RTT::OwnThread);
            this->addOperation("print",&orca::optim::QPSolver::print,&qp_,RTT::OwnThread);
            this->addAttribute("buildOptimisationProblemTime",build_time_);
            this->addAttribute("solveTime",solve_time_);
            this->provides("output")->addPort("JointTorque",port_torque_out_);
            this->provides("output")->addPort("JointAcceleration",port_acc_out_);
        }
        
        void updateHook()
        {
            if(orca::optim::OptimisationVector().getNrOfDegreesOfFreedom() == 0)
            {
                RTT::log(RTT::Warning) << "[" << RTT::TaskContext::getName() << "] " << "OptimisationVector is not yet initialized" << RTT::endlog();
                return;
            }
            
            static const int trq_idx = orca::optim::OptimisationVector().getIndex(orca::optim::ControlVariable::JointSpaceTorque);
            static const int trq_size = orca::optim::OptimisationVector().getSize(orca::optim::ControlVariable::JointSpaceTorque);
            static const int acc_idx = orca::optim::OptimisationVector().getIndex(orca::optim::ControlVariable::JointSpaceAcceleration);
            static const int acc_size = orca::optim::OptimisationVector().getSize(orca::optim::ControlVariable::JointSpaceAcceleration);
            
            timestamp_ = RTT::os::TimeService::Instance()->getTicks();
            qp_.buildOptimisationProblem();
            build_time_ = RTT::os::TimeService::Instance()->secondsSince( timestamp_ );
            
            timestamp_ = RTT::os::TimeService::Instance()->getTicks();
            int ret = qp_.solve();
            solve_time_ = RTT::os::TimeService::Instance()->secondsSince( timestamp_ );
            
            if( ret == 0 )
            {
                port_torque_out_.write( qp_.getPrimalSolution().segment(trq_idx,trq_size) );
                port_acc_out_.write( qp_.getPrimalSolution().segment(acc_idx,acc_size) );
            }
        }

    protected:
        QP qp_;
        RTT::OutputPort<Eigen::VectorXd> port_torque_out_;
        RTT::OutputPort<Eigen::VectorXd> port_acc_out_;
        RTT::Seconds build_time_;
        RTT::Seconds solve_time_;
        RTT::os::TimeService::ticks timestamp_;
    };
    
}
}

ORO_LIST_COMPONENT_TYPE(rtt_orca::optim::RttQPSolver<orca::optim::WeightedQPSolver>)
ORO_CREATE_COMPONENT_LIBRARY()
