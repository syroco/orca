#include <orca/rtt_orca/task/RttGenericTask.h>

namespace rtt_orca
{
namespace task
{
    class RttWrenchTask: public orca::task::WrenchTask, public task::RttGenericTask
    {
    public:
        RttWrenchTask(const std::string& name)
        : task::RttGenericTask(this,this,name)
        {
            this->addOperation("insertInProblem",&orca::task::GenericTask::insertInProblem,this,RTT::OwnThread);
            this->addOperation("removeFromProblem",&orca::task::GenericTask::removeFromProblem,this,RTT::OwnThread);
            this->addOperation("setWeight",&orca::task::GenericTask::setWeight,this,RTT::OwnThread);
            this->addOperation("getWeight",&orca::task::GenericTask::getWeight,this,RTT::OwnThread);
            
            this->addOperation("setBaseFrame",&orca::task::WrenchTask::setBaseFrame,this,RTT::OwnThread);
            this->addOperation("setControlFrame",&orca::task::WrenchTask::setControlFrame,this,RTT::OwnThread);
            this->addOperation("getBaseFrame",&orca::task::WrenchTask::getBaseFrame,this,RTT::OwnThread);
            this->addOperation("getControlFrame",&orca::task::WrenchTask::getControlFrame,this,RTT::OwnThread);
            
            this->provides("pid")->addOperation("setProportionalGain",&orca::common::PIDController<6>::setProportionalGain,&this->pid(),RTT::OwnThread);
            this->provides("pid")->addOperation("setDerivativeGain",&orca::common::PIDController<6>::setDerivativeGain,&this->pid(),RTT::OwnThread);
            
            this->addOperation("setCurrent",&orca::task::WrenchTask::setCurrent,this,RTT::OwnThread);
            this->addOperation("setDesired",&orca::task::WrenchTask::setDesired,this,RTT::OwnThread);
        }

        void updateHook()
        {
            this->updateRobotModel();
            port_wrench_des_.readNewest(this->wrench_des_);
            port_current_wrench_.readNewest(this->current_wrench_);
            orca::task::WrenchTask::update();
        }

    protected:
        RTT::InputPort<Eigen::Matrix<double,6,1> > port_wrench_des_;
        RTT::InputPort<Eigen::Matrix<double,6,1> > port_current_wrench_;
    };
}
}

ORO_CREATE_COMPONENT(rtt_orca::task::RttWrenchTask)
