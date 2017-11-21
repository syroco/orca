#include <orca/rtt_orca/task/RttGenericTask.h>

namespace rtt_orca
{
namespace task
{
    class RttCartesianTask: public orca::task::CartesianTask, public task::RttGenericTask
    {
    public:
        RttCartesianTask(const std::string& name)
        : task::RttGenericTask(this,this,name)
        {
            this->addOperation("setBaseFrame",&orca::task::CartesianTask::setBaseFrame,this,RTT::OwnThread);
            this->addOperation("setControlFrame",&orca::task::CartesianTask::setControlFrame,this,RTT::OwnThread);
            this->addOperation("getBaseFrame",&orca::task::CartesianTask::getBaseFrame,this,RTT::OwnThread);
            this->addOperation("getControlFrame",&orca::task::CartesianTask::getControlFrame,this,RTT::OwnThread);
        }

        void updateHook()
        {
            this->updateRobotModel();
            port_cartesian_acceleration_des_.readNewest(this->cart_acc_des_);
            orca::task::CartesianTask::update();
        }

    protected:
        RTT::InputPort<Eigen::Matrix<double,6,1> > port_cartesian_acceleration_des_;
    };
}
}

ORO_CREATE_COMPONENT(rtt_orca::task::RttCartesianTask)
