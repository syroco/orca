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

#include <orca/orca.h>
#include <orca/rtt_orca/robot/RobotModelHelper.h>

namespace rtt_orca
{
namespace task
{
    class RttCartesianTask: public orca::task::CartesianTask, public RTT::TaskContext
    {
    public:
        RttCartesianTask(const std::string& name)
        : RTT::TaskContext(name)
        , robotHelper_(this,this,this->robot())
        {
            orca::task::CartesianTask::setName(name);
            this->addOperation("insertInProblem",&orca::task::GenericTask::insertInProblem,this,RTT::OwnThread);
            this->addOperation("removeFromProblem",&orca::task::GenericTask::removeFromProblem,this,RTT::OwnThread);
            
            this->addOperation("setBaseFrame",&orca::task::CartesianTask::setBaseFrame,this,RTT::OwnThread);
            this->addOperation("setControlFrame",&orca::task::CartesianTask::setControlFrame,this,RTT::OwnThread);
            this->addOperation("getBaseFrame",&orca::task::CartesianTask::getBaseFrame,this,RTT::OwnThread);
            this->addOperation("getControlFrame",&orca::task::CartesianTask::getControlFrame,this,RTT::OwnThread);
        }

        bool configureHook()
        {
            robotHelper_.configureRobotPorts();
            return true;
        }

        void updateHook()
        {
            robotHelper_.updateRobotModel();
            port_cartesian_acceleration_des_.readNewest(this->cart_acc_des_);
            orca::task::CartesianTask::update();
        }

    protected:
        robot::RobotModelHelper robotHelper_;
        RTT::InputPort<Eigen::Matrix<double,6,1> > port_cartesian_acceleration_des_;
    };
}
}

ORO_CREATE_COMPONENT(rtt_orca::task::RttCartesianTask)
