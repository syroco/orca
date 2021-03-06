#include "orca/common/PIDController.h"
#include <orca/common/Factory.h>

using namespace orca::common;

PIDController::PIDController(const std::string& name)
: ConfigurableOrcaObject(name)
{
    this->addParameter( "dimension", &dimension_ ,ParamPolicy::Optional);
    this->addParameter( "p_gain", &p_gain_ );
    this->addParameter( "i_gain", &i_gain_ ,ParamPolicy::Optional);
    this->addParameter( "d_gain", &d_gain_ ,ParamPolicy::Optional);
    this->addParameter( "windup_limit", &windup_limit_ ,ParamPolicy::Optional);
    this->onConfigureSuccess([&](){ resize(p_gain_.get().size()); });
}

void PIDController::resize(int dim)
{
    if(dim <= 0)
    {
        LOG_ERROR << "Dimension as to be > 0";
        return;
    }
    if(dimension_.get() != dim)
    {
        dimension_ = dim;
        if(!p_gain_.isSet() || p_gain_.get().size() != dim)
            p_gain_ = Eigen::VectorXd::Zero(dim);
        
        if(!i_gain_.isSet() || i_gain_.get().size() != dim)
            i_gain_ = Eigen::VectorXd::Zero(dim);
        
        if(!d_gain_.isSet() || d_gain_.get().size() != dim)
            d_gain_ = Eigen::VectorXd::Zero(dim);
        
        if(i_error_.size() != dim)
            i_error_ = Eigen::VectorXd::Zero(dim);
        
        if(d_error_.size() != dim)
            d_error_ = Eigen::VectorXd::Zero(dim);
        
        if(cmd_.size() != dim)
            cmd_ = Eigen::VectorXd::Zero(dim);
        
        if(!windup_limit_.isSet() || windup_limit_.get().size() != dim)
            windup_limit_ = Eigen::VectorXd::Constant(dim,math::Infinity);
    }
}

void PIDController::setProportionalGain(const Eigen::VectorXd& P_gain)
{
    p_gain_ = P_gain;
}

void PIDController::setProportionalGain(const std::vector<double>& P_gain)
{
    setProportionalGain( Eigen::VectorXd::Map(P_gain.data(),P_gain.size()) );
}

const Eigen::VectorXd& PIDController::P() const
{
    return p_gain_.get();
}

void PIDController::setIntegralGain(const Eigen::VectorXd& I_gain)
{
    i_gain_ = I_gain;
}

void PIDController::setIntegralGain(const std::vector<double>& I_gain)
{
    setIntegralGain( Eigen::VectorXd::Map(I_gain.data(),I_gain.size()) );
}

void PIDController::setWindupLimit(const Eigen::VectorXd& windup_lim)
{
    windup_limit_ = windup_lim;
}

void PIDController::setWindupLimit(const std::vector<double>& windup_lim)
{
    setWindupLimit( Eigen::VectorXd::Map(windup_lim.data(),windup_lim.size()) );
}

const Eigen::VectorXd& PIDController::windupLimit()
{
    return windup_limit_.get();
}

const Eigen::VectorXd& PIDController::I() const
{
    return i_gain_.get();
}

void PIDController::setDerivativeGain(const Eigen::VectorXd& D_gain)
{
    d_gain_ = D_gain;
}

void PIDController::setDerivativeGain(const std::vector<double>& D_gain)
{
    setDerivativeGain( Eigen::VectorXd::Map(D_gain.data(),D_gain.size()) );
}

const Eigen::VectorXd& PIDController::D() const
{
    return d_gain_.get();
}

const Eigen::VectorXd& PIDController::computeCommand(const Eigen::VectorXd& Error
                                  , const Eigen::VectorXd& DError
                                  , double dt)
{
    i_error_ += dt * Error;
    // saturate integral_term
    i_error_.cwiseMin(   windupLimit() );
    i_error_.cwiseMax( - windupLimit() );

    cmd_.noalias() = p_gain_.get().asDiagonal() * Error + i_gain_.get().asDiagonal() * i_error_ + d_gain_.get().asDiagonal() * DError;
    return cmd_;
}

const Eigen::VectorXd& PIDController::computeCommand(const Eigen::VectorXd& Error, double dt)
{
    i_error_ += dt * Error;
    // saturate integral_term
    i_error_.cwiseMin(   windupLimit() );
    i_error_.cwiseMax( - windupLimit() );

    d_error_ = Error / dt;

    cmd_.noalias() = p_gain_.get().asDiagonal() * Error + i_gain_.get().asDiagonal() * i_error_ + d_gain_.get().asDiagonal() * d_error_;
    return cmd_;
}

void PIDController::print() const
{
    std::cout << "[PID]" << '\n';
    std::cout << "      Dimension " << p_gain_.get().size() << '\n';
    std::cout << "      P " << p_gain_.get().transpose() << '\n';
    std::cout << "      I " << i_gain_.get().transpose() << '\n';
    std::cout << "      D " << d_gain_.get().transpose() << '\n';
    std::cout << "      Windup limit " << windup_limit_.get().transpose() << '\n';
    std::cout << "      Ierror " << i_error_.transpose() << '\n';
    std::cout << "      derror " << d_error_.transpose() << '\n';
    std::cout << "      Command " << cmd_.transpose() << '\n';
}

ORCA_REGISTER_CLASS(orca::common::PIDController)
