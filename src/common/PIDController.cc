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
    this->config()->onSuccess([&](){ dimension_.set(p_gain_.get().size()); });
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
        dimension_.set(dim);
        p_gain_.get().setZero(dim);
        i_gain_.get().setZero(dim);
        d_gain_.get().setZero(dim);
        i_error_.setZero(dim);
        d_error_.setZero(dim);
        cmd_.setZero(dim);
        windup_limit_ = Eigen::VectorXd::Constant(dim,math::Infinity);
    }
}

void PIDController::setProportionalGain(const Eigen::VectorXd& P_gain)
{
    utils::assertSize(P_gain,p_gain_.get());
    p_gain_ = P_gain;
}

const Eigen::VectorXd& PIDController::P() const
{
    return p_gain_.get();
}

void PIDController::setIntegralGain(const Eigen::VectorXd& I_gain)
{
    utils::assertSize(I_gain,i_gain_.get());
    i_gain_ = I_gain;
}

void PIDController::setWindupLimit(const Eigen::VectorXd& windup_lim)
{
    utils::assertSize(windup_lim,windup_limit_.get());
    windup_limit_ = windup_lim;
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
    utils::assertSize(D_gain,d_gain_.get());
    d_gain_ = D_gain;
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
