#include "orca/common/PIDController.h"

using namespace orca::common;

PIDController::PIDController(unsigned int dim /*= 0*/)
{
    if(dim > 0)
        resize(dim);
}

void PIDController::resize(unsigned int dim)
{
    if(dim > 0)
    {
        p_gain_.setZero(dim);
        i_gain_.setZero(dim);
        d_gain_.setZero(dim);
        i_error_.setZero(dim);
        d_error_.setZero(dim);
        cmd_.setZero(dim);
        windup_limit_.resize(dim);
        windup_limit_.setConstant( math::Infinity );
    }
    else
    {
        LOG_ERROR << "Dimension as to be > 0";
    }
}

void PIDController::setProportionalGain(const Eigen::VectorXd& P_gain)
{
    utils::assertSize(P_gain,p_gain_);
    p_gain_ = P_gain;
}

const Eigen::VectorXd& PIDController::P() const
{
    return p_gain_;
}

void PIDController::setIntegralGain(const Eigen::VectorXd& I_gain)
{
    utils::assertSize(I_gain,i_gain_);
    i_gain_ = I_gain;
}

void PIDController::setWindupLimit(const Eigen::VectorXd& windup_lim)
{
    utils::assertSize(windup_lim,windup_limit_);
    windup_limit_ = windup_lim;
}

const Eigen::VectorXd& PIDController::windupLimit()
{
    return windup_limit_;
}

const Eigen::VectorXd& PIDController::I() const
{
    return i_gain_;
}

void PIDController::setDerivativeGain(const Eigen::VectorXd& D_gain)
{
    utils::assertSize(D_gain,d_gain_);
    d_gain_ = D_gain;
}

const Eigen::VectorXd& PIDController::D() const
{
    return d_gain_;
}

const Eigen::VectorXd& PIDController::computeCommand(const Eigen::VectorXd& Error
                                  , const Eigen::VectorXd& DError
                                  , double dt)
{
    i_error_ += dt * Error;
    // saturate integral_term
    i_error_.cwiseMin(   windupLimit() );
    i_error_.cwiseMax( - windupLimit() );

    cmd_.noalias() = p_gain_.asDiagonal() * Error + i_gain_.asDiagonal() * i_error_ + d_gain_.asDiagonal() * DError;
    return cmd_;
}

const Eigen::VectorXd& PIDController::computeCommand(const Eigen::VectorXd& Error, double dt)
{
    i_error_ += dt * Error;
    // saturate integral_term
    i_error_.cwiseMin(   windupLimit() );
    i_error_.cwiseMax( - windupLimit() );

    d_error_ = Error / dt;

    cmd_.noalias() = p_gain_.asDiagonal() * Error + i_gain_.asDiagonal() * i_error_ + d_gain_.asDiagonal() * d_error_;
    return cmd_;
}

const void PIDController::print() const
{
    std::cout << "[PID]" << '\n';
    std::cout << "      Dimension " << p_gain_.size() << '\n';
    std::cout << "      P " << p_gain_.transpose() << '\n';
    std::cout << "      I " << i_gain_.transpose() << '\n';
    std::cout << "      D " << d_gain_.transpose() << '\n';
    std::cout << "      Windup limit " << windup_limit_.transpose() << '\n';
    std::cout << "      Ierror " << i_error_.transpose() << '\n';
    std::cout << "      derror " << d_error_.transpose() << '\n';
    std::cout << "      Command " << cmd_.transpose() << '\n';
}
