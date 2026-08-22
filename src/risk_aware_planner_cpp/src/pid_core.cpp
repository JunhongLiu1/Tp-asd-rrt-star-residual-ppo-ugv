#include "risk_aware_planner_cpp/pid_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace risk_aware_planner_cpp
{
namespace
{

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(upper, value));
}

void validateConfig(const PidConfig & config)
{
  const double values[] = {
    config.kp, config.ki, config.kd,
    config.integral_min, config.integral_max,
    config.output_min, config.output_max};
  for (double value : values) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("PID configuration must be finite");
    }
  }
  if (config.kp < 0.0 || config.ki < 0.0 || config.kd < 0.0 ||
    config.integral_min > config.integral_max ||
    config.output_min > config.output_max)
  {
    throw std::invalid_argument("PID configuration is outside its valid range");
  }
}

}  // namespace

PidController::PidController(const PidConfig & config)
{
  setConfig(config);
}

void PidController::setConfig(const PidConfig & config)
{
  validateConfig(config);
  config_ = config;
  reset();
}

void PidController::reset()
{
  integral_state_ = 0.0;
  previous_error_ = 0.0;
  has_previous_error_ = false;
}

PidResult PidController::update(
  double setpoint, double measurement, double dt_sec)
{
  return update(
    setpoint, measurement, dt_sec,
    config_.output_min, config_.output_max);
}

PidResult PidController::update(
  double setpoint, double measurement, double dt_sec,
  double output_min, double output_max)
{
  PidResult result;
  if (!std::isfinite(setpoint) || !std::isfinite(measurement) ||
    !std::isfinite(dt_sec) || dt_sec <= 0.0 ||
    !std::isfinite(output_min) || !std::isfinite(output_max) ||
    output_min > output_max)
  {
    reset();
    return result;
  }

  // Runtime limits let the caller include feed-forward and actuator bounds in
  // the anti-windup decision. They may only tighten the configured PID limits.
  const double effective_output_min = std::max(config_.output_min, output_min);
  const double effective_output_max = std::min(config_.output_max, output_max);
  if (effective_output_min > effective_output_max) {
    reset();
    return result;
  }

  result.valid = true;
  result.error = setpoint - measurement;
  result.proportional = config_.kp * result.error;
  result.derivative = has_previous_error_ ?
    config_.kd * (result.error - previous_error_) / dt_sec : 0.0;

  const double candidate_integral = clamp(
    integral_state_ + result.error * dt_sec,
    config_.integral_min, config_.integral_max);
  const double candidate_output = result.proportional +
    config_.ki * candidate_integral + result.derivative;

  const bool pushes_high =
    candidate_output > effective_output_max && result.error > 0.0;
  const bool pushes_low =
    candidate_output < effective_output_min && result.error < 0.0;
  if (!pushes_high && !pushes_low) {
    integral_state_ = candidate_integral;
  }

  result.integral = config_.ki * integral_state_;
  const double raw_output =
    result.proportional + result.integral + result.derivative;
  result.output = clamp(
    raw_output, effective_output_min, effective_output_max);
  result.saturated = std::abs(result.output - raw_output) > 1.0e-12;
  previous_error_ = result.error;
  has_previous_error_ = true;
  return result;
}

double PidController::integralState() const
{
  return integral_state_;
}

}  // namespace risk_aware_planner_cpp
