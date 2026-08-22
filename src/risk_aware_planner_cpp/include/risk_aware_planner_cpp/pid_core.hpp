#ifndef RISK_AWARE_PLANNER_CPP__PID_CORE_HPP_
#define RISK_AWARE_PLANNER_CPP__PID_CORE_HPP_

namespace risk_aware_planner_cpp
{

struct PidConfig
{
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
  double integral_min{0.0};
  double integral_max{0.0};
  double output_min{0.0};
  double output_max{0.0};
};

struct PidResult
{
  bool valid{false};
  bool saturated{false};
  double error{0.0};
  double proportional{0.0};
  double integral{0.0};
  double derivative{0.0};
  double output{0.0};
};

class PidController
{
public:
  explicit PidController(const PidConfig & config = PidConfig{});

  void setConfig(const PidConfig & config);
  void reset();
  PidResult update(double setpoint, double measurement, double dt_sec);
  PidResult update(
    double setpoint, double measurement, double dt_sec,
    double output_min, double output_max);

  double integralState() const;

private:
  PidConfig config_;
  double integral_state_{0.0};
  double previous_error_{0.0};
  bool has_previous_error_{false};
};

}  // namespace risk_aware_planner_cpp

#endif  // RISK_AWARE_PLANNER_CPP__PID_CORE_HPP_
