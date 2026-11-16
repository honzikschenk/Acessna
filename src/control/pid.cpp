#include "acessna/control/pid.hpp"

#include <algorithm>

namespace acessna {

Pid::Pid(const PidGains& gains, const PidLimits& limits) : gains_(gains), limits_(limits) {}

void Pid::reset(double output) {
  integrator_ = std::clamp(output, limits_.integrator_min, limits_.integrator_max);
  prev_error_ = 0.0;
  has_prev_ = false;
}

double Pid::update(double error, double dt) {
  const double error_rate = (has_prev_ && dt > 0.0) ? (error - prev_error_) / dt : 0.0;
  prev_error_ = error;
  has_prev_ = true;
  return update(error, error_rate, dt);
}

double Pid::update(double error, double error_rate, double dt) {
  const double p = gains_.kp * error;
  const double d = gains_.kd * error_rate;

  const double step = gains_.ki * error * dt;
  const double integrated =
      std::clamp(integrator_ + step, limits_.integrator_min, limits_.integrator_max);

  // Anti-windup
  const double unsaturated = p + integrated + d;
  const bool winds_up = (unsaturated > limits_.output_max && step > 0.0) ||
                        (unsaturated < limits_.output_min && step < 0.0);
  if (!winds_up) integrator_ = integrated;

  return std::clamp(p + integrator_ + d, limits_.output_min, limits_.output_max);
}

}  // namespace acessna
