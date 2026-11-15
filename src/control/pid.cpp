#include "acessna/control/pid.hpp"

namespace acessna {

Pid::Pid(const PidGains& gains, const PidLimits& limits) : gains_(gains), limits_(limits) {}

void Pid::reset(double output) {
  (void)output;
  integrator_ = 0.0;  // TODO: preload so the first output ~= `output`
  prev_error_ = 0.0;
  has_prev_ = false;
}

double Pid::update(double error, double dt) {
  (void)error;
  (void)dt;
  // TODO: error rate from (error - prev_error_) / dt, zero on the first call after reset
  // TODO: forward to update(error, error_rate, dt)
  return 0.0;
}

double Pid::update(double error, double error_rate, double dt) {
  (void)error;
  (void)error_rate;
  (void)dt;
  // TODO: P term
  // TODO: integrate ki * error * dt, clamp to the integrator limits
  // TODO: D term from error_rate
  // TODO: clamp the output, stop integrating while saturated (anti-windup)
  return 0.0;
}

}  // namespace acessna
