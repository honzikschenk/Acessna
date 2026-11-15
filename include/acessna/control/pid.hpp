// PID controller
#pragma once

#include <limits>

namespace acessna {

struct PidGains {
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
};

struct PidLimits {
  double output_min = -std::numeric_limits<double>::infinity();
  double output_max = std::numeric_limits<double>::infinity();
  double integrator_min = -std::numeric_limits<double>::infinity();
  double integrator_max = std::numeric_limits<double>::infinity();
};

class Pid {
 public:
  Pid() = default;
  explicit Pid(const PidGains& gains, const PidLimits& limits = {});

  // Clears the integrator and derivative history. `output` preloads the integrator
  void reset(double output = 0.0);

  double update(double error, double dt);

  double update(double error, double error_rate, double dt);

  void set_gains(const PidGains& gains) { gains_ = gains; }
  void set_limits(const PidLimits& limits) { limits_ = limits; }
  const PidGains& gains() const { return gains_; }
  const PidLimits& limits() const { return limits_; }

  double integrator() const { return integrator_; }

 private:
  PidGains gains_;
  PidLimits limits_;
  double integrator_ = 0.0;
  double prev_error_ = 0.0;
  bool has_prev_ = false;
};

}  // namespace acessna
