#pragma once

#include <limits>

namespace acessna {

// First-order low-pass filter
class LowPassFilter {
 public:
  LowPassFilter() = default;
  explicit LowPassFilter(double time_constant_s) : tau_s_(time_constant_s) {}

  void reset(double value = 0.0);

  double update(double x, double dt);

  double value() const { return y_; }
  void set_time_constant(double time_constant_s) { tau_s_ = time_constant_s; }

 private:
  double tau_s_ = 0.0;  // 0 = pass through
  double y_ = 0.0;
};

class RateLimiter {
 public:
  RateLimiter() = default;
  explicit RateLimiter(double max_rate_per_s) : max_rate_(max_rate_per_s) {}

  void reset(double value = 0.0);

  double update(double x, double dt);

  double value() const { return y_; }
  void set_max_rate(double max_rate_per_s) { max_rate_ = max_rate_per_s; }

 private:
  double max_rate_ = std::numeric_limits<double>::infinity();
  double y_ = 0.0;
};

}  // namespace acessna
