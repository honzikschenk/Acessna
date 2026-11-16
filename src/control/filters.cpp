#include "acessna/control/filters.hpp"

#include <algorithm>
#include <cmath>

namespace acessna {

void LowPassFilter::reset(double value) { y_ = value; }

double LowPassFilter::update(double x, double dt) {
  if (tau_s_ <= 0.0) {
    y_ = x;
  } else {
    y_ += (x - y_) * dt / (tau_s_ + dt);
  }
  return y_;
}

void RateLimiter::reset(double value) { y_ = value; }

double RateLimiter::update(double x, double dt) {
  if (std::isinf(max_rate_)) {
    y_ = x;
  } else {
    const double max_step = max_rate_ * dt;
    y_ += std::clamp(x - y_, -max_step, max_step);
  }
  return y_;
}

}  // namespace acessna
