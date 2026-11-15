#include "acessna/control/filters.hpp"

namespace acessna {

void LowPassFilter::reset(double value) { y_ = value; }

double LowPassFilter::update(double x, double dt) {
  (void)dt;
  // TODO: y_ += (x - y_) * dt / (tau_s_ + dt)
  y_ = x;
  return y_;
}

void RateLimiter::reset(double value) { y_ = value; }

double RateLimiter::update(double x, double dt) {
  (void)dt;
  // TODO: limit the step (x - y_) to +-max_rate_ * dt
  y_ = x;
  return y_;
}

}  // namespace acessna
