#include "acessna/control/attitude_controller.hpp"

namespace acessna {

void AttitudeController::reset(const State& initial) {
  (void)initial;
}

RateSetpoint AttitudeController::update(const AttitudeSetpoint& sp, const State& s, double dt) {
  (void)sp;
  (void)s;
  (void)dt;
  RateSetpoint r;  // zero rates
  // TODO: roll error -> roll rate (p)
  // TODO: pitch error -> pitch rate (q)
  // TODO: sideslip / turn coordination -> yaw rate (r)
  // TODO: rate limits
  return r;
}

}  // namespace acessna
