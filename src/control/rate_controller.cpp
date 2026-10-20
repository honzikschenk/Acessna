#include "acessna/control/rate_controller.hpp"

namespace acessna {

void RateController::reset(const State& initial, const Controls& trim) {
  (void)initial;
  trim_ = {trim.elevator, trim.aileron, trim.rudder};
}

SurfaceCommands RateController::update(const RateSetpoint& sp, const State& s, double dt) {
  (void)sp;
  (void)s;
  (void)dt;
  SurfaceCommands c = trim_;  // start from the trimmed positions
  // TODO: roll rate (p) -> aileron
  // TODO: pitch rate (q) -> elevator
  // TODO: yaw rate (r) -> rudder
  return c;
}

}  // namespace acessna
