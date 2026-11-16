#include "acessna/control/rate_controller.hpp"

#include <algorithm>

namespace acessna {

namespace {
// Corrections about trim in normalized surface units
constexpr PidLimits kSurfaceLimits{-1.0, 1.0, -1.0, 1.0};
}  // namespace

RateController::RateController(const RateControllerConfig& cfg)
    : cfg_(cfg),
      roll_(cfg.roll, kSurfaceLimits),
      pitch_(cfg.pitch, kSurfaceLimits),
      yaw_(cfg.yaw, kSurfaceLimits) {}

void RateController::reset(const State& initial, const Controls& trim) {
  (void)initial;
  trim_ = {trim.elevator, trim.aileron, trim.rudder};
  roll_.reset();
  pitch_.reset();
  yaw_.reset();
}

SurfaceCommands RateController::update(const RateSetpoint& sp, const State& s, double dt) {
  SurfaceCommands c = trim_;

  // +aileron rolls right, +rudder yaws right
  c.aileron += roll_.update(sp.roll_rate_dps - s.p_dps, dt);
  c.rudder += yaw_.update(sp.yaw_rate_dps - s.r_dps, dt);
  // +elevator is nose down
  c.elevator -= pitch_.update(sp.pitch_rate_dps - s.q_dps, dt);

  c.aileron = std::clamp(c.aileron, -1.0, 1.0);
  c.elevator = std::clamp(c.elevator, -1.0, 1.0);
  c.rudder = std::clamp(c.rudder, -1.0, 1.0);
  return c;
}

}  // namespace acessna
