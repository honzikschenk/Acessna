#include "acessna/control/attitude_controller.hpp"

#include <algorithm>
#include <cmath>

#include "acessna/math.hpp"

namespace acessna {

AttitudeController::AttitudeController(const AttitudeControllerConfig& cfg)
    : cfg_(cfg),
      roll_(cfg.roll, {-cfg.max_roll_rate_dps, cfg.max_roll_rate_dps, -cfg.max_roll_rate_dps, cfg.max_roll_rate_dps}),
      pitch_(cfg.pitch, {-cfg.max_pitch_rate_dps, cfg.max_pitch_rate_dps, -cfg.max_pitch_rate_dps, cfg.max_pitch_rate_dps}),
      sideslip_(cfg.sideslip, {-cfg.max_yaw_rate_dps, cfg.max_yaw_rate_dps, -cfg.max_yaw_rate_dps, cfg.max_yaw_rate_dps}) {}

void AttitudeController::reset(const State& initial) {
  (void)initial;
  roll_.reset();
  pitch_.reset();
  sideslip_.reset();
}

RateSetpoint AttitudeController::update(const AttitudeSetpoint& sp, const State& s, double dt) {
  // Euler angle rates demands
  const double phi_dot = roll_.update(sp.roll_deg - s.phi_deg, dt);
  const double theta_dot = pitch_.update(sp.pitch_deg - s.theta_deg, dt);

  // Heading rate of a coordinated turn at a bank
  const double v_fps = std::max(s.tas_kts * kKtsToFps, 1.0);
  const double phi = s.phi_deg * kDegToRad;
  const double theta = s.theta_deg * kDegToRad;
  const double psi_dot = kGravityFps2 * std::tan(phi) / v_fps * kRadToDeg;

  // Euler angle rates -> body rates
  RateSetpoint r;
  r.roll_rate_dps = phi_dot - psi_dot * std::sin(theta);
  r.pitch_rate_dps = theta_dot * std::cos(phi) + psi_dot * std::sin(phi) * std::cos(theta);
  r.yaw_rate_dps = -theta_dot * std::sin(phi) + psi_dot * std::cos(phi) * std::cos(theta);

  // Sideslip correction - +beta (relative wind from the right) needs +r (yaw right)
  r.yaw_rate_dps += sideslip_.update(s.beta_deg - sp.sideslip_deg, dt);

  r.roll_rate_dps = std::clamp(r.roll_rate_dps, -cfg_.max_roll_rate_dps, cfg_.max_roll_rate_dps);
  r.pitch_rate_dps =
      std::clamp(r.pitch_rate_dps, -cfg_.max_pitch_rate_dps, cfg_.max_pitch_rate_dps);
  r.yaw_rate_dps = std::clamp(r.yaw_rate_dps, -cfg_.max_yaw_rate_dps, cfg_.max_yaw_rate_dps);
  return r;
}

}  // namespace acessna
