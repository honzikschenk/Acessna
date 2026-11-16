// Attitude controller
#pragma once

#include "acessna/control/pid.hpp"
#include "acessna/control/setpoints.hpp"
#include "acessna/sim/sim.hpp"
#include "acessna/telemetry.hpp"

namespace acessna {

struct AttitudeControllerConfig {
  PidGains roll{2.0, 0.1, 0.0};      // roll error -> p
  PidGains pitch{2.0, 0.2, 0.0};     // pitch error -> q
  PidGains sideslip{1.0, 0.2, 0.0};  // sideslip error -> r
  double max_roll_rate_dps = 20.0;
  double max_pitch_rate_dps = 10.0;
  double max_yaw_rate_dps = 15.0;
};

class AttitudeController : public TelemetrySource {
 public:
  explicit AttitudeController(const AttitudeControllerConfig& cfg = {});

  void reset(const State& initial);

  RateSetpoint update(const AttitudeSetpoint& sp, const State& s, double dt);

  const AttitudeControllerConfig& config() const { return cfg_; }

 private:
  AttitudeControllerConfig cfg_;
  Pid roll_;
  Pid pitch_;
  Pid sideslip_;
};

}  // namespace acessna
