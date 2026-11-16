// Rate controller
#pragma once

#include "acessna/control/pid.hpp"
#include "acessna/control/setpoints.hpp"
#include "acessna/sim/sim.hpp"
#include "acessna/telemetry.hpp"

namespace acessna {

struct RateControllerConfig {
  PidGains roll{0.02, 0.01, 0.0};   // p error -> aileron
  PidGains pitch{0.03, 0.02, 0.0};  // q error -> elevator
  PidGains yaw{0.05, 0.02, 0.0};    // r error -> rudder
};

class RateController : public TelemetrySource {
 public:
  explicit RateController(const RateControllerConfig& cfg = {});

  void reset(const State& initial, const Controls& trim);

  SurfaceCommands update(const RateSetpoint& sp, const State& s, double dt);

  const RateControllerConfig& config() const { return cfg_; }

 private:
  RateControllerConfig cfg_;
  Pid roll_;
  Pid pitch_;
  Pid yaw_;
  SurfaceCommands trim_;
};

}  // namespace acessna
