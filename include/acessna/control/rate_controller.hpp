// Rate controller
#pragma once

#include "acessna/control/pid.hpp"
#include "acessna/control/setpoints.hpp"
#include "acessna/sim/sim.hpp"
#include "acessna/telemetry.hpp"

namespace acessna {

struct RateControllerConfig {
  PidGains roll;   // p error -> aileron
  PidGains pitch;  // q error -> elevator
  PidGains yaw;    // r error -> rudder
};

class RateController : public TelemetrySource {
 public:
  explicit RateController(const RateControllerConfig& cfg = {}) : cfg_(cfg) {}

  void reset(const State& initial, const Controls& trim);

  SurfaceCommands update(const RateSetpoint& sp, const State& s, double dt);

  const RateControllerConfig& config() const { return cfg_; }

 private:
  RateControllerConfig cfg_;
  SurfaceCommands trim_;
};

}  // namespace acessna
