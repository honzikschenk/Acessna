// Flight-path controller: TECS (longitudinal) + LateralController (lateral)
#pragma once

#include "acessna/control/lateral_controller.hpp"
#include "acessna/control/setpoints.hpp"
#include "acessna/control/tecs.hpp"
#include "acessna/sim/sim.hpp"
#include "acessna/telemetry.hpp"

namespace acessna {

struct FlightPathControllerConfig {
  TecsConfig tecs;
  LateralControllerConfig lateral;
};

class FlightPathController {
 public:
  explicit FlightPathController(const FlightPathControllerConfig& cfg = {})
      : tecs_(cfg.tecs), lateral_(cfg.lateral) {}

  void attach_telemetry(Telemetry& t);

  void reset(const State& s, const Controls& current);

  FlightPathOutput update(const Target& t, const State& s, double dt);

 private:
  Tecs tecs_;
  LateralController lateral_;
  double sideslip_deg_ = 0.0;
};

}  // namespace acessna
