// Top-level controller flown by the harness
#pragma once

#include "acessna/autopilot/mode_manager.hpp"
#include "acessna/control/attitude_controller.hpp"
#include "acessna/control/flight_path_controller.hpp"
#include "acessna/control/rate_controller.hpp"
#include "acessna/sim/controller.hpp"
#include "acessna/telemetry.hpp"

namespace acessna {

struct AutopilotConfig {
  FlightPathControllerConfig flight_path;
  AttitudeControllerConfig attitude;
  RateControllerConfig rate;
};

class Autopilot : public Controller {
 public:
  explicit Autopilot(const AutopilotConfig& cfg = {});

  void attach_telemetry(Telemetry& t) override;
  void reset(const State& initial, const Controls& trim) override;
  Controls update(const State& s, double dt) override;

  ModeManager& modes() { return modes_; }
  const ModeManager& modes() const { return modes_; }

 private:
  void record(const Target& t, const FlightPathOutput& path, const RateSetpoint& rates,
              const SurfaceCommands& surfaces);

  ModeManager modes_;
  FlightPathController flight_path_;
  AttitudeController attitude_;
  RateController rate_;
  Telemetry* tlm_ = &Telemetry::discard();
  Controls trim_;
  Controls last_;  // last command issued, for bumpless transfer on mode change
};

}  // namespace acessna
