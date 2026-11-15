// Runs the active lateral and vertical modes and their transitions
#pragma once

#include <memory>

#include "acessna/autopilot/modes.hpp"
#include "acessna/control/setpoints.hpp"
#include "acessna/sim/sim.hpp"

namespace acessna {

class ModeManager {
 public:
  ModeManager();

  // Engages HeadingHold + AltitudeHold with the initial airspeed
  void reset(const State& initial);

  // Engages a mode
  void engage(std::unique_ptr<LateralMode> mode);
  void engage(std::unique_ptr<VerticalMode> mode);

  void set_airspeed_kts(double kts) { airspeed_kts_ = kts; }

  // Runs engaged modes and applies transitions. Returns true if either axis changed modes this step
  bool update(const State& s);

  // Targets from the active modes
  Target targets(const State& s, double dt);

  LateralModeId lateral_mode() const { return lateral_->id(); }
  VerticalModeId vertical_mode() const { return vertical_->id(); }
  ActiveModes active() const { return {lateral_mode(), vertical_mode()}; }

 private:
  std::unique_ptr<LateralMode> lateral_;
  std::unique_ptr<VerticalMode> vertical_;
  std::unique_ptr<LateralMode> pending_lateral_;
  std::unique_ptr<VerticalMode> pending_vertical_;
  double airspeed_kts_ = 0.0;
};

}  // namespace acessna
