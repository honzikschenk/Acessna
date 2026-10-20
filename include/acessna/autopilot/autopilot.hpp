// Top-level controller flown by the harness
#pragma once

#include "acessna/autopilot/mode_manager.hpp"
#include "acessna/control/attitude_controller.hpp"
#include "acessna/control/flight_path_controller.hpp"
#include "acessna/control/rate_controller.hpp"
#include "acessna/sim/controller.hpp"

namespace acessna {

class Autopilot : public Controller {
 public:
  void reset(const State& initial, const Controls& trim) override;
  Controls update(const State& s, double dt) override;

  Mode mode() const { return modes_.mode(); }

 private:
  ModeManager modes_;
  FlightPathController flight_path_;
  AttitudeController attitude_;
  RateController rate_;
  Controls trim_;
};

}  // namespace acessna
