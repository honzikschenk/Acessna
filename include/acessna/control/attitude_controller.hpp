// Attitude controller
#pragma once

#include "acessna/control/setpoints.hpp"
#include "acessna/sim/sim.hpp"

namespace acessna {

class AttitudeController {
 public:
  void reset(const State& initial);

  RateSetpoint update(const AttitudeSetpoint& sp, const State& s, double dt);
};

}  // namespace acessna
