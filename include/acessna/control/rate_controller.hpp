// Rate controller
#pragma once

#include "acessna/control/setpoints.hpp"
#include "acessna/sim/sim.hpp"

namespace acessna {

class RateController {
 public:
  void reset(const State& initial, const Controls& trim);

  SurfaceCommands update(const RateSetpoint& sp, const State& s, double dt);

 private:
  SurfaceCommands trim_;
};

}  // namespace acessna
