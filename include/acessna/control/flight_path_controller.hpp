// Flight-path controller
#pragma once

#include "acessna/control/setpoints.hpp"
#include "acessna/sim/sim.hpp"

namespace acessna {

class FlightPathController {
 public:
  void reset(const State& s, const Controls& trim);

  FlightPathOutput update(const Target& t, const State& s, double dt);

 private:
  FlightPathOutput hold_;
};

}  // namespace acessna
