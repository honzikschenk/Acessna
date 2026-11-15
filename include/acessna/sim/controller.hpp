// Control interface that is flown by the sim harness
#pragma once

#include "acessna/sim/sim.hpp"
#include "acessna/telemetry.hpp"

namespace acessna {

class Controller {
 public:
  virtual ~Controller() = default;

  virtual void attach_telemetry(Telemetry& t) { (void)t; }

  // Called once before the run with the post-trim state and control positions
  virtual void reset(const State& initial, const Controls& trim) {
    (void)initial;
    (void)trim;
  }

  // Called at the control rate. `dt` is the control period in seconds
  virtual Controls update(const State& s, double dt) = 0;
};

}  // namespace acessna
