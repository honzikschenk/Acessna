// State machine
#pragma once

#include "acessna/control/setpoints.hpp"
#include "acessna/sim/sim.hpp"

namespace acessna {

enum class Mode {
  kHold,
};

class ModeManager {
 public:
  void reset(const State& initial);

  // Runs mode transitions. Returns true if the mode changed this step
  bool update(const State& s);

  // Targets for the current mode
  Target targets(const State& s) const;

  Mode mode() const { return mode_; }

 private:
  Mode mode_ = Mode::kHold;
  Target hold_;
};

}  // namespace acessna
