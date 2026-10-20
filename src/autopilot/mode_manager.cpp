#include "acessna/autopilot/mode_manager.hpp"

namespace acessna {

void ModeManager::reset(const State& initial) {
  mode_ = Mode::kHold;
  hold_ = {initial.alt_ft, initial.psi_deg, initial.airspeed_kts};
}

bool ModeManager::update(const State& s) {
  (void)s;
  const Mode prev = mode_;
  // TODO: mode transitions
  return mode_ != prev;
}

Target ModeManager::targets(const State& s) const {
  (void)s;
  switch (mode_) {
    case Mode::kHold:
      return hold_;  // hold the initial condition
  }
  return hold_;
}

}  // namespace acessna
