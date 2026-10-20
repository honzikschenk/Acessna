#include "acessna/control/flight_controller.hpp"

namespace acessna {

void FlightController::reset(const State& initial, const Controls& trim) {
  (void)initial;
  trim_ = trim;
}

Controls FlightController::update(const State& s, double dt) {
  (void)s;
  (void)dt;
  Controls c = trim_;  // start from the trimmed positions
  return c;
}

}  // namespace acessna
