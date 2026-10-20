#include "acessna/control/flight_path_controller.hpp"

namespace acessna {

void FlightPathController::reset(const State& s, const Controls& trim) {
  hold_.attitude = {s.phi_deg, s.theta_deg, s.beta_deg};
  hold_.throttle = trim.throttle;
}

FlightPathOutput FlightPathController::update(const Target& t, const State& s, double dt) {
  (void)t;
  (void)s;
  (void)dt;
  FlightPathOutput out = hold_;  // hold the condition at the last reset
  // TODO: altitude -> pitch setpoint
  // TODO: heading -> roll setpoint
  // TODO: airspeed -> throttle
  return out;
}

}  // namespace acessna
