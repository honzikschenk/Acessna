#include "acessna/control/flight_path_controller.hpp"

namespace acessna {

void FlightPathController::attach_telemetry(Telemetry& t) {
  tecs_.attach_telemetry(t);
  lateral_.attach_telemetry(t);
}

void FlightPathController::reset(const State& s, const Controls& current) {
  tecs_.reset(s, current);
  lateral_.reset(s);
}

FlightPathOutput FlightPathController::update(const Target& t, const State& s, double dt) {
  const TecsOutput lon = tecs_.update(t, s, dt);
  const double roll_deg = lateral_.update(t, s, dt);

  FlightPathOutput out;
  out.attitude = {roll_deg, lon.pitch_deg, sideslip_deg_};
  out.throttle = lon.throttle;
  return out;
}

}  // namespace acessna
