#include "acessna/autopilot/autopilot.hpp"

namespace acessna {

void Autopilot::reset(const State& initial, const Controls& trim) {
  trim_ = trim;
  modes_.reset(initial);
  flight_path_.reset(initial, trim);
  attitude_.reset(initial);
  rate_.reset(initial, trim);
}

Controls Autopilot::update(const State& s, double dt) {
  if (modes_.update(s)) flight_path_.reset(s, trim_);  // bumpless transfer on mode change

  const Target targets = modes_.targets(s);
  const FlightPathOutput path = flight_path_.update(targets, s, dt);
  const RateSetpoint rates = attitude_.update(path.attitude, s, dt);
  const SurfaceCommands surfaces = rate_.update(rates, s, dt);

  Controls c = trim_;  // start from the trimmed positions
  c.elevator = surfaces.elevator;
  c.aileron = surfaces.aileron;
  c.rudder = surfaces.rudder;
  c.throttle = path.throttle;
  return c;
}

}  // namespace acessna
