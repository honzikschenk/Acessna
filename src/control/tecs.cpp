#include "acessna/control/tecs.hpp"

#include <variant>

#include "acessna/math.hpp"

namespace acessna {

void Tecs::reset(const State& s, const Controls& current) {
  hold_ = {s.theta_deg, current.throttle};
}

TecsOutput Tecs::update(const Target& t, const State& s, double dt) {
  (void)dt;
  const double h_dot_dem_fps =
      std::visit([&](const auto& cmd) { return height_rate_demand(cmd, s); }, t.vertical);
  (void)h_dot_dem_fps;
  TecsOutput out = hold_;  // hold the pitch and throttle at the last reset
  // TODO: limit h_dot_dem_fps to cfg_.max_climb_fpm / cfg_.max_sink_fpm
  // TODO: specific energy rate errors (potential from height rate, kinetic from t.airspeed_kts)
  // TODO: total energy rate error -> throttle (about the reset value)
  // TODO: energy balance rate error -> pitch setpoint
  // TODO: limits (cfg_ pitch and throttle)
  return out;
}

double Tecs::height_rate_demand(const AltitudeCommand& c, const State& s) {
  (void)c;
  (void)s;
  return 0.0;  // TODO: cfg_.altitude_gain_per_s * (c.altitude_ft - s.alt_ft)
}

double Tecs::height_rate_demand(const FlightPathCommand& c, const State& s) {
  (void)c;
  (void)s;
  return 0.0;  // TODO: V * sin(gamma), V from s.tas_kts * kKtsToFps
}

double Tecs::height_rate_demand(const ClimbRateCommand& c, const State& s) {
  (void)s;
  return c.climb_rate_fpm / 60.0;
}

}  // namespace acessna
