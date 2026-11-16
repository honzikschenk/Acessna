#include "acessna/control/tecs.hpp"

#include <algorithm>
#include <cmath>
#include <variant>

#include "acessna/math.hpp"

namespace acessna {

Tecs::Tecs(const TecsConfig& cfg)
    : cfg_(cfg),
      total_energy_(cfg.total_energy,
                    {cfg.throttle_min, cfg.throttle_max, cfg.throttle_min, cfg.throttle_max}),
      energy_balance_(cfg.energy_balance,
                      {cfg.pitch_min_deg, cfg.pitch_max_deg, cfg.pitch_min_deg, cfg.pitch_max_deg}),
      speed_rate_filter_(cfg.speed_rate_tau_s) {}

void Tecs::reset(const State& s, const Controls& current) {
  total_energy_.reset(current.throttle);
  energy_balance_.reset(s.theta_deg);
  speed_rate_filter_.reset(0.0);
  prev_airspeed_fps_ = s.airspeed_kts * kKtsToFps;
}

TecsOutput Tecs::update(const Target& t, const State& s, double dt) {
  double h_dot_dem_fps =
      std::visit([&](const auto& cmd) { return height_rate_demand(cmd, s); }, t.vertical);
  h_dot_dem_fps = std::clamp(h_dot_dem_fps, -cfg_.max_sink_fpm / 60.0, cfg_.max_climb_fpm / 60.0);

  // Measured speed and acceleration (derivative of airspeed)
  const double airspeed_fps = s.airspeed_kts * kKtsToFps;
  const double raw_v_dot = dt > 0.0 ? (airspeed_fps - prev_airspeed_fps_) / dt : 0.0;
  prev_airspeed_fps_ = airspeed_fps;
  const double v_dot_fps2 = speed_rate_filter_.update(raw_v_dot, dt);
  const double v_dot_dem_fps2 =
      cfg_.airspeed_gain_per_s * (t.airspeed_kts - s.airspeed_kts) * kKtsToFps;

  // Energy rate errors
  const double v_fps = std::max(s.tas_kts * kKtsToFps, 1.0);
  const double potential_err = (h_dot_dem_fps - s.vs_fpm / 60.0) / v_fps;
  const double kinetic_err = (v_dot_dem_fps2 - v_dot_fps2) / kGravityFps2;

  TecsOutput out;
  out.throttle = total_energy_.update(potential_err + kinetic_err, dt);
  out.pitch_deg = energy_balance_.update(potential_err - kinetic_err, dt);
  return out;
}

double Tecs::height_rate_demand(const AltitudeCommand& c, const State& s) {
  return cfg_.altitude_gain_per_s * (c.altitude_ft - s.alt_ft);
}

double Tecs::height_rate_demand(const FlightPathCommand& c, const State& s) {
  return s.tas_kts * kKtsToFps * std::sin(c.flight_path_deg * kDegToRad);
}

double Tecs::height_rate_demand(const ClimbRateCommand& c, const State& s) {
  (void)s;
  return c.climb_rate_fpm / 60.0;
}

}  // namespace acessna
