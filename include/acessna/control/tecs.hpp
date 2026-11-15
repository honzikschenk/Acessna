// TECS, longitudinal flight-path control
#pragma once

#include "acessna/control/pid.hpp"
#include "acessna/control/setpoints.hpp"
#include "acessna/sim/sim.hpp"
#include "acessna/telemetry.hpp"

namespace acessna {

struct TecsConfig {
  PidGains total_energy;    // total energy rate error -> throttle
  PidGains energy_balance;  // energy balance rate error -> pitch
  double altitude_gain_per_s = 0.0;  // altitude error (ft) -> height-rate demand (ft/s)
  double pitch_min_deg = -10.0;
  double pitch_max_deg = 15.0;
  double throttle_min = 0.0;
  double throttle_max = 1.0;
  double max_climb_fpm = 700.0;
  double max_sink_fpm = 1000.0;
};

class Tecs : public TelemetrySource {
 public:
  explicit Tecs(const TecsConfig& cfg = {}) : cfg_(cfg) {}

  void reset(const State& s, const Controls& current);

  TecsOutput update(const Target& t, const State& s, double dt);

  const TecsConfig& config() const { return cfg_; }

 private:
  double height_rate_demand(const AltitudeCommand& c, const State& s);
  double height_rate_demand(const FlightPathCommand& c, const State& s);
  double height_rate_demand(const ClimbRateCommand& c, const State& s);

  TecsConfig cfg_;
  TecsOutput hold_;
};

}  // namespace acessna
