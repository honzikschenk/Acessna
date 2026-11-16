// Approach to KROA runway 06
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "acessna/autopilot/autopilot.hpp"
#include "acessna/guidance/glideslope.hpp"
#include "acessna/guidance/route.hpp"
#include "acessna/sim/harness.hpp"

namespace acessna {

struct Runway {
  std::string name;
  double threshold_lat_deg = 0.0;
  double threshold_lon_deg = 0.0;
  double threshold_elev_ft = 0.0;
  double course_deg = 0.0;
  double length_m = 0.0;  // threshold to the far end, where the localizer is
};

struct Approach {
  Runway runway;
  Waypoint start;
  double start_heading_deg = 0.0;

  std::vector<Waypoint> route;
  double faf_distance_m = 0.0;  // final approach fix to threshold
  double faf_alt_ft = 0.0;      // on the glidepath
  double glidepath_deg = 3.0;
  double threshold_crossing_height_ft = 50.0;
  double decision_height_ft = 200.0;  // above the threshold

  GlidePath glidepath() const;
};

Approach kroa_rwy06_approach();

TrackPosition final_position(const Approach& a, const State& s);

RunConfig approach_run(const Approach& a);

struct Dispersions {
  double start_offset_m = 300.0;
  double heading_deg = 15.0;
  double altitude_ft = 150.0;
  double airspeed_kts = 10.0;
  double max_wind_kts = 20.0;
  int max_turbulence = 3;  // severity 0 -> max (1 light, 3 moderate)
};

// approach_run with wind, turbulence and initial conditions drawn from `seed`
RunConfig dispersed_approach_run(const Approach& a, std::uint64_t seed, const Dispersions& d = {});

// Autopilot that flies the approach from the start of every run
std::unique_ptr<Autopilot> make_approach_autopilot(const Approach& a,
                                                   const AutopilotConfig& cfg = {});

}  // namespace acessna
