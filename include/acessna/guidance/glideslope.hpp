// Glideslope guidance, tracks a straight glidepath to the runway threshold
// Outputs a flight path angle for TECS
#pragma once

#include "acessna/sim/sim.hpp"

namespace acessna {

struct GlidePath {
  double threshold_lat_deg = 0.0;
  double threshold_lon_deg = 0.0;
  double threshold_elev_ft = 0.0;
  double course_deg = 0.0;  // true, final approach course
  double angle_deg = 3.0;
  double threshold_crossing_height_ft = 50.0;
};

struct GlideslopeConfig {
  double height_gain_per_s = 0.35;    // height error (ft) -> climb-rate correction (ft/s)
  double max_correction_fpm = 500.0;  // limit on that correction
};

// Glidepath height (MSL) at the aircraft's distance from the threshold along the course
double glidepath_height_ft(const GlidePath& gp, const State& s);

// Flight path angle command (deg, + = climb)
double glideslope_flight_path_deg(const GlideslopeConfig& cfg, const GlidePath& gp, const State& s);

}  // namespace acessna
