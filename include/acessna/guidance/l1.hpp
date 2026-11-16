// L1 lateral guidance
#pragma once

#include "acessna/sim/sim.hpp"

namespace acessna {

struct L1Config {
  double period_s = 20.0;  // tracking period: larger is smoother and slower
  double damping = 0.75;   // 0.7 -> 0.85
  double min_l1_m = 100.0;
};

struct L1Output {
  double lateral_accel_mps2 = 0.0;  // + = turn right
  double bank_deg = 0.0;            // coordinated bank for the acceleration, atan(a / g)
  double l1_distance_m = 0.0;
  double cross_track_m = 0.0;  // + = right of the line
};

// Tracks the line through (lat0, lon0) along `course_deg`
L1Output l1_track_line(const L1Config& cfg, double lat0_deg, double lon0_deg, double course_deg,
                       const State& s);

}  // namespace acessna
