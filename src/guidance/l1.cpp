#include "acessna/guidance/l1.hpp"

#include <algorithm>
#include <cmath>

#include "acessna/guidance/route.hpp"
#include "acessna/math.hpp"

namespace acessna {

namespace {
constexpr double kGravityMps2 = kGravityFps2 * kFtToM;
}  // namespace

L1Output l1_track_line(const L1Config& cfg, double lat0_deg, double lon0_deg, double course_deg,
                       const State& s) {
  const double v_mps = std::max(s.groundspeed_kts * kKtsToFps * kFtToM, 1.0);
  // Reference distance ahead on the path
  const double l1_m = std::max(cfg.damping * cfg.period_s * v_mps / kPi, cfg.min_l1_m);

  const TrackPosition tp = track_position(lat0_deg, lon0_deg, course_deg, s.lat_deg, s.lon_deg);

  // Velocity relative to the line
  const double rel_track = (s.track_deg - course_deg) * kDegToRad;
  const double v_along = v_mps * std::cos(rel_track);
  const double v_cross = v_mps * std::sin(rel_track);

  // Angle from the velocity to the L1 point
  const double nu1 = std::asin(std::clamp(tp.cross_m / l1_m, -0.7071, 0.7071));
  const double nu2 = std::atan2(v_cross, v_along);
  const double nu = std::clamp(nu1 + nu2, -kPi / 2.0, kPi / 2.0);

  // Steers left (-) when right of or heading right of the line
  const double k = 4.0 * cfg.damping * cfg.damping;
  L1Output out;
  out.lateral_accel_mps2 = -k * v_mps * v_mps / l1_m * std::sin(nu);
  out.bank_deg = std::atan(out.lateral_accel_mps2 / kGravityMps2) * kRadToDeg;
  out.l1_distance_m = l1_m;
  out.cross_track_m = tp.cross_m;
  return out;
}

}  // namespace acessna
