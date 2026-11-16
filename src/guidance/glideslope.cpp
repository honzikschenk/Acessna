#include "acessna/guidance/glideslope.hpp"

#include <algorithm>
#include <cmath>

#include "acessna/guidance/route.hpp"
#include "acessna/math.hpp"

namespace acessna {

double glidepath_height_ft(const GlidePath& gp, const State& s) {
  const TrackPosition tp = track_position(gp.threshold_lat_deg, gp.threshold_lon_deg, gp.course_deg,
                                          s.lat_deg, s.lon_deg);
  const double to_threshold_ft = std::max(-tp.along_m, 0.0) * kMToFt;
  return gp.threshold_elev_ft + gp.threshold_crossing_height_ft +
         std::tan(gp.angle_deg * kDegToRad) * to_threshold_ft;
}

double glideslope_flight_path_deg(const GlideslopeConfig& cfg, const GlidePath& gp,
                                  const State& s) {
  const double groundspeed_fps = s.groundspeed_kts * kKtsToFps;
  const double path_rate_fps = -groundspeed_fps * std::tan(gp.angle_deg * kDegToRad);

  const double max_corr_fps = cfg.max_correction_fpm / 60.0;
  const double correction_fps = std::clamp(
      cfg.height_gain_per_s * (glidepath_height_ft(gp, s) - s.alt_ft), -max_corr_fps, max_corr_fps);

  const double tas_fps = std::max(s.tas_kts * kKtsToFps, 1.0);
  const double ratio = std::clamp((path_rate_fps + correction_fps) / tas_fps, -1.0, 1.0);
  return std::asin(ratio) * kRadToDeg;
}

}  // namespace acessna
