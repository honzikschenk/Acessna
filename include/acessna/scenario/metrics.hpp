// Metrics for an approach run, for Monte Carlo summaries
#pragma once

#include <limits>
#include <string>

#include "acessna/scenario/kroa.hpp"
#include "acessna/sim/harness.hpp"

namespace acessna {

struct ApproachMetrics {
  static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

  bool reached_faf = false;  // crossed the final approach fix on final
  bool gs_captured = false;  // glideslope mode engaged
  bool reached_dh = false;   // descended to decision height on final
  bool hit_ground = false;
  std::string error;  // JSBSim errors, if present

  // At the final approach fix (NaN if not reached)
  double faf_time_s = kNaN;
  double faf_cross_track_m = kNaN;  // + = right
  double faf_alt_error_ft = kNaN;   // minus the final approach fix altitude

  double gs_capture_time_s = kNaN;

  // At decision height (NaN if not reached)
  double dh_time_s = kNaN;
  double dh_cross_track_m = kNaN;    // + = right
  double dh_gs_deviation_ft = kNaN;  // altitude minus the glidepath
  double dh_airspeed_kts = kNaN;
  double dh_sink_fpm = kNaN;

  // Final approach fix to decision height (NaN if the final approach fix isn't reached)
  double final_max_cross_track_m = kNaN;    // largest |cross-track|
  double final_max_gs_deviation_ft = kNaN;  // largest |glidepath deviation|
  double final_max_sink_fpm = kNaN;
  double final_max_gs_deviation_deg = kNaN;

  // Largest overshoot past the centerline
  double loc_overshoot_deg = 0.0;

  // Below 1000 ft above the threshold, until decision height
  double max_airspeed_error_kts = 0.0;
  double max_sink_below_1000ft_fpm = 0.0;
  // Below 500 ft above the threshold, until decision height
  double max_bank_below_500ft_deg = 0.0;

  // Whole run
  double max_bank_deg = 0.0;
  double max_sideslip_deg = 0.0;
  double min_airspeed_kts = std::numeric_limits<double>::infinity();
};

ApproachMetrics approach_metrics(const Approach& a, const RunResult& r);

}  // namespace acessna
