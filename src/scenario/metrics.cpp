#include "acessna/scenario/metrics.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "acessna/autopilot/modes.hpp"
#include "acessna/math.hpp"

namespace acessna {

namespace {

constexpr double kOnFinalCrossM = 3000.0;

// Telemetry signal index, or -1
std::ptrdiff_t signal_index(const RunResult& r, const std::string& name) {
  const auto it = std::find(r.signal_names.begin(), r.signal_names.end(), name);
  return it == r.signal_names.end() ? -1 : it - r.signal_names.begin();
}

}  // namespace

ApproachMetrics approach_metrics(const Approach& a, const RunResult& r) {
  ApproachMetrics m;
  m.hit_ground = r.hit_ground;
  m.error = r.error;

  const GlidePath gp = a.glidepath();
  const double dh_alt_ft = a.runway.threshold_elev_ft + a.decision_height_ft;
  const std::ptrdiff_t vertical_mode = signal_index(r, "mode.vertical");
  const std::ptrdiff_t target_airspeed = signal_index(r, "tgt.airspeed_kts");
  auto signal = [](const Sample& sample, std::ptrdiff_t i) {
    return i >= 0 && static_cast<std::size_t>(i) < sample.signals.size()
               ? sample.signals[static_cast<std::size_t>(i)]
               : std::nan("");
  };
  // Where the glidepath meets the runway
  const double gpip_m =
      a.threshold_crossing_height_ft * kFtToM / std::tan(a.glidepath_deg * kDegToRad);
  // Side of the centerline the run starts on
  double start_side = 0.0;
  // Whether it has been crossed
  bool crossed = false;
  const auto glideslope_id = static_cast<double>(VerticalModeId::kGlideslope);

  double prev_along_m = 0.0;
  bool has_prev = false;

  for (const Sample& sample : r.log) {
    const State& s = sample.state;
    m.max_bank_deg = std::max(m.max_bank_deg, std::abs(s.phi_deg));
    m.max_sideslip_deg = std::max(m.max_sideslip_deg, std::abs(s.beta_deg));
    m.min_airspeed_kts = std::min(m.min_airspeed_kts, s.airspeed_kts);

    const TrackPosition fp = final_position(a, s);
    const bool on_final = std::abs(fp.cross_m) < kOnFinalCrossM;
    const double gs_dev_ft = s.alt_ft - glidepath_height_ft(gp, s);
    const double sink_fpm = -s.vs_fpm;

    const double hat_ft = s.alt_ft - a.runway.threshold_elev_ft;

    if (!m.gs_captured && signal(sample, vertical_mode) == glideslope_id) {
      m.gs_captured = true;
      m.gs_capture_time_s = s.t;
      m.final_max_gs_deviation_ft = 0.0;
      m.final_max_gs_deviation_deg = 0.0;
      m.final_max_sink_fpm = sink_fpm;
    }

    if (!m.reached_dh) {
      if (start_side == 0.0) start_side = fp.cross_m < 0.0 ? -1.0 : 1.0;
      const double far_side_m = -start_side * fp.cross_m;  // + once past the centerline
      if (far_side_m > 0.0 && fp.along_m < 0.0) crossed = true;
      if (crossed && far_side_m > 0.0) {
        const double to_localizer_m = a.runway.length_m - fp.along_m;
        m.loc_overshoot_deg =
            std::max(m.loc_overshoot_deg, std::atan2(far_side_m, to_localizer_m) * kRadToDeg);
      }
      if (hat_ft < 1000.0) {
        const double target_kts = signal(sample, target_airspeed);
        if (!std::isnan(target_kts)) {
          m.max_airspeed_error_kts =
              std::max(m.max_airspeed_error_kts, std::abs(s.airspeed_kts - target_kts));
        }
        m.max_sink_below_1000ft_fpm = std::max(m.max_sink_below_1000ft_fpm, sink_fpm);
      }
      if (hat_ft < 500.0) {
        m.max_bank_below_500ft_deg = std::max(m.max_bank_below_500ft_deg, std::abs(s.phi_deg));
      }
    }

    if (has_prev && on_final && !m.reached_faf && prev_along_m < -a.faf_distance_m &&
        fp.along_m >= -a.faf_distance_m) {
      m.reached_faf = true;
      m.faf_time_s = s.t;
      m.faf_cross_track_m = fp.cross_m;
      m.faf_alt_error_ft = s.alt_ft - a.faf_alt_ft;
      m.final_max_cross_track_m = 0.0;
    }

    if (m.reached_faf && !m.reached_dh) {
      m.final_max_cross_track_m = std::max(m.final_max_cross_track_m, std::abs(fp.cross_m));
      if (m.gs_captured) {
        m.final_max_gs_deviation_ft = std::max(m.final_max_gs_deviation_ft, std::abs(gs_dev_ft));
        const double to_gpip_m = gpip_m - fp.along_m;
        m.final_max_gs_deviation_deg =
            std::max(m.final_max_gs_deviation_deg,
                     std::abs(std::atan2(gs_dev_ft * kFtToM, to_gpip_m)) * kRadToDeg);
        m.final_max_sink_fpm = std::max(m.final_max_sink_fpm, sink_fpm);
      }
      if (on_final && fp.along_m < 0.0 && s.alt_ft <= dh_alt_ft) {
        m.reached_dh = true;
        m.dh_time_s = s.t;
        m.dh_cross_track_m = fp.cross_m;
        m.dh_gs_deviation_ft = gs_dev_ft;
        m.dh_airspeed_kts = s.airspeed_kts;
        m.dh_sink_fpm = sink_fpm;
      }
    }
    prev_along_m = fp.along_m;
    has_prev = true;
  }
  return m;
}

}  // namespace acessna
