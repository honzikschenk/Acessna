#include "acessna/scenario/kroa.hpp"

#include <cmath>
#include <random>

#include "acessna/math.hpp"

namespace acessna {

namespace {

constexpr double kNmToM = 1852.0;
constexpr double kOnFinalCrossM = 3000.0;

}  // namespace

Approach kroa_rwy06_approach() {
  // OurAirports runways.csv, KROA 06/24
  constexpr double kThrLat = 37.322201;
  constexpr double kThrLon = -79.984001;
  constexpr double kThrElevFt = 1152.0;
  constexpr double kEnd24Lat = 37.334301;
  constexpr double kEnd24Lon = -79.966202;

  Approach a;
  const LocalOffset runway = local_offset_m(kThrLat, kThrLon, kEnd24Lat, kEnd24Lon);
  a.runway = {"KROA 06", kThrLat, kThrLon, kThrElevFt, bearing_deg(runway), distance_m(runway)};
  a.faf_distance_m = 1.0 * kNmToM;
  const double final_turn_m = 1.6 * kNmToM;  // turn onto final, ahead of the final approach fix
  const double start_out_m = 0.4 * kNmToM;   // start abeam this far out from the threshold

  const double course = a.runway.course_deg;
  const double outbound = wrap_360(course + 180.0);
  const double left = wrap_360(course - 90.0);
  const double downwind_m = 0.7 * kNmToM;
  const double pattern_alt_ft = kThrElevFt + 1000.0;
  a.faf_alt_ft = kThrElevFt + a.threshold_crossing_height_ft +
                 std::tan(a.glidepath_deg * kDegToRad) * a.faf_distance_m * kMToFt;

  const Waypoint abeam = project("", kThrLat, kThrLon, outbound, start_out_m, 0.0);
  a.start = project("START", abeam.lat_deg, abeam.lon_deg, left, downwind_m, pattern_alt_ft);
  a.start_heading_deg = outbound;

  // Level at the final approach fix altitude after the turn onto final, so the glideslope is
  // intercepted from below
  const Waypoint final_turn =
      project("FINAL", kThrLat, kThrLon, outbound, final_turn_m, a.faf_alt_ft);
  a.route = {
      project("BASE", final_turn.lat_deg, final_turn.lon_deg, left, downwind_m,
              pattern_alt_ft - 200.0),
      final_turn,
      project("FAF", kThrLat, kThrLon, outbound, a.faf_distance_m, a.faf_alt_ft),
      {"THR", kThrLat, kThrLon, a.faf_alt_ft},
  };
  return a;
}

GlidePath Approach::glidepath() const {
  return {runway.threshold_lat_deg,
          runway.threshold_lon_deg,
          runway.threshold_elev_ft,
          runway.course_deg,
          glidepath_deg,
          threshold_crossing_height_ft};
}

TrackPosition final_position(const Approach& a, const State& s) {
  return track_position(a.runway.threshold_lat_deg, a.runway.threshold_lon_deg, a.runway.course_deg,
                        s.lat_deg, s.lon_deg);
}

RunConfig approach_run(const Approach& a) {
  RunConfig cfg;
  cfg.ic.lat_deg = a.start.lat_deg;
  cfg.ic.lon_deg = a.start.lon_deg;
  cfg.ic.alt_ft = a.start.alt_ft;
  cfg.ic.heading_deg = a.start_heading_deg;
  cfg.ic.terrain_elev_ft = a.runway.threshold_elev_ft;
  cfg.duration_s = 300.0;

  struct Progress {
    double prev_along_m = 0.0;
    bool has_prev = false;
    bool passed_faf = false;
  };
  const Runway rwy = a.runway;
  const double faf_distance_m = a.faf_distance_m;
  const double dh_alt_ft = rwy.threshold_elev_ft + a.decision_height_ft;
  cfg.on_step = [lat = rwy.threshold_lat_deg, lon = rwy.threshold_lon_deg, course = rwy.course_deg,
                 faf_distance_m, dh_alt_ft, p = Progress{}](C172Sim&, const State& s) mutable {
    const TrackPosition fp = track_position(lat, lon, course, s.lat_deg, s.lon_deg);
    const bool on_final = std::abs(fp.cross_m) < kOnFinalCrossM;
    bool keep_going = true;
    if (p.has_prev && on_final) {
      if (p.prev_along_m < -faf_distance_m && fp.along_m >= -faf_distance_m) {
        p.passed_faf = true;
      }
      if (p.passed_faf && (s.alt_ft <= dh_alt_ft || fp.along_m >= 0.0)) keep_going = false;
    }
    p.prev_along_m = fp.along_m;
    p.has_prev = true;
    return keep_going;
  };
  return cfg;
}

RunConfig dispersed_approach_run(const Approach& a, std::uint64_t seed, const Dispersions& d) {
  RunConfig cfg = approach_run(a);
  std::mt19937_64 rng(seed);
  auto uniform = [&rng](double lo, double hi) {
    return std::uniform_real_distribution<double>(lo, hi)(rng);
  };

  InitialConditions& ic = cfg.ic;
  const double north_m = uniform(-d.start_offset_m, d.start_offset_m);
  const double east_m = uniform(-d.start_offset_m, d.start_offset_m);
  const Waypoint start = project("start", ic.lat_deg, ic.lon_deg, bearing_deg({north_m, east_m}),
                                 std::hypot(north_m, east_m), 0.0);
  ic.lat_deg = start.lat_deg;
  ic.lon_deg = start.lon_deg;
  ic.heading_deg = wrap_360(ic.heading_deg + uniform(-d.heading_deg, d.heading_deg));
  ic.alt_ft += uniform(-d.altitude_ft, d.altitude_ft);
  ic.airspeed_kts += uniform(-d.airspeed_kts, d.airspeed_kts);

  // Wind from `from_deg`
  const double wind_fps = uniform(0.0, d.max_wind_kts) * kKtsToFps;
  const double from_deg = uniform(0.0, 360.0);
  ic.wind_north_fps = -wind_fps * std::cos(from_deg * kDegToRad);
  ic.wind_east_fps = -wind_fps * std::sin(from_deg * kDegToRad);
  ic.turbulence = std::uniform_int_distribution<int>(0, d.max_turbulence)(rng);
  ic.random_seed = static_cast<unsigned int>(seed);
  return cfg;
}

std::unique_ptr<Autopilot> make_approach_autopilot(const Approach& a, const AutopilotConfig& cfg) {
  auto ap = std::make_unique<Autopilot>(cfg);
  GlideslopeArm arm;
  arm.path = a.glidepath();
  ap->modes().set_initial_modes([route = a.route, arm] {
    auto tracker = std::make_shared<RouteTracker>(route);
    return ModeManager::InitialModes{std::make_unique<RouteNav>(tracker),
                                     std::make_unique<RouteAltitude>(tracker, arm)};
  });
  return ap;
}

}  // namespace acessna
