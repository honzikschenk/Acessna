// JSBSim wrapper with Cessna 172P
#pragma once

#include <memory>
#include <optional>
#include <string>

#include "acessna/math.hpp"

namespace JSBSim {
class FGFDMExec;
}

namespace acessna {

struct InitialConditions {
  double alt_ft = 3000.0;       // MSL
  double airspeed_kts = 100.0;  // calibrated
  double heading_deg = 0.0;     // true
  double lat_deg = 37.325776;
  double lon_deg = -79.978586;
  double gamma_deg = 0.0;  // flight path angle
  double terrain_elev_ft = 0.0;
  double wind_north_fps = 0.0;  // direction the air moves *toward*
  double wind_east_fps = 0.0;
  double wind_down_fps = 0.0;
  double turbulence = 0.0;       // 0 = off, 1 light, 3 moderate, 5 severe
  unsigned int random_seed = 0;  // JSBSim RNG seed (turbulence)
  bool trim = true;              // trim for steady level flight at start
};

// Pilot inputs: Surfaces are normalized [-1, 1], throttle/mixture/flaps [0, 1]
// Values are clamped before being sent to JSBSim.
struct Controls {
  double elevator = 0.0;  // + = nose down (stick forward)
  double aileron = 0.0;   // + = roll right
  double rudder = 0.0;    // + = yaw right
  double throttle = 0.75;
  double mixture = 0.87;
  double flaps = 0.0;  // 0, 1/3, 2/3, 1 = 0, 10, 20, 30 deg
  // Unset leaves the trim wheel where the initial trim put it
  // `elevator` acts as a delta about the trimmed condition
  std::optional<double> pitch_trim;
};

struct State {
  double t;        // s
  double north_m;  // displacement from start
  double east_m;
  double lat_deg;
  double lon_deg;
  double alt_ft;  // MSL
  double agl_ft;
  double phi_deg;       // roll
  double theta_deg;     // pitch
  double psi_deg;       // true heading [0, 360)
  double p_dps;         // body roll rate
  double q_dps;         // body pitch rate
  double r_dps;         // body yaw rate
  double airspeed_kts;  // calibrated
  double tas_kts;
  double groundspeed_kts;
  double track_deg;  // ground track
  double vs_fpm;     // vertical speed, + = climb
  double alpha_deg;
  double beta_deg;
  double rpm;
};

// One entry per State field: CSV column name, JSBSim property and scale to State units
struct StateField {
  double State::* field;
  const char* name;
  const char* prop;
  double scale;
};

inline constexpr StateField kStateFields[] = {
    {&State::t, "t", "simulation/sim-time-sec", 1.0},
    {&State::north_m, "north_m", "position/distance-from-start-lat-mt", 1.0},
    {&State::east_m, "east_m", "position/distance-from-start-lon-mt", 1.0},
    {&State::lat_deg, "lat_deg", "position/lat-geod-deg", 1.0},
    {&State::lon_deg, "lon_deg", "position/long-gc-deg", 1.0},
    {&State::alt_ft, "alt_ft", "position/h-sl-ft", 1.0},
    {&State::agl_ft, "agl_ft", "position/h-agl-ft", 1.0},
    {&State::phi_deg, "phi_deg", "attitude/phi-deg", 1.0},
    {&State::theta_deg, "theta_deg", "attitude/theta-deg", 1.0},
    {&State::psi_deg, "psi_deg", "attitude/psi-deg", 1.0},
    {&State::p_dps, "p_dps", "velocities/p-rad_sec", kRadToDeg},
    {&State::q_dps, "q_dps", "velocities/q-rad_sec", kRadToDeg},
    {&State::r_dps, "r_dps", "velocities/r-rad_sec", kRadToDeg},
    {&State::airspeed_kts, "airspeed_kts", "velocities/vc-kts", 1.0},
    {&State::tas_kts, "tas_kts", "velocities/vtrue-kts", 1.0},
    {&State::groundspeed_kts, "groundspeed_kts", "velocities/vg-fps", kFpsToKts},
    {&State::track_deg, "track_deg", "flight-path/psi-gt-rad", kRadToDeg},
    {&State::vs_fpm, "vs_fpm", "velocities/h-dot-fps", 60.0},
    {&State::alpha_deg, "alpha_deg", "aero/alpha-deg", 1.0},
    {&State::beta_deg, "beta_deg", "aero/beta-deg", 1.0},
    {&State::rpm, "rpm", "propulsion/engine/engine-rpm", 1.0},
};

class C172Sim {
 public:
  explicit C172Sim(const InitialConditions& ic = {}, double sim_hz = 120.0);
  ~C172Sim();
  C172Sim(const C172Sim&) = delete;
  C172Sim& operator=(const C172Sim&) = delete;

  // Resets the FDM
  State reset();
  State reset(const InitialConditions& ic);

  void apply(const Controls& c);
  State step(int n = 1);  // advance n integration steps of dt()
  State state() const;

  double dt() const { return 1.0 / sim_hz_; }
  const InitialConditions& ic() const { return ic_; }
  // Control positions right after trim (pitch_trim is set)
  const Controls& trim_controls() const { return trim_controls_; }

  // Raw access to JSBSim properties
  double get(const std::string& prop) const;
  void set(const std::string& prop, double value);

  // Stream state to FlightGear over UDP
  void enable_flightgear_output();

  JSBSim::FGFDMExec& fdm() { return *fdm_; }

  // Directory containing aircraft/, engine/, systems/
  static std::string root_dir();

 private:
  InitialConditions ic_;
  double sim_hz_;
  std::unique_ptr<JSBSim::FGFDMExec> fdm_;
  Controls trim_controls_;
};

}  // namespace acessna
