// JSBSim wrapper with Cessna 172P
#pragma once

#include <memory>
#include <optional>
#include <string>

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
  double gamma_deg = 0.0;       // flight path angle
  double terrain_elev_ft = 0.0;
  double wind_north_fps = 0.0;  // direction the air moves *toward*
  double wind_east_fps = 0.0;
  double wind_down_fps = 0.0;
  double turbulence = 0.0;      // 0 = off, 1 light, 3 moderate, 5 severe
  bool trim = true;             // trim for steady level flight at start
};

// Pilot inputs: Surfaces are normalized [-1, 1], throttle/mixture/flaps [0, 1]
// Values are clamped before being sent to JSBSim.
struct Controls {
  double elevator = 0.0;  // + = nose down (stick forward)
  double aileron = 0.0;   // + = roll right
  double rudder = 0.0;    // + = yaw right
  double throttle = 0.75;
  double mixture = 0.87;
  double flaps = 0.0;     // 0, 1/3, 2/3, 1 = 0, 10, 20, 30 deg
  // Unset leaves the trim wheel where the initial trim put it
  // `elevator` acts as a delta about the trimmed condition
  std::optional<double> pitch_trim;
};

struct State {
  double t;               // s
  double north_m;         // displacement from start
  double east_m;
  double lat_deg;
  double lon_deg;
  double alt_ft;          // MSL
  double agl_ft;
  double phi_deg;         // roll
  double theta_deg;       // pitch
  double psi_deg;         // true heading [0, 360)
  double p_dps;           // body roll rate
  double q_dps;           // body pitch rate
  double r_dps;           // body yaw rate
  double airspeed_kts;    // calibrated
  double tas_kts;
  double groundspeed_kts;
  double track_deg;       // ground track
  double vs_fpm;          // vertical speed, + = climb
  double alpha_deg;
  double beta_deg;
  double rpm;
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
