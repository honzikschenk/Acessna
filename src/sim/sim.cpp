#include "acessna/sim/sim.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <mutex>
#include <stdexcept>

#include "FGFDMExec.h"
#include "input_output/FGPropertyManager.h"
#include "models/FGOutput.h"
#include "simgear/misc/sg_path.hxx"

namespace acessna {

std::string C172Sim::root_dir() {
  if (const char* env = std::getenv("JSBSIM_ROOT")) return env;
  return ACESSNA_JSBSIM_ROOT;
}

C172Sim::C172Sim(const InitialConditions& ic, double sim_hz) : ic_(ic), sim_hz_(sim_hz) {
  reset();
}

C172Sim::~C172Sim() = default;

State C172Sim::reset(const InitialConditions& ic) {
  ic_ = ic;
  return reset();
}

State C172Sim::reset() {
  // Prevents JSBSim console output from race conditions via Monte Carlo simulations
  static std::once_flag debug_once;
  std::call_once(debug_once, [] {
    if (!std::getenv("JSBSIM_DEBUG")) JSBSim::FGJSBBase::debug_lvl = 0;
  });

  auto fdm = std::make_unique<JSBSim::FGFDMExec>();
  fdm->SetRootDir(SGPath(root_dir()));
  fdm->SetAircraftPath(SGPath("aircraft"));
  fdm->SetEnginePath(SGPath("engine"));
  fdm->SetSystemsPath(SGPath("systems"));
  fdm->Setdt(dt());
  if (!fdm->LoadModel("c172p")) {
    throw std::runtime_error("failed to load c172p from " + root_dir());
  }

  auto pm = fdm->GetPropertyManager();
  for (const auto& sp : kStateFields) {
    if (!pm->HasNode(sp.prop)) throw std::runtime_error(std::string("missing property ") + sp.prop);
  }

  fdm->SetPropertyValue("simulation/randomseed", ic_.random_seed);
  fdm->SetPropertyValue("ic/lat-geod-deg", ic_.lat_deg);
  fdm->SetPropertyValue("ic/long-gc-deg", ic_.lon_deg);
  fdm->SetPropertyValue("ic/terrain-elevation-ft", ic_.terrain_elev_ft);
  fdm->SetPropertyValue("ic/h-sl-ft", ic_.alt_ft);
  fdm->SetPropertyValue("ic/psi-true-deg", ic_.heading_deg);
  fdm->SetPropertyValue("ic/gamma-deg", ic_.gamma_deg);
  fdm->SetPropertyValue("ic/vc-kts", ic_.airspeed_kts);
  fdm->SetPropertyValue("atmosphere/wind-north-fps", ic_.wind_north_fps);
  fdm->SetPropertyValue("atmosphere/wind-east-fps", ic_.wind_east_fps);
  fdm->SetPropertyValue("atmosphere/wind-down-fps", ic_.wind_down_fps);
  if (ic_.turbulence > 0) {
    fdm->SetPropertyValue("atmosphere/turb-type", 3);  // Milspec Tustin
    fdm->SetPropertyValue("atmosphere/turbulence/milspec/severity", ic_.turbulence);
    fdm->SetPropertyValue("atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps",
                          15.0 * ic_.turbulence);
  }

  if (!fdm->RunIC()) throw std::runtime_error("JSBSim RunIC() failed");
  fdm->SetPropertyValue("propulsion/set-running", -1);  // start all engines
  const Controls defaults;
  fdm->SetPropertyValue("fcs/mixture-cmd-norm", defaults.mixture);
  fdm->SetPropertyValue("fcs/throttle-cmd-norm", defaults.throttle);
  if (ic_.trim) {
    try {
      fdm->DoTrim(1);  // full trim: longitudinal + lateral
    } catch (const JSBSim::TrimFailureException& e) {
      throw std::runtime_error(std::string("trim failed: ") + e.what());
    }
  }

  fdm_ = std::move(fdm);
  trim_controls_.elevator = get("fcs/elevator-cmd-norm");
  trim_controls_.aileron = get("fcs/aileron-cmd-norm");
  trim_controls_.rudder = get("fcs/rudder-cmd-norm");
  trim_controls_.throttle = get("fcs/throttle-cmd-norm");
  trim_controls_.mixture = get("fcs/mixture-cmd-norm");
  trim_controls_.flaps = get("fcs/flap-cmd-norm");
  trim_controls_.pitch_trim = get("fcs/pitch-trim-cmd-norm");
  return state();
}

void C172Sim::apply(const Controls& c) {
  set("fcs/elevator-cmd-norm", std::clamp(c.elevator, -1.0, 1.0));
  set("fcs/aileron-cmd-norm", std::clamp(c.aileron, -1.0, 1.0));
  set("fcs/rudder-cmd-norm", std::clamp(c.rudder, -1.0, 1.0));
  set("fcs/throttle-cmd-norm", std::clamp(c.throttle, 0.0, 1.0));
  set("fcs/mixture-cmd-norm", std::clamp(c.mixture, 0.0, 1.0));
  set("fcs/flap-cmd-norm", std::clamp(c.flaps, 0.0, 1.0));
  if (c.pitch_trim) set("fcs/pitch-trim-cmd-norm", std::clamp(*c.pitch_trim, -1.0, 1.0));
}

State C172Sim::step(int n) {
  for (int i = 0; i < n; ++i) {
    if (!fdm_->Run()) throw std::runtime_error("JSBSim Run() returned false");
  }
  return state();
}

State C172Sim::state() const {
  State s{};
  for (const auto& sp : kStateFields) s.*sp.field = fdm_->GetPropertyValue(sp.prop) * sp.scale;
  s.track_deg = wrap_360(s.track_deg);
  return s;
}

double C172Sim::get(const std::string& prop) const { return fdm_->GetPropertyValue(prop); }

void C172Sim::set(const std::string& prop, double value) { fdm_->SetPropertyValue(prop, value); }

void C172Sim::enable_flightgear_output() {
  SGPath directive = SGPath(root_dir()) / "data_output" / "flightgear.xml";
  if (!fdm_->SetOutputDirectives(directive)) {
    throw std::runtime_error("failed to load " + directive.utf8Str());
  }
  // Sockets are only opened in InitModel
  fdm_->GetOutput()->InitModel();
}

}  // namespace acessna
