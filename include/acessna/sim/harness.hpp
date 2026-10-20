// Closed-loop runner: steps JSBSim, calls Controller at a fixed rate, logs
#pragma once

#include <functional>
#include <string>
#include <vector>

#include "acessna/sim/controller.hpp"
#include "acessna/sim/sim.hpp"

namespace acessna {

struct RunConfig {
  InitialConditions ic;
  double duration_s = 60.0;
  double control_hz = 60.0;   // must divide sim_hz evenly
  double sim_hz = 120.0;      // JSBSim integration rate
  bool realtime = false;      // pace to wall clock (for FlightGear viewing)
  bool flightgear = false;    // stream to FlightGear on UDP 5550
  bool stop_on_ground = true; // end the run if AGL drops below 1 ft
  std::string csv_path;       // empty = don't write a CSV
  // Optional hook run every control step before the controller, e.g. to
  // inject failures or gusts via sim.set(...). Return false to end the run.
  std::function<bool(C172Sim&, const State&)> on_step;
};

struct Sample {
  State state;
  Controls controls;  // command issued at this state
};

struct RunResult {
  std::vector<Sample> log;
  bool hit_ground = false;
  std::string error;  // non-empty if JSBSim threw mid-run
  const State& final_state() const { return log.back().state; }
};

RunResult run(Controller& controller, const RunConfig& cfg);

void write_csv(const std::string& path, const std::vector<Sample>& log);

}  // namespace acessna
