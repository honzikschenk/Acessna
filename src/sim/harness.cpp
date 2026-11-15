#include "acessna/sim/harness.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <thread>

namespace acessna {

RunResult run(Controller& controller, const RunConfig& cfg) {
  const double ratio = cfg.sim_hz / cfg.control_hz;
  const int substeps = static_cast<int>(std::lround(ratio));
  if (substeps < 1 || std::abs(ratio - substeps) > 1e-9) {
    throw std::invalid_argument("control_hz must evenly divide sim_hz");
  }
  const double control_dt = 1.0 / cfg.control_hz;

  C172Sim sim(cfg.ic, cfg.sim_hz);
  if (cfg.flightgear) sim.enable_flightgear_output();

  const bool unbounded = cfg.duration_s < 0;  // run until ground, error or on_step stops it

  RunResult result;
  if (!unbounded) result.log.reserve(static_cast<size_t>(cfg.duration_s * cfg.control_hz) + 1);

  Telemetry telemetry;
  controller.attach_telemetry(telemetry);

  State s = sim.state();
  controller.reset(s, sim.trim_controls());

  const auto wall_start = std::chrono::steady_clock::now();
  const double t0 = s.t;
  try {
    while (unbounded || s.t - t0 < cfg.duration_s - 1e-9) {
      if (cfg.on_step && !cfg.on_step(sim, s)) break;
      telemetry.begin_step();
      Controls c = controller.update(s, control_dt);
      sim.apply(c);
      result.log.push_back({s, c, telemetry.values()});
      s = sim.step(substeps);

      if (cfg.stop_on_ground && s.agl_ft < 1.0) {
        result.hit_ground = true;
        break;
      }
      if (cfg.realtime) {
        std::this_thread::sleep_until(
            wall_start + std::chrono::duration<double>(s.t - t0));
      }
    }
  } catch (const std::exception& e) {
    result.error = e.what();
  }
  // Final state with the last command held, so the log covers the full run
  result.log.push_back({s, result.log.empty() ? sim.trim_controls() : result.log.back().controls, {}});
  result.signal_names = telemetry.names();
  controller.attach_telemetry(Telemetry::discard());  // `telemetry` dies with this call

  if (!cfg.csv_path.empty()) write_csv(cfg.csv_path, result);
  return result;
}

void write_csv(const std::string& path, const RunResult& result) {
  std::ofstream out(path);
  if (!out) throw std::runtime_error("cannot open " + path);

  for (const auto& f : kStateFields) out << f.name << ',';
  out << "elevator,aileron,rudder,throttle,mixture,flaps,pitch_trim";
  for (const auto& name : result.signal_names) out << ',' << name;
  out << '\n';

  out.precision(8);
  for (const auto& [s, c, signals] : result.log) {
    for (const auto& f : kStateFields) out << s.*f.field << ',';
    out << c.elevator << ',' << c.aileron << ',' << c.rudder << ',' << c.throttle << ','
        << c.mixture << ',' << c.flaps << ',';
    if (c.pitch_trim) out << *c.pitch_trim;
    for (std::size_t i = 0; i < result.signal_names.size(); ++i) {
      out << ',';
      if (i < signals.size() && !std::isnan(signals[i])) out << signals[i];
    }
    out << '\n';
  }
}

}  // namespace acessna
