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

  RunResult result;
  result.log.reserve(static_cast<size_t>(cfg.duration_s * cfg.control_hz) + 1);

  State s = sim.state();
  controller.reset(s, sim.trim_controls());

  const auto wall_start = std::chrono::steady_clock::now();
  const double t0 = s.t;
  try {
    while (s.t - t0 < cfg.duration_s - 1e-9) {
      if (cfg.on_step && !cfg.on_step(sim, s)) break;
      Controls c = controller.update(s, control_dt);
      sim.apply(c);
      result.log.push_back({s, c});
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
  result.log.push_back({s, result.log.empty() ? sim.trim_controls() : result.log.back().controls});

  if (!cfg.csv_path.empty()) write_csv(cfg.csv_path, result.log);
  return result;
}

void write_csv(const std::string& path, const std::vector<Sample>& log) {
  std::ofstream out(path);
  if (!out) throw std::runtime_error("cannot open " + path);
  out << "t,north_m,east_m,lat_deg,lon_deg,alt_ft,agl_ft,phi_deg,theta_deg,psi_deg,"
         "p_dps,q_dps,r_dps,airspeed_kts,tas_kts,groundspeed_kts,track_deg,vs_fpm,"
         "alpha_deg,beta_deg,rpm,"
         "elevator,aileron,rudder,throttle,mixture,flaps,pitch_trim\n";
  out.precision(8);
  for (const auto& [s, c] : log) {
    out << s.t << ',' << s.north_m << ',' << s.east_m << ',' << s.lat_deg << ',' << s.lon_deg
        << ',' << s.alt_ft << ',' << s.agl_ft << ',' << s.phi_deg << ',' << s.theta_deg << ','
        << s.psi_deg << ',' << s.p_dps << ',' << s.q_dps << ',' << s.r_dps << ','
        << s.airspeed_kts << ',' << s.tas_kts << ',' << s.groundspeed_kts << ',' << s.track_deg
        << ',' << s.vs_fpm << ',' << s.alpha_deg << ',' << s.beta_deg << ',' << s.rpm << ','
        << c.elevator << ',' << c.aileron << ',' << c.rudder << ',' << c.throttle << ','
        << c.mixture << ',' << c.flaps << ',';
    if (c.pitch_trim) out << *c.pitch_trim;
    out << '\n';
  }
}

}  // namespace acessna
