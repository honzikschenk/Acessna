#include <cstdio>

#include "acessna/control/flight_controller.hpp"
#include "acessna/sim/harness.hpp"

int main() {
  acessna::RunConfig cfg;
  cfg.csv_path = "run.csv";

  acessna::FlightController controller;

  const auto result = acessna::run(controller, cfg);

  if (!result.error.empty()) {
    std::fprintf(stderr, "error: %s\n", result.error.c_str());
    return 1;
  }
  const auto& s = result.final_state();
  std::printf("t=%.1fs alt=%.0fft ias=%.1fkts hdg=%.1fdeg%s\n", s.t, s.alt_ft, s.airspeed_kts,
              s.psi_deg, result.hit_ground ? " (hit ground)" : "");
  return 0;
}
