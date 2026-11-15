#include <cstdio>
#include <cstring>

#include "acessna/autopilot/autopilot.hpp"
#include "acessna/sim/harness.hpp"

int main(int argc, char** argv) {
  acessna::RunConfig cfg;
  cfg.csv_path = "run.csv";
  cfg.duration_s = 60.0;
  cfg.realtime = false;
  cfg.flightgear = false;

  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--flightgear") == 0) {
      cfg.duration_s = -1.0;
      cfg.realtime = true;
      cfg.flightgear = true;
    } else {
      std::fprintf(stderr, "usage: %s [--flightgear]\n", argv[0]);
      return 2;
    }
  }

  acessna::Autopilot autopilot;

  const auto result = acessna::run(autopilot, cfg);

  if (!result.error.empty()) {
    std::fprintf(stderr, "error: %s\n", result.error.c_str());
    return 1;
  }
  const auto& s = result.final_state();
  std::printf("t=%.1fs alt=%.0fft ias=%.1fkts hdg=%.1fdeg%s\n", s.t, s.alt_ft, s.airspeed_kts,
              s.psi_deg, result.hit_ground ? " (hit ground)" : "");
  return 0;
}
