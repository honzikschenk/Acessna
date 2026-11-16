#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fstream>
#include <string>
#include <vector>

#include "acessna/math.hpp"
#include "acessna/scenario/kroa.hpp"
#include "acessna/scenario/metrics.hpp"
#include "acessna/sim/harness.hpp"
#include "acessna/sim/monte_carlo.hpp"

namespace {

using namespace acessna;

int usage(const char* argv0) {
  std::fprintf(stderr,
               "usage: %s [--flightgear]\n"
               "       %s --monte-carlo RUNS [--seed SEED] [--threads N]\n",
               argv0, argv0);
  return 2;
}

void print_metrics(const ApproachMetrics& m) {
  if (m.reached_faf) {
    std::printf("FAF: t=%.0fs cross-track=%+.0fm alt error=%+.0fft\n", m.faf_time_s,
                m.faf_cross_track_m, m.faf_alt_error_ft);
  }
  if (m.gs_captured) std::printf("glideslope captured: t=%.0fs\n", m.gs_capture_time_s);
  if (m.reached_dh) {
    std::printf(
        "DH: t=%.0fs cross-track=%+.1fm glidepath deviation=%+.1fft ias=%.1fkts sink=%.0ffpm\n",
        m.dh_time_s, m.dh_cross_track_m, m.dh_gs_deviation_ft, m.dh_airspeed_kts, m.dh_sink_fpm);
    std::printf(
        "final: max |cross-track|=%.1fm max |glidepath deviation|=%.1fft max sink=%.0ffpm\n",
        m.final_max_cross_track_m, m.final_max_gs_deviation_ft, m.final_max_sink_fpm);
  } else {
    std::printf("did not reach decision height%s\n", m.hit_ground ? " (hit ground)" : "");
  }
  std::printf("run: max bank=%.1fdeg max sideslip=%.1fdeg min ias=%.1fkts\n", m.max_bank_deg,
              m.max_sideslip_deg, m.min_airspeed_kts);
}

int fly_nominal(const Approach& a, bool flightgear) {
  RunConfig cfg = approach_run(a);
  cfg.csv_path = "run.csv";
  if (flightgear) {
    cfg.realtime = true;
    cfg.flightgear = true;
  }

  auto autopilot = make_approach_autopilot(a);
  const RunResult result = run(*autopilot, cfg);
  if (!result.error.empty()) {
    std::fprintf(stderr, "error: %s\n", result.error.c_str());
    return 1;
  }
  print_metrics(approach_metrics(a, result));
  return 0;
}

// Mean, 95%, and max
void print_stat(const char* name, std::vector<double> v) {
  v.erase(std::remove_if(v.begin(), v.end(), [](double x) { return !std::isfinite(x); }), v.end());
  if (v.empty()) {
    std::printf("  %-28s no data\n", name);
    return;
  }
  std::sort(v.begin(), v.end());
  double sum = 0.0;
  for (double x : v) sum += x;
  const auto n = static_cast<double>(v.size());
  const double p95 = v[static_cast<std::size_t>(std::ceil(0.95 * n)) - 1];
  std::printf("  %-28s mean %8.1f   p95 %8.1f   max %8.1f\n", name, sum / n, p95, v.back());
}

int fly_monte_carlo(const Approach& a, int runs, std::uint64_t seed, int threads) {
  MonteCarloConfig mc;
  mc.runs = runs;
  mc.base_seed = seed;
  mc.threads = threads;

  const auto metrics = run_monte_carlo<ApproachMetrics>(
      mc, [&a] { return make_approach_autopilot(a); },
      [&a](std::uint64_t s) { return dispersed_approach_run(a, s); },
      [&a](std::uint64_t, const RunResult& r) { return approach_metrics(a, r); });

  // Columns: CSV name, summary label (empty = CSV only), value
  struct Column {
    const char* csv;
    const char* label;
    double (*get)(const ApproachMetrics&);
  };
  static const Column kColumns[] = {
      {"faf_cross_track_m", "|cross-track| at FAF (m)",
       [](const ApproachMetrics& m) { return m.faf_cross_track_m; }},
      {"faf_alt_error_ft", "|alt error| at FAF (ft)",
       [](const ApproachMetrics& m) { return m.faf_alt_error_ft; }},
      {"gs_capture_time_s", "", [](const ApproachMetrics& m) { return m.gs_capture_time_s; }},
      {"dh_time_s", "time to DH (s)", [](const ApproachMetrics& m) { return m.dh_time_s; }},
      {"dh_cross_track_m", "|cross-track| at DH (m)",
       [](const ApproachMetrics& m) { return m.dh_cross_track_m; }},
      {"dh_gs_deviation_ft", "|glidepath dev| at DH (ft)",
       [](const ApproachMetrics& m) { return m.dh_gs_deviation_ft; }},
      {"dh_airspeed_kts", "", [](const ApproachMetrics& m) { return m.dh_airspeed_kts; }},
      {"dh_sink_fpm", "sink at DH (fpm)", [](const ApproachMetrics& m) { return m.dh_sink_fpm; }},
      {"final_max_cross_track_m", "max |cross-track| FAF-DH (m)",
       [](const ApproachMetrics& m) { return m.final_max_cross_track_m; }},
      {"final_max_gs_deviation_ft", "max |glidepath dev| (ft)",
       [](const ApproachMetrics& m) { return m.final_max_gs_deviation_ft; }},
      {"final_max_sink_fpm", "max sink on glideslope (fpm)",
       [](const ApproachMetrics& m) { return m.final_max_sink_fpm; }},
      {"final_max_gs_deviation_deg", "max |glidepath dev| (deg)",
       [](const ApproachMetrics& m) { return m.final_max_gs_deviation_deg; }},
      {"loc_overshoot_deg", "localizer overshoot (deg)",
       [](const ApproachMetrics& m) { return m.loc_overshoot_deg; }},
      {"max_airspeed_error_kts", "max |ias error| <1000ft (kts)",
       [](const ApproachMetrics& m) { return m.max_airspeed_error_kts; }},
      {"max_sink_below_1000ft_fpm", "max sink <1000ft (fpm)",
       [](const ApproachMetrics& m) { return m.max_sink_below_1000ft_fpm; }},
      {"max_bank_below_500ft_deg", "max bank <500ft (deg)",
       [](const ApproachMetrics& m) { return m.max_bank_below_500ft_deg; }},
      {"max_bank_deg", "max bank (deg)", [](const ApproachMetrics& m) { return m.max_bank_deg; }},
      {"max_sideslip_deg", "max sideslip (deg)",
       [](const ApproachMetrics& m) { return m.max_sideslip_deg; }},
      {"min_airspeed_kts", "min airspeed (kts)",
       [](const ApproachMetrics& m) { return m.min_airspeed_kts; }},
  };

  std::ofstream csv("monte_carlo.csv");
  csv << "seed,wind_kts,wind_from_deg,turbulence,reached_faf,gs_captured,reached_dh,hit_ground,"
         "error";
  for (const Column& c : kColumns) csv << ',' << c.csv;
  csv << '\n';
  int faf = 0;
  int captured = 0;
  int dh = 0;
  int ground = 0;
  int errors = 0;
  for (std::size_t i = 0; i < metrics.size(); ++i) {
    const ApproachMetrics& m = metrics[i];
    faf += m.reached_faf;
    captured += m.gs_captured;
    dh += m.reached_dh;
    ground += m.hit_ground;
    errors += !m.error.empty();
    // Dispersions, regenerated from the seed
    const InitialConditions ic = dispersed_approach_run(a, seed + i).ic;
    const double wind_kts = std::hypot(ic.wind_north_fps, ic.wind_east_fps) * kFpsToKts;
    const double wind_from_deg =
        wrap_360(std::atan2(-ic.wind_east_fps, -ic.wind_north_fps) * kRadToDeg);
    csv << seed + i << ',' << wind_kts << ',' << wind_from_deg << ',' << ic.turbulence << ','
        << m.reached_faf << ',' << m.gs_captured << ',' << m.reached_dh << ',' << m.hit_ground
        << ',' << (m.error.empty() ? 0 : 1);
    for (const Column& c : kColumns) csv << ',' << c.get(m);
    csv << '\n';
  }

  std::printf(
      "%d runs (seeds %llu -> %llu): %d reached the FAF, %d captured the glideslope, "
      "%d reached DH, %d hit the ground, %d errors\n",
      runs, static_cast<unsigned long long>(seed), static_cast<unsigned long long>(seed + runs - 1),
      faf, captured, dh, ground, errors);
  for (const Column& c : kColumns) {
    if (*c.label == '\0') continue;
    std::vector<double> v;
    v.reserve(metrics.size());
    for (const ApproachMetrics& m : metrics) v.push_back(std::abs(c.get(m)));
    print_stat(c.label, v);
  }
  std::printf("result written to monte_carlo.csv\n");
  return errors > 0 ? 1 : 0;
}

// Parses an int argument
bool parse_count(const char* s, long long& out) {
  char* end = nullptr;
  errno = 0;
  out = std::strtoll(s, &end, 10);
  return errno == 0 && end != s && *end == '\0' && out >= 0;
}

int run_main(int argc, char** argv) {
  bool flightgear = false;
  long long mc_runs = 0;
  long long seed = 1;
  long long threads = 0;

  for (int i = 1; i < argc; ++i) {
    const bool has_value = i + 1 < argc;
    if (std::strcmp(argv[i], "--flightgear") == 0) {
      flightgear = true;
    } else if (std::strcmp(argv[i], "--monte-carlo") == 0 && has_value) {
      if (!parse_count(argv[++i], mc_runs)) return usage(argv[0]);
    } else if (std::strcmp(argv[i], "--seed") == 0 && has_value) {
      if (!parse_count(argv[++i], seed)) return usage(argv[0]);
    } else if (std::strcmp(argv[i], "--threads") == 0 && has_value) {
      if (!parse_count(argv[++i], threads)) return usage(argv[0]);
    } else {
      return usage(argv[0]);
    }
  }
  if (flightgear && mc_runs > 0) return usage(argv[0]);

  const Approach approach = kroa_rwy06_approach();
  if (mc_runs > 0) {
    return fly_monte_carlo(approach, static_cast<int>(mc_runs), static_cast<std::uint64_t>(seed),
                           static_cast<int>(threads));
  }
  return fly_nominal(approach, flightgear);
}

}  // namespace

int main(int argc, char** argv) {
  try {
    return run_main(argc, argv);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
