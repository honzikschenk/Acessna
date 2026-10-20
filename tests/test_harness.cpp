// Minimal tests

#include <cmath>
#include <cstdio>
#include <functional>

#include "acessna/sim/harness.hpp"

namespace {

int failures = 0;

#define CHECK(cond)                                                    \
  do {                                                                 \
    if (!(cond)) {                                                     \
      std::fprintf(stderr, "  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      ++failures;                                                      \
    }                                                                  \
  } while (0)

// Holds the trimmed controls, optionally adding a fixed delta
struct HoldTrim : acessna::Controller {
  double d_elevator = 0.0, d_aileron = 0.0;
  acessna::Controls trim;
  void reset(const acessna::State&, const acessna::Controls& t) override { trim = t; }
  acessna::Controls update(const acessna::State&, double) override {
    acessna::Controls c = trim;
    c.elevator += d_elevator;
    c.aileron += d_aileron;
    return c;
  }
};

void test_trim_is_steady() {
  HoldTrim ctl;
  acessna::RunConfig cfg;
  cfg.duration_s = 30.0;
  auto r = acessna::run(ctl, cfg);
  const auto& s = r.final_state();
  CHECK(r.error.empty());
  CHECK(std::abs(s.t - 30.0) < 0.05);
  CHECK(std::abs(s.alt_ft - cfg.ic.alt_ft) < 20.0);
  CHECK(std::abs(s.airspeed_kts - cfg.ic.airspeed_kts) < 2.0);
  CHECK(std::abs(s.phi_deg) < 2.0);
  CHECK(s.north_m > 1000.0);  // ~100 kts north for 30 s = 1.5 km
}

void test_control_sign_conventions() {
  HoldTrim ctl;
  acessna::RunConfig cfg;
  cfg.duration_s = 2.0;
  ctl.d_elevator = -0.2;
  CHECK(acessna::run(ctl, cfg).final_state().theta_deg > 2.0);
  ctl.d_elevator = 0.0;
  ctl.d_aileron = 0.2;
  CHECK(acessna::run(ctl, cfg).final_state().phi_deg > 5.0);
}

}  // namespace

int main() {
  const std::pair<const char*, std::function<void()>> tests[] = {
      {"trim_is_steady", test_trim_is_steady},
      {"control_sign_conventions", test_control_sign_conventions},
  };
  for (const auto& [name, fn] : tests) {
    std::printf("%s\n", name);
    fn();
  }
  std::printf(failures ? "%d check(s) failed\n" : "all passed\n", failures);
  return failures ? 1 : 0;
}
