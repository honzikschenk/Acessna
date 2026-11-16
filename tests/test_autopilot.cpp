// Tuned autopilot regression testing, step responses and runway approach

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>

#include "acessna/autopilot/autopilot.hpp"
#include "acessna/math.hpp"
#include "acessna/scenario/kroa.hpp"
#include "acessna/scenario/metrics.hpp"
#include "acessna/sim/harness.hpp"

namespace {

using namespace acessna;

template <class Engage>
RunResult fly(Autopilot& ap, double duration_s, Engage engage) {
  RunConfig cfg;
  cfg.duration_s = duration_s;
  bool engaged = false;
  cfg.on_step = [&](C172Sim&, const State& s) {
    if (!engaged && s.t >= 1.0) {
      engage(ap);
      engaged = true;
    }
    return true;
  };
  return run(ap, cfg);
}

TEST(Autopilot, CoordinatedHeadingChange) {
  Autopilot ap;
  const RunResult r =
      fly(ap, 40.0, [](Autopilot& a) { a.modes().engage(std::make_unique<HeadingHold>(90.0)); });
  ASSERT_TRUE(r.error.empty()) << r.error;

  double max_beta = 0.0;
  double max_phi = 0.0;
  double max_alt_dev = 0.0;
  for (const Sample& s : r.log) {
    max_beta = std::max(max_beta, std::abs(s.state.beta_deg));
    max_phi = std::max(max_phi, std::abs(s.state.phi_deg));
    max_alt_dev = std::max(max_alt_dev, std::abs(s.state.alt_ft - 3000.0));
  }
  EXPECT_NEAR(wrap_180(r.final_state().psi_deg - 90.0), 0.0, 1.0);
  EXPECT_LT(max_phi, 28.0);
  EXPECT_LT(max_beta, 2.0);
  EXPECT_LT(max_alt_dev, 20.0);
}

TEST(Autopilot, ClimbHoldsAirspeed) {
  Autopilot ap;
  const RunResult r =
      fly(ap, 80.0, [](Autopilot& a) { a.modes().engage(std::make_unique<AltitudeHold>(3500.0)); });
  ASSERT_TRUE(r.error.empty()) << r.error;

  double max_vs = 0.0;
  double max_ias_dev = 0.0;
  for (const Sample& s : r.log) {
    max_vs = std::max(max_vs, s.state.vs_fpm);
    max_ias_dev = std::max(max_ias_dev, std::abs(s.state.airspeed_kts - 100.0));
  }
  EXPECT_NEAR(r.final_state().alt_ft, 3500.0, 20.0);
  EXPECT_LT(max_vs, 750.0);
  EXPECT_LT(max_ias_dev, 3.0);
}

TEST(Autopilot, FliesKroaApproachToDecisionHeight) {
  const Approach a = kroa_rwy06_approach();
  auto ap = make_approach_autopilot(a);
  const RunResult r = run(*ap, approach_run(a));
  ASSERT_TRUE(r.error.empty()) << r.error;

  const ApproachMetrics m = approach_metrics(a, r);
  ASSERT_TRUE(m.reached_faf);
  ASSERT_TRUE(m.gs_captured);
  ASSERT_TRUE(m.reached_dh);
  EXPECT_FALSE(m.hit_ground);
  EXPECT_LT(m.dh_time_s, 120.0);  // demo length
  EXPECT_LT(std::abs(m.dh_cross_track_m), 10.0);
  EXPECT_LT(std::abs(m.dh_gs_deviation_ft), 15.0);
  EXPECT_LT(m.final_max_cross_track_m, 20.0);
  EXPECT_LT(m.final_max_sink_fpm, 1000.0);
}

}  // namespace
