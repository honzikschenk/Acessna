// Sim/harness sanity tests, flown with a controller that just holds trim

#include <gtest/gtest.h>

#include "acessna/sim/harness.hpp"

namespace {

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

TEST(Harness, TrimIsSteady) {
  HoldTrim ctl;
  acessna::RunConfig cfg;
  cfg.duration_s = 30.0;
  const auto r = acessna::run(ctl, cfg);
  ASSERT_TRUE(r.error.empty()) << r.error;

  const auto& s = r.final_state();
  EXPECT_NEAR(s.t, 30.0, 0.05);
  EXPECT_NEAR(s.alt_ft, cfg.ic.alt_ft, 20.0);
  EXPECT_NEAR(s.airspeed_kts, cfg.ic.airspeed_kts, 2.0);
  EXPECT_NEAR(s.phi_deg, 0.0, 2.0);
  EXPECT_GT(s.north_m, 1000.0);  // ~100 kts north for 30 s = 1.5 km
}

TEST(Harness, NegativeDurationRunsUntilGround) {
  HoldTrim ctl;
  ctl.d_elevator = 0.6;  // steep dive into the ground
  acessna::RunConfig cfg;
  cfg.duration_s = -1.0;
  cfg.on_step = [](acessna::C172Sim&, const acessna::State& s) {
    return s.t < 600.0;
  };  // safety cap
  const auto r = acessna::run(ctl, cfg);
  ASSERT_TRUE(r.error.empty()) << r.error;
  EXPECT_TRUE(r.hit_ground);
  EXPECT_GT(r.final_state().t, 1.0);
}

// Sign conventions
TEST(Harness, NegativeElevatorPitchesNoseUp) {
  HoldTrim ctl;
  ctl.d_elevator = -0.2;
  acessna::RunConfig cfg;
  cfg.duration_s = 2.0;
  EXPECT_GT(acessna::run(ctl, cfg).final_state().theta_deg, 2.0);
}

TEST(Harness, PositiveAileronRollsRight) {
  HoldTrim ctl;
  ctl.d_aileron = 0.2;
  acessna::RunConfig cfg;
  cfg.duration_s = 2.0;
  EXPECT_GT(acessna::run(ctl, cfg).final_state().phi_deg, 5.0);
}

}  // namespace
