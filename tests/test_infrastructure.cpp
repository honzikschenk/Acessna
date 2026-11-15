// Tests for the framework around the control laws: math helpers, telemetry, modes,
// CSV logging, and the Monte Carlo runner

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <string>
#include <variant>

#include "acessna/autopilot/autopilot.hpp"
#include "acessna/math.hpp"
#include "acessna/sim/harness.hpp"
#include "acessna/sim/monte_carlo.hpp"
#include "acessna/telemetry.hpp"

namespace {

using namespace acessna;

State state_at(double t, double psi_deg = 10.0, double alt_ft = 3000.0) {
  State s{};
  s.t = t;
  s.psi_deg = psi_deg;
  s.alt_ft = alt_ft;
  s.airspeed_kts = 100.0;
  return s;
}

TEST(Math, WrapAngles) {
  EXPECT_DOUBLE_EQ(wrap_180(190.0), -170.0);
  EXPECT_DOUBLE_EQ(wrap_180(-190.0), 170.0);
  EXPECT_DOUBLE_EQ(wrap_180(180.0), -180.0);
  EXPECT_DOUBLE_EQ(wrap_180(-180.0), -180.0);
  EXPECT_DOUBLE_EQ(wrap_180(720.0 + 5.0), 5.0);
  EXPECT_DOUBLE_EQ(wrap_360(-10.0), 350.0);
  EXPECT_DOUBLE_EQ(wrap_360(360.0), 0.0);
}

TEST(Telemetry, RecordsInFirstUseOrderAndClearsEachStep) {
  Telemetry t;
  t.record("b", 2.0);
  t.record("a", 1.0);
  t.record("b", 3.0);
  ASSERT_EQ(t.names(), (std::vector<std::string>{"b", "a"}));
  EXPECT_EQ(t.values(), (std::vector<double>{3.0, 1.0}));

  t.begin_step();
  EXPECT_TRUE(std::isnan(t.values()[0]));
  EXPECT_TRUE(std::isnan(t.values()[1]));
}

TEST(Telemetry, DiscardIgnoresRecords) {
  Telemetry::discard().record("x", 1.0);
  EXPECT_TRUE(Telemetry::discard().names().empty());
}

TEST(ModeManager, ResetHoldsInitialHeadingAltitudeAndAirspeed) {
  ModeManager m;
  m.reset(state_at(0.0, 42.0, 3500.0));
  EXPECT_EQ(m.lateral_mode(), LateralModeId::kHeadingHold);
  EXPECT_EQ(m.vertical_mode(), VerticalModeId::kAltitudeHold);

  EXPECT_FALSE(m.update(state_at(1.0, 50.0, 3600.0)));
  const Target t = m.targets(state_at(1.0, 50.0, 3600.0), 1.0 / 60);
  EXPECT_DOUBLE_EQ(std::get<HeadingCommand>(t.lateral).heading_deg, 42.0);
  EXPECT_DOUBLE_EQ(std::get<AltitudeCommand>(t.vertical).altitude_ft, 3500.0);
  EXPECT_DOUBLE_EQ(t.airspeed_kts, 100.0);
}

TEST(ModeManager, EngageTakesEffectOnNextUpdateAndReportsChange) {
  ModeManager m;
  m.reset(state_at(0.0));
  m.engage(std::make_unique<HeadingHold>(270.0));
  m.engage(std::make_unique<AltitudeHold>());  // captures altitude at engagement

  EXPECT_TRUE(m.update(state_at(1.0, 10.0, 3200.0)));
  const Target t = m.targets(state_at(1.0), 1.0 / 60);
  EXPECT_DOUBLE_EQ(std::get<HeadingCommand>(t.lateral).heading_deg, 270.0);
  EXPECT_DOUBLE_EQ(std::get<AltitudeCommand>(t.vertical).altitude_ft, 3200.0);
  EXPECT_FALSE(m.update(state_at(2.0)));
}

// Holds bank until t > 1 s, then hands over to a heading hold
struct BankThenHeading : LateralMode {
  LateralModeId id() const override { return LateralModeId::kHeadingHold; }
  LateralCommand command(const State&, double) override { return BankCommand{15.0}; }
  std::unique_ptr<LateralMode> next(const State& s, const ActiveModes&) override {
    if (s.t > 1.0) return std::make_unique<HeadingHold>(90.0);
    return nullptr;
  }
};

TEST(ModeManager, ModeTransitionsItself) {
  ModeManager m;
  m.reset(state_at(0.0));
  m.engage(std::make_unique<BankThenHeading>());
  EXPECT_TRUE(m.update(state_at(0.5)));
  EXPECT_TRUE(std::holds_alternative<BankCommand>(m.targets(state_at(0.5), 0.1).lateral));

  EXPECT_TRUE(m.update(state_at(1.5)));
  const Target t = m.targets(state_at(1.5), 0.1);
  EXPECT_DOUBLE_EQ(std::get<HeadingCommand>(t.lateral).heading_deg, 90.0);
}

TEST(Harness, AutopilotTelemetryIsLoggedToCsv) {
  Autopilot ap;
  RunConfig cfg;
  cfg.duration_s = 0.5;
  cfg.csv_path = testing::TempDir() + "acessna_telemetry.csv";
  const RunResult r = run(ap, cfg);
  ASSERT_TRUE(r.error.empty()) << r.error;

  ASSERT_FALSE(r.signal_names.empty());
  EXPECT_EQ(r.log.front().signals.size(), r.signal_names.size());
  EXPECT_TRUE(r.log.back().signals.empty());  // final state, no controller step

  std::ifstream in(cfg.csv_path);
  std::string header, row;
  ASSERT_TRUE(std::getline(in, header));
  ASSERT_TRUE(std::getline(in, row));
  EXPECT_EQ(header.rfind("t,north_m,east_m,", 0), 0u);
  EXPECT_NE(header.find(",pitch_trim,mode.lateral,"), std::string::npos);
  EXPECT_NE(header.find("tgt.heading_deg"), std::string::npos);
  EXPECT_NE(header.find("att_sp.roll_deg"), std::string::npos);
  EXPECT_EQ(std::count(header.begin(), header.end(), ','), std::count(row.begin(), row.end(), ','));
}

// Holds trim, for exercising the Monte Carlo runner
struct HoldTrim : Controller {
  Controls trim;
  void reset(const State&, const Controls& t) override { trim = t; }
  Controls update(const State&, double) override { return trim; }
};

std::vector<double> turbulent_runs(int threads) {
  MonteCarloConfig mc;
  mc.runs = 4;
  mc.base_seed = 7;
  mc.threads = threads;
  return run_monte_carlo<double>(
      mc, [] { return std::make_unique<HoldTrim>(); },
      [](std::uint64_t seed) {
        RunConfig cfg;
        cfg.duration_s = 3.0;
        cfg.ic.turbulence = 3.0;
        cfg.ic.random_seed = static_cast<unsigned int>(seed);
        return cfg;
      },
      [](std::uint64_t, const RunResult& r) { return r.final_state().phi_deg; });
}

TEST(MonteCarlo, SeedsAreReproducibleAcrossThreadCounts) {
  const auto serial = turbulent_runs(1);
  const auto parallel = turbulent_runs(4);
  ASSERT_EQ(serial.size(), 4u);
  EXPECT_EQ(serial, parallel);
  EXPECT_NE(serial[0], serial[1]);  // different seeds, different turbulence
}

}  // namespace
