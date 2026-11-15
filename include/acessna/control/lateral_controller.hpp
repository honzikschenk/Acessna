// Lateral controller
// heading / course / bank command -> roll setpoint
#pragma once

#include "acessna/control/pid.hpp"
#include "acessna/control/setpoints.hpp"
#include "acessna/sim/sim.hpp"
#include "acessna/telemetry.hpp"

namespace acessna {

struct LateralControllerConfig {
  PidGains heading;  // heading error -> roll setpoint
  PidGains course;   // course error -> roll setpoint
  double max_bank_deg = 25.0;
};

class LateralController : public TelemetrySource {
 public:
  explicit LateralController(const LateralControllerConfig& cfg = {}) : cfg_(cfg) {}

  void reset(const State& s);

  double update(const Target& t, const State& s, double dt);

  const LateralControllerConfig& config() const { return cfg_; }

 private:
  double track(const HeadingCommand& c, const State& s, double dt);
  double track(const CourseCommand& c, const State& s, double dt);
  double track(const BankCommand& c, const State& s, double dt);

  LateralControllerConfig cfg_;
  double hold_roll_deg_ = 0.0;
};

}  // namespace acessna
