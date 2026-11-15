#include "acessna/control/lateral_controller.hpp"

#include <variant>

#include "acessna/math.hpp"

namespace acessna {

void LateralController::reset(const State& s) {
  hold_roll_deg_ = s.phi_deg;
}

double LateralController::update(const Target& t, const State& s, double dt) {
  return std::visit([&](const auto& cmd) { return track(cmd, s, dt); }, t.lateral);
}

double LateralController::track(const HeadingCommand& c, const State& s, double dt) {
  (void)c;
  (void)s;
  (void)dt;
  // TODO: wrap_180(c.heading_deg - s.psi_deg) -> roll setpoint, limit to cfg_.max_bank_deg
  return hold_roll_deg_;
}

double LateralController::track(const CourseCommand& c, const State& s, double dt) {
  (void)c;
  (void)s;
  (void)dt;
  // TODO: wrap_180(c.course_deg - s.track_deg) -> roll setpoint, limit to cfg_.max_bank_deg
  return hold_roll_deg_;
}

double LateralController::track(const BankCommand& c, const State& s, double dt) {
  (void)s;
  (void)dt;
  return c.bank_deg;  // TODO: limit to cfg_.max_bank_deg
}

}  // namespace acessna
