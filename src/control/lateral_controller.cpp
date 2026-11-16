#include "acessna/control/lateral_controller.hpp"

#include <algorithm>
#include <variant>

#include "acessna/math.hpp"

namespace acessna {

LateralController::LateralController(const LateralControllerConfig& cfg)
    : cfg_(cfg),
      heading_(cfg.heading,
               {-cfg.max_bank_deg, cfg.max_bank_deg, -cfg.max_bank_deg, cfg.max_bank_deg}),
      course_(cfg.course,
              {-cfg.max_bank_deg, cfg.max_bank_deg, -cfg.max_bank_deg, cfg.max_bank_deg}) {}

void LateralController::reset(const State& s) {
  heading_.reset(s.phi_deg);
  course_.reset(s.phi_deg);
}

double LateralController::update(const Target& t, const State& s, double dt) {
  return std::visit([&](const auto& cmd) { return track(cmd, s, dt); }, t.lateral);
}

double LateralController::track(const HeadingCommand& c, const State& s, double dt) {
  return heading_.update(wrap_180(c.heading_deg - s.psi_deg), dt);
}

double LateralController::track(const CourseCommand& c, const State& s, double dt) {
  return course_.update(wrap_180(c.course_deg - s.track_deg), dt);
}

double LateralController::track(const BankCommand& c, const State& s, double dt) {
  (void)s;
  (void)dt;
  return std::clamp(c.bank_deg, -cfg_.max_bank_deg, cfg_.max_bank_deg);
}

}  // namespace acessna
