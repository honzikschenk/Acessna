#include "acessna/autopilot/mode_manager.hpp"

namespace acessna {

ModeManager::ModeManager()
    : lateral_(std::make_unique<HeadingHold>()), vertical_(std::make_unique<AltitudeHold>()) {}

void ModeManager::reset(const State& initial) {
  pending_lateral_.reset();
  pending_vertical_.reset();
  lateral_ = std::make_unique<HeadingHold>();
  vertical_ = std::make_unique<AltitudeHold>();
  lateral_->enter(initial);
  vertical_->enter(initial);
  airspeed_kts_ = initial.airspeed_kts;
}

void ModeManager::engage(std::unique_ptr<LateralMode> mode) { pending_lateral_ = std::move(mode); }

void ModeManager::engage(std::unique_ptr<VerticalMode> mode) { pending_vertical_ = std::move(mode); }

bool ModeManager::update(const State& s) {
  bool changed = false;
  if (pending_lateral_) {
    lateral_ = std::move(pending_lateral_);
    lateral_->enter(s);
    changed = true;
  }
  if (pending_vertical_) {
    vertical_ = std::move(pending_vertical_);
    vertical_->enter(s);
    changed = true;
  }

  const ActiveModes before = active();
  auto next_lateral = lateral_->next(s, before);
  auto next_vertical = vertical_->next(s, before);
  if (next_lateral) {
    lateral_ = std::move(next_lateral);
    lateral_->enter(s);
    changed = true;
  }
  if (next_vertical) {
    vertical_ = std::move(next_vertical);
    vertical_->enter(s);
    changed = true;
  }
  return changed;
}

Target ModeManager::targets(const State& s, double dt) {
  Target t;
  t.lateral = lateral_->command(s, dt);
  t.vertical = vertical_->command(s, dt);
  t.airspeed_kts = airspeed_kts_;
  return t;
}

}  // namespace acessna
