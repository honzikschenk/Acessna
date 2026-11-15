#include "acessna/autopilot/modes.hpp"

namespace acessna {

void HeadingHold::enter(const State& s) { heading_deg_ = selected_deg_.value_or(s.psi_deg); }

LateralCommand HeadingHold::command(const State& s, double dt) {
  (void)s;
  (void)dt;
  return HeadingCommand{heading_deg_};
}

void AltitudeHold::enter(const State& s) { altitude_ft_ = selected_ft_.value_or(s.alt_ft); }

VerticalCommand AltitudeHold::command(const State& s, double dt) {
  (void)s;
  (void)dt;
  return AltitudeCommand{altitude_ft_};
}

}  // namespace acessna
