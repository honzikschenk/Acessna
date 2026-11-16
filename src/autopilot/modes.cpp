#include "acessna/autopilot/modes.hpp"

#include <algorithm>
#include <cmath>

#include "acessna/math.hpp"

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

void RouteNav::enter(const State& s) { route_->start(s); }

LateralCommand RouteNav::command(const State& s, double dt) {
  (void)dt;
  // Fly-by lead
  const double v_mps = s.groundspeed_kts * kKtsToFps * kFtToM;
  const double radius_m =
      v_mps * v_mps / (kGravityFps2 * kFtToM * std::tan(cfg_.max_bank_deg * kDegToRad));
  const double turn = std::min(std::abs(route_->turn_deg()), 150.0) * kDegToRad;
  const double lead_m = std::min(radius_m * std::tan(turn / 2.0), cfg_.max_turn_lead_m);
  route_->update(s, lead_m);

  const Leg& leg = route_->leg();
  return BankCommand{
      l1_track_line(cfg_.l1, leg.from_lat_deg, leg.from_lon_deg, leg.course_deg, s).bank_deg};
}

VerticalCommand RouteAltitude::command(const State& s, double dt) {
  (void)s;
  (void)dt;
  return AltitudeCommand{route_->active().alt_ft};
}

std::unique_ptr<VerticalMode> RouteAltitude::next(const State& s, const ActiveModes& active) {
  (void)active;
  if (!arm_) return nullptr;
  const GlidePath& gp = arm_->path;
  const TrackPosition tp = track_position(gp.threshold_lat_deg, gp.threshold_lon_deg, gp.course_deg,
                                          s.lat_deg, s.lon_deg);
  const bool on_final =
      tp.along_m < 0.0 && std::abs(tp.cross_m) <= arm_->max_cross_track_m &&
      std::abs(wrap_180(s.track_deg - gp.course_deg)) <= arm_->max_track_error_deg;
  if (on_final && s.alt_ft >= glidepath_height_ft(gp, s) - arm_->capture_band_ft) {
    return std::make_unique<Glideslope>(gp, arm_->guidance);
  }
  return nullptr;
}

VerticalCommand Glideslope::command(const State& s, double dt) {
  (void)dt;
  return FlightPathCommand{glideslope_flight_path_deg(cfg_, path_, s)};
}

}  // namespace acessna
