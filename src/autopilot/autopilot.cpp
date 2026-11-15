#include "acessna/autopilot/autopilot.hpp"

#include <variant>

namespace acessna {

Autopilot::Autopilot(const AutopilotConfig& cfg)
    : flight_path_(cfg.flight_path), attitude_(cfg.attitude), rate_(cfg.rate) {}

void Autopilot::attach_telemetry(Telemetry& t) {
  tlm_ = &t;
  flight_path_.attach_telemetry(t);
  attitude_.attach_telemetry(t);
  rate_.attach_telemetry(t);
}

void Autopilot::reset(const State& initial, const Controls& trim) {
  trim_ = trim;
  last_ = trim;
  modes_.reset(initial);
  flight_path_.reset(initial, trim);
  attitude_.reset(initial);
  rate_.reset(initial, trim);
}

Controls Autopilot::update(const State& s, double dt) {
  if (modes_.update(s)) flight_path_.reset(s, last_);  // bumpless transfer on mode change

  const Target targets = modes_.targets(s, dt);
  const FlightPathOutput path = flight_path_.update(targets, s, dt);
  const RateSetpoint rates = attitude_.update(path.attitude, s, dt);
  const SurfaceCommands surfaces = rate_.update(rates, s, dt);
  record(targets, path, rates, surfaces);

  Controls c = trim_;  // start from the trimmed positions
  c.elevator = surfaces.elevator;
  c.aileron = surfaces.aileron;
  c.rudder = surfaces.rudder;
  c.throttle = path.throttle;
  last_ = c;
  return c;
}

void Autopilot::record(const Target& t, const FlightPathOutput& path, const RateSetpoint& rates,
                       const SurfaceCommands& surfaces) {
  Telemetry& tlm = *tlm_;
  tlm.record("mode.lateral", static_cast<double>(modes_.lateral_mode()));
  tlm.record("mode.vertical", static_cast<double>(modes_.vertical_mode()));

  struct {
    Telemetry& tlm;
    void operator()(const HeadingCommand& c) { tlm.record("tgt.heading_deg", c.heading_deg); }
    void operator()(const CourseCommand& c) { tlm.record("tgt.course_deg", c.course_deg); }
    void operator()(const BankCommand& c) { tlm.record("tgt.bank_deg", c.bank_deg); }
    void operator()(const AltitudeCommand& c) { tlm.record("tgt.altitude_ft", c.altitude_ft); }
    void operator()(const FlightPathCommand& c) {
      tlm.record("tgt.flight_path_deg", c.flight_path_deg);
    }
    void operator()(const ClimbRateCommand& c) {
      tlm.record("tgt.climb_rate_fpm", c.climb_rate_fpm);
    }
  } rec{tlm};
  std::visit(rec, t.lateral);
  std::visit(rec, t.vertical);
  tlm.record("tgt.airspeed_kts", t.airspeed_kts);

  tlm.record("att_sp.roll_deg", path.attitude.roll_deg);
  tlm.record("att_sp.pitch_deg", path.attitude.pitch_deg);
  tlm.record("att_sp.sideslip_deg", path.attitude.sideslip_deg);
  tlm.record("rate_sp.p_dps", rates.roll_rate_dps);
  tlm.record("rate_sp.q_dps", rates.pitch_rate_dps);
  tlm.record("rate_sp.r_dps", rates.yaw_rate_dps);
  tlm.record("cmd.elevator", surfaces.elevator);
  tlm.record("cmd.aileron", surfaces.aileron);
  tlm.record("cmd.rudder", surfaces.rudder);
  tlm.record("cmd.throttle", path.throttle);
}

}  // namespace acessna
