// Autopilot modes
// Lateral and vertical modes run independently
// Each mode is a command source, a hold value or a guidance law
#pragma once

#include <memory>
#include <optional>
#include <utility>

#include "acessna/control/setpoints.hpp"
#include "acessna/guidance/glideslope.hpp"
#include "acessna/guidance/l1.hpp"
#include "acessna/guidance/route.hpp"
#include "acessna/sim/sim.hpp"

namespace acessna {

enum class LateralModeId {
  kHeadingHold,
  kRoute,
};

enum class VerticalModeId {
  kAltitudeHold,
  kRouteAltitude,
  kGlideslope,
};

struct ActiveModes {
  LateralModeId lateral;
  VerticalModeId vertical;
};

class LateralMode {
 public:
  virtual ~LateralMode() = default;

  virtual LateralModeId id() const = 0;

  // Called once when the mode engages
  virtual void enter(const State& s) { (void)s; }

  // The command for this step
  virtual LateralCommand command(const State& s, double dt) = 0;

  // Runs every step before command(). Returns the mode to switch to, or nullptr to stay
  virtual std::unique_ptr<LateralMode> next(const State& s, const ActiveModes& active) {
    (void)s;
    (void)active;
    return nullptr;
  }
};

class VerticalMode {
 public:
  virtual ~VerticalMode() = default;

  virtual VerticalModeId id() const = 0;
  virtual void enter(const State& s) { (void)s; }
  virtual VerticalCommand command(const State& s, double dt) = 0;
  virtual std::unique_ptr<VerticalMode> next(const State& s, const ActiveModes& active) {
    (void)s;
    (void)active;
    return nullptr;
  }
};

// Holds a heading, the one given, or the heading when the mode engages
class HeadingHold : public LateralMode {
 public:
  HeadingHold() = default;
  explicit HeadingHold(double heading_deg) : selected_deg_(heading_deg) {}

  LateralModeId id() const override { return LateralModeId::kHeadingHold; }
  void enter(const State& s) override;
  LateralCommand command(const State& s, double dt) override;

 private:
  std::optional<double> selected_deg_;
  double heading_deg_ = 0.0;
};

// Holds an altitude, the one given, or the altitude when the mode engages
class AltitudeHold : public VerticalMode {
 public:
  AltitudeHold() = default;
  explicit AltitudeHold(double altitude_ft) : selected_ft_(altitude_ft) {}

  VerticalModeId id() const override { return VerticalModeId::kAltitudeHold; }
  void enter(const State& s) override;
  VerticalCommand command(const State& s, double dt) override;

 private:
  std::optional<double> selected_ft_;
  double altitude_ft_ = 0.0;
};

// Flies a route with L1 guidance on each leg and fly-by transitions
struct RouteNavConfig {
  L1Config l1;
  double max_bank_deg = 25.0;
  double max_turn_lead_m = 2000.0;
};

class RouteNav : public LateralMode {
 public:
  explicit RouteNav(std::shared_ptr<RouteTracker> route, const RouteNavConfig& cfg = {})
      : route_(std::move(route)), cfg_(cfg) {}

  LateralModeId id() const override { return LateralModeId::kRoute; }
  void enter(const State& s) override;
  LateralCommand command(const State& s, double dt) override;

 private:
  std::shared_ptr<RouteTracker> route_;
  RouteNavConfig cfg_;
};

// Arms glideslope capture on final
struct GlideslopeArm {
  GlidePath path;
  GlideslopeConfig guidance;
  double capture_band_ft = 30.0;
  double max_cross_track_m = 300.0;
  double max_track_error_deg = 30.0;
};

// Holds the active waypoint's altitude
// Hands over to Glideslope on capture
class RouteAltitude : public VerticalMode {
 public:
  explicit RouteAltitude(std::shared_ptr<const RouteTracker> route,
                         std::optional<GlideslopeArm> arm = std::nullopt)
      : route_(std::move(route)), arm_(arm) {}

  VerticalModeId id() const override { return VerticalModeId::kRouteAltitude; }
  VerticalCommand command(const State& s, double dt) override;
  std::unique_ptr<VerticalMode> next(const State& s, const ActiveModes& active) override;

 private:
  std::shared_ptr<const RouteTracker> route_;
  std::optional<GlideslopeArm> arm_;
};

// Tracks the glidepath down
class Glideslope : public VerticalMode {
 public:
  Glideslope(const GlidePath& path, const GlideslopeConfig& cfg) : path_(path), cfg_(cfg) {}

  VerticalModeId id() const override { return VerticalModeId::kGlideslope; }
  VerticalCommand command(const State& s, double dt) override;

 private:
  GlidePath path_;
  GlideslopeConfig cfg_;
};

}  // namespace acessna
