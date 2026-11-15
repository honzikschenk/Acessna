// Autopilot modes
// Lateral and vertical modes run independently
// Each mode is a command source, a hold value or a guidance law
#pragma once

#include <memory>
#include <optional>

#include "acessna/control/setpoints.hpp"
#include "acessna/sim/sim.hpp"

namespace acessna {

enum class LateralModeId {
  kHeadingHold,
};

enum class VerticalModeId {
  kAltitudeHold,
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

}  // namespace acessna
