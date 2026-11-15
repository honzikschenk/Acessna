// Data passed between the layers
#pragma once

#include <variant>

namespace acessna {

// Lateral commands
struct HeadingCommand {
  double heading_deg = 0.0;  // true
};
struct CourseCommand {
  double course_deg = 0.0;  // ground track
};
struct BankCommand {
  double bank_deg = 0.0;  // direct bank command
};
using LateralCommand = std::variant<HeadingCommand, CourseCommand, BankCommand>;

// Vertical commands
struct AltitudeCommand {
  double altitude_ft = 0.0;  // MSL
};
struct FlightPathCommand {
  double flight_path_deg = 0.0;  // gamma
};
struct ClimbRateCommand {
  double climb_rate_fpm = 0.0;
};
using VerticalCommand = std::variant<AltitudeCommand, FlightPathCommand, ClimbRateCommand>;

// Flight-path controller targets
struct Target {
  LateralCommand lateral;
  VerticalCommand vertical;
  double airspeed_kts = 0.0;  // TECS tracks
};

// TECS output, longitudinal part of the flight-path controller
struct TecsOutput {
  double pitch_deg = 0.0;
  double throttle = 0.0;
};

// Attitude controller commands, Set by the flight-path controller
struct AttitudeSetpoint {
  double roll_deg = 0.0;
  double pitch_deg = 0.0;
  double sideslip_deg = 0.0;
};

// Flight-path controller output
struct FlightPathOutput {
  AttitudeSetpoint attitude;
  double throttle = 0.0;
};

// Attitude controller output
struct RateSetpoint {
  double roll_rate_dps = 0.0;   // p
  double pitch_rate_dps = 0.0;  // q
  double yaw_rate_dps = 0.0;    // r
};

// Rate controller output
struct SurfaceCommands {
  double elevator = 0.0;
  double aileron = 0.0;
  double rudder = 0.0;
};

}  // namespace acessna
