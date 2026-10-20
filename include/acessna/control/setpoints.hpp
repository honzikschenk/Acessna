// Data passed between the layers
#pragma once

namespace acessna {

// What the flight-path controller tracks. Set by the autopilot
struct Target {
  double altitude_ft = 0.0;  // MSL
  double heading_deg = 0.0;  // true
  double airspeed_kts = 0.0; // calibrated
};

// What the attitude controller tracks. Set by the flight-path controller
struct AttitudeSetpoint {
  double roll_deg = 0.0;
  double pitch_deg = 0.0;
  double sideslip_deg = 0.0;
};

// Flight-path controller output. Attitude for the attitude controller and throttle
struct FlightPathOutput {
  AttitudeSetpoint attitude;
  double throttle = 0.0;
};

// Set by the attitude controller
struct RateSetpoint {
  double roll_rate_dps = 0.0;   // p
  double pitch_rate_dps = 0.0;  // q
  double yaw_rate_dps = 0.0;    // r
};

// Rate controller output, normalized like Controls
struct SurfaceCommands {
  double elevator = 0.0;
  double aileron = 0.0;
  double rudder = 0.0;
};

}  // namespace acessna
