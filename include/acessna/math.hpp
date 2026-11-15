// Unit conversions and angle helpers
#pragma once

#include <cmath>

namespace acessna {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kKtsToFps = 1.6878098570997825;
constexpr double kFpsToKts = 1.0 / kKtsToFps;
constexpr double kFtToM = 0.3048;
constexpr double kMToFt = 1.0 / kFtToM;
constexpr double kGravityFps2 = 32.174049;

// Wraps an angle to [-180, 180)
inline double wrap_180(double deg) {
  double w = std::fmod(deg + 180.0, 360.0);
  if (w < 0.0) w += 360.0;
  return w - 180.0;
}

// Wraps an angle to [0, 360)
inline double wrap_360(double deg) {
  double w = std::fmod(deg, 360.0);
  if (w < 0.0) w += 360.0;
  return w;
}

}  // namespace acessna
