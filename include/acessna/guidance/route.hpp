// Waypoint route
#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "acessna/sim/sim.hpp"

namespace acessna {

struct Waypoint {
  std::string name;
  double lat_deg = 0.0;
  double lon_deg = 0.0;
  double alt_ft = 0.0;  // MSL, commanded while this waypoint is active
};

struct LocalOffset {
  double north_m = 0.0;
  double east_m = 0.0;
};

LocalOffset local_offset_m(double lat0_deg, double lon0_deg, double lat_deg, double lon_deg);
double distance_m(const LocalOffset& d);
double bearing_deg(const LocalOffset& d);

// The point `distance_m` from (lat0, lon0) along true bearing `bearing_deg`
Waypoint project(const std::string& name, double lat0_deg, double lon0_deg, double bearing_deg,
                 double distance_m, double alt_ft);

// Position of a point relative to a line through (lat0, lon0) along true course `course_deg`
struct TrackPosition {
  double along_m = 0.0;  // + = ahead of the reference point
  double cross_m = 0.0;  // + = right of the course
};

TrackPosition track_position(double lat0_deg, double lon0_deg, double course_deg, double lat_deg,
                             double lon_deg);

// A straight leg between two points
struct Leg {
  double from_lat_deg = 0.0;
  double from_lon_deg = 0.0;
  double to_lat_deg = 0.0;
  double to_lon_deg = 0.0;
  double course_deg = 0.0;  // from -> to
};

// Sequences a route as straight legs
class RouteTracker {
 public:
  explicit RouteTracker(std::vector<Waypoint> waypoints);

  void start(const State& s);

  void update(const State& s, double lead_m = 0.0);

  bool complete() const { return index_ >= waypoints_.size(); }
  std::size_t active_index() const { return index_; }

  // The active waypoint, or the last one once complete
  const Waypoint& active() const;

  // The leg being flown
  const Leg& leg() const { return leg_; }

  // Course change at the active waypoint onto the following leg (+ = right), 0 at the last
  double turn_deg() const;

  const std::vector<Waypoint>& waypoints() const { return waypoints_; }

 private:
  void set_leg(double from_lat_deg, double from_lon_deg, const Waypoint& to);

  std::vector<Waypoint> waypoints_;
  std::size_t index_ = 0;
  Leg leg_;
};

}  // namespace acessna
