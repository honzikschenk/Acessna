#include "acessna/guidance/route.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "acessna/math.hpp"

namespace acessna {

namespace {
constexpr double kEarthRadiusM = 6371000.0;
}  // namespace

LocalOffset local_offset_m(double lat0_deg, double lon0_deg, double lat_deg, double lon_deg) {
  const double mean_lat = 0.5 * (lat0_deg + lat_deg) * kDegToRad;
  return {(lat_deg - lat0_deg) * kDegToRad * kEarthRadiusM,
          wrap_180(lon_deg - lon0_deg) * kDegToRad * kEarthRadiusM * std::cos(mean_lat)};
}

double distance_m(const LocalOffset& d) { return std::hypot(d.north_m, d.east_m); }

double bearing_deg(const LocalOffset& d) {
  return wrap_360(std::atan2(d.east_m, d.north_m) * kRadToDeg);
}

Waypoint project(const std::string& name, double lat0_deg, double lon0_deg, double bearing_deg,
                 double distance_m, double alt_ft) {
  const double north = distance_m * std::cos(bearing_deg * kDegToRad);
  const double east = distance_m * std::sin(bearing_deg * kDegToRad);
  const double lat = lat0_deg + north / kEarthRadiusM * kRadToDeg;
  const double mean_lat = 0.5 * (lat0_deg + lat) * kDegToRad;
  const double lon = lon0_deg + east / (kEarthRadiusM * std::cos(mean_lat)) * kRadToDeg;
  return {name, lat, lon, alt_ft};
}

TrackPosition track_position(double lat0_deg, double lon0_deg, double course_deg, double lat_deg,
                             double lon_deg) {
  const LocalOffset d = local_offset_m(lat0_deg, lon0_deg, lat_deg, lon_deg);
  const double c = std::cos(course_deg * kDegToRad);
  const double s = std::sin(course_deg * kDegToRad);
  return {d.north_m * c + d.east_m * s, -d.north_m * s + d.east_m * c};
}

RouteTracker::RouteTracker(std::vector<Waypoint> waypoints) : waypoints_(std::move(waypoints)) {
  if (waypoints_.empty()) throw std::invalid_argument("route has no waypoints");
}

void RouteTracker::set_leg(double from_lat_deg, double from_lon_deg, const Waypoint& to) {
  leg_ = {from_lat_deg, from_lon_deg, to.lat_deg, to.lon_deg,
          bearing_deg(local_offset_m(from_lat_deg, from_lon_deg, to.lat_deg, to.lon_deg))};
}

void RouteTracker::start(const State& s) {
  index_ = 0;
  set_leg(s.lat_deg, s.lon_deg, waypoints_.front());
}

void RouteTracker::update(const State& s, double lead_m) {
  while (!complete()) {
    const double along_m =
        track_position(leg_.to_lat_deg, leg_.to_lon_deg, leg_.course_deg, s.lat_deg, s.lon_deg)
            .along_m;  // < 0 before the waypoint
    if (along_m < -lead_m) break;

    ++index_;
    if (!complete()) set_leg(leg_.to_lat_deg, leg_.to_lon_deg, waypoints_[index_]);
  }
}

const Waypoint& RouteTracker::active() const {
  return complete() ? waypoints_.back() : waypoints_[index_];
}

double RouteTracker::turn_deg() const {
  if (index_ + 1 >= waypoints_.size()) return 0.0;
  const Waypoint& a = waypoints_[index_];
  const Waypoint& b = waypoints_[index_ + 1];
  const double next = bearing_deg(local_offset_m(a.lat_deg, a.lon_deg, b.lat_deg, b.lon_deg));
  return wrap_180(next - leg_.course_deg);
}

}  // namespace acessna
