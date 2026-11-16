// Route geometry, waypoint sequencing, L1 and glideslope guidance, and route modes

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <variant>

#include "acessna/autopilot/mode_manager.hpp"
#include "acessna/guidance/glideslope.hpp"
#include "acessna/guidance/l1.hpp"
#include "acessna/guidance/route.hpp"
#include "acessna/math.hpp"

namespace {

using namespace acessna;

constexpr double kLat = 37.3;
constexpr double kLon = -80.0;

State at(double lat, double lon) {
  State s{};
  s.lat_deg = lat;
  s.lon_deg = lon;
  s.alt_ft = 3000.0;
  s.airspeed_kts = 100.0;
  return s;
}

TEST(Geometry, ProjectAndOffsetRoundTrip) {
  for (double brg : {0.0, 49.5, 90.0, 229.5, 315.0}) {
    const Waypoint p = project("p", kLat, kLon, brg, 10000.0, 0.0);
    const LocalOffset d = local_offset_m(kLat, kLon, p.lat_deg, p.lon_deg);
    EXPECT_NEAR(distance_m(d), 10000.0, 1.0) << brg;
    EXPECT_NEAR(bearing_deg(d), brg, 0.01) << brg;
  }
}

TEST(Geometry, TrackPositionSigns) {
  const Waypoint ahead_right = project("p", kLat, kLon, 45.0, 1000.0, 0.0);
  const TrackPosition tp =
      track_position(kLat, kLon, 0.0, ahead_right.lat_deg, ahead_right.lon_deg);
  EXPECT_NEAR(tp.along_m, 707.1, 1.0);
  EXPECT_NEAR(tp.cross_m, 707.1, 1.0);  // east of a northbound course is right
}

std::vector<Waypoint> square_route() {
  return {project("A", kLat, kLon, 0.0, 5000.0, 3500.0),
          project("B", kLat, kLon, 45.0, 7071.0, 4000.0)};
}

TEST(RouteTracker, SequencesWithinTheLeadThenKeepsTheLastLeg) {
  RouteTracker r(square_route());
  r.start(at(kLat, kLon));
  EXPECT_EQ(r.active().name, "A");
  EXPECT_NEAR(r.leg().course_deg, 0.0, 0.01);
  EXPECT_NEAR(r.turn_deg(), 90.0, 0.5);  // A -> B heads east, right turn

  const Waypoint near_a = project("", kLat, kLon, 0.0, 4600.0, 0.0);
  r.update(at(near_a.lat_deg, near_a.lon_deg), 300.0);  // 400 m to go, outside the lead
  EXPECT_EQ(r.active().name, "A");
  r.update(at(near_a.lat_deg, near_a.lon_deg), 500.0);  // inside the lead
  EXPECT_EQ(r.active().name, "B");
  EXPECT_NEAR(r.leg().course_deg, 90.0, 0.5);
  EXPECT_DOUBLE_EQ(r.turn_deg(), 0.0);  // last waypoint

  const Waypoint b = square_route()[1];
  r.update(at(b.lat_deg, b.lon_deg));
  EXPECT_TRUE(r.complete());
  EXPECT_EQ(r.active().name, "B");
  EXPECT_NEAR(r.leg().course_deg, 90.0, 0.5);  // last leg stays active
}

TEST(RouteTracker, PassingAWaypointSequences) {
  RouteTracker r(square_route());
  r.start(at(kLat, kLon));
  // 2 km east of A and slightly past it: past the line through A perpendicular to the leg
  const Waypoint a = square_route()[0];
  const Waypoint abeam = project("", a.lat_deg, a.lon_deg, 80.0, 2000.0, 0.0);
  r.update(at(abeam.lat_deg, abeam.lon_deg));
  EXPECT_EQ(r.active().name, "B");
}

// Moving at 100 kts along `track_deg` at `cross_m` right of a northbound line through (kLat, kLon)
State moving(double cross_m, double track_deg) {
  const Waypoint p = project("", kLat, kLon, 90.0, cross_m, 0.0);
  State s = at(p.lat_deg, p.lon_deg);
  s.groundspeed_kts = 100.0;
  s.tas_kts = 100.0;
  s.track_deg = track_deg;
  return s;
}

TEST(L1, OnTheLineFliesStraight) {
  const L1Output out = l1_track_line({}, kLat, kLon, 0.0, moving(0.0, 0.0));
  EXPECT_NEAR(out.bank_deg, 0.0, 1e-6);
  EXPECT_NEAR(out.cross_track_m, 0.0, 1e-6);
}

TEST(L1, SteersBackTowardTheLine) {
  EXPECT_LT(l1_track_line({}, kLat, kLon, 0.0, moving(100.0, 0.0)).bank_deg, 0.0);   // right: left
  EXPECT_GT(l1_track_line({}, kLat, kLon, 0.0, moving(-100.0, 0.0)).bank_deg, 0.0);  // left: right
  EXPECT_LT(l1_track_line({}, kLat, kLon, 0.0, moving(0.0, 20.0)).bank_deg, 0.0);  // heading right
  // Converging at the 45 degree intercept from far right: no further turn needed
  EXPECT_NEAR(l1_track_line({}, kLat, kLon, 0.0, moving(5000.0, 315.0)).bank_deg, 0.0, 0.01);
}

GlidePath north_glidepath() { return {kLat, kLon, 1000.0, 0.0, 3.0, 50.0}; }

// 2 km before the threshold, `dh_ft` above the glidepath
State on_approach(double dh_ft) {
  const Waypoint p = project("", kLat, kLon, 180.0, 2000.0, 0.0);
  State s = moving(0.0, 0.0);
  s.lat_deg = p.lat_deg;
  s.lon_deg = p.lon_deg;
  s.alt_ft = glidepath_height_ft(north_glidepath(), s) + dh_ft;
  return s;
}

TEST(Glideslope, GlidepathHeight) {
  // 50 ft threshold crossing + 2 km * tan(3 deg)
  EXPECT_NEAR(glidepath_height_ft(north_glidepath(), on_approach(0.0)) - 1000.0,
              50.0 + 2000.0 * kMToFt * std::tan(3.0 * kDegToRad), 0.5);
}

TEST(Glideslope, CommandsThePathAngleAndCorrectsHeight) {
  EXPECT_NEAR(glideslope_flight_path_deg({}, north_glidepath(), on_approach(0.0)), -3.0, 0.01);
  EXPECT_GT(glideslope_flight_path_deg({}, north_glidepath(), on_approach(-50.0)), -3.0);  // low
  EXPECT_LT(glideslope_flight_path_deg({}, north_glidepath(), on_approach(50.0)), -3.0);   // high
}

TEST(Glideslope, RouteAltitudeCapturesNearThePathOnFinal) {
  auto route = std::make_shared<RouteTracker>(square_route());
  GlideslopeArm arm;
  arm.path = north_glidepath();
  RouteAltitude mode(route, arm);
  const ActiveModes active{LateralModeId::kRoute, VerticalModeId::kRouteAltitude};
  EXPECT_EQ(mode.next(on_approach(-100.0), active), nullptr);  // well below: not yet
  auto next = mode.next(on_approach(-10.0), active);
  ASSERT_NE(next, nullptr);
  EXPECT_EQ(next->id(), VerticalModeId::kGlideslope);
  State off_course = on_approach(0.0);
  off_course.track_deg = 90.0;
  EXPECT_EQ(mode.next(off_course, active), nullptr);
}

TEST(ModeManager, InitialModesAreBuiltFreshEachReset) {
  ModeManager m;
  m.set_initial_modes([] {
    auto route = std::make_shared<RouteTracker>(square_route());
    return ModeManager::InitialModes{std::make_unique<RouteNav>(route),
                                     std::make_unique<RouteAltitude>(route)};
  });
  for (int i = 0; i < 2; ++i) {
    m.reset(at(kLat, kLon));
    EXPECT_EQ(m.lateral_mode(), LateralModeId::kRoute);
    EXPECT_EQ(m.vertical_mode(), VerticalModeId::kRouteAltitude);
    const Target t = m.targets(at(kLat, kLon), 1.0 / 60);
    EXPECT_TRUE(std::holds_alternative<BankCommand>(t.lateral));
    EXPECT_DOUBLE_EQ(std::get<AltitudeCommand>(t.vertical).altitude_ft, 3500.0);
  }
}

}  // namespace
