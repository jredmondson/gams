#include <iostream>
#include <math.h>
#include <gams/pose/CartesianFrame.h>
#include <gams/pose/GPSFrame.h>
#include <madara/knowledge/KnowledgeBase.h>

using namespace gams::pose;

/* multiplicative factor for deciding if a TEST is sufficiently close */
const double TEST_epsilon = 0.01;

double round_nearest(double in)
{
  return floor(in + 0.5);
}

#define LOG(expr) \
  std::cout << #expr << " == " << (expr) << std::endl

#define TEST_EQ(expr, expect) \
  do {\
    auto v = (expr); \
    auto e = (expect); \
    if(v == e) \
    { \
      std::cout << #expr << " ?= " << e << "  SUCCESS! got " << v << std::endl; \
    } \
    else \
    { \
      std::cout << #expr << " ?= " << e << "  FAIL! got " << v << " instead" << std::endl; \
    } \
  } while(0)

#define TEST(expr, expect) \
  do {\
    double bv = (expr); \
    double v = round_nearest((bv) * 1024)/1024; \
    double e = round_nearest((expect) * 1024)/1024; \
    bool ok = \
      e >= 0 ? (v >= e * (1 - TEST_epsilon) && v <= e * (1 + TEST_epsilon)) \
             : (v >= e * (1 + TEST_epsilon) && v <= e * (1 - TEST_epsilon)); \
    if(ok) \
    { \
      std::cout << #expr << " ?= " << e << "  SUCCESS! got " << bv << std::endl; \
    } \
    else \
    { \
      std::cout << #expr << " ?= " << e << "  FAIL! got " << bv << " instead" << std::endl; \
    } \
  } while(0)

int main(int, char *[])
{
  std::cout.precision(4);
  std::cout << std::fixed;
  std::cout << "Testing default frame support:" << std::endl;
  Position dloc0(0,0);
  Position dloc1(3,4);
  LOG(dloc0);
  LOG(dloc1);
  TEST(dloc0.distance_to(dloc1), 5);
  TEST(dloc1.distance_to(dloc0), 5);

  std::cout << std::endl << "Testing GPS frame support:" << std::endl;
  ReferenceFrame gpsframe = gps_frame();
  Position gloc0(gpsframe,0,0);
  Position gloc1(gpsframe,1,1);
  Position gloc2(gpsframe,90,0);
  Position gloc3(gpsframe,90,90);
  LOG(gloc0);
  LOG(gloc1);
  LOG(gloc2);
  LOG(gloc3);
  double gps_one_degree = EARTH_CIRC/360;
  TEST(gloc0.distance_to(gloc1), sqrt(2 * (gps_one_degree * gps_one_degree)));
  TEST(gloc0.distance_to(gloc2), EARTH_CIRC/4);
  TEST(gloc0.distance_to(gloc3), EARTH_CIRC/4);
  TEST(gloc2.distance_to(gloc3), 0);
  Position gloc4(gpsframe,120,0);
  Position gloc5(gpsframe,60,180);
  LOG(gloc4);
  LOG(gloc5);
  TEST(gloc4.distance_to(gloc5), 0);
  Position gloc6(gpsframe,360,180);
  LOG(gloc6);
  TEST(gloc6.distance_to(gloc0), EARTH_CIRC/2);
  gloc6.normalize();
  LOG(gloc6);
  TEST(gloc6.distance_to(gloc0), EARTH_CIRC/2);

  std::cout << std::endl << "Testing CartesianFrame tree:" << std::endl;
  Position gloc(gpsframe,0,90);
  ReferenceFrame cart_frame0(gloc);
  ReferenceFrame cart_frame1(Pose(cart_frame0, 3, 4));
  ReferenceFrame cart_frame2(Pose(cart_frame1, 3, 4));
  ReferenceFrame cart_frame3(Pose(cart_frame2, 3, 4));

  Position cloc0(cart_frame0, 0, 0);
  Position cloc2(cart_frame2, 0, 0);
  Position cloc3(cart_frame3, 0, 0);
  Position cloc3a(cart_frame0, cloc3);

  LOG(cloc0);
  LOG(cloc3);
  TEST(cloc0.frame() == cloc3.frame(), 0);
  TEST(cloc0.distance_to(cloc2), 10);
  TEST(cloc0.distance_to(cloc3), 15);
  TEST(cloc3.distance_to(cloc0), 15);
  TEST(cloc3.distance_to(cloc2), 5);

  LOG(cloc3a);
  TEST(cloc0.frame() == cloc3a.frame(), 1);
  TEST(cloc3a.distance_to(cloc0), 15);
  cloc3.transform_this_to(cart_frame0);

  LOG(cloc3);
  TEST(cloc0.frame() == cloc3.frame(), 1);
  TEST(cloc3.distance_to(cloc0), 15);

  std::cout << std::endl << "Testing Cartesian/GPS conversion:" << std::endl;
  TEST(cloc2.distance_to(gloc), 10);
  TEST(cloc3.distance_to(gloc), 15);
  TEST(cloc0.distance_to(gloc), 0);
  TEST(gloc.distance_to(cloc2), 10);
  TEST(gloc.distance_to(cloc3), 15);
  TEST(gloc.distance_to(cloc0), 0);
  LOG(cloc3.transform_to(cloc0.frame()));
  LOG(cloc3.transform_to(gloc.frame()));
  LOG(gloc.distance_to(cloc3));
  TEST(cloc0.distance_to(gloc0), EARTH_CIRC/4);
  TEST(gloc0.distance_to(cloc0), EARTH_CIRC/4);
  LOG(cloc3.distance_to(gloc0));
  LOG(gloc0.distance_to(cloc3));

  std::cout << std::endl << "Testing orientations between Cartesian frames:" << std::endl;
  ReferenceFrame rot_frame0(gloc0);
  ReferenceFrame rot_frame1(Pose(rot_frame0, Position(50, 100), Orientation(0, 0, 90, degrees)));

  Orientation rot0(rot_frame0, 0, 0, 0);
  Orientation rot1(rot_frame1, 0, 0, 0);
  LOG(Orientation(rot_frame1.origin()));
  TEST(rot1.transform_to(rot_frame0).rz(), M_PI / 2);
  TEST(rot0.transform_to(rot_frame1).rz(), - (M_PI / 2));
  TEST(rot0.distance_to(rot1), M_PI / 2);
  TEST(rot1.distance_to(rot0), M_PI / 2);

  Position rloc0(rot_frame0, 0, 0);
  Position rloc1(rot_frame1, 4, 0);
  LOG(rloc1);
  LOG(rloc1.transform_to(rot_frame0));
  Position rloc2(rot_frame0, 4, 0);
  LOG(rloc2);
  LOG(rloc2.transform_to(rot_frame1));
  Position glocr(gpsframe, 1, 2);
  LOG(glocr.transform_to(rot_frame1));
  Position rloc3(rot_frame1, 2000, 8000);
  LOG(rloc3.transform_to(gpsframe));

  std::cout << std::endl << "Testing orientations between Cartesian/GPS frames:" << std::endl;
  Orientation grot0(gpsframe, 0, 0, 0);
  TEST(rot1.transform_to(gpsframe).rz(), M_PI / 2);
  TEST(grot0.transform_to(rot_frame1).rz(), - (M_PI / 2));
  
  std::cout << std::endl << "Testing Poses, with orientations between Cartesian frames:" << std::endl;
  Pose pose0(rot_frame0, 0, 0);
  Pose pose1(rot_frame1, 0, 0);
  LOG(pose0);
  LOG(pose1);
  TEST(pose1.transform_to(rot_frame0).rz(), M_PI / 2);
  TEST(pose0.transform_to(gpsframe).rz(), 0);
  TEST(pose1.transform_to(gpsframe).rz(), M_PI / 2);

  std::cout << std::endl << "Forming a hexagon with a chain of Cartesian frames:" << std::endl;
  Orientation sixty_degrees(0, 0, 60, degrees);
  ReferenceFrame hex_frame0({gps_frame(), gloc0, sixty_degrees});
  Pose hex0(hex_frame0, 0, 0);
  ReferenceFrame hex_frame1({hex_frame0, {10, 0}, sixty_degrees});
  Pose hex1(hex_frame1, 0, 0);
  ReferenceFrame hex_frame2({hex_frame1, {10, 0}, sixty_degrees});
  Pose hex2(hex_frame2, 0, 0);
  ReferenceFrame hex_frame3({hex_frame2, {10, 0}, sixty_degrees});
  Pose hex3(hex_frame3, 0, 0);
  ReferenceFrame hex_frame4({hex_frame3, {10, 0}, sixty_degrees});
  Pose hex4(hex_frame4, 0, 0);
  ReferenceFrame hex_frame5({hex_frame4, {10, 0}, sixty_degrees});
  Pose hex5(hex_frame5, 0, 0);
  ReferenceFrame hex_frame6({hex_frame5, {10, 0}, sixty_degrees});
  Pose hex6(hex_frame6, 0, 0);
  TEST(hex6.distance_to(hex0), 0);
  TEST(hex0.distance_to(hex6), 0);
  TEST(hex0.distance_to(hex1), 10);
  TEST(hex0.distance_to(hex2), 17.32);
  TEST(hex0.distance_to(hex3), 20);
  TEST(hex6.distance_to(gloc0), 0);
  TEST(gloc0.distance_to(hex6), 0);
  TEST(gloc0.distance_to(hex1), 10);
  TEST(gloc0.distance_to(hex2), 17.32);
  TEST(gloc0.distance_to(hex3), 20);
  TEST(hex0.angle_to(hex1, degrees), 60);
  TEST(hex0.angle_to(hex2, degrees), 120);
  TEST(hex0.angle_to(hex3, degrees), 180);
  TEST(hex0.angle_to(hex4, degrees), 120);
  TEST(hex0.angle_to(hex5, degrees), 60);
  TEST(hex0.angle_to(hex6, degrees), 0);
  LOG(Orientation(hex0));
  LOG(Orientation(hex6));
  LOG(Orientation(hex6.transform_to(hex_frame0)));
  TEST(hex0.angle_to(hex0), 0);

  std::cout << std::endl << "Test saving and loading frame tree (TODO):"
            << std::endl;

  madara::knowledge::KnowledgeBase kb;

  {
    ReferenceFrame building_frame("Building", {gps_frame(), 70, -40}, -1);
    ReferenceFrame room_frame("LivingRoom", {building_frame, 10, 20}, -1);
    ReferenceFrame kitchen_frame("Kitchen", {building_frame, 30, 50}, -1);
    ReferenceFrame drone_frame("Drone", {kitchen_frame, 3, 2, -2}, 1000);
    ReferenceFrame camera_frame("Camera", {drone_frame, 0, 0, 0.5}, 1000);
    ReferenceFrame drone2_frame("Drone2", {room_frame, 3, 2, -2}, 1000);

    gps_frame().save(kb);
    building_frame.save(kb);
    room_frame.save(kb);
    kitchen_frame.save(kb);
    drone_frame.save(kb);
    camera_frame.save(kb);
    drone2_frame.save(kb);

    ReferenceFrame drone_frame1 = drone_frame.move({kitchen_frame, 3, 4, -2}, 2000);
    ReferenceFrame camera_frame1 = camera_frame.orient({drone_frame1, 0, 0, M_PI/4}, 2000);
    ReferenceFrame drone2_frame1 = drone2_frame.move({room_frame, 3, 6, -2}, 2000);

    drone_frame1.save(kb);
    camera_frame1.save(kb);
    drone2_frame1.save(kb);

    drone_frame.move({room_frame, 3, 6, -2}, 2250).save(kb);
    camera_frame1.orient({drone_frame1, 0, 0, M_PI/2}, 2250).save(kb);
    drone2_frame.move({room_frame, 3, 7, -2}, 2500).save(kb);
  }

  ReferenceFrameIdentity::gc();

  std::vector<std::string> ids = {"Drone", "Drone2", "Camera"};

  std::vector<ReferenceFrame> frames = ReferenceFrame::load_tree(kb, ids, 1000);

  TEST_EQ(frames.size(), ids.size());

  if (frames.size() == ids.size()) {
    TEST_EQ(frames[0].id(), "Drone");
    TEST_EQ(frames[1].id(), "Drone2");

    TEST(frames[0].timestamp(), 1000);
    TEST(frames[1].timestamp(), 1000);

    TEST_EQ(frames[0].interpolated(), false);
    TEST_EQ(frames[1].interpolated(), false);

    TEST(frames[0].origin().x(), 3);
    TEST(frames[0].origin().y(), 2);
    TEST(frames[1].origin().rz(), 0);

    Position d2pos(frames[1], 1, 1);
    Position d1pos = d2pos.transform_to(frames[0]);
    LOG(d2pos);
    LOG(d1pos);
  }

  frames = ReferenceFrame::load_tree(kb, ids, 1500);

  TEST_EQ(frames.size(), ids.size());

  if (frames.size() == ids.size()) {
    TEST_EQ(frames[0].id(), "Drone");
    TEST_EQ(frames[1].id(), "Drone2");

    TEST(frames[0].timestamp(), 1500);
    TEST(frames[1].timestamp(), 1500);

    TEST_EQ(frames[0].interpolated(), true);
    TEST_EQ(frames[1].interpolated(), false);

    TEST(frames[0].origin().x(), 3);
    TEST(frames[0].origin().y(), 3);
    TEST(frames[2].origin().rz(), M_PI/8);
  }

  frames = ReferenceFrame::load_tree(kb, ids);

  TEST_EQ(frames.size(), ids.size());

  if (frames.size() == ids.size()) {
    TEST_EQ(frames[0].id(), "Drone");
    TEST_EQ(frames[1].id(), "Drone2");

    TEST(frames[0].timestamp(), 2250);
    TEST(frames[1].timestamp(), 2250);

    TEST_EQ(frames[0].interpolated(), false);
    TEST_EQ(frames[1].interpolated(), true);

    TEST(frames[0].origin().x(), 3);
    TEST(frames[0].origin().y(), 6);
    TEST(frames[2].origin().rz(), M_PI/2);

    Linear<Position> cpose = frames[0].origin();
  }

  frames = ReferenceFrame::load_tree(kb, ids, 2500);

  TEST_EQ(frames.size(), 0UL);

  ReferenceFrameIdentity::default_expiry(4000);
  for (int x = 0; x <= 10000; x += 500) {
    ReferenceFrame exp("expiring", Pose(), x);
    exp.save(kb);
  }

  std::string dump;
  kb.to_string(dump);
  LOG(dump);

  ReferenceFrame exp("expiring", Pose(), -1);
  exp.save(kb, -1);

  kb.to_string(dump);
  LOG(dump);

  exp.save(kb);

  kb.to_string(dump);
  LOG(dump);

  return 0;
}
