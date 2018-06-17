#include <iostream>
#include <fstream>
#include <streambuf>
#include <math.h>
#include <gams/pose/Position.h>
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
  std::cout << __LINE__ << ": " << #expr << " == " << (expr) << std::endl

#define TEST_EQ(expr, expect) \
  do {\
    auto v = (expr); \
    auto e = (expect); \
    if(v == e) \
    { \
      std::cout << __LINE__ << ": " << #expr << " ?= " << e << "  SUCCESS! got " << v << std::endl; \
    } \
    else \
    { \
      std::cout << __LINE__ << ": " << #expr << " ?= " << e << "  FAIL! got " << v << " instead" << std::endl; \
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
      std::cout << __LINE__ << ": " << #expr << " ?= " << e << "  SUCCESS! got " << bv << std::endl; \
    } \
    else \
    { \
      std::cout << __LINE__ << ": " << #expr << " ?= " << e << "  FAIL! got " << bv << " instead" << std::endl; \
    } \
  } while(0)

int main(int, char *[])
{
  static_assert(!supports_transform_to<PositionVector>::value, "");
  static_assert(supports_transform_to<Position>::value, "");
  static_assert(supports_transform_to<StampedPosition>::value, "");

  static_assert(!supports_timestamp<PositionVector>::value, "");
  static_assert(!supports_timestamp<Position>::value, "");
  static_assert(supports_timestamp<StampedPosition>::value, "");

  static_assert(!supports_timestamp<OrientationVector>::value, "");
  static_assert(!supports_timestamp<Orientation>::value, "");
  static_assert(supports_timestamp<StampedOrientation>::value, "");

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
  ReferenceFrame rot_frame1(Pose(rot_frame0,
        PositionVector(50, 100),
        OrientationVector(0, 0, 90, degrees)));

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
  OrientationVector sixty_degrees(0, 0, 60, degrees);
  ReferenceFrame hex_frame0(Pose{gps_frame(), gloc0, sixty_degrees});
  Pose hex0(hex_frame0, 0, 0);
  ReferenceFrame hex_frame1(Pose{hex_frame0, PositionVector{10, 0}, sixty_degrees});
  Pose hex1(hex_frame1, 0, 0);
  ReferenceFrame hex_frame2(Pose{hex_frame1, PositionVector{10, 0}, sixty_degrees});
  Pose hex2(hex_frame2, 0, 0);
  ReferenceFrame hex_frame3(Pose{hex_frame2, PositionVector{10, 0}, sixty_degrees});
  Pose hex3(hex_frame3, 0, 0);
  ReferenceFrame hex_frame4(Pose{hex_frame3, PositionVector{10, 0}, sixty_degrees});
  Pose hex4(hex_frame4, 0, 0);
  ReferenceFrame hex_frame5(Pose{hex_frame4, PositionVector{10, 0}, sixty_degrees});
  Pose hex5(hex_frame5, 0, 0);
  ReferenceFrame hex_frame6(Pose{hex_frame5, PositionVector{10, 0}, sixty_degrees});
  Pose hex6(hex_frame6, 0, 0);
  TEST(hex6.distance_to(hex0), 0);
  TEST(hex0.distance_to(hex6), 0);
  TEST(hex0.distance_to(hex1), 10);
  TEST(hex0.distance_to(hex2), 17.32);
  TEST(hex0.distance_to(hex3), 20);
  TEST(hex6.distance_to(gloc0), 0);
  LOG(hex1);
  LOG(hex1.transform_to(gps_frame()));
  LOG(gloc0.transform_to(hex_frame0));
  LOG(gloc0.transform_to(hex_frame1));
  LOG(gloc0.transform_to(hex_frame2));
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
    ReferenceFrame building_frame("Building", Pose{gps_frame(), 70, -40}, -1);
    ReferenceFrame room_frame("LivingRoom", Pose{building_frame, 10, 20}, -1);
    ReferenceFrame kitchen_frame("Kitchen", Pose{building_frame, 30, 50}, -1);

    TEST_EQ(building_frame.timestamp(), -1UL);

    ReferenceFrame drone_frame("Drone", Pose{kitchen_frame, 3, 2, -2}, 1000);
    ReferenceFrame camera_frame("Camera", Pose{drone_frame, 0, 0, 0.5}, 1000);
    ReferenceFrame drone2_frame("Drone2", Pose{room_frame, 3, 2, -2}, 1000);

    TEST_EQ(drone_frame.timestamp(), 1000UL);
    TEST_EQ(camera_frame.timestamp(), 1000UL);

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

    TEST_EQ(drone_frame1.timestamp(), 2000UL);
    TEST_EQ(camera_frame1.timestamp(), 2000UL);
    TEST_EQ(camera_frame1.origin().rz(), M_PI/4);

    drone_frame1.save(kb);
    camera_frame1.save(kb);
    drone2_frame1.save(kb);

    gps_frame().save(kb, "public_frames");
    building_frame.save(kb, "public_frames");
    room_frame.save(kb, "public_frames");
    kitchen_frame.save(kb, "public_frames");
    drone_frame.save(kb, "public_frames");
    camera_frame.save(kb, "public_frames");
    drone2_frame.save(kb, "public_frames");
    drone_frame1.save(kb, "public_frames");
    camera_frame1.save(kb, "public_frames");
    drone2_frame1.save(kb, "public_frames");

    drone_frame.move({room_frame, 3, 6, -2}, 2250).save(kb);
    camera_frame1.orient({drone_frame1, 0, 0, M_PI/2}, 2250).save(kb);
    drone2_frame.move({room_frame, 3, 7, -2}, 2500).save(kb);
  }

  ReferenceFrameIdentity::gc();

  std::string dump;
  kb.to_string(dump);
  LOG(dump);

  std::vector<std::string> ids = {"Drone", "Drone2", "Camera"};

  std::vector<ReferenceFrame> frames = ReferenceFrame::load_tree(kb, ids, 1000);

  TEST_EQ(frames.size(), ids.size());

  if (frames.size() == ids.size()) {
    TEST_EQ(frames[0].id(), "Drone");
    TEST_EQ(frames[1].id(), "Drone2");

    TEST((double)frames[0].timestamp(), 1000);
    TEST((double)frames[1].timestamp(), 1000);

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

  auto pframes = ReferenceFrame::load_tree(kb, ids, 1500, "public_frames");

  TEST_EQ(pframes.size(), ids.size());

  if (pframes.size() == ids.size()) {
    TEST_EQ(pframes[0].id(), "Drone");
    TEST_EQ(pframes[1].id(), "Drone2");

    TEST((double)pframes[0].timestamp(), 1500);
    TEST((double)pframes[1].timestamp(), 1500);

    TEST_EQ(pframes[0].interpolated(), true);
    TEST_EQ(pframes[1].interpolated(), true);

    TEST(pframes[0].origin().x(), 3);
    TEST(pframes[0].origin().y(), 3);
    TEST(pframes[1].origin().rz(), 0);

    Position d2pos(pframes[1], 1, 1);
    Position d1pos = d2pos.transform_to(pframes[0]);
    LOG(d2pos);
    LOG(d1pos);
  }

  frames = ReferenceFrame::load_tree(kb, ids, 1500);

  TEST_EQ(frames.size(), ids.size());

  if (frames.size() == ids.size()) {
    TEST_EQ(frames[0].id(), "Drone");
    TEST_EQ(frames[1].id(), "Drone2");

    TEST((double)frames[0].timestamp(), 1500);
    TEST((double)frames[1].timestamp(), 1500);

    TEST_EQ(frames[0].interpolated(), true);
    TEST_EQ(frames[1].interpolated(), true);

    TEST(frames[0].origin().x(), 3);
    TEST(frames[0].origin().y(), 3);
    TEST(frames[2].origin().rz(), M_PI/8);

    Position d2pos(pframes[1], 1, 1);
    Position d1pos = d2pos.transform_to(pframes[0]);
    LOG(d2pos);
    LOG(d1pos);
  }

  frames = ReferenceFrame::load_tree(kb, ids);

  TEST_EQ(frames.size(), ids.size());

  if (frames.size() == ids.size()) {
    TEST_EQ(frames[0].id(), "Drone");
    TEST_EQ(frames[1].id(), "Drone2");

    TEST((double)frames[0].timestamp(), 2250);
    TEST((double)frames[1].timestamp(), 2250);

    TEST_EQ(frames[0].interpolated(), false);
    TEST_EQ(frames[1].interpolated(), true);

    TEST(frames[0].origin().x(), 3);
    TEST(frames[0].origin().y(), 6);
    TEST(frames[2].origin().rz(), M_PI/2);

    Position cpos = frames[0].origin();
  }

  FrameStore frame_store(kb, 4000);
  frames = frame_store.load_tree(ids, 2500);

  TEST_EQ(frames.size(), 0UL);

  for (int x = 0; x <= 10000; x += 500) {
    ReferenceFrame exp("expiring", Pose(), x);
    frame_store.save(std::move(exp));
  }

  kb.to_string(dump);
  LOG(dump);

  ReferenceFrame exp("expiring", Pose(), -1);
  exp.save(kb, -1);

  kb.to_string(dump);
  LOG(dump);

  exp.save(kb);
  frame_store.save(std::move(exp));

  kb.to_string(dump);
  LOG(dump);

  madara::knowledge::KnowledgeBase tkb;
  std::ifstream t("frames.kb");
  std::string karl((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
  tkb.evaluate (karl);

  tkb.to_string(dump);
  LOG(dump);

  auto f = ReferenceFrame::load(tkb, "p1base_link");
  LOG (f.valid());

  Pose pf(f, 1, 2, 3);
  Position loc(pf);
  madara::knowledge::containers::NativeDoubleArray nda(".foo", tkb);
  loc.to_container(nda);

  tkb.to_string(dump);
  LOG(dump);

  {
    madara::knowledge::KnowledgeBase data_;

    gams::pose::default_frame().save(data_);
    auto map = gams::pose::ReferenceFrame("map", gams::pose::Pose());
    map.save(data_);

    auto odom = gams::pose::ReferenceFrame("odom", gams::pose::Pose(map, 0, 0, 1.0));
    odom.save(data_);

    auto base =
        gams::pose::ReferenceFrame("base", gams::pose::Pose(odom, 0.0, 0.0, 0.0), 1000);
    base.save(data_);

    auto laser = gams::pose::ReferenceFrame("laser", gams::pose::Pose(base, 0.0, 0.5, 0.75));
    laser.save(data_);

    gams::pose::ReferenceFrame frame =
      base.pose(gams::pose::Pose(odom, 4, 4, 0.0), 5000);
    frame.save(data_);


    data_.to_string(dump);
    LOG(dump);

    std::vector<std::string> frame_ids = {"laser", "map", "base", "odom"};

    std::vector<gams::pose::ReferenceFrame> frames =
      gams::pose::ReferenceFrame::load_tree(data_, frame_ids, 3000);

    TEST ((double)frames.size(), 4);
    if (frames.size() == 4)
    {
      gams::pose::ReferenceFrame laser_frame = frames[0];
      auto origin = laser_frame.origin();
      gams::pose::ReferenceFrame map = frames[1];
      LOG (laser_frame.timestamp());
      LOG (map.timestamp());
      gams::pose::ReferenceFrame base = frames[2];
      gams::pose::ReferenceFrame odom = frames[3];
      LOG (base.timestamp());
      LOG (odom.timestamp());
      if (map.valid())
      {
        gams::pose::Pose transformed = origin.transform_to(map);
        TEST (transformed.x(), 2);
        TEST (transformed.y(), 2.5);
        TEST (transformed.z(), 1.75);
      }
    }
  }

  {
    PositionVector pv{1, 2, 3};
    Position p{default_frame(), 4, 5, 6};
    Position p1{4, 5, 6};
    StampedPosition sp{TimeValue{Duration{100}}, default_frame(), 4, 5, 6};
    sp.nanos(1234567);
    TEST_EQ(sp.nanos(), 1234567UL);
    LOG(sp.secs());
    sp.secs(123.456);
    TEST(sp.secs(), 123.456);
    LOG(sp.nanos());
    StampedPosition sp1{default_frame(), 4, 5, 6};
    StampedPosition sp2{TimeValue{Duration{100}}, 4, 5, 6};

    DisplacementVector dvec{1, 2, 3};
    dvec *= 12;
    TEST_EQ(dvec.x(), 12);
    TEST_EQ(dvec.y(), 24);

    auto dvec2 = dvec * 2;
    TEST_EQ(dvec2.x(), 24);
    TEST_EQ(dvec2.y(), 48);

    auto dvec3 = 3 * dvec;
    TEST_EQ(dvec3.x(), 36);
    TEST_EQ(dvec3.y(), 72);

    auto dvec4 = dvec3 / 6;
    TEST_EQ(dvec4.x(), 6);
    TEST_EQ(dvec4.y(), 12);

    Displacement dis{default_frame(), 3, 4, 5};
    dis *= 10;
    TEST_EQ(dis.x(), 30);
    TEST_EQ(dis.y(), 40);
    TEST_EQ(dis.frame().id(), default_frame().id());

    auto dis2 = dis * 2;
    TEST_EQ(dis2.x(), 60);
    TEST_EQ(dis2.y(), 80);
    TEST_EQ(dis2.frame().id(), default_frame().id());

    dvec += dis;
    TEST_EQ(dvec.x(), 42);
    TEST_EQ(dvec.y(), 64);

    pv += dis;
    TEST_EQ(pv.x(), 31);
    TEST_EQ(pv.y(), 42);

    auto pv2 = pv + dis;
    TEST_EQ(pv2.x(), 61);
    TEST_EQ(pv2.y(), 82);

    Rotation rote{default_frame(), M_PI/2, 0, 0};
    rote *= 2;
    TEST_EQ(rote.rx(), M_PI);

    StampedPosition stamped_position(TimeValue{Duration{1234}}, default_frame(), 2.0, 3.0);
    StampedOrientation ori(TimeValue{Duration{4321}}, gps_frame(), 0, 0, 0);
    StampedPose stamped_pose(stamped_position, ori);

    TEST_EQ(stamped_position.time().time_since_epoch().count(), 1234);
    TEST_EQ(stamped_position.frame() == default_frame(), 1);
    TEST_EQ(stamped_pose.time().time_since_epoch().count(), 1234);
    TEST_EQ(stamped_pose.frame() == default_frame(), 1);
    TEST_EQ(stamped_pose.frame() == gps_frame(), 0);
  }

  {
    madara::knowledge::KnowledgeBase kb;

    auto root = ReferenceFrame("root", Pose(0, 0, 0), 10);
    auto child_5 = ReferenceFrame("child", Pose(root, 1, 0, 0), 5);
    // auto child_10 = ReferenceFrame("child", Pose(root, 1, 0, 5), 10);
    auto child_15 = ReferenceFrame("child", Pose(root, 1, 0, 10), 15);
    auto gchild = ReferenceFrame("gchild", Pose(child_5, 0, 1, 0), 10);

    root.save(kb);
    child_5.save(kb);
    // child_10.save(kb);
    child_15.save(kb);
    gchild.save(kb);

    std::vector<std::string> frame_ids = {"root", "child", "gchild"};
    auto frames = ReferenceFrame::load_tree(kb, frame_ids);
    TEST_EQ(frames.empty(), 0);

    frames = ReferenceFrame::load_tree(kb, frame_ids, 15);
    TEST_EQ(frames.empty(), 1);

    frames = ReferenceFrame::load_tree(kb, frame_ids, 10);
    TEST_EQ(frames.empty(), 0);

    TEST_EQ(frames[2].origin().transform_to(frames[0]).z(), 5);

    std::cout << frames[0].origin() << std::endl;
    std::cout << frames[1].origin().transform_to(frames[0]) << std::endl;
    std::cout << frames[2].origin().transform_to(frames[1]) << std::endl;
    std::cout << frames[2].origin().transform_to(frames[0]) << std::endl;

    kb.print();
  }
  {
    Orientation o0(0, 0, 0);
    Orientation o1(0, 0, M_PI/2);

    auto q0 = o0.into_quat();
    auto q1 = o1.into_quat();

    auto q = q0.slerp(0.5, q1);

    Orientation o(q);

    TEST(o.rz(), M_PI/4);

    auto o2 = o.slerp(0.5, o0);

    TEST(o2.rz(), M_PI/8);

    auto o3 = o0.cross(o1);
    double d = o0.dot(o1);
  }
  {
    Position p(3, 4, 0);
    TEST(p.norm(), 5);
    TEST(p.squaredNorm(), 25);

    Position p1(0, 4, 0);
    TEST(p1.normalized().y(), 1);
  }


  return 0;
}
