#include <iostream>
#include <math.h>
#include <gams/utility/CartesianFrame.h>
#include <gams/utility/GPSFrame.h>

using namespace gams::utility;

/* multiplicative factor for deciding if a TEST is sufficiently close */
const double TEST_epsilon = 0.0001;

double round_nearest(double in)
{
  return floor(in + 0.5);
}

#define LOG(expr) \
  std::cout << #expr << " == " << (expr) << std::endl

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

int main(int argc, char *argv[])
{
  std::cout.precision(4);
  std::cout << std::fixed;
  std::cout << "Testing default frame support:" << std::endl;
  Location dloc0(0,0);
  Location dloc1(3,4);
  LOG(dloc0);
  LOG(dloc1);
  TEST(dloc0.distance_to(dloc1), 5);
  TEST(dloc1.distance_to(dloc0), 5);

  std::cout << std::endl << "Testing GPS frame support:" << std::endl;
  GPSFrame gps_frame;
  Location gloc0(gps_frame,0,0);
  Location gloc1(gps_frame,1,1);
  Location gloc2(gps_frame,0,90);
  Location gloc3(gps_frame,90,90);
  LOG(gps_frame.radius());
  LOG(gps_frame.circ());
  LOG(gloc0);
  LOG(gloc1);
  LOG(gloc2);
  LOG(gloc3);
  double gps_one_degree = gps_frame.circ()/360;
  TEST(gloc0.distance_to(gloc1), sqrt(2 * (gps_one_degree * gps_one_degree)));
  TEST(gloc0.distance_to(gloc2), gps_frame.circ()/4);
  TEST(gloc0.distance_to(gloc3), gps_frame.circ()/4);
  TEST(gloc2.distance_to(gloc3), 0);
  Location gloc4(gps_frame,0,120);
  Location gloc5(gps_frame,180,60);
  LOG(gloc4);
  LOG(gloc5);
  TEST(gloc4.distance_to(gloc5), 0);
  Location gloc6(gps_frame,180,360);
  LOG(gloc6);
  TEST(gloc6.distance_to(gloc0), gps_frame.circ()/2);
  gloc6.normalize();
  LOG(gloc6);
  TEST(gloc6.distance_to(gloc0), gps_frame.circ()/2);

  std::cout << std::endl << "Testing CartesianFrame tree:" << std::endl;
  Location gloc(gps_frame,-90,0);
  CartesianFrame cart_frame0(gloc);
  CartesianFrame cart_frame1(Pose(cart_frame0, 3, 4));
  CartesianFrame cart_frame2(Pose(cart_frame1, 3, 4));
  CartesianFrame cart_frame3(Pose(cart_frame2, 3, 4));

  Location cloc0(cart_frame0, 0, 0);
  Location cloc2(cart_frame2, 0, 0);
  Location cloc3(cart_frame3, 0, 0);
  Location cloc3a(cart_frame0, cloc3);

  LOG(cloc0);
  LOG(cloc3);
  TEST(cloc0.frame() == cloc3.frame(), 0);
  TEST(cloc0.distance_to(cloc3), 15);
  TEST(cloc3.distance_to(cloc0), 15);

  LOG(cloc3a);
  TEST(cloc0.frame() == cloc3a.frame(), 1);
  TEST(cloc3a.distance_to(cloc0), 15);
  cloc3.transform_this_to(cart_frame0);

  LOG(cloc3);
  TEST(cloc0.frame() == cloc3.frame(), 1);
  TEST(cloc3.distance_to(cloc0), 15);

  std::cout << std::endl << "Testing Cartesian/GPS conversion:" << std::endl;
  TEST(cloc2.distance_to(gloc), 15);
  TEST(cloc0.distance_to(gloc), 15);
  LOG(cloc3.transform_to(cloc0.frame()));
  LOG(cloc3.transform_to(gloc.frame()));
  LOG(gloc.distance_to(cloc3));
  TEST(cloc0.distance_to(gloc0), gps_frame.circ()/4);
  TEST(gloc0.distance_to(cloc0), gps_frame.circ()/4);
  LOG(cloc3.distance_to(gloc0));
  LOG(gloc0.distance_to(cloc3));

  std::cout << std::endl << "Testing rotations between Cartesian frames:" << std::endl;
  CartesianFrame rot_frame0(gloc0);
  CartesianFrame rot_frame1(Pose(rot_frame0, Location(50, 100), Orientation(0, 0, 90, degrees)));

  Orientation rot0(rot_frame0, 0, 0, 0);
  Orientation rot1(rot_frame1, 0, 0, 0);
  LOG(Orientation(rot_frame1.origin()));
  TEST(rot1.transform_to(rot_frame0).rz(), M_PI / 2);
  TEST(rot0.transform_to(rot_frame1).rz(), - (M_PI / 2));
  TEST(rot0.distance_to(rot1), 90);
  TEST(rot1.distance_to(rot0), 90);

  Location rloc0(rot_frame0, 0, 0);
  Location rloc1(rot_frame1, 4, 0);
  LOG(rloc1);
  LOG(rloc1.transform_to(rot_frame0));
  Location rloc2(rot_frame0, 4, 0);
  LOG(rloc2);
  LOG(rloc2.transform_to(rot_frame1));
  Location glocr(gps_frame, 1, 2);
  LOG(glocr.transform_to(rot_frame1));
  Location rloc3(rot_frame1, 2000, 8000);
  LOG(rloc3.transform_to(gps_frame));

  std::cout << std::endl << "Testing rotations between Cartesian/GPS frames:" << std::endl;
  Orientation grot0(gps_frame, 0, 0, 0);
  TEST(rot1.transform_to(gps_frame).rz(), M_PI / 2);
  TEST(grot0.transform_to(rot_frame1).rz(), - (M_PI / 2));
  
  std::cout << std::endl << "Testing Poses, with rotations between Cartesian frames:" << std::endl;
  Pose pose0(rot_frame0, 0, 0);
  Pose pose1(rot_frame1, 0, 0);
  LOG(pose0);
  LOG(pose1);
  TEST(pose1.transform_to(rot_frame0).rz(), M_PI / 2);
  TEST(pose0.transform_to(gps_frame).rz(), 0);
  TEST(pose1.transform_to(gps_frame).rz(), M_PI / 2);

  std::cout << std::endl << "Forming a hexagon with a chain of Cartesian frames:" << std::endl;
  CartesianFrame hex_frame0(gloc0);
  Pose hex0(hex_frame0, 0, 0);
  CartesianFrame hex_frame1(Pose(hex_frame0, Location(10, 0), Orientation(0, 0, 60, degrees)));
  Pose hex1(hex_frame1, 0, 0);
  CartesianFrame hex_frame2(Pose(hex_frame1, Location(10, 0), Orientation(0, 0, 60, degrees)));
  Pose hex2(hex_frame2, 0, 0);
  CartesianFrame hex_frame3(Pose(hex_frame2, Location(10, 0), Orientation(0, 0, 60, degrees)));
  Pose hex3(hex_frame3, 0, 0);
  CartesianFrame hex_frame4(Pose(hex_frame3, Location(10, 0), Orientation(0, 0, 60, degrees)));
  Pose hex4(hex_frame4, 0, 0);
  CartesianFrame hex_frame5(Pose(hex_frame4, Location(10, 0), Orientation(0, 0, 60, degrees)));
  Pose hex5(hex_frame5, 0, 0);
  CartesianFrame hex_frame6(Pose(hex_frame5, Location(10, 0), Orientation(0, 0, 60, degrees)));
  Pose hex6(hex_frame6, 0, 0);
  TEST(hex6.distance_to(hex0), 0);
  TEST(hex0.distance_to(hex6), 0);
  TEST(hex0.distance_to(hex1), 10);
  TEST(hex0.distance_to(hex2), 17.32);
  TEST(hex0.distance_to(hex3), 20);
  TEST(hex0.angle_to(hex1), 60);
  TEST(hex0.angle_to(hex2), 120);
  TEST(hex0.angle_to(hex3), 180);
  TEST(hex0.angle_to(hex4), 120);
  TEST(hex0.angle_to(hex5), 60);
  TEST(hex0.angle_to(hex6), 0);
  LOG(Orientation(hex0));
  LOG(Orientation(hex6));
  LOG(Orientation(hex6.transform_to(hex_frame0)));
  TEST(hex0.angle_to(hex0), 0);
  return 0;
}
