#include <iostream>
#include <gams/utility/Base_Frame.h>
#include <gams/utility/Cartesian_Frame.h>
#include <gams/utility/GPS_Frame.h>

using namespace gams::utility;

/* multiplicative factor for deciding if a TEST is sufficiently close */
const double TEST_epsilon = 0.0001;

#define LOG(expr) \
  std::cout << #expr << " == " << (expr) << std::endl

#define TEST(expr, expect) \
  do {\
    double bv = (expr); \
    double v = round((bv) * 1024)/1024; \
    double e = round((expect) * 1024)/1024; \
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

int main()
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
  GPS_Frame gps_frame;
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

  std::cout << std::endl << "Testing Cartesian_Frame tree:" << std::endl;
  Location gloc(gps_frame,-90,0);
  Cartesian_Frame cart_frame0(gloc);
  Cartesian_Frame cart_frame1(Pose(cart_frame0, 3, 4));
  Cartesian_Frame cart_frame2(Pose(cart_frame1, 3, 4));
  Cartesian_Frame cart_frame3(Pose(cart_frame2, 3, 4));

  Location cloc0(cart_frame0, 0, 0);
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
  TEST(cloc3.distance_to(gloc), 15);
  LOG(cloc3.transform_to(cloc0.frame()));
  LOG(cloc3.transform_to(gloc.frame()));
  LOG(gloc.distance_to(cloc3));
  TEST(cloc0.distance_to(gloc0), gps_frame.circ()/4);
  TEST(gloc0.distance_to(cloc0), gps_frame.circ()/4);
  LOG(cloc3.distance_to(gloc0));
  LOG(gloc0.distance_to(cloc3));

  std::cout << std::endl << "Testing rotations between Cartesian frames:" << std::endl;
  Cartesian_Frame rot_frame0(gloc0);
  Cartesian_Frame rot_frame1(Rotation(rot_frame0, Rotation::Z_axis, 90));

  Rotation rot0(rot_frame0, 0, 0, 0);
  Rotation rot1(rot_frame1, 0, 0, 0);
  LOG(Rotation(rot_frame1.origin()));
  TEST(rot1.transform_to(rot_frame0).rz(), M_PI / 2);
  TEST(rot0.transform_to(rot_frame1).rz(), - (M_PI / 2));
  TEST(rot0.distance_to(rot1), 90);
  TEST(rot1.distance_to(rot0), 90);

  std::cout << std::endl << "Testing rotations between Cartesian/GPS frames:" << std::endl;
  Rotation grot0(gps_frame, 0, 0, 0);
  TEST(rot1.transform_to(gps_frame).rz(), M_PI / 2);
  TEST(grot0.transform_to(rot_frame1).rz(), - (M_PI / 2));

  std::cout << std::endl << "Testing string conversions:" << std::endl;
  Location sloc0(gps_frame, 4, 7);
  LOG(sloc0.to_string());
  sloc0.from_string("1, 2, 3");
  LOG(sloc0);

  std::cout << std::endl << "Testing Poses, with rotations between Cartesian frames:" << std::endl;
  Pose pose0(rot_frame0, 0, 0);
  Pose pose1(rot_frame1, 0, 0);
  LOG(pose0);
  LOG(pose1);
  TEST(pose1.transform_to(rot_frame0).rz(), M_PI / 2);
  TEST(pose0.transform_to(gps_frame).rz(), 0);
  TEST(pose1.transform_to(gps_frame).rz(), M_PI / 2);
}
