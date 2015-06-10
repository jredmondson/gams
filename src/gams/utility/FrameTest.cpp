#include <iostream>
#include "Base_Frame.h"

using namespace gams::utility;

int main()
{
  Cartesian_Frame cart_frame;
  Location loc1(cart_frame,0,0,0);
  Cartesian_Frame cart_frame2(Pose(cart_frame, 3, 4));
  Location loc2(cart_frame2,0,0,0);
  std::cout << loc1.distance_to(loc2) << std::endl;
  std::cout << loc2.distance_to(loc1) << std::endl;
  Cartesian_Frame cart_frame3(Location(cart_frame2, 3, 4));
  Location loc3(cart_frame3,0,0,0);
  std::cout << loc1.distance_to(loc3) << std::endl;
  std::cout << loc3.distance_to(loc1) << std::endl;
  Cartesian_Frame cart_frame4(Location(cart_frame, -3, -4));
  Location loc4(cart_frame4,0,0,0);
  std::cout << loc1.distance_to(loc4) << std::endl;
  std::cout << loc4.distance_to(loc1) << std::endl;
  std::cout << loc3.distance_to(loc4) << std::endl;
  std::cout << loc4.distance_to(loc3) << std::endl;

  Rotation rot1(cart_frame,0,0,0);
  Rotation rot2(cart_frame2,0,0,0);
  std::cout << rot1.distance_to(rot2) << std::endl;

  Pose pose1(cart_frame,0,0);
  Pose pose2(cart_frame2,0,0);
  std::cout << pose1.distance_to(pose2) << std::endl;
  return 0;
}
