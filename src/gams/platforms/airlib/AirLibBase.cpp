/**
 * @file AirLibBase.cpp
 * @author Devon Ash <noobaca2@gmail.com>
 *
 * This file contains the definition of the AirLibBase abstract class
 **/

#ifdef _GAMS_AIRLIB_ // only compile this if we are simulating in Unreal's AirSim


#include <iostream>
#include <cmath>
#include <map>
#include <vector>

#include "madara/knowledge/containers/DoubleVector.h"

#include "gams/platforms/airlib/AirLibBase.h"

using std::endl;
using std::cout;
using std::cerr;
using std::string;
using std::map;

gams::platforms::AirLibBase::AirLibBase(madara::knowledge::KnowledgeBase * knowledge,
  variables::Sensors * sensors, variables::Self * self) :
  BasePlatform(knowledge, sensors, self), ready_(false)
{
  static bool init = false;

  if(!init)
  {
//    string node_name(knowledge->get(".ros_node").to_string());
//    map<string, string> remap;
//    ros::init(remap, node_name);
    init = true;
  }
}

int
gams::platforms::AirLibBase::sense()
{
  return 1;
}

int
gams::platforms::AirLibBase::analyze()
{
  return 1;
}

int
gams::platforms::AirLibBase::land()
{
  return 1;
}

int
gams::platforms::AirLibBase::move(const pose::Position & position/*position*/,
        const pose::PositionBounds & bounds/*bounds*/)
{
  return 1;
}

void
gams::platforms::AirLibBase::set_move_speed(const double & /*speed*/)
{

}

int
gams::platforms::AirLibBase::takeoff()
{
  return 1;
}

void
gams::platforms::AirLibBase::wait_for_go() const
{
}

const gams::pose::ReferenceFrame &
gams::platforms::AirLibBase::get_frame(void) const
{
  return pose::gps_frame();
}

#endif // _GAMS_AIRLIB_

