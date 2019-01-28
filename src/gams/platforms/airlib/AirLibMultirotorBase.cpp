/**
 * @file AirLibMultirotorBase.cpp
 * @author Devon Ash <noobaca2@gmail.com>
 *
 * This file contains the definition of the AirLibMultirotorBase abstract class
 **/

#ifdef _GAMS_AIRLIB_ // only compile this if we are simulating in Unreal's AirSim


#include <iostream>
#include <cmath>
#include <map>
#include <vector>

#include "madara/knowledge/containers/DoubleVector.h"

#include "gams/platforms/airlib/AirLibMultirotorBase.h"

using std::endl;
using std::cout;
using std::cerr;
using std::string;
using std::map;

gams::platforms::AirLibMultirotorBase::AirLibMultirotorBase (madara::knowledge::KnowledgeBase * knowledge,
  variables::Sensors * sensors, variables::Self * self) :
  BasePlatform (knowledge, sensors, self), ready_ (false)
{
  static bool init = false;

  client.confirmConnection();
  client.enableApiControl(true);
  client.armDisarm(true);
  takeoff();

  if (!init)
  {
//    string node_name (knowledge->get (".ros_node").to_string ());
//    map<string, string> remap;
//    ros::init (remap, node_name);
    init = true;
  }
}

int
gams::platforms::AirLibMultirotorBase::sense ()
{
  return 1;
}

int
gams::platforms::AirLibMultirotorBase::analyze ()
{
  return 1;
}

double
gams::platforms::AirLibMultirotorBase::get_accuracy () const
{
  return 1;
}

int
gams::platforms::AirLibMultirotorBase::land ()
{
  client.landAsync()->waitOnLastTask();
  return 1;
}

int
land ()
{
  return 1;
}

int
gams::platforms::AirLibMultirotorBase::move (const pose::Position & position/*position*/,
        const PositionBounds & bounds/*bounds*/)
{
  client.moveToPositionAsync(position.x(), position.y(), position.z(), 1);
  return 1;
}

void
gams::platforms::AirLibMultirotorBase::set_move_speed (const double & /*speed*/)
{

}

int
gams::platforms::AirLibMultirotorBase::takeoff ()
{
  client.takeoffAsync(5)->waitOnLastTask();
  return 1;
}

void
gams::platforms::AirLibMultirotorBase::wait_for_go () const
{
}

const gams::pose::ReferenceFrame &
gams::platforms::AirLibMultirotorBase::get_frame (void) const
{
  return pose::gps_frame();
}

#endif // _GAMS_AIRLIB_

