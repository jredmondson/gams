/**
 * @file AirLibQuadcopter.cpp
 * @author Devon Ash <noobaca2@gmail.com>
 *
 * This file contains the definition of the AirLibQuadcopter simulator robot class
 */

#ifdef _GAMS_AIRLIB_ // only compile this if we are simulating in airsim

#include "gams/platforms/airlib/AirLibQuadcopter.h"

#include <iostream>
using std::endl;
using std::cout;
using std::string;
#include <cmath>

#include "madara/knowledge/containers/DoubleVector.h"

#include "gams/variables/Sensor.h"


gams::platforms::BasePlatform *
gams::platforms::AirLibQuadcopterFactory::create (
  const madara::knowledge::KnowledgeMap & args,
  madara::knowledge::KnowledgeBase * knowledge,
  variables::Sensors * sensors,
  variables::Platforms * platforms,
  variables::Self * self)
{
  BasePlatform * result (0);
  
  if (knowledge && sensors && platforms && self)
  {
    if (knowledge->get_num_transports () == 0)
    {
      madara::transport::QoSTransportSettings settings;

      settings.type = madara::transport::MULTICAST;
      settings.hosts.push_back ("239.255.0.1:4150");

      knowledge_->attach_transport ("", settings);
      knowledge_->activate_transport ();
    }

    result = new AirLibQuadcopter (knowledge, sensors, 
      platforms, self);
  }

  return result;
}

gams::platforms::AirLibQuadcopter::AirLibQuadcopter (
  madara::knowledge::KnowledgeBase * knowledge,
  variables::Sensors * sensors,
  variables::Platforms * platforms,
  variables::Self * self)
  : AirLibQuadcopter (knowledge, sensors, self)
{
  if (platforms && knowledge)
  {
    (*platforms)[get_id ()].init_vars (*knowledge, get_id ());
    status_ = (*platforms)[get_id ()];
  }

  self_->agent.desired_altitude = 0.05;
}

gams::platforms::AirLibQuadcopter::~AirLibQuadcopter()
{

}

std::string
gams::platforms::AirLibQuadcopter::get_id () const
{
  return "airlib_quad";
}

std::string
gams::platforms::AirLibQuadcopter::get_name () const
{
  return "airlib quad";
}

#endif // _GAMS_AIRLIB_
