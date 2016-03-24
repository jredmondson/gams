/**
 * Copyright (c) 2014 Carnegie Mellon University. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following acknowledgments and disclaimers.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. The names "Carnegie Mellon University," "SEI" and/or "Software
 *    Engineering Institute" shall not be used to endorse or promote products
 *    derived from this software without prior written permission. For written
 *    permission, please contact permission@sei.cmu.edu.
 * 
 * 4. Products derived from this software may not be called "SEI" nor may "SEI"
 *    appear in their names without prior written permission of
 *    permission@sei.cmu.edu.
 * 
 * 5. Redistributions of any form whatsoever must retain the following
 *    acknowledgment:
 * 
 *      This material is based upon work funded and supported by the Department
 *      of Defense under Contract No. FA8721-05-C-0003 with Carnegie Mellon
 *      University for the operation of the Software Engineering Institute, a
 *      federally funded research and development center. Any opinions,
 *      findings and conclusions or recommendations expressed in this material
 *      are those of the author(s) and do not necessarily reflect the views of
 *      the United States Department of Defense.
 * 
 *      NO WARRANTY. THIS CARNEGIE MELLON UNIVERSITY AND SOFTWARE ENGINEERING
 *      INSTITUTE MATERIAL IS FURNISHED ON AN "AS-IS" BASIS. CARNEGIE MELLON
 *      UNIVERSITY MAKES NO WARRANTIES OF ANY KIND, EITHER EXPRESSED OR
 *      IMPLIED, AS TO ANY MATTER INCLUDING, BUT NOT LIMITED TO, WARRANTY OF
 *      FITNESS FOR PURPOSE OR MERCHANTABILITY, EXCLUSIVITY, OR RESULTS
 *      OBTAINED FROM USE OF THE MATERIAL. CARNEGIE MELLON UNIVERSITY DOES
 *      NOT MAKE ANY WARRANTY OF ANY KIND WITH RESPECT TO FREEDOM FROM PATENT,
 *      TRADEMARK, OR COPYRIGHT INFRINGEMENT.
 * 
 *      This material has been approved for public release and unlimited
 *      distribution.
 **/

#ifdef _GAMS_VREP_ // only compile this if we are simulating in VREP

#include "VREPQuadLaser.h"


#include <iostream>
#include <cmath>

#include "madara/knowledge/containers/DoubleVector.h"

#include "gams/variables/Sensor.h"

using std::endl;
using std::cout;
using std::string;
using madara::knowledge::containers::NativeDoubleVector;
using madara::knowledge::containers::Double;

const string gams::platforms::VREPQuadLaser::DEFAULT_MODEL (
  (getenv ("GAMS_ROOT") == 0) ? 
  "" : // if GAMS_ROOT is not defined, then just leave this as empty string
  (string (getenv ("GAMS_ROOT")) + "/resources/vrep/Quadricopter_Laser.ttm")
  );

std::string
gams::platforms::VREPQuadLaserFactory::get_default_model()
{
  return VREPQuadLaser::DEFAULT_MODEL;
}

gams::platforms::VREPQuadLaser *
gams::platforms::VREPQuadLaserFactory::create_quad (
  std::string model_file, 
  simxUChar is_client_side, 
  madara::knowledge::KnowledgeBase * knowledge,
  variables::Sensors * sensors,
  variables::Platforms * platforms,
  variables::Self * self)
{
  return new VREPQuadLaser (model_file, is_client_side, knowledge, sensors, platforms, self);
}

gams::platforms::VREPQuadLaser::VREPQuadLaser (
  std::string model_file, 
  simxUChar is_client_side, 
  madara::knowledge::KnowledgeBase * knowledge,
  variables::Sensors * sensors,
  variables::Platforms * platforms,
  variables::Self * self) :
  VREPQuad (model_file, is_client_side, knowledge, sensors, platforms, self),
  laser_sensor_(-1), sonar_sensor_(-1)
{
  if (knowledge && sensors && platforms && self)
  {
    get_sensor_handles();
  }
}

std::string gams::platforms::VREPQuadLaser::get_id () const
{
  return "vrep_quad_laser";
}

std::string gams::platforms::VREPQuadLaser::get_name () const
{
  return "VREP Laser Quadcopter";
}

double
gams::platforms::VREPQuadLaser::read_sensor(simxInt handle, double range) const
{
  simxUChar detectionState;
  simxFloat detectedPoint[3];
  simxInt ret = simxReadProximitySensor(client_id_, handle,
    &detectionState, detectedPoint, NULL, NULL, simx_opmode_oneshot_wait);
  if(detectionState == 0)
    return -range;
  else
    return sqrt(detectedPoint[0] * detectedPoint[0] +
                detectedPoint[1] * detectedPoint[1] +
                detectedPoint[2] * detectedPoint[2]);
}

double gams::platforms::VREPQuadLaser::get_range() const
{
  return read_sensor(laser_sensor_, 2);
}

uint32_t gams::platforms::VREPQuadLaser::get_color() const
{
  std::ostringstream ss;
  ss << "sig_laser_" << node_id_;
  simxUChar *msg;
  simxInt len;
  simxInt result = simxGetStringSignal(client_id_, ss.str().c_str(), &msg, &len,
                                       simx_opmode_oneshot_wait);
  if(result == simx_return_ok)
  {
    uint32_t ret = 0;
    bool started = false;
    for(int i = 0; i < len; ++i)
    {
      if(started)
      {
        char c = msg[i];
        if(c == ':' || c == '\n')
          break;
        ret <<= 4;
        if(c >= '0' && c <= '9')
          c |= c - '0';
        else if(c >= 'A' && c <= 'F')
          c |= c - 'A' + 10;
        else if(c >= 'a' && c <= 'f')
          c |= c - 'a' + 10;
      }
      else if(msg[i] = '#')
        started = true;
    }
    return ret;
  }
  return 0x000000;
}

void gams::platforms::VREPQuadLaser::set_color(uint32_t color) const
{
  std::ostringstream ss_val;
  ss_val << "#" << std::hex << std::setfill('0') << std::setw(6) << color;

  std::ostringstream ss_name;
  ss_name << "sig_bodyColor_" << node_id_;

  std::string name = ss_name.str();
  std::string val = ss_val.str();
  simxInt result = (simxInt) simxSetStringSignal(
    client_id_, (const char *) name.c_str(),
    (const unsigned char *) val.c_str(), (simxInt)val.size(),
    simx_opmode_oneshot_wait);
}

double gams::platforms::VREPQuadLaser::get_altitude() const
{
  return read_sensor(sonar_sensor_, 1);
}

void
gams::platforms::VREPQuadLaser::get_sensor_handles ()
{
  //find the dummy base sub-object
  simxInt handlesCount = 0,*handles = NULL;
  simxInt parentsCount = 0,*parents = NULL;
  simxGetObjectGroupData (client_id_, sim_object_proximitysensor_type, 2, &handlesCount,
    &handles, &parentsCount, &parents, NULL, NULL, NULL, NULL,
    simx_opmode_oneshot_wait);

  int count = 0;
  // find sensors; first is laser, second is sonar
  for(simxInt i = 0; i < handlesCount; ++i)
  {
    if(parents[i] == node_id_)
    {
      switch(count)
      {
        case 0:
          laser_sensor_ = handles[i];
          break;
        case 1:
          sonar_sensor_ = handles[i];
          break;
        default:
          continue;
      }
      ++count;
    }
  }
}

#endif // _GAMS_VREP_
