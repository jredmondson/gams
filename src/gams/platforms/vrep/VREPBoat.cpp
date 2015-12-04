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

/**
 * @file VREPBoat.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * This file contains the definition of the VREPBoat simulator ant robot class
 */

#ifdef _GAMS_VREP_ // only compile this if we are simulating in VREP

#include "gams/platforms/vrep/VREPBoat.h"

#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)

#include <iostream>
using std::endl;
using std::cout;
using std::string;
#include <cmath>

#include "madara/knowledge/containers/DoubleVector.h"

#include "gams/variables/Sensor.h"

const string gams::platforms::VREPBoat::DEFAULT_BOAT_MODEL (
  (getenv ("GAMS_ROOT") == 0) ? 
  "" : // if GAMS_ROOT is not defined, then just leave this as empty string
  (string (getenv ("GAMS_ROOT")) + "/resources/vrep/boat.ttm")
  );


gams::platforms::BasePlatform *
gams::platforms::VREPBoatFactory::create (
  const madara::knowledge::KnowledgeVector & args,
  madara::knowledge::KnowledgeBase * knowledge,
  variables::Sensors * sensors,
  variables::Platforms * platforms,
  variables::Self * self)
{
  const static string DEFAULT_BOAT_MODEL (string (getenv ("GAMS_ROOT")) + 
    "/resources/vrep/boat.ttm");

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

    string file;
    simxUChar is_client_side;
    if (args.size () >= 1)
    {
      file = args[0].to_string ();
      is_client_side = 1;
    }
    else
    {
      file = VREPBoat::DEFAULT_BOAT_MODEL;
      is_client_side = 0;
    }

    result = new VREPBoat (file, is_client_side, knowledge, sensors, 
      platforms, self);
  }

  return result;
}

gams::platforms::VREPBoat::VREPBoat (
  const std::string& file, 
  const simxUChar client_side,
  madara::knowledge::KnowledgeBase * knowledge,
  variables::Sensors * sensors,
  variables::Platforms * platforms,
  variables::Self * self)
  : VREPBase (knowledge, sensors, self)
{
  if (platforms && knowledge)
  {
    (*platforms)[get_id ()].init_vars (*knowledge, get_id ());
    status_ = (*platforms)[get_id ()];
  }

  self_->agent.desired_altitude = 0.05;
  add_model_to_environment (file, client_side);
  set_initial_position ();
  get_target_handle ();
  wait_for_go ();
}

void
gams::platforms::VREPBoat::add_model_to_environment (const std::string& file,
  const simxUChar client_side)
{
  if (simxLoadModel (client_id_, file.c_str (), client_side, &node_id_,
    simx_opmode_oneshot_wait) != simx_error_noerror)
  {
    cerr << "error loading VREPBoat model in vrep" << endl;
    exit (-1);
  }

  if (node_id_ < 0)
  {
    cerr << "invalid handle id for VREPBoat base" << endl;
    exit (-1);
  }
}

std::string
gams::platforms::VREPBoat::get_id () const
{
  return "vrep_boat";
}

std::string
gams::platforms::VREPBoat::get_name () const
{
  return "VREP Boat";
}

double
gams::platforms::VREPBoat::get_accuracy () const
{
  return 5;
}

void
gams::platforms::VREPBoat::get_target_handle ()
{
  //find the dummy base sub-object
  simxInt handlesCount = 0,*handles = NULL;
  simxInt parentsCount = 0,*parents = NULL;
  simxGetObjectGroupData (client_id_, sim_object_dummy_type, 2, &handlesCount,
    &handles, &parentsCount, &parents, NULL, NULL, NULL, NULL,
    simx_opmode_oneshot_wait);

  // find node base
  simxInt nodeBase = -1;
  for(simxInt i = 0; i < handlesCount; ++i)
  {
    if(parents[i] == node_id_)
    {
      nodeBase = handles[i];
      break;
    }
  }

  // find the target sub-object of the base sub-object
  node_target_ = -1;
  simxGetObjectChild (client_id_, nodeBase, 0, &node_target_,
    simx_opmode_oneshot_wait);
}

double
gams::platforms::VREPBoat::get_initial_z() const
{
  return 0.16;
}

#endif // _GAMS_VREP_
