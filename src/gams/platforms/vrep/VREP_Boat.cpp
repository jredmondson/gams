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
 * @file VREP_Boat.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * This file contains the definition of the VREP_Boat simulator ant robot class
 */

#ifdef _GAMS_VREP_ // only compile this if we are simulating in VREP

#include "gams/platforms/vrep/VREP_Boat.h"

#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)

#include <iostream>
using std::endl;
using std::cout;
using std::string;
#include <cmath>

#include "madara/knowledge_engine/containers/Double_Vector.h"

#include "gams/variables/Sensor.h"

const string gams::platforms::VREP_Boat::DEFAULT_BOAT_MODEL (
  (getenv ("GAMS_ROOT") == 0) ? 
  "" : // if GAMS_ROOT is not defined, then just leave this as empty string
  (string (getenv ("GAMS_ROOT")) + "/resources/vrep/boat.ttm")
  );


gams::platforms::Base_Platform *
gams::platforms::VREP_Boat_Factory::create (
  const Madara::Knowledge_Vector & args,
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  variables::Sensors * sensors,
  variables::Platforms * platforms,
  variables::Self * self)
{
  const static string DEFAULT_BOAT_MODEL (string (getenv ("GAMS_ROOT")) + 
    "/resources/vrep/boat.ttm");

  Base_Platform * result (0);
  
  if (knowledge && sensors && platforms && self)
  {
    if (knowledge->get_num_transports () == 0)
    {
      Madara::Transport::QoS_Transport_Settings settings;

      settings.type = Madara::Transport::MULTICAST;
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
      file = VREP_Boat::DEFAULT_BOAT_MODEL;
      is_client_side = 0;
    }

    result = new VREP_Boat (file, is_client_side, knowledge, sensors, 
      platforms, self);
  }

  return result;
}

gams::platforms::VREP_Boat::VREP_Boat (
  const std::string& file, 
  const simxUChar client_side,
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  variables::Sensors * sensors,
  variables::Platforms * platforms,
  variables::Self * self)
  : VREP_Base (knowledge, sensors, self)
{
  if (platforms && knowledge)
  {
    (*platforms)[get_id ()].init_vars (*knowledge, get_id ());
    status_ = (*platforms)[get_id ()];
  }

  self_->device.desired_altitude = 0.05;
  add_model_to_environment (file, client_side);
  set_initial_position ();
  get_target_handle ();
  wait_for_go ();
}

void
gams::platforms::VREP_Boat::add_model_to_environment (const std::string& file,
  const simxUChar client_side)
{
  if (simxLoadModel (client_id_, file.c_str (), client_side, &node_id_,
    simx_opmode_oneshot_wait) != simx_error_noerror)
  {
    cerr << "error loading VREP_Boat model in vrep" << endl;
    exit (-1);
  }

  if (node_id_ < 0)
  {
    cerr << "invalid handle id for VREP_Boat base" << endl;
    exit (-1);
  }
}

std::string
gams::platforms::VREP_Boat::get_id () const
{
  return "vrep_boat";
}

std::string
gams::platforms::VREP_Boat::get_name () const
{
  return "VREP Boat";
}

double
gams::platforms::VREP_Boat::get_accuracy () const
{
  return 5;
}

void
gams::platforms::VREP_Boat::get_target_handle ()
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

void
gams::platforms::VREP_Boat::set_initial_position () const
{
  // get initial position
  simxFloat pos[3];
  if (knowledge_->get (".initial_lat").to_double () != 0)
  {
    // get gps coords
    utility::GPS_Position gps_coord;
    gps_coord.latitude (knowledge_->get (".initial_lat").to_double ());
    gps_coord.longitude (knowledge_->get (".initial_lon").to_double ());

    // convert to vrep
    utility::Position vrep_coord;
    gps_to_vrep (gps_coord, vrep_coord);
    position_to_array (vrep_coord, pos);
  }
  else
  {
    // get vrep coords
    pos[0] = knowledge_->get (".initial_x").to_double ();
    pos[1] = knowledge_->get (".initial_y").to_double ();
  }
  pos[2] = (simxFloat) 0.16;

  // send set object position command
  simxSetObjectPosition (client_id_, node_id_, sim_handle_parent, pos,
    simx_opmode_oneshot_wait);
}

#endif // _GAMS_VREP_
