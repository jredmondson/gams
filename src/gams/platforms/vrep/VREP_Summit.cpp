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
 * @file VREP_Summit.cpp
 * @author Anton Dukeman <anton.dukeman@gmail.com>
 *
 * This file contains the definition of the VREP_Summit simulator ant robot class
 */

#ifdef _GAMS_VREP_ // only compile this if we are simulating in VREP

#include "gams/platforms/vrep/VREP_Summit.h"

#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)

#include <iostream>
using std::endl;
using std::cout;
using std::string;
#include <cmath>

#include "madara/knowledge_engine/containers/Double_Vector.h"

#include "gams/variables/Sensor.h"

gams::platforms::Base_Platform *
gams::platforms::VREP_Summit_Factory::create (
        const Madara::Knowledge_Vector & args,
        Madara::Knowledge_Engine::Knowledge_Base * knowledge,
        variables::Sensors * sensors,
        variables::Platforms * platforms,
        variables::Self * self)
{
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

      GAMS_DEBUG (gams::utility::LOG_MINOR_EVENT, (LM_DEBUG, 
        DLINFO "gams::platforms::VREP_Summit_Factory::create:" \
        " no transports found, attaching multicast\n"));
    }

    GAMS_DEBUG (gams::utility::LOG_EVENT_TRACE, (LM_DEBUG, 
      DLINFO "gams::platforms::VREP_Summit_Factory::create:" \
      " creating VREP_Summit object\n"));

    result = new VREP_Summit (knowledge, sensors, platforms, self);
  }
  else
  {
    GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
      DLINFO "gams::platforms::VREP_Summit_Factory::create:" \
      " invalid knowledge, sensors, platforms, or self\n"));
  }

  if (result == 0)
  {
    GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
      DLINFO "gams::platforms::VREP_Summit_Factory::create:" \
      " error creating VREP_Summit object\n"));
  }

  return result;
}

gams::platforms::VREP_Summit::VREP_Summit (
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
  add_model_to_environment ();
  set_initial_position ();
  get_target_handle ();
  wait_for_go ();
}

void
gams::platforms::VREP_Summit::add_model_to_environment ()
{
  string modelFile (getenv ("GAMS_ROOT"));
  modelFile += "/resources/vrep/summit.ttm";
  if (simxLoadModel (client_id_, modelFile.c_str (), 0, &node_id_,
    simx_opmode_oneshot_wait) != simx_error_noerror)
  {
    GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
      DLINFO "gams::platforms::VREP_Summit::add_model_to_environment:" \
      " error loading model in vrep\n"));
    exit (-1);
  }

  if (node_id_ < 0)
  {
    GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
      DLINFO "gams::platforms::VREP_Summit::add_model_to_environment:" \
      " invalid handle id\n"));
    exit (-1);
  }
}

std::string
gams::platforms::VREP_Summit::get_id () const
{
  return "vrep_summit";
}

std::string
gams::platforms::VREP_Summit::get_name () const
{
  return "VREP Summit";
}

void
gams::platforms::VREP_Summit::get_target_handle ()
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

  if (node_target_ < 0)
  {
    GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
      DLINFO "gams::platforms::VREP_Summit::get_target_handle:" \
      " invalid target handle id\n"));
  }
}

void
gams::platforms::VREP_Summit::set_initial_position () const
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
  pos[2] = (simxFloat) 0.3;

  // send set object position command
  simxSetObjectPosition (client_id_, node_id_, sim_handle_parent, pos,
    simx_opmode_oneshot_wait);
}

#endif // _GAMS_VREP_
