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

#include "VREP_UAV.h"


#include <iostream>
#include <cmath>

#include "madara/knowledge_engine/containers/Double_Vector.h"

#include "gams/variables/Sensor.h"

#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)

using std::endl;
using std::cout;
using std::string;
using Madara::Knowledge_Engine::Containers::Native_Double_Vector;

const string gams::platforms::VREP_UAV::DEFAULT_UAV_MODEL (
  (getenv ("GAMS_ROOT") == 0) ? 
  "" : // if GAMS_ROOT is not defined, then just leave this as empty string
  (string (getenv ("GAMS_ROOT")) + "/resources/vrep/Quadricopter_NoCamera.ttm")
  );

const double gams::platforms::VREP_UAV::Target_Mover::RATE (30.0);

const std::string gams::platforms::VREP_UAV::Target_Mover::DEST_CONTAINER_NAME
  (".platform.vrep_uav.thread.destination");

const std::string gams::platforms::VREP_UAV::MOVE_THREAD_NAME ("move_thread");

gams::platforms::Base_Platform *
gams::platforms::VREP_UAV_Factory::create (
  const Madara::Knowledge_Vector & args,
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  variables::Sensors * sensors,
  variables::Platforms * platforms,
  variables::Self * self)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MINOR,
    "entering gams::platforms::VREP_UAV_Factory::create\n");

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

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MINOR,
         "gams::platforms::VREP_UAV_Factory::create:" \
        " no transports found, attaching multicast\n");
    }

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
       "gams::platforms::VREP_UAV_Factory::create:" \
      " creating VREP_UAV object\n");

    // specify the model file
    string model_file;
    simxUChar is_client_side; // file is on server
    if (args.size () >= 1)
    {
      model_file = args[0].to_string ();
      is_client_side = 1;
    }
    else
    {
      model_file = VREP_UAV::DEFAULT_UAV_MODEL;
      is_client_side = 0;
    }

    result = new VREP_UAV (model_file, is_client_side, knowledge, sensors, 
      platforms, self);
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
       "gams::platforms::VREP_UAV_Factory::create:" \
      " invalid knowledge, sensors, platforms, or self\n");
  }

  if (result == 0)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
       "gams::platforms::VREP_UAV_Factory::create:" \
      " error creating VREP_UAV object\n");
  }

  return result;
}

gams::platforms::VREP_UAV::VREP_UAV (
  std::string model_file, 
  simxUChar is_client_side, 
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  variables::Sensors * sensors,
  variables::Platforms * platforms,
  variables::Self * self) :
  VREP_Base (knowledge, sensors, self), 
  mover_ (
    Native_Double_Vector (Target_Mover::DEST_CONTAINER_NAME, *knowledge, 3)),
  threader_ (*knowledge), 
  thread_dest_ (Target_Mover::DEST_CONTAINER_NAME, *knowledge, 3)
{
  if (knowledge && sensors && platforms && self)
  {
    (*platforms)[get_id ()].init_vars (*knowledge, get_id ());
    status_ = (*platforms)[get_id ()];

    self->device.desired_altitude = self->id.to_integer () + 1;
    add_model_to_environment (model_file, is_client_side);
    set_initial_position ();
    get_target_handle ();
    wait_for_go ();

    double move_speed = knowledge_->get (".vrep_uav_move_speed").to_double ();
    if (move_speed > 0)
    {
      mover_.set_move_speed (move_speed);
      set_move_speed (move_speed);
    }

    mover_.set_client_id (client_id_);
    threader_.run (Target_Mover::RATE, MOVE_THREAD_NAME, &mover_);
  }
}

int
gams::platforms::VREP_UAV::move (const utility::Position & position, const double & epsilon)
{
  // update variables
  Base_Platform::move (position);

  // inform the thread
  const utility::GPS_Position* dest_gps_pos = 
    dynamic_cast<const utility::GPS_Position*>(&position);
  if (dest_gps_pos == 0)
  {
    position.to_container (thread_dest_);
  }
  else
  {
    utility::Position pos;
    gps_to_vrep (*dest_gps_pos, pos);
    pos.to_container (thread_dest_);
  }

  return 1;
}

void
gams::platforms::VREP_UAV::add_model_to_environment (const string &model_file, 
  const simxUChar is_client_side)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_ERROR,
    "gams::platforms::VREP_UAV::add_model_to_environment(" \
    "%s, %u)\n", model_file.c_str (), is_client_side);
  if (simxLoadModel (client_id_, model_file.c_str (), is_client_side, &node_id_,
    simx_opmode_oneshot_wait) != simx_error_noerror)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
       "gams::platforms::VREP_UAV::add_model_to_environment:" \
      " error loading model in vrep\n");
    exit (-1);
  }

  if (node_id_ < 0)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
       "gams::platforms::VREP_UAV::add_model_to_environment:" \
      " invalid handle id\n");
    exit (-1);
  }
}

std::string gams::platforms::VREP_UAV::get_id () const
{
  return "vrep_uav";
}

std::string gams::platforms::VREP_UAV::get_name () const
{
  return "VREP UAV";
}

void
gams::platforms::VREP_UAV::get_target_handle ()
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
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
       "gams::platforms::VREP_UAV::get_target_handle:" \
      " invalid target handle id\n");
  }

  mover_.set_node_target (node_target_);
}

void
gams::platforms::VREP_UAV::set_initial_position ()
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

  // send set object position command
  pos[2] = knowledge_->get (".initial_alt").to_double ();
  simxSetObjectPosition (client_id_, node_id_, -1, pos,
    simx_opmode_oneshot_wait);

  // initial position
  utility::Position initial;
  array_to_position (pos, initial);
  mover_.set_target_pos (initial);
}

gams::platforms::VREP_UAV::Target_Mover::Target_Mover (
  const Madara::Knowledge_Engine::Containers::Native_Double_Vector& d, 
  double m) :
  client_id_ (-1), node_target_ (-1), move_speed_ (m), target_pos_ (), 
  destination_ (d)
{
}

void
gams::platforms::VREP_UAV::Target_Mover::run ()
{
  double local_move_speed = move_speed_ / RATE;

  // get destination
  utility::Position dest_pos;
  dest_pos.from_container (destination_);
  simxFloat dest_arr[3];
  position_to_array (dest_pos, dest_arr);

  // set current position of node target
  simxFloat curr_arr[3];
  position_to_array (target_pos_, curr_arr);

  // get distance to destination
  double distance_to_destination = dest_pos.distance_to_2d (target_pos_);
  
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::platforms::VREP_UAV::Target_Mover::run:" \
    " moving to (%f,%f, distance %f m)\n",
    dest_pos.x, dest_pos.y, distance_to_destination);

  // check if quadrotor has reached target (within epsilon)
  if(distance_to_destination <= local_move_speed)
  {
    target_pos_ = dest_pos;
    simxSetObjectPosition (client_id_, node_target_, -1, dest_arr,
      simx_opmode_oneshot_wait);
    return;
  }

  // move quadrotor target closer to the desired position
  if(distance_to_destination < local_move_speed) // we can get to target in one step
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_DETAILED,
      "gams::platforms::VREP_UAV::Target_Mover::run:" \
      " moving to target instantly\n");

    curr_arr[0] = dest_arr[0];
    curr_arr[1] = dest_arr[1];
    curr_arr[2] = dest_arr[2];
  }
  else // we cannot reach target in this step
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_DETAILED,
      "gams::platforms::VREP_UAV::Target_Mover::run:" \
      " calculating new target location\n");

    // how far do we have to go in each dimension
    double dist[3];
    for (int i = 0; i < 3; ++i)
      dist[i] = fabs (curr_arr[i] - dest_arr[i]);

    // update target position
    for (int i = 0; i < 3; ++i)
    {
      if(curr_arr[i] < dest_arr[i])
        curr_arr[i] += dist[i] * local_move_speed / distance_to_destination;
      else
        curr_arr[i] -= dist[i] * local_move_speed / distance_to_destination;
    }
  }

  // send movement command
  simxSetObjectPosition (client_id_, node_target_, -1, curr_arr,
    simx_opmode_oneshot_wait);
  array_to_position (curr_arr, target_pos_);

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_ERROR,
    "gams::platforms::VREP_UAV::move:" \
    " setting target to \"%f,%f,%f\"\n", curr_arr[0], curr_arr[1], curr_arr[2]);
}

void
gams::platforms::VREP_UAV::Target_Mover::set_node_target (simxInt n)
{
  node_target_ = n;
}

void
gams::platforms::VREP_UAV::Target_Mover::set_client_id (simxInt c)
{
  client_id_ = c;
}

void
gams::platforms::VREP_UAV::Target_Mover::set_move_speed (double m)
{
  move_speed_ = m;
}

void
gams::platforms::VREP_UAV::Target_Mover::set_target_pos (
  const utility::Position& p)
{
  target_pos_ = p;
}

#endif // _GAMS_VREP_
