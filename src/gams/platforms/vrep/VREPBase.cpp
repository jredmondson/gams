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

#include "gams/platforms/vrep/VREPBase.h"

#include <iostream>
#include <cmath>

#include "madara/knowledge/containers/DoubleVector.h"

#include "gams/variables/Sensor.h"

#include <ace/Guard_T.h>

typedef ACE_Guard<MADARA_LOCK_TYPE> Guard;

#define VREP_LOCK_MUTEX(mutex) \
  for(Guard guard__(mutex), *ptr__ = &guard__; ptr__; ptr__ = NULL)

#define VREP_LOCK VREP_LOCK_MUTEX(this->vrep_mutex_)

using std::endl;
using std::cout;
using std::string;

const std::string gams::platforms::VREPBase::TargetMover::NAME("move_thread");

gams::platforms::VREPBase::VREPBase (
  madara::knowledge::KnowledgeBase * knowledge,
  variables::Sensors * sensors,
  variables::Self * self)
  : BasePlatform (knowledge, sensors, self), airborne_ (false),
    move_speed_ (0.8), sw_pose_ (get_sw_pose(get_frame())),
    vrep_frame_ (sw_pose_), mover_ (NULL)
{
  if (sensors && knowledge)
  {
    // setup containers to access movement configuration values
    thread_rate_.set_name(".vrep_move_thread_rate", *knowledge);
    if(!thread_rate_.exists())
      thread_rate_ = 0;

    thread_move_speed_.set_name(".vrep_thread_move_speed", *knowledge);
    if(!thread_move_speed_.exists())
      thread_move_speed_ = 0;

    max_delta_.set_name(".vrep_max_delta", *knowledge);
    if(!max_delta_.exists())
      max_delta_ = 0;

    max_delta_.set_name(".vrep_max_rotate_delta", *knowledge);
    if(!max_rotate_delta_.exists())
      max_rotate_delta_ = 0;

    // grab coverage sensor
    variables::Sensors::iterator it = sensors->find ("coverage");
    if (it == sensors->end ()) // create coverage sensor
    {
      // get origin
      utility::GPSPosition origin;
      madara::knowledge::containers::NativeDoubleArray origin_container;
      origin_container.set_name ("sensor.coverage.origin", *knowledge, 3);
      origin.from_container (origin_container);

      // establish sensor
      variables::Sensor* coverage_sensor =
        new variables::Sensor ("coverage", knowledge, 2.5, origin);
      (*sensors)["coverage"] = coverage_sensor;
    }
    (*sensors_)["coverage"] = (*sensors)["coverage"];

    // get client id
    string vrep_host = knowledge->get (".vrep_host").to_string ();
    int vrep_port = knowledge->get (".vrep_port").to_integer ();
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::platforms::VREPBase():" \
      " attempting to connect to VREP at %s:%d\n",
      vrep_host.c_str (), vrep_port);
    client_id_ = simxStart(vrep_host.c_str (), vrep_port, true, true, 2000, 5);
    if (client_id_ == -1)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_EMERGENCY,
        "gams::platforms::VREPBase():" \
        " VREP connection failure, exit(-1)\n");
      exit (-1);
    }
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::platforms::VREPBase():" \
      " successfully connected to VREP at %s:%d\n",
      vrep_host.c_str (), vrep_port);
    knowledge->wait ("vrep_ready == 1;");
  }
}

gams::utility::Pose
gams::platforms::VREPBase::get_sw_pose(const utility::ReferenceFrame &frame)
{
  if(knowledge_)
  {
    // get vrep environment data
    string sw = knowledge_->get (".vrep_sw_position").to_string ();
    double lat, lon;
    if(sscanf(sw.c_str (), "%lf,%lf", &lat, &lon) == 2)
      return utility::Pose(frame, lon, lat);
  }
  return utility::Pose(frame, 0, 0);
}

gams::platforms::VREPBase::~VREPBase ()
{
  simxInt retVal;
  simxInt childId = 0;

  if(mover_ != NULL)
  {
    threader_.terminate(TargetMover::NAME);
    mover_ = NULL;
  }

  while (childId != -1)
  {
    VREP_LOCK
    {
      retVal = simxGetObjectChild (client_id_, node_id_,
                                   0, &childId, simx_opmode_oneshot_wait);
    }

    if (retVal != simx_error_noerror)
      cerr << "error getting child of node: " << node_id_ << endl;

    if(childId != -1)
    {
      VREP_LOCK
      {
        retVal = simxRemoveObject (client_id_, childId,
                                   simx_opmode_oneshot_wait);
      }

      if(retVal != simx_error_noerror)
        cerr << "error removing child id " << childId << endl;
    }
  }

  VREP_LOCK
  {
    retVal = simxRemoveObject (client_id_, node_id_,
                               simx_opmode_oneshot_wait);
  }

  if (retVal != simx_error_noerror)
  {
    cerr << "error deleting node " << node_id_ << endl;
  }
}

void
gams::platforms::VREPBase::operator= (const VREPBase & rhs)
{
  if (this != &rhs)
  {
    this->BasePlatform::operator= (rhs);
    this->airborne_ = rhs.airborne_;
    this->client_id_ = rhs.client_id_;
    this->move_speed_ = rhs.move_speed_;
    this->node_id_ = rhs.node_id_;
    this->node_target_ = rhs.node_target_;
  }
}

int
gams::platforms::VREPBase::sense (void)
{
  // get position
  simxFloat curr_arr[3];
  VREP_LOCK
  {
    simxGetObjectPosition (client_id_, node_id_, -1, curr_arr,
                                   simx_opmode_oneshot_wait);

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_DETAILED,
      "gams::algorithms::platforms::VREPBase:" \
      " vrep position: %f,%f,%f\n", curr_arr[0], curr_arr[1], curr_arr[2]);

    utility::Location vrep_loc(get_vrep_frame(), curr_arr);
    utility::Location loc(get_frame(), vrep_loc);

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_DETAILED,
      "gams::algorithms::platforms::VREPBase:" \
      " gps position: %f,%f,%f\n", loc.lat(), loc.lng(), loc.alt());

    // set position in madara
    loc.to_container<utility::order::GPS> (self_->agent.location);
  }

  return 0;
}

int
gams::platforms::VREPBase::analyze (void)
{
  // set position on coverage map
  utility::Position* pos = get_position();
  (*sensors_)["coverage"]->set_value (
    utility::GPSPosition(*pos),
    knowledge_->get_context ().get_clock ());
  delete pos;

  return 0;
}

double
gams::platforms::VREPBase::get_accuracy () const
{
  return 0.5;
}

double
gams::platforms::VREPBase::get_move_speed () const
{
  return move_speed_;
}

int
gams::platforms::VREPBase::land (void)
{
  if (airborne_)
  {
    airborne_ = false;
  }

  return 0;
}

int
gams::platforms::VREPBase::move (const utility::Position & position,
  const double & epsilon)
{
  const utility::GPSPosition *gps_pos =
    dynamic_cast<const utility::GPSPosition *>(&position);

  if(gps_pos != NULL)
  {
    return move(utility::Location(get_frame(),
              gps_pos->longitude(), gps_pos->latitude(), gps_pos->altitude()),
              epsilon);
  }
  else
  {
    return move(utility::Location(get_vrep_frame(),
              position.x, position.y, position.z),
              epsilon);
  }
}

int
gams::platforms::VREPBase::do_move (const utility::Location & target,
                                    const utility::Location & current,
                                    double max_delta)
{
  simxFloat dest_arr[3];
  target.to_array(dest_arr);

  simxFloat curr_arr[3];
  current.to_array(curr_arr);

  double distance = target.distance_to(current);

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_TRACE,
    "gams::platforms::VREPQuad::do_move:" \
    " moving from (%f, %f, %f) to (%f, %f, %f, distance %f m).",
    current.x(), current.y(), current.z(),
    target.x(), target.y(), target.z(), distance);

  if (max_delta > 0)
  {
    // move quadrotor target closer to the desired position
    if(distance < max_delta) // we can get to target in one step
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_TRACE,
        "gams::platforms::VREPQuad::do_move:" \
        " moving to target instantly\n");

      curr_arr[0] = dest_arr[0];
      curr_arr[1] = dest_arr[1];
      curr_arr[2] = dest_arr[2];
    }
    else // we cannot reach target in this step
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_TRACE,
        "gams::platforms::VREPQuad::do_move:" \
        " calculating new target location\n");

      // how far do we have to go in each dimension
      double dist[3];
      for (int i = 0; i < 3; ++i)
        dist[i] = curr_arr[i] - dest_arr[i];

      // update target position
      for (int i = 0; i < 3; ++i)
      {
        curr_arr[i] -= dist[i] * max_delta / distance;
      }
    }

    // send movement command
    VREP_LOCK
    {
      simxSetObjectPosition (client_id_, node_target_, -1, curr_arr,
                             simx_opmode_oneshot_wait);
    }

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_TRACE,
      "gams::platforms::VREPQuad::do_move:" \
      " setting target to \"%f,%f,%f\"\n", curr_arr[0], curr_arr[1], curr_arr[2]);
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_TRACE,
      "gams::platforms::VREPQuad::do_move:" \
      " setting target to \"%f,%f,%f\"\n", dest_arr[0], dest_arr[1], dest_arr[2]);

    // send movement command
    VREP_LOCK
    {
      simxSetObjectPosition (client_id_, node_target_, -1, dest_arr,
                             simx_opmode_oneshot_wait);
    }
  }
  return 1;
}

int
gams::platforms::VREPBase::do_rotate (const utility::Rotation & target,
                                      const utility::Rotation & current,
                                      double max_delta)
{
  // TODO: handle non-Z-axis rotation; for now ignore X and Y as workaround
  simxFloat dest_arr[3];
  dest_arr[0] = 0;
  dest_arr[1] = 0;
  dest_arr[2] = target.rz();

  simxFloat curr_arr[3];
  curr_arr[0] = 0;
  curr_arr[1] = 0;
  curr_arr[2] = target.rz();

  double distance = target.distance_to(current);

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_TRACE,
    "gams::platforms::VREPQuad::do_rotate:" \
    " rotating from (%f, %f, %f) to (%f, %f, %f, distance %f m).",
    current.rx(), current.ry(), current.rz(),
    target.rx(), target.ry(), target.rz(), distance);

  if (max_delta > 0)
  {
    // move quadrotor target closer to the desired position
    if(distance < max_delta) // we can get to target in one step
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_TRACE,
        "gams::platforms::VREPQuad::do_rotate:" \
        " rotating to target instantly\n");

      curr_arr[0] = dest_arr[0];
      curr_arr[1] = dest_arr[1];
      curr_arr[2] = dest_arr[2];
    }
    else // we cannot reach target in this step
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_TRACE,
        "gams::platforms::VREPQuad::do_rotate:" \
        " calculating new target location\n");

      // how far do we have to go in each dimension
      double dist[3];
      for (int i = 0; i < 3; ++i)
        dist[i] = curr_arr[i] - dest_arr[i];

      // update target position
      for (int i = 0; i < 3; ++i)
      {
        curr_arr[i] -= dist[i] * max_delta / distance;
      }
    }

    // send movement command
    VREP_LOCK
    {
      simxSetObjectOrientation (client_id_, node_target_, -1, curr_arr,
                                simx_opmode_oneshot_wait);
    }

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_TRACE,
      "gams::platforms::VREPQuad::do_rotate:" \
      " setting target to \"%f,%f,%f\"\n", curr_arr[0], curr_arr[1], curr_arr[2]);
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_TRACE,
      "gams::platforms::VREPQuad::do_rotate:" \
      " setting target to \"%f,%f,%f\"\n", dest_arr[0], dest_arr[1], dest_arr[2]);

    // send movement command
    VREP_LOCK
    {
      simxSetObjectOrientation (client_id_, node_target_, -1, dest_arr,
                                simx_opmode_oneshot_wait);
    }
  }
  return 1;
}

int
gams::platforms::VREPBase::move (const utility::Location & target,
  double epsilon)
{
  // update variables
  BasePlatform::move (target);

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_TRACE,
    "gams::platforms::VREPQuad::move:" \
    " requested target \"%f,%f,%f\"\n", target.x(), target.y(), target.z());

  // convert form input reference frame to vrep reference frame, if necessary
  utility::Location vrep_target(get_vrep_frame(), target);

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_TRACE,
    "gams::platforms::VREPQuad::move:" \
    " vrep target \"%f,%f,%f\" rate %f\n", vrep_target.x(), vrep_target.y(), vrep_target.z(), thread_rate_.to_double());

  // get current position in VREP frame
  simxFloat curr_arr[3];
  VREP_LOCK
  {
    simxGetObjectPosition (client_id_, node_target_, -1, curr_arr,
                           simx_opmode_oneshot_wait);
  }
  utility::Location vrep_loc(get_vrep_frame(), curr_arr);

  if (thread_rate_.to_double() == 0)
  {
    if(mover_ != NULL)
    {
      threader_.terminate(TargetMover::NAME);
      mover_ = NULL;
    }

    if(do_move(vrep_target, vrep_loc, max_delta_.to_double()) == 0)
      return 0;
  }
  else
  {
    if(mover_ == NULL)
    {
      mover_ = new TargetMover(*this);
      threader_.run(thread_rate_.to_double(), TargetMover::NAME, mover_);
    }
  }

  // return code
  if (vrep_loc.distance_to (vrep_target) < epsilon)
    return 2;
  else
    return 1;
}

int
gams::platforms::VREPBase::rotate (const utility::Rotation & target,
  double epsilon)
{
  // update variables
  BasePlatform::rotate (target);

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_TRACE,
    "gams::platforms::VREPQuad::rotate:" \
    " requested target \"%f,%f,%f\"\n", target.rx(), target.ry(), target.rz());

  // convert form input reference frame to vrep reference frame, if necessary
  utility::Rotation vrep_target(get_vrep_frame(), target);

  // get current position in VREP frame
  simxFloat curr_arr[3];
  VREP_LOCK
  {
    simxGetObjectOrientation (client_id_, node_target_, -1, curr_arr,
                              simx_opmode_oneshot_wait);
  }

  // TODO: handle non-Z-axis rotation; for now ignore X and Y as workaround
  curr_arr[0] = 0;
  curr_arr[1] = 0;

  utility::Rotation vrep_rot(get_vrep_frame(), 0, 0, curr_arr[2]);

  if(do_rotate(vrep_target, vrep_rot, max_rotate_delta_.to_double()) == 0)
    return 0;

  #if 0
  simxFloat dest_arr[3];

  // TODO: handle non-Z-axis rotation; for now ignore X and Y as workaround
  dest_arr[0] = 0;
  dest_arr[1] = 0;

  dest_arr[2] = vrep_target.rz();

  // send movement command
  VREP_LOCK
  {
    simxSetObjectOrientation (client_id_, node_target_, -1, dest_arr,
                           simx_opmode_oneshot_wait);
  }
  #endif

  // return code
  if (vrep_rot.distance_to (vrep_target) < epsilon)
    return 2;
  else
    return 1;
}

void
gams::platforms::VREPBase::set_move_speed (const double& speed)
{
  move_speed_ = speed;
}

int
gams::platforms::VREPBase::takeoff (void)
{
  if (!airborne_)
  {
    airborne_ = true;
  }

  return 0;
}

void
gams::platforms::VREPBase::get_target_handle ()
{
}

void
gams::platforms::VREPBase::set_initial_position ()
{
  // get initial position
  simxFloat pos[3];
  if (knowledge_->exists(".initial_lat") && knowledge_->exists(".initial_lon"))
  {
    // get gps coords
    utility::Location gps_loc(get_frame(),
                       knowledge_->get (".initial_lon").to_double (),
                       knowledge_->get (".initial_lat").to_double ());

    // convert to vrep
    utility::Location vrep_loc(get_vrep_frame(), gps_loc);
    vrep_loc.to_array(pos);
  }
  else
  {
    // get vrep coords
    pos[0] = knowledge_->get (".initial_x").to_double ();
    pos[1] = knowledge_->get (".initial_y").to_double ();
  }
  pos[2] = get_initial_z();

  // send set object position command
  VREP_LOCK
  {
    simxSetObjectPosition (client_id_, node_id_, -1, pos,
                           simx_opmode_oneshot_wait);
  }

  utility::Location vrep_loc(get_vrep_frame(), pos);
  utility::Location gps_loc(get_frame(), vrep_loc);
  gps_loc.to_container<utility::order::GPS>(self_->agent.location);
  //BasePlatform::move(vrep_loc);
}

double
gams::platforms::VREPBase::get_initial_z() const
{
  if (knowledge_->exists(".initial_alt"))
    return knowledge_->get (".initial_alt").to_double();
  else
    return 0.08;
}

void
gams::platforms::VREPBase::wait_for_go () const
{
  // sync with other nodes; wait for all processes to get up
  std::stringstream buffer, init_string;
  init_string << "S";
  init_string << self_->id.to_integer ();
  init_string << ".init";

  buffer << "(" << init_string.str () << " = 1)";
  buffer << " && begin_sim";
  std::string expression = buffer.str ();
  madara::knowledge::WaitSettings wait_settings;
  wait_settings.send_list [init_string.str ()] = true;
  madara::knowledge::CompiledExpression compiled;
  compiled = knowledge_->compile (expression);
  knowledge_->wait (compiled, wait_settings);
}

gams::platforms::VREPBase::TargetMover::TargetMover (
  VREPBase &base)
  : base_(base)
{
}

void
gams::platforms::VREPBase::TargetMover::run ()
{
  double rate = base_.thread_rate_.to_double();
  double local_move_speed = base_.thread_move_speed_.to_double () / rate;

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_TRACE,
    "gams::platforms::VREPQuad::TargetMover::run:" \
    " moving at speed %f\n", local_move_speed);

  // convert form input reference frame to vrep reference frame, if necessary
  utility::Location target(get_frame(), 0, 0);
  target.from_container<utility::order::GPS>(base_.self_->agent.dest);
  utility::Location vrep_target(base_.get_vrep_frame(), target);

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_TRACE,
    "gams::platforms::VREPQuad::TargetMover::run:" \
    " vrep target \"%f,%f,%f\"\n", vrep_target.x(), vrep_target.y(), vrep_target.z());

  // get current target position in VREP frame
  simxFloat curr_arr[3];
  VREP_LOCK_MUTEX(base_.vrep_mutex_)
  {
    simxGetObjectPosition (base_.client_id_, base_.node_target_, -1,
                           curr_arr, simx_opmode_oneshot_wait);
  }

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_TRACE,
    "gams::platforms::VREPQuad::TargetMover::run:" \
    " vrep target position \"%f,%f,%f\"\n", curr_arr[0], curr_arr[1], curr_arr[2]);

  utility::Location vrep_loc(base_.get_vrep_frame(), curr_arr);

  base_.do_move(vrep_target, vrep_loc, local_move_speed);
}

const gams::utility::ReferenceFrame &
gams::platforms::VREPBase::get_vrep_frame() const
{
  return vrep_frame_;
}

#endif // _GAMS_VREP_
