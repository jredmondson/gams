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
#include "madara/knowledge/containers/NativeDoubleVector.h"

#include "gams/variables/Sensor.h"

#include "gams/pose/Euler.h"

typedef MADARA_GUARD_TYPE Guard;

#define VREP_LOCK_MUTEX(mutex) \
  for(Guard guard__(mutex), *ptr__ = &guard__; ptr__; ptr__ = NULL)

#define VREP_LOCK VREP_LOCK_MUTEX(this->vrep_mutex_)

using std::endl;
using std::cout;
using std::string;

using namespace gams::pose::euler;

namespace containers = madara::knowledge::containers;

const std::string gams::platforms::VREPBase::TargetMover::NAME("move_thread");

gams::platforms::VREPBase::VREPBase (
  std::string model_file,
  simxUChar is_client_side,
  madara::knowledge::KnowledgeBase * knowledge,
  variables::Sensors * sensors,
  variables::Self * self)
  : BasePlatform (knowledge, sensors, self), airborne_ (false),
    move_speed_ (0.8), 
    node_target_ (-1),
    sw_pose_ (get_sw_pose(pose::gps_frame())),
    vrep_frame_ (sw_pose_), mover_ (NULL),
    agent_is_ready_ (false),
    vrep_is_ready_ (false),
    sim_is_running_ (false),
    model_file_ (model_file),
    is_client_side_ (is_client_side),
    begin_sim_ ("begin_sim", *knowledge),
    vrep_ready_ ("vrep_ready", *knowledge),
    agent_ready_ ("S" + self->id.to_string () + ".init", *knowledge_)
{
  if (sensors && knowledge)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::platforms::VREPBase ():" \
      " sw_pose_ is %s\n",
      sw_pose_.to_string ().c_str ());

    // setup containers to access movement configuration values
    thread_rate_.set_name(".vrep_move_thread_rate", *knowledge);
    if(!thread_rate_.exists ())
      thread_rate_ = 0;

    thread_move_speed_.set_name(".vrep_thread_move_speed", *knowledge);
    if(!thread_move_speed_.exists ())
      thread_move_speed_ = 0;

    max_delta_.set_name(".vrep_max_delta", *knowledge);
    if(!max_delta_.exists ())
      max_delta_ = 0;

    max_orient_delta_.set_name(".vrep_max_orient_delta", *knowledge);
    if(!max_orient_delta_.exists ())
      max_orient_delta_ = 0;

    // grab coverage sensor
    variables::Sensors::iterator it = sensors->find ("coverage");
    if (it == sensors->end ()) // create coverage sensor
    {
      // get origin
      pose::Position origin(pose::gps_frame());
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
      "gams::platforms::VREPBase ():" \
      " attempting to connect to VREP at %s:%d\n",
      vrep_host.c_str (), vrep_port);
    client_id_ = simxStart(vrep_host.c_str (), vrep_port, true, true, 2000, 5);
    if (client_id_ == -1)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_EMERGENCY,
        "gams::platforms::VREPBase ():" \
        " VREP connection failure, exit(-1)\n");
      exit (-1);
    }
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::platforms::VREPBase ():" \
      " successfully connected to VREP at %s:%d\n",
      vrep_host.c_str (), vrep_port);
  }
}

gams::pose::Pose
gams::platforms::VREPBase::get_sw_pose(const pose::ReferenceFrame &frame)
{
  if (knowledge_)
  {
    // get vrep environment data
    containers::NativeDoubleVector sw (".vrep_sw_position", *knowledge_);

    // VREP apparently has weird lat/lon x/y stuff going on. Reverse order.
    return pose::Pose (frame, sw[1], sw[0], 0, 0, 0, 0);
  }

  return pose::Pose (frame, 0, 0, 0, 0, 0, 0);
}

gams::platforms::VREPBase::~VREPBase ()
{
  simxInt retVal = -1;
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
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::platforms::VREPBase ~():" \
        " error getting child of node: %d\n",
        (int)node_id_);
    }

    if(childId != -1)
    {
      VREP_LOCK
      {
        retVal = simxRemoveObject (client_id_, childId,
                                   simx_opmode_oneshot_wait);
      }

      if(retVal != simx_error_noerror)
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
          "gams::platforms::VREPBase ~():" \
          " error removing child id %d\n",
          (int)childId);
      }
    }
  }

  VREP_LOCK
  {
    retVal = simxRemoveObject (client_id_, node_id_,
                               simx_opmode_oneshot_wait);
  }

  if (retVal != simx_error_noerror)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::platforms::VREPBase ~():" \
      " error deleting node %d\n",
      (int)node_id_);
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
  if (get_ready ())
  {
    // get position
    simxFloat curr_arr[3];
    simxFloat curr_orientation[3];
    VREP_LOCK
    {
      simxGetObjectPosition (client_id_, node_id_, -1, curr_arr,
                                     simx_opmode_oneshot_wait);

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_DETAILED,
        "gams::algorithms::platforms::VREPBase:" \
        " vrep position: %f,%f,%f\n", curr_arr[0], curr_arr[1], curr_arr[2]);

      simxGetObjectOrientation (client_id_, node_id_, -1, curr_orientation,
        simx_opmode_oneshot_wait);

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_DETAILED,
        "gams::algorithms::platforms::VREPBase:" \
        " vrep orientation: %f,%f,%f\n",
        curr_orientation[0], curr_orientation[1], curr_orientation[2]);

      pose::Position vrep_loc(get_vrep_frame (), curr_arr);
      pose::Position loc(pose::gps_frame(), vrep_loc);

      pose::euler::EulerVREP vrep_euler (
        curr_orientation[0], curr_orientation[1], curr_orientation[2]);

      pose::Orientation vrep_orient (get_vrep_frame (), vrep_euler.to_quat ());

      pose::euler::YawPitchRoll vrep_yawpitchroll (vrep_orient);

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_DETAILED,
        "gams::algorithms::platforms::VREPBase:" \
        " gps position: %f,%f,%f\n", loc.lat (), loc.lng (), loc.alt ());

      // set position in madara
      loc.to_container (self_->agent.location);
      self_->agent.orientation.set (2, vrep_yawpitchroll.c ());
      self_->agent.orientation.set (0, vrep_yawpitchroll.a ());
      self_->agent.orientation.set (1, vrep_yawpitchroll.b ());

      // now that location is set, make sure movement_available is enabled
      status_.movement_available = 1;
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::platforms::VREPBase::sense:" \
      " Unable to sense. Waiting on vrep_ready and begin_sim\n");

    // now that location is set, make sure movement_available is enabled
    status_.movement_available = 0;
  }

  return 0;
}

int
gams::platforms::VREPBase::analyze (void)
{
  if (get_ready ())
  {
    // set position on coverage map
    pose::Position pos = get_location ();
    (*sensors_)["coverage"]->set_value (
      pos, knowledge_->get_context ().get_clock ());
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::platforms::VREPBase::analyze:" \
      " Unable to analyze. Waiting on vrep_ready and begin_sim\n");
  }

  return 0;
}

double
gams::platforms::VREPBase::get_accuracy () const
{
  return 1.0;
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
gams::platforms::VREPBase::do_move (const pose::Position & target,
                                    const pose::Position & current,
                                    double max_delta)
{
  if (get_ready ())
  {
    simxFloat dest_arr[3];
    target.to_array (dest_arr);

    simxFloat curr_arr[3];
    current.to_array (curr_arr);

    double distance = target.distance_to (current);

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_DETAILED,
      "gams::platforms::VREPBase::do_move:" \
      " moving from (%f, %f, %f) to (%f, %f, %f, distance %f m).",
      current.x (), current.y (), current.z (),
      target.x (), target.y (), target.z (), distance);

    if (max_delta > 0)
    {
      // move quadrotor target closer to the desired position
      if (distance < max_delta) // we can get to target in one step
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_DETAILED,
          "gams::platforms::VREPBase::do_move:" \
          " moving to target instantly\n");

        curr_arr[0] = dest_arr[0];
        curr_arr[1] = dest_arr[1];
        curr_arr[2] = dest_arr[2];
      }
      else // we cannot reach target in this step
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_DETAILED,
          "gams::platforms::VREPBase::do_move:" \
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
        gams::loggers::LOG_DETAILED,
        "gams::platforms::VREPBase::do_move:" \
        " setting target to \"%f,%f,%f\"\n",
        curr_arr[0], curr_arr[1], curr_arr[2]);
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_DETAILED,
        "gams::platforms::VREPBase::do_move:" \
        " setting target to \"%f,%f,%f\"\n",
        dest_arr[0], dest_arr[1], dest_arr[2]);

      // send movement command
      VREP_LOCK
      {
        simxSetObjectPosition (client_id_, node_target_, -1, dest_arr,
        simx_opmode_oneshot_wait);
      }
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::platforms::VREPBase::do_move:" \
      " Unable to move. Waiting on vrep_ready and begin_sim\n");
  }

  return 1;
}

int
gams::platforms::VREPBase::do_orient (pose::Orientation target,
                                      const pose::Orientation & current,
                                      double max_delta)
{
  // TODO: handle non-Z-axis orientation; for now ignore X and Y as workaround

  /*
  EulerVREP euler_current(current);

  simxFloat curr_arr[3];
  curr_arr[0] = euler_current.a ();
  curr_arr[1] = euler_current.b ();
  curr_arr[2] = euler_current.c ();
  */

  if (get_ready ())
  {
    double distance = target.distance_to (current);

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_TRACE,
      "gams::platforms::VREPBase::do_orient:" \
      " rotating from (%f, %f, %f) to (%f, %f, %f, distance %f m).",
      current.rx (), current.ry (), current.rz (),
      target.rx (), target.ry (), target.rz (), distance);

    if (max_delta == 0 || distance < max_delta + 0.01)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_TRACE,
        "gams::platforms::VREPBase::do_orient:" \
        " rotating to target instantly\n");
    }
    else // we cannot reach target in this step
    {
      double fraction = max_delta / distance;

      // move quadrotor target closer to the desired orientation
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_TRACE,
        "gams::platforms::VREPBase::do_orient:" \
        " calculating new target location %f / %f == %f\n",
        max_delta, distance, fraction);

      target.slerp_this (current, 1.0 - fraction);
    }

    EulerVREP euler_target (target);
    simxFloat dest_arr[3];
    dest_arr[0] = euler_target.a ();
    dest_arr[1] = euler_target.b ();
    dest_arr[2] = euler_target.c ();

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_TRACE,
      "gams::platforms::VREPBase::do_orient:" \
      " setting target to \"%f,%f,%f\"\n", dest_arr[0], dest_arr[1], dest_arr[2]);

    // send movement command
    VREP_LOCK
    {
      simxSetObjectOrientation (client_id_, node_target_, -1, dest_arr,
      simx_opmode_oneshot_wait);
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::platforms::VREPBase::do_orient:" \
      " Unable to orient. Waiting on vrep_ready and begin_sim\n");
  }

  return 1;
}

int
gams::platforms::VREPBase::move (const pose::Position & target,
        const PositionBounds &bounds)
{
  if (get_ready ())
  {
    // update variables
    BasePlatform::move (target, bounds);

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_TRACE,
      "gams::platforms::VREPBase::move:" \
      " requested target \"%f,%f,%f\"\n", target.x (), target.y (), target.z ());

    // convert from input reference frame to vrep reference frame, if necessary
    pose::Position vrep_target (get_vrep_frame (), target);

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_TRACE,
      "gams::platforms::VREPBase::move:" \
      " vrep target \"%f,%f,%f\" rate %f\n", vrep_target.x (), vrep_target.y (), vrep_target.z (), thread_rate_.to_double ());

    // get current position in VREP frame
    simxFloat curr_arr[3];
    VREP_LOCK
    {
      simxGetObjectPosition (client_id_, node_target_, -1, curr_arr,
      simx_opmode_oneshot_wait);
    }
    pose::Position vrep_loc (get_vrep_frame (), curr_arr);

    if (thread_rate_.to_double () == 0)
    {
      if (mover_ != NULL)
      {
        threader_.terminate (TargetMover::NAME);
        mover_ = NULL;
      }

      if (do_move (vrep_target, vrep_loc, max_delta_.to_double ()) == 0)
        return 0;
    }
    else
    {
      if (mover_ == NULL)
      {
        mover_ = new TargetMover (*this);
        threader_.run (thread_rate_.to_double (), TargetMover::NAME, mover_);
      }
    }

    VREP_LOCK
    {
      simxGetObjectPosition (client_id_, node_id_, -1, curr_arr,
      simx_opmode_oneshot_wait);
    }

    pose::Position vrep_node_loc(get_vrep_frame (), curr_arr);
    // return code
    if (bounds.check_position(vrep_node_loc, vrep_target))
      return 2;
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::platforms::VREPBase::move:" \
      " Unable to move. Waiting on vrep_ready and begin_sim\n");
  }
  
  return 1;
}

int
gams::platforms::VREPBase::orient (const pose::Orientation & target,
        const OrientationBounds &bounds)
{
  if (get_ready ())
  {
    // update variables
    BasePlatform::orient (target, bounds);

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_TRACE,
      "gams::platforms::VREPBase::orient:" \
      " requested target \"%f,%f,%f\"\n", target.rx (), target.ry (), target.rz ());

    // convert from input reference frame to vrep reference frame, if necessary
    pose::Orientation vrep_target (get_vrep_frame (), target);

    // get current position in VREP frame
    simxFloat curr_arr[3];
    VREP_LOCK
    {
      simxGetObjectOrientation (client_id_, node_target_, -1, curr_arr,
      simx_opmode_oneshot_wait);
    }

    EulerVREP euler_curr (curr_arr[0], curr_arr[1], curr_arr[2]);

    pose::Orientation vrep_rot (euler_curr.to_orientation (get_vrep_frame ()));

    if (do_orient (vrep_target, vrep_rot, max_orient_delta_.to_double ()) == 0)
      return 0;

    VREP_LOCK
    {
      simxGetObjectOrientation (client_id_, node_id_, -1, curr_arr,
      simx_opmode_oneshot_wait);
    }

    EulerVREP euler_node_curr (curr_arr[0], curr_arr[1], curr_arr[2]);

    pose::Orientation vrep_node_rot (
      euler_node_curr.to_orientation (get_vrep_frame ()));

    // return code
    if (bounds.check_orientation(vrep_node_rot, vrep_target))
      return 2;
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::platforms::VREPBase::orient:" \
      " Unable to orient. Waiting on vrep_ready and begin_sim\n");
  }

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
  bool is_gps = false;
  pose::Position init_loc;
  if (knowledge_->exists(".initial_lat") && knowledge_->exists(".initial_lon"))
  {
    is_gps = true;
    // get gps coords
    pose::Position gps_loc(pose::gps_frame(),
                       knowledge_->get (".initial_lon").to_double (),
                       knowledge_->get (".initial_lat").to_double ());

    // convert to vrep
    pose::Position vrep_loc(get_vrep_frame (), gps_loc);
    vrep_loc.to_array(pos);

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::platforms::VREPBase::set_inital_position:" \
    " Converting from [%s] to [%s] via origin [%s]\n",
    gps_loc.to_string().c_str(), vrep_loc.to_string().c_str(),
    vrep_loc.frame().origin().to_string().c_str()
    );

    init_loc = gps_loc;
  }
  else
  {
    // get vrep coords
    pos[0] = knowledge_->get (".initial_x").to_double ();
    pos[1] = knowledge_->get (".initial_y").to_double ();

    init_loc = pose::Position(get_vrep_frame(), pos[0], pos[1], pos[2]);
  }
  pos[2] = get_initial_z ();

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::platforms::VREPBase::set_inital_position:" \
    " Placing VREP platform at %s[%s], vrep [%f, %f, %f]\n",
    is_gps ? "GPS" : "", init_loc.to_string().c_str(), pos[0], pos[1], pos[2]
    );

  // send set object position command
  VREP_LOCK
  {
    simxSetObjectPosition (client_id_, node_id_, -1, pos,
                           simx_opmode_oneshot_wait);
  }

  pose::Position vrep_loc(get_vrep_frame (), pos);
  pose::Position gps_loc(pose::gps_frame(), vrep_loc);
  gps_loc.to_container (self_->agent.location);
  //BasePlatform::move(vrep_loc);
}

double
gams::platforms::VREPBase::get_initial_z () const
{
  if (knowledge_->exists(".initial_alt"))
    return knowledge_->get (".initial_alt").to_double ();
  else
    return 0.08;
}

gams::platforms::VREPBase::TargetMover::TargetMover (
  VREPBase &base)
  : base_(base)
{
}

void
gams::platforms::VREPBase::TargetMover::run ()
{
  double rate = base_.thread_rate_.to_double ();
  double local_move_speed = base_.thread_move_speed_.to_double () / rate;

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_TRACE,
    "gams::platforms::VREPBase::TargetMover::run:" \
    " moving at speed %f\n", local_move_speed);

  // convert form input reference frame to vrep reference frame, if necessary
  pose::Position target(base_.get_frame (), 0, 0);
  target.from_container (base_.self_->agent.dest);
  pose::Position vrep_target(base_.get_vrep_frame (), target);

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_TRACE,
    "gams::platforms::VREPBase::TargetMover::run:" \
    " vrep target \"%f,%f,%f\"\n", vrep_target.x (), vrep_target.y (), vrep_target.z ());

  // get current target position in VREP frame
  simxFloat curr_arr[3];
  VREP_LOCK_MUTEX(base_.vrep_mutex_)
  {
    simxGetObjectPosition (base_.client_id_, base_.node_target_, -1,
                           curr_arr, simx_opmode_oneshot_wait);
  }

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_TRACE,
    "gams::platforms::VREPBase::TargetMover::run:" \
    " vrep target position \"%f,%f,%f\"\n", curr_arr[0], curr_arr[1], curr_arr[2]);

  pose::Position vrep_loc(base_.get_vrep_frame (), curr_arr);

  base_.do_move(vrep_target, vrep_loc, local_move_speed);
}

const gams::pose::ReferenceFrame &
gams::platforms::VREPBase::get_vrep_frame (void) const
{
  return vrep_frame_;
}


const gams::pose::ReferenceFrame &
gams::platforms::VREPBase::get_frame (void) const
{
  return pose::gps_frame();
}

#endif // _GAMS_VREP_
