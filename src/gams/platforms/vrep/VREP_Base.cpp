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

#include "gams/platforms/vrep/VREP_Base.h"

#include <iostream>
#include <cmath>

#include "madara/knowledge_engine/containers/Double_Vector.h"

#include "gams/variables/Sensor.h"

#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)

using std::endl;
using std::cout;
using std::string;

gams::platforms::VREP_Base::VREP_Base (
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  variables::Sensors * sensors,
  variables::Self * self)
  : Base_Platform (knowledge, sensors, self), airborne_ (false),
    move_speed_ (0.8)
{
  if (sensors && knowledge)
  {
    // grab coverage sensor
    variables::Sensors::iterator it = sensors->find ("coverage");
    if (it == sensors->end ()) // create coverage sensor
    {
      // get origin
      utility::GPS_Position origin;
      Madara::Knowledge_Engine::Containers::Native_Double_Array origin_container;
      origin_container.set_name ("sensor.coverage.origin", *knowledge, 3);
      origin.from_container (origin_container);

      // establish sensor
      variables::Sensor* coverage_sensor =
        new variables::Sensor ("coverage", knowledge, 2.5, origin);
      (*sensors)["coverage"] = coverage_sensor;
    }
    (*sensors_)["coverage"] = (*sensors)["coverage"];

    // get vrep environment data
    string sw = knowledge->get (".vrep_sw_position").to_string ();
    double lat, lon;
    sscanf(sw.c_str (), "%lf,%lf", &lat, &lon);
    sw_position_.latitude (lat); sw_position_.longitude (lon);

    // get client id
    string vrep_host = knowledge->get (".vrep_host").to_string ();
    int vrep_port = knowledge->get (".vrep_port").to_integer ();
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::platforms::VREP_Base():" \
      " attempting to connect to VREP at %s:%d\n",
      vrep_host.c_str (), vrep_port);
    client_id_ = simxStart(vrep_host.c_str (), vrep_port, true, true, 2000, 5);
    if (client_id_ == -1)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_EMERGENCY,
        "gams::platforms::VREP_Base():" \
        " VREP connection failure, exit(-1)\n");
      exit (-1);
    }
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::platforms::VREP_Base():" \
      " successfully connected to VREP at %s:%d\n",
      vrep_host.c_str (), vrep_port);
    knowledge->wait ("vrep_ready == 1;");
  }
}

gams::platforms::VREP_Base::~VREP_Base ()
{
  simxInt childId = 0;
  while (childId != -1)
  {
    simxInt retVal = simxGetObjectChild (client_id_, node_id_, 0, &childId,
      simx_opmode_oneshot_wait);
    if (retVal != simx_error_noerror)
      cerr << "error getting child of node: " << node_id_ << endl;

    if(childId != -1)
    {
      retVal = simxRemoveObject (client_id_, childId, simx_opmode_oneshot_wait);
      if(retVal != simx_error_noerror)
        cerr << "error removing child id " << childId << endl;
    }
  }

  if (simxRemoveObject (client_id_, node_id_, simx_opmode_oneshot_wait)
    != simx_error_noerror)
  {
    cerr << "error deleting node " << node_id_ << endl;
  }
}

void
gams::platforms::VREP_Base::operator= (const VREP_Base & rhs)
{
  if (this != &rhs)
  {
    this->Base_Platform::operator= (rhs);
    this->airborne_ = rhs.airborne_;
    this->client_id_ = rhs.client_id_;
    this->move_speed_ = rhs.move_speed_;
    this->node_id_ = rhs.node_id_;
    this->node_target_ = rhs.node_target_;
    this->sw_position_ = rhs.sw_position_;
  }
}

int
gams::platforms::VREP_Base::sense (void)
{
  // get position
  simxFloat curr_arr[3];
  simxGetObjectPosition (client_id_, node_id_, sim_handle_parent, curr_arr,
                        simx_opmode_oneshot_wait);

  utility::Position vrep_pos;
  array_to_position (curr_arr, vrep_pos);
  utility::GPS_Position position;
  vrep_to_gps (vrep_pos, position);

  // set position in madara
  position.to_container (self_->device.location);

  return 0;
}

int
gams::platforms::VREP_Base::analyze (void)
{
  // set position on coverage map
  utility::Position* pos = get_position();
  (*sensors_)["coverage"]->set_value (
    utility::GPS_Position(*pos),
    knowledge_->get_context ().get_clock ());
  delete pos;

  return 0;
}

double
gams::platforms::VREP_Base::get_accuracy () const
{
  return 1.0;
}

double
gams::platforms::VREP_Base::get_move_speed () const
{
  return move_speed_;
}

int
gams::platforms::VREP_Base::land (void)
{
  if (airborne_)
  {
    airborne_ = false;
  }

  return 0;
}

int
gams::platforms::VREP_Base::move (const utility::Position & position,
  const double & epsilon)
{
  // update variables
  Base_Platform::move (position);

  // convert form gps reference frame to vrep reference frame
  simxFloat dest_arr[3];
  utility::Position dest_pos;
  gps_to_vrep (position, dest_pos);
  position_to_array (dest_pos, dest_arr);
  if (dest_arr[2] == 0)
    dest_arr[2] = (simxFloat)0.08;

  // send movement command
  simxSetObjectPosition (client_id_, node_target_, sim_handle_parent, dest_arr,
                        simx_opmode_oneshot_wait);

  // check if we have reached target
  simxFloat curr_arr[3];
  simxGetObjectPosition (client_id_, node_id_, sim_handle_parent, curr_arr,
                        simx_opmode_oneshot_wait);
  utility::Position vrep_pos;
  array_to_position (curr_arr, vrep_pos);

  // return code
  if (vrep_pos.distance_to (dest_pos) < epsilon)
    return 2;
  else
    return 1;
}

void
gams::platforms::VREP_Base::set_move_speed (const double& speed)
{
  move_speed_ = speed;
}

int
gams::platforms::VREP_Base::takeoff (void)
{
  if (!airborne_)
  {
    airborne_ = true;
  }

  return 0;
}

void
gams::platforms::VREP_Base::get_target_handle ()
{
}

void
gams::platforms::VREP_Base::array_to_position(const simxFloat (&arr)[3],
  utility::Position & pos) const
{
  pos.x = arr[0];
  pos.y = arr[1];
  pos.z = arr[2];
}

void 
gams::platforms::VREP_Base::gps_to_vrep (const utility::GPS_Position & position,
  utility::Position & converted) const
{
  // assume the Earth is a perfect sphere
  const double EARTH_RADIUS = 6371000.0;
  const double EARTH_CIRCUMFERENCE = 2 * EARTH_RADIUS * M_PI;

  // convert the latitude/y coordinates
  converted.y = (position.latitude () - sw_position_.latitude ()) / 360.0 * EARTH_CIRCUMFERENCE;
  
  // assume the meters/degree longitude is constant throughout environment
  // convert the longitude/x coordinates
  double r_prime = EARTH_RADIUS * cos (DEG_TO_RAD (sw_position_.latitude ()));
  double circumference = 2 * r_prime * M_PI;
  converted.x = (position.longitude () - sw_position_.longitude ()) / 360.0 * circumference;

  // do nothing to altitude
  converted.z = position.altitude ();
}

void
gams::platforms::VREP_Base::position_to_array (const utility::Position & pos,
  simxFloat (&arr)[3]) const
{
  arr[0] = pos.x;
  arr[1] = pos.y;
  arr[2] = pos.z;
}

void 
gams::platforms::VREP_Base::vrep_to_gps (const utility::Position & position,
  utility::GPS_Position & converted) const
{
  // assume the Earth is a perfect sphere
  const double EARTH_RADIUS = 6371000.0;
  const double EARTH_CIRCUMFERENCE = 2 * EARTH_RADIUS * M_PI;

  // convert the latitude/y coordinates
  // VREP uses y for latitude
  converted.latitude ((360.0 * position.y / EARTH_CIRCUMFERENCE) + sw_position_.latitude ());
  
  // assume the meters/degree longitude is constant throughout environment
  // convert the longitude/x coordinates
  // VREP uses x for longitude
  double r_prime = EARTH_RADIUS * cos (DEG_TO_RAD (sw_position_.latitude ()));
  double circumference = 2 * r_prime * M_PI;
  converted.longitude ((360.0 * position.x / circumference) + sw_position_.longitude ());

  // do nothing to altitude
  converted.altitude (position.z);
}

void
gams::platforms::VREP_Base::set_initial_position () const
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
  pos[2] = (simxFloat) 0.08;

  // send set object position command
  simxSetObjectPosition (client_id_, node_id_, sim_handle_parent, pos,
    simx_opmode_oneshot_wait);
}

void
gams::platforms::VREP_Base::wait_for_go () const
{
  // sync with other nodes; wait for all processes to get up
  std::stringstream buffer, init_string;
  init_string << "S";
  init_string << self_->id.to_integer ();
  init_string << ".init";

  buffer << "(" << init_string.str () << " = 1)";
  buffer << " && begin_sim";
  std::string expression = buffer.str ();
  Madara::Knowledge_Engine::Wait_Settings wait_settings;
  wait_settings.send_list [init_string.str ()] = true;
  Madara::Knowledge_Engine::Compiled_Expression compiled;
  compiled = knowledge_->compile (expression);
  knowledge_->wait (compiled, wait_settings);
}

#endif // _GAMS_VREP_
