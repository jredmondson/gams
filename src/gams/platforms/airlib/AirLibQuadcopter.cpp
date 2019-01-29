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

const string gams::platforms::AirLibQuadcopter::DEFAULT_BOAT_MODEL (
  (getenv ("GAMS_ROOT") == 0) ? 
  "" : // if GAMS_ROOT is not defined, then just leave this as empty string
  (string (getenv ("GAMS_ROOT")) + "/resources/airlib/boat.ttm")
  );


gams::platforms::BasePlatform *
gams::platforms::AirLibQuadcopterFactory::create (
  const madara::knowledge::KnowledgeMap & args,
  madara::knowledge::KnowledgeBase * knowledge,
  variables::Sensors * sensors,
  variables::Platforms * platforms,
  variables::Self * self)
{
  const static string DEFAULT_BOAT_MODEL (string (getenv ("GAMS_ROOT")) + 
    "/resources/airlib/boat.ttm");

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

    madara::knowledge::KnowledgeMap::const_iterator found =
      args.find ("client_side");

    if (found != args.end () && found->second.to_integer () == 1)
    {
      is_client_side = 1;
    }
    else
    {
      file = AirLibQuadcopter::DEFAULT_BOAT_MODEL;
      is_client_side = 0;
    }

    result = new AirLibQuadcopter (file, is_client_side, knowledge, sensors, 
      platforms, self);
  }

  return result;
}

gams::platforms::AirLibQuadcopter::AirLibQuadcopter (
  const std::string& file,
  const simxUChar client_side,
  madara::knowledge::KnowledgeBase * knowledge,
  variables::Sensors * sensors,
  variables::Platforms * platforms,
  variables::Self * self)
  : airlibBase (file, client_side, knowledge, sensors, self)
{
  if (platforms && knowledge)
  {
    (*platforms)[get_id ()].init_vars (*knowledge, get_id ());
    status_ = (*platforms)[get_id ()];
  }

  self_->agent.desired_altitude = 0.05;
}

void
gams::platforms::AirLibQuadcopter::add_model_to_environment (const std::string& file,
  const simxUChar client_side)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
    "gams::platforms::AirLibQuadcopter::add_model_to_environment(" \
    "%s, %d)\n", file.c_str (), (int)client_side);

  if (simxLoadModel (client_id_, file.c_str (), client_side, &node_id_,
    simx_opmode_oneshot_wait) != simx_error_noerror)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::platforms::AirLibQuadcopter::add_model_to_environment:" \
      " error loading model in airlib\n");
    exit (-1);
  }

  if (node_id_ < 0)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::platforms::AirLibQuadcopter::add_model_to_environment:" \
      " invalid handle id\n");
    exit (-1);
  }
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

double
gams::platforms::AirLibQuadcopter::get_accuracy () const
{
  return 5;
}

void
gams::platforms::AirLibQuadcopter::get_target_handle ()
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
gams::platforms::AirLibQuadcopter::get_initial_z() const
{
  return 0.16;
}

#endif // _GAMS_AIRLIB_
