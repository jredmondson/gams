/**
 * @file AirLibQuadcopter.cpp
 * @authors Alex Rozgo, Devon Ash<noobaca2@gmail.com>
 *
 * This file contains the definition of the AirLibQuadcopter simulator robot class
 */

#ifdef _GAMS_AIRLIB_ // only compile this if we are simulating in airsim

#include "gams/platforms/airlib/AirLibQuadcopter.h"

#include <iostream>
#include <cmath>

#include "madara/knowledge/containers/DoubleVector.h"

#include "gams/variables/Sensor.h"

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif
#include "rpc/rpc_error.h"
STRICT_MODE_ON

gams::platforms::BasePlatform *
gams::platforms::AirLibQuadcopterFactory::create (
        const madara::knowledge::KnowledgeMap & args,
        madara::knowledge::KnowledgeBase * knowledge,
        variables::Sensors * sensors,
        variables::Platforms * platforms,
        variables::Self * self)
{
  return new AirLibQuadcopter(knowledge, sensors, self);
}

gams::platforms::AirLibQuadcopter::AirLibQuadcopter (
  madara::knowledge::KnowledgeBase * knowledge,
  variables::Sensors * sensors,
  variables::Self * self)
  : AirLibBase (knowledge, sensors, self)
{
  vehicle_name_ = "agent." + std::to_string(self_->id.to_integer());
  std::cout << "vehicle_name: " << vehicle_name_ << std::endl;

  client_ = new msr::airlib::MultirotorRpcLibClient();

  // as an example of what to do here, create a coverage sensor
  if (knowledge && sensors)
  {
    // set the data plane for the threader
    threader_.set_data_plane(*knowledge);

    // create a coverage sensor
    gams::variables::Sensors::iterator it = sensors->find("coverage");
    if (it == sensors->end()) // create coverage sensor
    {
      // get origin
      gams::pose::Position origin(gams::pose::gps_frame());
      madara::knowledge::containers::NativeDoubleArray origin_container;
      origin_container.set_name("sensor.coverage.origin", *knowledge, 3);
      origin.from_container(origin_container);

      // establish sensor
      gams::variables::Sensor *coverage_sensor =
          new gams::variables::Sensor("coverage", knowledge, 2.5, origin);
      (*sensors)["coverage"] = coverage_sensor;
    }
    (*sensors_)["coverage"] = (*sensors)["coverage"];
    status_.init_vars(*knowledge, get_id());

    // create threads
    // end create threads

    client_->confirmConnection();

    client_->enableApiControl(true, vehicle_name_);

    client_->armDisarm(true, vehicle_name_);
    client_->takeoffAsync(5, vehicle_name_);
    std::this_thread::sleep_for(std::chrono::duration<double>(5));
    client_->enableApiControl(true, vehicle_name_);

    status_.movement_available = 1;
  }
}

gams::platforms::AirLibQuadcopter::~AirLibQuadcopter()
{
  threader_.terminate();
  threader_.wait();
  delete client_;
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

int 
gams::platforms::AirLibQuadcopter::sense(void)
{
  auto location = client_->simGetVehiclePose(vehicle_name_).position;

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
                        gams::loggers::LOG_MAJOR,
                        "SIMBOTIC position: %f,%f,%f\n", location.x() / 100.0, location.y() / 100.0,  location.z() / 100.0);

  gams::pose::Position loc(get_frame(), location.x() / 100.0, location.y() / 100.0, 0);

  loc.to_container(self_->agent.location);
  self_->agent.orientation.set(0, 0.0);
  self_->agent.orientation.set(1, 0.0);
  self_->agent.orientation.set(2, 0.0);

  status_.waiting = 0;
  status_.moving = 1;

  return gams::platforms::PLATFORM_OK;
}

int
gams::platforms::AirLibQuadcopter::home(void)
{
  client_->goHomeAsync(Utils::max<float>(), vehicle_name_);
  return gams::platforms::PLATFORM_IN_PROGRESS;
}

int 
gams::platforms::AirLibQuadcopter::land(void)
{
  client_->landAsync(60, vehicle_name_);
  return gams::platforms::PLATFORM_IN_PROGRESS;
}

int 
gams::platforms::AirLibQuadcopter::move(
    const gams::pose::Position &location,
    const gams::platforms::PositionBounds &bounds)
{
  client_->enableApiControl(true, vehicle_name_);
  client_->moveToPositionAsync(location.x(), location.y(), -40.0, 10,
                               Utils::max<float>(), msr::airlib::DrivetrainType::MaxDegreeOfFreedom,
                               YawMode(), -1, 1, vehicle_name_);
  // client_->hoverAsync(vehicle_name_)->waitOnLastTask();
  return BasePlatform::move(location, bounds);
}

int 
gams::platforms::AirLibQuadcopter::rotate(
    const gams::pose::Orientation &target,
    double epsilon)
{
  gams::pose::euler::EulerXYZ e(target);
  client_->rotateToYawAsync(e.c(), Utils::max<float>(), 5, vehicle_name_);
  return gams::platforms::PLATFORM_MOVING;
}

int 
gams::platforms::AirLibQuadcopter::takeoff(void)
{
  client_->takeoffAsync(20, vehicle_name_)->waitOnLastTask();
  return gams::platforms::PLATFORM_OK;
}

const gams::pose::ReferenceFrame &
gams::platforms::AirLibQuadcopter::get_frame(void) const
{
  // For cartesian, replace with gams::pose::default_frame()
  return gams::pose::gps_frame();
}

#endif // _GAMS_AIRLIB_
