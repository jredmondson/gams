#include <algorithm>
#include <math.h>
#include <sstream>

#include "madara/knowledge/containers/NativeDoubleVector.h"
#include "madara/utility/Utility.h"
#include "OscPlatform.h"

namespace knowledge = madara::knowledge;
typedef knowledge::KnowledgeRecord  KnowledgeRecord;

gams::platforms::OscPlatformFactory::~OscPlatformFactory()
{
  
}

// factory class for creating a OscPlatform 
gams::platforms::BasePlatform *
gams::platforms::OscPlatformFactory::create(
        const madara::knowledge::KnowledgeMap &,
        madara::knowledge::KnowledgeBase * knowledge,
        gams::variables::Sensors * sensors,
        gams::variables::Platforms * ,
        gams::variables::Self * self)
{
  BasePlatform * result (0);
  
  if (knowledge && sensors && self)
  {
    result = new OscPlatform (knowledge, sensors, self);
  }

  return result;
}

// Constructor
gams::platforms::OscPlatform::OscPlatform(
  madara::knowledge::KnowledgeBase * knowledge,
  gams::variables::Sensors * sensors,
  gams::variables::Self * self)
: gams::platforms::BasePlatform(knowledge, sensors, self)
{
  // as an example of what to do here, create a coverage sensor
  if (knowledge && sensors)
  {
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
      gams::variables::Sensor* coverage_sensor =
        new gams::variables::Sensor("coverage", knowledge, 2.5, origin);
     (*sensors)["coverage"] = coverage_sensor;
    }
   (*sensors_)["coverage"] =(*sensors)["coverage"];
    status_.init_vars(*knowledge, get_id());
    
    build_prefixes();

    madara::transport::QoSTransportSettings settings_;

    settings_.hosts.push_back(
      knowledge->get(".osc.local.endpoint").to_string());
    settings_.hosts.push_back(
      knowledge->get(".osc.server.endpoint").to_string());

    knowledge::KnowledgeRecord initial_pose = knowledge->get(".initial_pose");

    if (!initial_pose.is_array_type())
    {
      // by default initialize agents to [.id, .id, .id]
      initial_pose.set_index(2, *(self_->id));
      initial_pose.set_index(1, *(self_->id));
      initial_pose.set_index(0, *(self_->id));
    }

    settings_.type =
      knowledge->get(".osc.transport.type").to_integer();
    if (settings_.type == 0)
    {
      settings_.type = madara::transport::UDP;
    }

    if (settings_.hosts[0] == "")
    {
      std::stringstream buffer;
      buffer << "127.0.0.1:";
      buffer << (*(self_->id) + 8000);
      settings_.hosts[0] = buffer.str();
    }

    if (settings_.hosts[1] == "")
    {
      settings_.hosts[1] = "127.0.0.1:5555";
    }

    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
      "gams::platforms::OscPlatform::const: OSC settings:" \
      " %s: .osc.local.endpoint=%s, .osc.server.endpoint=%s,"
      " .osc.transport_type=%d\n",
      self_->agent.prefix.c_str(),
      settings_.hosts[0].c_str(),
      settings_.hosts[1].c_str(),
      (int)settings_.type);

    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
      "gams::platforms::OscPlatform::const: OSC prefixes:" \
      " %s: xy_vel=%s, xy_z=%s,"
      " pos=%s, rot=%s\n",
      self_->agent.prefix.c_str(),
      xy_velocity_prefix_.c_str(),
      z_velocity_prefix_.c_str(),
      position_prefix_.c_str(),
      rotation_prefix_.c_str());

    std::stringstream json_buffer;
    json_buffer << "{\n  \"id\": ";
    json_buffer << *(self_->id);
    json_buffer << ",\n  \"port\": ";
    json_buffer << (*(self_->id) + 8000);
    json_buffer << ",\n  \"location\": {\n";
    json_buffer << "    \"x\": " <<
      initial_pose.retrieve_index (0).to_double() << ", ";
    json_buffer << "\"y\": " <<
      initial_pose.retrieve_index (1).to_double() << ", ";
    json_buffer << "\"z\": " <<
      initial_pose.retrieve_index (2).to_double() << "\n";
    json_buffer << "  },\n  \"rotation\": {\n";
    json_buffer << "    \"x\": " <<
      initial_pose.retrieve_index (3).to_double() << ", ";
    json_buffer << "\"y\": " <<
      initial_pose.retrieve_index (4).to_double() << ", ";
    json_buffer << "\"z\": " <<
      initial_pose.retrieve_index (5).to_double() << "\n";
    json_buffer << "  }\n}";
    

    json_creation_ = json_buffer.str();

    
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
      "gams::platforms::OscPlatform::const: creating agent with JSON:" \
      " %s:\n%s\n",
      self_->agent.prefix.c_str(),
      json_creation_.c_str());

    osc_.create_socket(settings_);

    utility::OscUdp::OscMap values;
    values["/spawn/quadcopter"] = KnowledgeRecord(json_creation_);

    osc_.send(values);

    last_move_.frame(get_frame());
    last_move_.x(0);
    last_move_.y(0);
    last_move_.z(0);

    last_orient_.frame(get_frame());
    last_orient_.rx(0);
    last_orient_.ry(0);
    last_orient_.rz(0);

    move_timer_.start();
    last_thrust_timer_.start();
    last_position_timer_.start();

    status_.movement_available = 1;
  }
}


// Destructor
gams::platforms::OscPlatform::~OscPlatform()
{
}

void
gams::platforms::OscPlatform::build_prefixes(void)
{
  if (xy_velocity_prefix_ == "" 
    && madara::utility::begins_with(self_->agent.prefix, "agent."))
  {
    // build the common prefix
    std::string common_prefix = "/";
    common_prefix += self_->agent.prefix;
    common_prefix[6] = '/';

    xy_velocity_prefix_ = common_prefix;
    z_velocity_prefix_ = common_prefix;
    yaw_velocity_prefix_ = common_prefix;
    position_prefix_ = common_prefix;
    rotation_prefix_ = common_prefix;

    // build the specialized prefixes
    xy_velocity_prefix_ += "/velocity/xy";
    z_velocity_prefix_ += "/velocity/z";
    yaw_velocity_prefix_ += "/velocity/yaw";
    position_prefix_ += "/pos";
    rotation_prefix_ += "/rot";
  }
}

std::vector<double>
gams::platforms::OscPlatform::calculate_thrust(
  const pose::Position & current, const pose::Position & target,
  bool & finished)
{
  finished = true;
  std::vector<double> difference (std::min(current.size(), target.size()));
  
  last_thrust_timer_.start();

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MAJOR,
    "gams::platforms::OscPlatform::calculate_thrust: " \
    "%s: current=[%s], target=[%s]\n",
    self_->agent.prefix.c_str(),
    current.to_string().c_str(), target.to_string().c_str());

  for (size_t i = 0; i < difference.size(); ++i)
  {
    difference[i] = target.get(i) - current.get(i);

    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_TRACE,
      "gams::platforms::OscPlatform::calculate_thrust: " \
      "%s: difference[%zu]=[%f]\n",
      self_->agent.prefix.c_str(), i, difference[i]);

    if (difference[i] <= 0.3 && difference[i] >= -0.3)
    {
      difference[i] = 0;
    }
    else if (difference[i] <= 0.5 && difference[i] >= -0.5)
    {
      difference[i] /= fabs(difference[i]);
      difference[i] *= 0.25;
      finished ? finished = false : 0;
    }
    else if (difference[i] <= 1.5 && difference[i] >= -1,5)
    {
      difference[i] /= fabs(difference[i]);
      difference[i] *= 0.5;
      finished ? finished = false : 0;
    }
    else
    {
      difference[i] /= fabs(difference[i]);
      finished ? finished = false : 0;
    }
  }
  
  madara::knowledge::KnowledgeRecord record (difference);

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MAJOR,
    "gams::platforms::OscPlatform::calculate_thrust: " \
    "%s: returning thrust of [%s]\n",
    self_->agent.prefix.c_str(),
    record.to_string().c_str());

  return difference;
}

// Polls the sensor environment for useful information. Required.
int
gams::platforms::OscPlatform::sense(void)
{
  utility::OscUdp::OscMap values;
  values[position_prefix_];
  values[rotation_prefix_];

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MINOR,
    "gams::platforms::OscPlatform::sense: " \
    "%s: entering receive on OSC UDP\n",
    self_->agent.prefix.c_str());

  osc_.receive(values);

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MINOR,
    "gams::platforms::OscPlatform::sense: " \
    "%s: leaving receive on OSC UDP with %zu values\n",
    self_->agent.prefix.c_str(), values.size());
  
  if (values[position_prefix_].size() != 3)
  {
    values.erase(position_prefix_);
  }

  if (values[rotation_prefix_].size() != 3)
  {
    values.erase(rotation_prefix_);
  }

  for (auto value: values)
  {
    if (value.first == position_prefix_)
    {
      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_MINOR,
        "gams::platforms::OscPlatform::sense: " \
        "%s: Processing %s => platform location with %d values\n",
        self_->agent.prefix.c_str(),
        value.first.c_str(), (int)value.second.size());

      
      // convert osc order to the frame order
      pose::Position loc(get_frame());
      loc.from_array(value.second.to_doubles());

      // The UnrealEngine provides us with centimeters. Convert to meters.
      loc.x(loc.x()/100);
      loc.y(loc.y()/100);
      loc.z(loc.z()/100);

      // save the location to the self container
      loc.to_container (self_->agent.location);

      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_MAJOR,
        "gams::platforms::OscPlatform::sense: " \
        "%s: Platform location is [%s]\n",
        self_->agent.prefix.c_str(),
        self_->agent.location.to_record().to_string().c_str());

      is_created_ = true;
      last_position_timer_.start();
    }
    else if (value.first == rotation_prefix_)
    {
      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_MINOR,
        "gams::platforms::OscPlatform::sense: " \
        " %s: Processing %s => platform orientation with %d values\n",
        self_->agent.prefix.c_str(),
        value.first.c_str(), (int)value.second.size());

      
      // convert osc order to the frame order
      pose::Orientation angles(get_frame());
      angles.from_array(value.second.to_doubles());

      // save the location to the self container
      angles.to_container (self_->agent.orientation);

      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_MAJOR,
        "gams::platforms::OscPlatform::sense: " \
        "%s: Platform orientation is [%s]\n",
        self_->agent.prefix.c_str(),
        self_->agent.orientation.to_record().to_string().c_str());

      is_created_ = true;
    }
    else
    {
      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_ALWAYS,
        "gams::platforms::OscPlatform::sense: " \
        "%s: Unable to map topic %s to platform info\n",
        self_->agent.prefix.c_str(),
        value.first.c_str());
    }
    
  }

  if (values.size() == 0)
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
      "gams::platforms::OscPlatform::sense: " \
      "%s: No received values from UnrealGAMS\n",
      self_->agent.prefix.c_str());

    // if we've never received a server packet for this agent, recreate
    if (!is_created_)
    {
      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_MAJOR,
        "gams::platforms::OscPlatform::sense: recreating agent with JSON:" \
        " %s:\n%s\n",
        self_->agent.prefix.c_str(),
        json_creation_.c_str());

      utility::OscUdp::OscMap values;
      values["/spawn/quadcopter"] = KnowledgeRecord(json_creation_);

      osc_.send(values);
    }
  }
  else
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
      "gams::platforms::OscPlatform::sense: " \
      "%s: finished processing updates from OSC\n",
      self_->agent.prefix.c_str());
  }

  // check for loiter timeout (have we thrusted recently?)
  last_thrust_timer_.stop();
  if (last_thrust_timer_.duration_ds() > 1)
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_ALWAYS,
      "gams::platforms::OscPlatform::sense: " \
      "%s: loiter timeout. Sending velocities [0,0,0]\n",
      self_->agent.prefix.c_str());

    // send 0 velocities to loiter

    values.clear();
    KnowledgeRecord record;

    record.set_index(1, 0.0);
    record.set_index(0, 0.0);
    values[xy_velocity_prefix_] = record;

    record.resize(1);
    values[z_velocity_prefix_] = record;
    osc_.send(values);

    // restart the timer
    last_thrust_timer_.start();
  }

  // check for last position timeout (need to recreate agent in sim)
  last_position_timer_.stop();
  if (last_position_timer_.duration_ds() > 2)
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_ALWAYS,
      "gams::platforms::OscPlatform::sense: " \
      "%s: haven't heard from simulator in %f seconds. Creating agent.\n",
      self_->agent.prefix.c_str(),
      last_position_timer_.duration_ds());

    is_created_ = false;
    last_position_timer_.start();
  }

  return gams::platforms::PLATFORM_OK;
}


// Analyzes platform information. Required.
int
gams::platforms::OscPlatform::analyze(void)
{
  return gams::platforms::PLATFORM_OK;
}


// Gets the name of the platform. Required.
std::string
gams::platforms::OscPlatform::get_name(void) const
{
  return "OscPlatform";
}


// Gets the unique identifier of the platform.
std::string
gams::platforms::OscPlatform::get_id(void) const
{
  return "OscPlatform";
}


// Gets the position accuracy in meters. Optional.
double
gams::platforms::OscPlatform::get_accuracy(void) const
{
  return 0.5;
}


// Gets sensor radius. Optional.
double
gams::platforms::OscPlatform::get_min_sensor_range(void) const
{
  // should be in square meters
  return 1.0;
}

// Gets move speed. Optional.
double
gams::platforms::OscPlatform::get_move_speed(void) const
{
  // should be in meters/s
  return 1.0;
}

// Instructs the agent to return home. Optional.
int
gams::platforms::OscPlatform::home(void)
{
  /**
   * Movement functions work in a polling manner. In a real implementation,
   * we would want to check a finite state machine, external thread or
   * platform status to determine what to return. For now, we will simply
   * return that we are in the process of moving to the final pose.
   **/
  return gams::platforms::PLATFORM_IN_PROGRESS;
}


// Instructs the agent to take off. Optional.
int
gams::platforms::OscPlatform::land(void)
{
  return gams::platforms::PLATFORM_OK;
}

int
gams::platforms::OscPlatform::move(const pose::Position & target,
  const PositionBounds & bounds)
{
  // update variables
  BasePlatform::move(target, bounds);
  int result = PLATFORM_MOVING;

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_TRACE,
    "gams::platforms::OscPlatform::move:" \
    " %s: requested target \"%f,%f,%f\"\n",
    self_->agent.prefix.c_str(),
    target.x(), target.y(), target.z());

  // convert from input reference frame to vrep reference frame, if necessary
  pose::Position new_target(get_frame(), target);

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_TRACE,
    "gams::platforms::OscPlatform::move:" \
    " %s: target \"%f,%f,%f\"\n",
    self_->agent.prefix.c_str(),
    new_target.x(), new_target.y(), new_target.z());

  // are we moving to a new location? If so, start an acceleration timer
  if (!last_move_.approximately_equal (new_target, 0.1))
  {
    move_timer_.start();
    last_move_ = new_target;
  }

  pose::Position cur_loc = get_location();

  utility::OscUdp::OscMap values;
  std::vector<double> xy_velocity;
  std::vector<double> z_velocity;

  // if (bounds.check_position(cur_loc, new_target))
  // quick hack to check position is within 0.5 meters

  bool finished = false;
  xy_velocity = calculate_thrust(cur_loc, new_target, finished);

  // if we have just started our movement, modify velocity to account
  // for acceleration. This will help animation be smooth in Unreal.
  move_timer_.stop();
  double move_time = move_timer_.duration_ds ();
  if (move_time < 1.0)
  {
    // have acceleration ramp up over a second at .1 per 1/20 second
    for (size_t i = 0; i < xy_velocity.size(); ++i)
    {
      double abs_velocity = fabs(xy_velocity[i]);
      double signed_move_time = xy_velocity[i] < 0 ? -move_time : move_time;
      xy_velocity[i] = move_time < abs_velocity ?
        signed_move_time : xy_velocity[i];
    }

    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_ALWAYS,
      "gams::platforms::OscPlatform::move:" \
      " %s: moving to new target at time %f with modified velocity "
      "[%f,%f,%f]\n",
      self_->agent.prefix.c_str(),
      move_time,
      xy_velocity[0], xy_velocity[1], xy_velocity[2]);

  }

  z_velocity.push_back(xy_velocity[2]);
  xy_velocity.resize(2);

  if (finished)
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MINOR,
      "gams::platforms::OscPlatform::move:" \
      " %s: ARRIVED at target \"%f,%f,%f\"\n",
      self_->agent.prefix.c_str(),
      new_target.x(), new_target.y(), new_target.z());

    result = PLATFORM_ARRIVED;
  }
  
  values[xy_velocity_prefix_] = xy_velocity;
  values[z_velocity_prefix_] = z_velocity;

  osc_.send(values);

  return result;
}

int
gams::platforms::OscPlatform::orient(const pose::Orientation & target,
        const OrientationBounds &bounds)
{
  // update variables
  BasePlatform::orient(target, bounds);

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_TRACE,
    "gams::platforms::OscPlatform::orient:" \
    " %s: requested target \"%f,%f,%f\"\n",
    self_->agent.prefix.c_str(), target.rx(), target.ry(), target.rz());

  // convert from input reference frame to vrep reference frame, if necessary
  pose::Orientation new_target(get_frame(), target);

  last_orient_ = new_target;

  utility::OscUdp::OscMap values;
  std::vector<double> yaw_velocity;
  yaw_velocity.push_back(0);
  values[yaw_velocity_prefix_] = yaw_velocity;
  osc_.send(values);

  // we're not changing orientation. this has to be done for move alg to work
  return PLATFORM_ARRIVED;
}

// Pauses movement, keeps source and dest at current values. Optional.
void
gams::platforms::OscPlatform::pause_move(void)
{
}


// Set move speed. Optional.
void
gams::platforms::OscPlatform::set_move_speed(const double& )
{
}


// Stops movement, resetting source and dest to current location. Optional.
void
gams::platforms::OscPlatform::stop_move(void)
{
}

// Instructs the agent to take off. Optional.
int
gams::platforms::OscPlatform::takeoff(void)
{
  return gams::platforms::PLATFORM_OK;
}

const gams::pose::ReferenceFrame &
gams::platforms::OscPlatform::get_frame(void) const
{
  // For cartesian, replace with gams::pose::default_frame()
  return gams::pose::default_frame();
}
