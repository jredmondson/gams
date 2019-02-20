#include <algorithm>
#include <math.h>

#include "madara/knowledge/containers/NativeDoubleVector.h"
#include "madara/utility/Utility.h"
#include "OscPlatform.h"

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
      gams::loggers::LOG_ALWAYS,
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

    osc_.create_socket(settings_);

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
    xy_velocity_prefix_ = "/";
    xy_velocity_prefix_ += self_->agent.prefix;
    xy_velocity_prefix_[6] = '/';

    // build the common prefix
    z_velocity_prefix_ = xy_velocity_prefix_;
    position_prefix_ = xy_velocity_prefix_;
    rotation_prefix_ = xy_velocity_prefix_;

    // build the specialized prefixes
    xy_velocity_prefix_ += "/velocity/xy";
    z_velocity_prefix_ += "/velocity/z";
    position_prefix_ += "/pos";
    rotation_prefix_ += "/rot";
  }
}

std::vector<double>
gams::platforms::OscPlatform::calculate_thrust(
  const pose::Position & current, const pose::Position & target, int)
{
  std::vector<double> difference (std::min(current.size(), target.size()));
  
  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_ALWAYS,
    "gams::platforms::OscPlatform::calculate_thrust: " \
    "%s: current=[%s], target=[%s]\n",
    self_->agent.prefix.c_str(),
    current.to_string().c_str(), target.to_string().c_str());

  for (size_t i = 0; i < difference.size(); ++i)
  {
    difference[i] = current.get(i) - target.get(i);

    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_TRACE,
      "gams::platforms::OscPlatform::calculate_thrust: " \
      "%s: difference[%zu]=[%f]\n",
      self_->agent.prefix.c_str(), i, difference[i]);

    if (difference[i] <= 1.0 && difference[i] >= -1.0)
    {
      difference[i] = 0;
    }
    else
    {
      difference[i] /= fabs(difference[i]);
    }
    
    // else if (difference[i] >= 50 || difference[i] <= -50)
    // {
    //   difference[i] /= fabs(difference[i]);
    // }
    // else if (difference[i] >= 20 || difference[i] <= -20)
    // {
    //   difference[i] /= fabs(difference[i]);
    //   difference[i] *= 2.0/3;
    // }
    // else if (difference[i] >= 10 || difference[i] <= -10)
    // {
    //   difference[i] /= fabs(difference[i]);
    //   difference[i] *= 1.0/3;
    // }
    // else if (difference[i] >= 1 || difference[i] <= -1)
    // {
    //   difference[i] /= fabs(difference[i]);
    //   difference[i] *= 1.0/6;
    // }
  }
  
  madara::knowledge::KnowledgeRecord record (difference);

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_ALWAYS,
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
      loc.from_array(value.second);

      // save the location to the self container
      loc.to_container (self_->agent.location);

      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_ALWAYS,
        "gams::platforms::OscPlatform::sense: " \
        "%s: Platform location is [%s]\n",
        self_->agent.prefix.c_str(),
        self_->agent.location.to_record().to_string().c_str());

    }
    if (value.first == rotation_prefix_)
    {
      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_MINOR,
        "gams::platforms::OscPlatform::sense: " \
        " %s: Processing %s => platform orientation with %d values\n",
        self_->agent.prefix.c_str(),
        value.first.c_str(), (int)value.second.size());

      
      // convert osc order to the frame order
      pose::Orientation angles(get_frame());
      angles.from_array(value.second);

      // save the location to the self container
      angles.to_container (self_->agent.orientation);

      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_ALWAYS,
        "gams::platforms::OscPlatform::sense: " \
        "%s: Platform orientation is [%s]\n",
        self_->agent.prefix.c_str(),
        self_->agent.orientation.to_record().to_string().c_str());

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
  }
  else
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
      "gams::platforms::OscPlatform::sense: " \
      "%s: finished processing updates from OSC\n",
      self_->agent.prefix.c_str());
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
  return 1;
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
  const PositionBounds &bounds)
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

  pose::Position cur_loc = get_location();

  utility::OscUdp::OscMap values;
  std::vector<double> xy_velocity;
  std::vector<double> z_velocity;

  if (bounds.check_position(cur_loc, new_target))
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MINOR,
      "gams::platforms::OscPlatform::move:" \
      " %s: ARRIVED at target \"%f,%f,%f\"\n",
      self_->agent.prefix.c_str(),
      new_target.x(), new_target.y(), new_target.z());

    xy_velocity.push_back(0);
    xy_velocity.push_back(0);
    z_velocity.push_back(0);

    result = PLATFORM_ARRIVED;
  }
  else
  {
    xy_velocity = calculate_thrust(cur_loc, new_target, 0);

    z_velocity.push_back(xy_velocity[2]);
    xy_velocity.resize(2);
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

  return 1;
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
