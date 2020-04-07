 #ifdef _GAMS_OSC_

#include <algorithm>
#include <math.h>
#include <sstream>

#include "madara/utility/Utility.h"
#include "OscJoystickPlatform.h"

#ifndef __WIN32__

#include "gams/utility/LinuxJoystick.h"

#endif // !__WIN32__

namespace knowledge = madara::knowledge;
typedef knowledge::KnowledgeRecord  KnowledgeRecord;

class SenseThread : public madara::threads::BaseThread
{
  private:
    /// data plane if we want to access the knowledge base
    madara::knowledge::KnowledgeBase data_;

    /// mapping of xyz_velocity_ so platform::sense method can read it
    madara::knowledge::containers::NativeDoubleVector xyz_velocity_;

    /// check to see if user wishes y-axis to be inverted
    bool inverted_y_ = false;

    /// check to see if user wishes z-axis to be inverted
    bool inverted_z_ = false;

    /// check to see if user wishes to flip the x and y axis values
    bool flip_xy_ = false;

    /// handle to the joystick
    gams::utility::Joystick joystick_;

  public:
    /**
     * Default constructor
     **/
    SenseThread() {}
    
    /**
     * Destructor
     **/
    virtual ~SenseThread() {}
    
    /**
      * Initializes thread with MADARA context
      * @param   context   context for querying current program state
      **/
    virtual void init(madara::knowledge::KnowledgeBase & knowledge)
    {
      data_ = knowledge;
      xyz_velocity_.set_name(".xyz_velocity", data_, 3);

      std::string handle = knowledge.get(".osc.local.handle").to_string();

      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_ALWAYS,
        "SenseThread::init: " \
        "%s: attempting to open handle %s\n",
        knowledge.get(".prefix").to_string().c_str(),
        handle.c_str());

      if (handle != "")
      {
        bool result = joystick_.open_handle(handle);

        if (result)
        {
          madara_logger_ptr_log(gams::loggers::global_logger.get(),
            gams::loggers::LOG_ALWAYS,
            "SenseThread::init: " \
            "%s: SUCCESS: mapped to joystick %s\n",
            knowledge.get(".prefix").to_string().c_str(),
            handle.c_str());
        }
        else
        {
          madara_logger_ptr_log(gams::loggers::global_logger.get(),
            gams::loggers::LOG_ALWAYS,
            "SenseThread::init: " \
            "%s: FAIL: cannot map to joystick %s\n",
            knowledge.get(".prefix").to_string().c_str(),
            handle.c_str());
        }
        
      }

      inverted_y_ = knowledge.get(".osc.local.inverted_y").is_true();
      inverted_z_ = knowledge.get(".osc.local.inverted_z").is_true();
      flip_xy_ = knowledge.get(".osc.local.flip_xy").is_true();
    }

    /**
      * Executes the main thread logic
      **/
    virtual void run(void)
    {
#ifndef __WIN32__

      // read the joystick
      gams::utility::JoystickEvent event;
      
      joystick_.get(event);

      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_ALWAYS,
        "Event: type=%d, number=%d, value=%d.\n",
       (int)event.type,(int)event.number,
       (int)event.value);

      // if (!joystick_.is_init(event))
      // {
      if (joystick_.is_axis(event))
      {
        std::stringstream actions;

        if (joystick_.is_x_move(event))
        {
          actions << "X MOVE: " << joystick_.to_double(event) << ", ";
          xyz_velocity_.set(0, joystick_.to_double(event));
        }

        if (joystick_.is_y_move(event))
        {
          actions << "Y MOVE: " << joystick_.to_double(event) << ", ";

          if (inverted_y_)
          {
            xyz_velocity_.set(1, -joystick_.to_double(event));
          }
          else
          {
            xyz_velocity_.set(1, joystick_.to_double(event));
          }
        }

        if (joystick_.is_z_move(event))
        {
          actions << "Z MOVE: " << joystick_.to_double(event) << ", ";

          if (inverted_z_)
          {
            xyz_velocity_.set(2, -joystick_.to_double(event));
          }
          else
          {
            xyz_velocity_.set(2, joystick_.to_double(event));
          }
        }

        if (joystick_.is_rotate(event))
        {
          actions << "ROTATE: " << joystick_.to_double(event) << ", ";
        }

        madara_logger_ptr_log(gams::loggers::global_logger.get(),
          gams::loggers::LOG_ALWAYS,
          "AXIS: type=%d, number=%d, value=%d, double_value=%f, %s\n",
         (int)event.type,(int)event.number,
         (int)event.value, joystick_.to_double(event),
          actions.str().c_str());
      }
      else if (joystick_.is_button(event))
      {
        madara_logger_ptr_log(gams::loggers::global_logger.get(),
          gams::loggers::LOG_MAJOR,
          "BUTTON: type=%d, number=%d, value=%d\n",
         (int)event.type,(int)event.number,
         (int)event.value);
      }
      else
      {
        madara_logger_ptr_log(gams::loggers::global_logger.get(),
          gams::loggers::LOG_MAJOR,
          "UNKNOWN: type=%d, number=%d, value=%d\n",
         (int)event.type,(int)event.number,
         (int)event.value);
      }
        
#endif // __WIN32__
    }
};

gams::platforms::OscJoystickPlatformFactory::OscJoystickPlatformFactory(const std::string & type)
: type_(type)
{
  
}

gams::platforms::OscJoystickPlatformFactory::~OscJoystickPlatformFactory()
{
  
}

// factory class for creating a OscJoystickPlatform 
gams::platforms::BasePlatform *
gams::platforms::OscJoystickPlatformFactory::create(
        const madara::knowledge::KnowledgeMap &,
        madara::knowledge::KnowledgeBase * knowledge,
        gams::variables::Sensors * sensors,
        gams::variables::Platforms * ,
        gams::variables::Self * self)
{
  BasePlatform * result(0);

  if (type_ != "quadcopter" &&
      type_ != "satellite")
  {
    type_ = "quadcopter";
  }
  
  if (knowledge && sensors && self)
  {
    result = new OscJoystickPlatform(knowledge, sensors, self,
      type_);
  }

  return result;
}

// Constructor
gams::platforms::OscJoystickPlatform::OscJoystickPlatform(
  madara::knowledge::KnowledgeBase * knowledge,
  gams::variables::Sensors * sensors,
  gams::variables::Self * self,
  const std::string & type)        
: gams::platforms::BasePlatform(knowledge, sensors, self),
  type_(type), event_fd_("/dev/input/js0")
{
  // as an example of what to do here, create a coverage sensor
  if (knowledge && sensors)
  {
    xyz_velocity_.set_name(".xyz_velocity", *knowledge, 3);

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

    knowledge::KnowledgeRecord event_handle =
      knowledge->get(".osc.local.handle");

    if (event_handle)
    {
      event_fd_ = event_handle.to_string();
    }

    settings_.hosts.push_back(
      knowledge->get(".osc.local.endpoint").to_string());

    settings_.hosts.push_back(
      knowledge->get(".osc.server.endpoint").to_string());

    knowledge::KnowledgeRecord initial_pose = knowledge->get(".initial_pose");

    is_created_ = knowledge->get(".osc.is_created").is_true();

    if (!initial_pose.is_array_type())
    {
      // by default initialize agents to [.id, .id, .id]
      initial_pose.set_index(2, 200);
      initial_pose.set_index(1, *(self_->id) * 300);
      initial_pose.set_index(0, *(self_->id) * 300);
    }
    else
    {
      double value = initial_pose.retrieve_index(0).to_double() * 100;
      initial_pose.set_index(0, value);
      value = initial_pose.retrieve_index(1).to_double() * 100;
      initial_pose.set_index(1, value);
      value = initial_pose.retrieve_index(2).to_double() * 100;
      initial_pose.set_index(2, value);
    }
    
    loiter_timeout_ = knowledge->get(".osc.loiter_timeout").to_double();

    if (loiter_timeout_ >= 0 && loiter_timeout_ < 5)
    {
      loiter_timeout_ = 5;
    }


    settings_.type = (uint32_t)
      knowledge->get(".osc.transport.type").to_integer();
    if (settings_.type == 0)
    {
      settings_.type = madara::transport::UDP;
    }

    if (settings_.hosts[0] == "")
    {
      std::stringstream buffer;
      buffer << "127.0.0.1:";
      buffer <<(*(self_->id) + 8000);
      settings_.hosts[0] = buffer.str();
    }

    if (settings_.hosts[1] == "")
    {
      settings_.hosts[1] = "127.0.0.1:5555";
    }

    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
      "gams::platforms::OscJoystickPlatform::const: OSC settings:" \
      " %s: .osc.local.endpoint=%s, .osc.server.endpoint=%s,"
      " .osc.transport_type=%d\n",
      self_->agent.prefix.c_str(),
      settings_.hosts[0].c_str(),
      settings_.hosts[1].c_str(),
     (int)settings_.type);

    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
      "gams::platforms::OscJoystickPlatform::const: OSC prefixes:" \
      " %s: xy_vel=%s, xy_z=%s,"
      " pos=%s, rot=%s\n",
      self_->agent.prefix.c_str(),
      xy_velocity_prefix_.c_str(),
      z_velocity_prefix_.c_str(),
      position_prefix_.c_str(),
      rotation_prefix_.c_str());

    std::stringstream json_buffer;
    json_buffer << "{\"id\":";
    json_buffer << *(self_->id);
    json_buffer << ",\"port\":";
    json_buffer <<(*(self_->id) + 8000);
    json_buffer << ",\"location\":{";
    json_buffer << "\"x\": " <<
      initial_pose.retrieve_index(0).to_double() << ",";
    json_buffer << "\"y\": " <<
      initial_pose.retrieve_index(1).to_double() << ",";
    json_buffer << "\"z\": " <<
      initial_pose.retrieve_index(2).to_double();
    json_buffer << "},\"rotation\":{\n";
    json_buffer << "\"x\":" <<
      initial_pose.retrieve_index(3).to_double() << ",";
    json_buffer << "\"y\":" <<
      initial_pose.retrieve_index(4).to_double() << ",";
    json_buffer << "\"z\":" <<
      initial_pose.retrieve_index(5).to_double();
    json_buffer << "}}";
    

    json_creation_ = json_buffer.str();

    
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
      "gams::platforms::OscJoystickPlatform::const: creation JSON:" \
      " %s:\n%s\n",
      self_->agent.prefix.c_str(),
      json_creation_.c_str());

    osc_.create_socket(settings_);


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

    if (!is_created_)
    {
      utility::OscUdp::OscMap values;
      std::string address = "/spawn/" + type_;
      values[address] = KnowledgeRecord(json_creation_);

      osc_.send(values);
    }

    status_.movement_available = 1;

    inverted_y_ = knowledge->get(".osc.local.inverted_y").is_true();
    inverted_z_ = knowledge->get(".osc.local.inverted_z").is_true();
    flip_xy_ = knowledge->get(".osc.local.flip_xy").is_true();

    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
      "gams::platforms::OscJoystickPlatform::const: " \
      "%s: mapping to joystick %s\n",
      self_->agent.prefix.c_str(),
      event_fd_.c_str());

    threader_.set_data_plane(*knowledge);

    threader_.run(20, "ReadInput", new ::SenseThread());
  }
}


// Destructor
gams::platforms::OscJoystickPlatform::~OscJoystickPlatform()
{
  threader_.terminate();
  threader_.wait();
}

void
gams::platforms::OscJoystickPlatform::build_prefixes(void)
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
gams::platforms::OscJoystickPlatform::calculate_thrust(
  const pose::Position & current, const pose::Position & target,
  bool & finished)
{
  finished = true;
  std::vector<double> difference(std::min(current.size(), target.size()));
  
  last_thrust_timer_.start();

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MAJOR,
    "gams::platforms::OscJoystickPlatform::calculate_thrust: " \
    "%s: current=[%s], target=[%s]\n",
    self_->agent.prefix.c_str(),
    current.to_string().c_str(), target.to_string().c_str());

  for (size_t i = 0; i < difference.size(); ++i)
  {
    difference[i] = target.get(i) - current.get(i);

    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_TRACE,
      "gams::platforms::OscJoystickPlatform::calculate_thrust: " \
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
    else if (difference[i] <= 1.5 && difference[i] >= -1.5)
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
  
  madara::knowledge::KnowledgeRecord record(difference);

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MAJOR,
    "gams::platforms::OscJoystickPlatform::calculate_thrust: " \
    "%s: returning thrust of [%s]\n",
    self_->agent.prefix.c_str(),
    record.to_string().c_str());

  return difference;
}

// Polls the sensor environment for useful information. Required.
int
gams::platforms::OscJoystickPlatform::sense(void)
{
  utility::OscUdp::OscMap values;
  values[position_prefix_];
  values[rotation_prefix_];

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MINOR,
    "gams::platforms::OscJoystickPlatform::sense: " \
    "%s: entering receive on OSC UDP\n",
    self_->agent.prefix.c_str());

  osc_.receive(values);

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_MINOR,
    "gams::platforms::OscJoystickPlatform::sense: " \
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
        "gams::platforms::OscJoystickPlatform::sense: " \
        "%s: Processing %s => platform location with %d values\n",
        self_->agent.prefix.c_str(),
        value.first.c_str(),(int)value.second.size());

      
      // convert osc order to the frame order
      pose::Position loc(get_frame());
      loc.from_array(value.second.to_doubles());

      // The UnrealEngine provides us with centimeters. Convert to meters.
      loc.x(loc.x()/100);
      loc.y(loc.y()/100);
      loc.z(loc.z()/100);

      // save the location to the self container
      loc.to_container(self_->agent.location);

      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_MAJOR,
        "gams::platforms::OscJoystickPlatform::sense: " \
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
        "gams::platforms::OscJoystickPlatform::sense: " \
        " %s: Processing %s => platform orientation with %d values\n",
        self_->agent.prefix.c_str(),
        value.first.c_str(),(int)value.second.size());

      
      // convert osc order to the frame order
      pose::Orientation angles(get_frame());
      angles.from_array(value.second.to_doubles());

      // save the location to the self container
      angles.to_container(self_->agent.orientation);

      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_MAJOR,
        "gams::platforms::OscJoystickPlatform::sense: " \
        "%s: Platform orientation is [%s]\n",
        self_->agent.prefix.c_str(),
        self_->agent.orientation.to_record().to_string().c_str());

      is_created_ = true;
    }
    else
    {
      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_ALWAYS,
        "gams::platforms::OscJoystickPlatform::sense: " \
        "%s: Unable to map topic %s to platform info\n",
        self_->agent.prefix.c_str(),
        value.first.c_str());
    }
    
  }

  if (values.size() == 0)
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
      "gams::platforms::OscJoystickPlatform::sense: " \
      "%s: No received values from UnrealGAMS\n",
      self_->agent.prefix.c_str());
  }
  else
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
      "gams::platforms::OscJoystickPlatform::sense: " \
      "%s: finished processing updates from OSC\n",
      self_->agent.prefix.c_str());
  }

  // check for last position timeout(need to recreate agent in sim)
  last_position_timer_.stop();

  //if we've never received a server packet for this agent, recreate
  if (last_position_timer_.duration_ds() > 60)
  {
    if (!is_created_)
    {
      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_ALWAYS,
        "gams::platforms::OscJoystickPlatform::sense: " \
        "%s: haven't heard from simulator in %f seconds. Recreating agent.\n",
        self_->agent.prefix.c_str(),
        last_position_timer_.duration_ds());

      is_created_ = false;

      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_ALWAYS,
        "gams::platforms::OscJoystickPlatform::sense: recreating agent with JSON:" \
        " %s:\n%s\n",
        self_->agent.prefix.c_str(),
        json_creation_.c_str());

      last_position_timer_.start();

      utility::OscUdp::OscMap values;
      std::string address = "/spawn/" + type_;
      values[address] = KnowledgeRecord(json_creation_);

      osc_.send(values);
    }
  }

  {
    // update the velocities that the user input through joystick
    utility::OscUdp::OscMap values;
    std::vector<double> xy_velocity;
    std::vector<double> z_velocity;

    if (flip_xy_)
    {
      xy_velocity.push_back(xyz_velocity_[1]);
      xy_velocity.push_back(xyz_velocity_[0]);
    }
    else
    {
      xy_velocity.push_back(xyz_velocity_[0]);
      xy_velocity.push_back(xyz_velocity_[1]);
    }
    
    z_velocity.push_back(xyz_velocity_[2]);

    values[xy_velocity_prefix_] = xy_velocity;
    values[z_velocity_prefix_] = z_velocity;

    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_ALWAYS,
      "Sending xyz velocities [%f, %f, %f]\n",
      xyz_velocity_[0], xyz_velocity_[1], xyz_velocity_[2]);

    osc_.send(values);

  }

  return gams::platforms::PLATFORM_OK;
}


// Analyzes platform information. Required.
int
gams::platforms::OscJoystickPlatform::analyze(void)
{
  return gams::platforms::PLATFORM_OK;
}


// Gets the name of the platform. Required.
std::string
gams::platforms::OscJoystickPlatform::get_name(void) const
{
  return "OscJoystickPlatform";
}


// Gets the unique identifier of the platform.
std::string
gams::platforms::OscJoystickPlatform::get_id(void) const
{
  return "OscJoystickPlatform";
}


// Gets the position accuracy in meters. Optional.
double
gams::platforms::OscJoystickPlatform::get_accuracy(void) const
{
  return 0.5;
}


// Gets sensor radius. Optional.
double
gams::platforms::OscJoystickPlatform::get_min_sensor_range(void) const
{
  // should be in square meters
  return 1.0;
}

// Gets move speed. Optional.
double
gams::platforms::OscJoystickPlatform::get_move_speed(void) const
{
  // should be in meters/s
  return 1.0;
}

// Instructs the agent to return home. Optional.
int
gams::platforms::OscJoystickPlatform::home(void)
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
gams::platforms::OscJoystickPlatform::land(void)
{
  return gams::platforms::PLATFORM_OK;
}

int
gams::platforms::OscJoystickPlatform::move(const pose::Position & target,
  const pose::PositionBounds & bounds)
{
  // update variables
  BasePlatform::move(target, bounds);
  int result = PLATFORM_MOVING;

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_TRACE,
    "gams::platforms::OscJoystickPlatform::move:" \
    " %s: requested target \"%f,%f,%f\"\n",
    self_->agent.prefix.c_str(),
    target.x(), target.y(), target.z());

  // convert from input reference frame to vrep reference frame, if necessary
  pose::Position new_target(get_frame(), target);

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_TRACE,
    "gams::platforms::OscJoystickPlatform::move:" \
    " %s: target \"%f,%f,%f\"\n",
    self_->agent.prefix.c_str(),
    new_target.x(), new_target.y(), new_target.z());

  // are we moving to a new location? If so, start an acceleration timer
  if (!last_move_.approximately_equal(new_target, 0.1))
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
  double move_time = move_timer_.duration_ds();
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
      gams::loggers::LOG_MAJOR,
      "gams::platforms::OscJoystickPlatform::move:" \
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
      "gams::platforms::OscJoystickPlatform::move:" \
      " %s: ARRIVED at target \"%f,%f,%f\"\n",
      self_->agent.prefix.c_str(),
      new_target.x(), new_target.y(), new_target.z());

    result = PLATFORM_ARRIVED;
  }
  
  values[xy_velocity_prefix_] = xy_velocity;
  values[z_velocity_prefix_] = z_velocity;

  osc_.send(values);

  // unlike other platforms, we do not send the velocities
  // Moving to the location is up to the user with the controller

  return result;
}

int
gams::platforms::OscJoystickPlatform::orient(const pose::Orientation & target,
        const pose::OrientationBounds &bounds)
{
  // update variables
  BasePlatform::orient(target, bounds);

  madara_logger_ptr_log(gams::loggers::global_logger.get(),
    gams::loggers::LOG_TRACE,
    "gams::platforms::OscJoystickPlatform::orient:" \
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
gams::platforms::OscJoystickPlatform::pause_move(void)
{
}


// Set move speed. Optional.
void
gams::platforms::OscJoystickPlatform::set_move_speed(const double& )
{
}


// Stops movement, resetting source and dest to current location. Optional.
void
gams::platforms::OscJoystickPlatform::stop_move(void)
{
}

// Instructs the agent to take off. Optional.
int
gams::platforms::OscJoystickPlatform::takeoff(void)
{
  return gams::platforms::PLATFORM_OK;
}

const gams::pose::ReferenceFrame &
gams::platforms::OscJoystickPlatform::get_frame(void) const
{
  // For cartesian, replace with gams::pose::default_frame()
  return gams::pose::default_frame();
}

#endif //  #ifdef _GAMS_OSC_