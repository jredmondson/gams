 
#include "Modify.h"

#include <iostream>

std::vector <double> static_location;
std::vector <double> static_orientation;

gams::algorithms::BaseAlgorithm *
algorithms::ModifyFactory::create (
  const madara::knowledge::KnowledgeMap & args,
  madara::knowledge::KnowledgeBase * knowledge,
  gams::platforms::BasePlatform * platform,
  gams::variables::Sensors * sensors,
  gams::variables::Self * self,
  gams::variables::Agents * agents)
{
  gams::algorithms::BaseAlgorithm * result (0);
  madara::knowledge::KnowledgeRecord::Integer payload_size (0);
  madara::knowledge::KnowledgeMap::const_iterator payload_found =
    args.find ("payload");

  if (payload_found != args.end ())
  {
    payload_size = payload_found->second.to_integer ();
  }

  static_location.resize (3);
  static_location[0] = 42.0;
  static_location[1] = 70.0;
  static_location[2] = 1000.0;

  static_orientation.resize (3, 0.0);

  if (knowledge && sensors && platform && self)
  {
    result = new Modify (knowledge, platform, sensors, self, agents,
      (int)payload_size);
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "algorithms::ModifyFactory::create:" 
      " failed to create due to invalid pointers. " 
      " knowledge=%p, sensors=%p, platform=%p, self=%p, agents=%p\n",
      knowledge, sensors, platform, self, agents);
  }

  /**
   * Note the usage of logger macros with the GAMS global logger. This
   * is highly optimized and is just an integer check if the log level is
   * not high enough to print the message
   **/
  if (result == 0)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "algorithms::ModifyFactory::create:" 
      " unknown error creating Modify algorithm\n");
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "algorithms::ModifyFactory::create:" 
      " successfully created Modify algorithm\n");
  }

  return result;
}

algorithms::Modify::Modify (
  madara::knowledge::KnowledgeBase * knowledge,
  gams::platforms::BasePlatform * platform,
  gams::variables::Sensors * sensors,
  gams::variables::Self * self,
  gams::variables::Agents * agents,
  int payload_size)
  : gams::algorithms::BaseAlgorithm (knowledge, platform, sensors, self, agents)
{
  status_.init_vars (*knowledge, "modify", self->id.to_integer ());
  status_.init_variable_values ();

  if (knowledge && self)
  {
    // setup payload
    if (payload_size > 0)
    {
      unsigned char * buffer = (unsigned char *)malloc ((size_t)payload_size);
      payload_ = knowledge->get_ref (self->prefix.to_string () + ".payload");
      knowledge->set_file (payload_, buffer, (size_t)payload_size);
    }
    payload_size_ = payload_size;

    kb_executions_.set_name (".executions", *knowledge);

    // setup initiale location and orientation
    self->agent.location.set (static_location);
    self->agent.home.set (static_location);
    self->agent.source.set (static_location);
    self->agent.dest.set (static_location);

    self->agent.orientation.set (static_orientation);
    self->agent.source_orientation.set (static_orientation);
    self->agent.dest_orientation.set (static_orientation);
  }
}

algorithms::Modify::~Modify ()
{
}

int
algorithms::Modify::analyze (void)
{
  return 0;
}
      

int
algorithms::Modify::execute (void)
{
  if (self_)
  {
    // modify the variable to resend them
    self_->agent.location.modify ();
    self_->agent.home.modify ();
    self_->agent.source.modify ();
    self_->agent.dest.modify ();

    self_->agent.orientation.modify ();
    self_->agent.source_orientation.modify ();
    self_->agent.dest_orientation.modify ();

    if (knowledge_ && payload_size_ > 0)
    {
      knowledge_->mark_modified (payload_);
    }
  }

  ++kb_executions_;

  return 0;
}


int
algorithms::Modify::plan (void)
{
  return 0;
}
