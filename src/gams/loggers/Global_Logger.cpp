#include "Global_Logger.h"

std::auto_ptr <Madara::Logger::Logger> gams::loggers::global_logger (
  new Madara::Logger::Logger ());
