#include "GlobalLogger.h"

std::auto_ptr <madara::logger::Logger> gams::loggers::global_logger (
  new madara::logger::Logger ());
