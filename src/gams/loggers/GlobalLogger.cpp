#include "GlobalLogger.h"

madara::utility::Refcounter <madara::logger::Logger>
  gams::loggers::global_logger (new madara::logger::Logger ());
