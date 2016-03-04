
#ifndef _GAMS_LOGGERS_GLOBAL_LOGGER_H_
#define _GAMS_LOGGERS_GLOBAL_LOGGER_H_

#include <memory>
#include "gams/GAMSExport.h"
#include "madara/logger/Logger.h"
#include "madara/utility/Refcounter.h"

namespace gams
{
  namespace loggers
  {
    /**
    * Logging levels available for the GAMS library
    **/
    enum LogLevels
    {
      LOG_EMERGENCY = 0,
      LOG_ALWAYS = 0,
      LOG_ERROR = 1,
      LOG_WARNING = 2,
      LOG_MAJOR = 3,
      LOG_MINOR = 4,
      LOG_TRACE = 5,
      LOG_DETAILED = 6,
      LOG_MAX = 6
    };

    extern GAMSExport madara::utility::Refcounter <madara::logger::Logger>
      global_logger;
  }
}

#endif // _GAMS_LOGGERS_GLOBAL_LOGGER_H_