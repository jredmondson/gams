/**
 * @file Log_Macros.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * Macros used for logging in GAMS, based on the DAnCE Log Macros
 * maintained by Will Otte <wotte@dre.vanderbilt.edu>.
 * 
 */


#ifndef _GAMS_UTILITY_LOGGING_H_
#define _GAMS_UTILITY_LOGGING_H_

#include "gams/GAMS_Export.h"
#include "ace/Log_Msg.h"

#if !defined (DLINFO)
# define DLINFO ACE_TEXT("(%P|%t) [%M] - %T - ")
#endif

extern GAMS_Export int GAMS_debug_level;

// Defines for logging levels

/// Used for errors that cause the fatal shutdown of any portion
/// of the infrastructure.
#define GAMS_LOG_EMERGENCY            0
/// Used for GAMS errors at the point the error
/// exits the process in question, or when a decision is made to
/// cause the GAMS suite to fail.
#define GAMS_LOG_TERMINAL_ERROR       1
/// Used for non-fatal deployment errors that do not cause deployment
/// failure.
#define GAMS_LOG_NONFATAL_ERROR       2
/// Used to log detailed error information at the point of failure
#define GAMS_LOG_ERROR                3
/// used to indicate that a questionable situation that doesn't cause
/// deployment failure, but can cause undefined conditions.
#define GAMS_LOG_WARNING              4
/// Used to indicate that a ``significant'' GAMS event has completed.
#define GAMS_LOG_MAJOR_EVENT          5
/// Used to inficate a ``minor'' GAMS event has completed.
#define GAMS_LOG_MINOR_EVENT          6
/// Used to trace significant actions within major/minor events.  This
/// will usually include starts for major/minor events.
#define GAMS_LOG_EVENT_TRACE          7
/// Used to display important configuration information that impacts
/// major GAMS events.
#define GAMS_LOG_MAJOR_DEBUG_INFO     8
/// used to display minor debug info
#define GAMS_LOG_MINOR_DEBUG_INFO     9
/// used to display extremely fine-grained trace information
#define GAMS_LOG_DETAILED_TRACE      10

namespace gams
{
  namespace utility
  {
    /**
     * Logging levels available for GAMS library
     **/
    enum Log_Levels
    {
      LOG_EMERGENCY = 0,
      LOG_TERMINAL_ERROR = 1,
      LOG_NONFATAL_ERROR = 2,
      LOG_ERROR = 3,
      LOG_WARNING = 4,
      LOG_MAJOR_EVENT = 5,
      LOG_MINOR_EVENT = 6,
      LOG_EVENT_TRACE = 7,
      LOG_MAJOR_DEBUG_INFO = 8,
      LOG_MINOR_DEBUG_INFO = 9,
      LOG_DETAILED_TRACE = 10
    };

    /**
     * Sets the log level
     * @param level  the log level to use @see Log_Levels
     **/
    inline void set_log_level (int level)
    {
      ::GAMS_debug_level = level;
    }

    /**
     * Gets the log level
     * @return lthe log level being used @see Log_Levels
     **/
    inline int get_log_level (void)
    {
      return ::GAMS_debug_level;
    }
  }
}

// By default tracing is turned off.
#if !defined (GAMS_NTRACE)
#  define GAMS_NTRACE 1
#endif /* GAMS_NTRACE */

#if (GAMS_NTRACE == 1)
#  if !defined (ACE_NTRACE)
#    define GAMS_TRACE(X) do {} while (0)
#    define GAMS_ENABLE_TRACE() do {} while (0)
#    define GAMS_DISABLE_TRACE() do {} while (0)
#  else
#    if (ACE_NTRACE == 0)
#      error GAMS_TRACE cannot be disabled if ACE_TRACE is enabled
#    else
#      define GAMS_TRACE(X) do {} while (0)
#      define GAMS_ENABLE_TRACE() do {} while (0)
#      define GAMS_DISABLE_TRACE() do {} while (0)
#    endif
#  endif
#else
#  if !defined (ACE_HAS_TRACE)
#    define ACE_HAS_TRACE
#  endif /* ACE_HAS_TRACE */
#  define GAMS_TRACE(X) ACE_TRACE_IMPL (X)
#  define GAMS_ENABLE_TRACE() ACE_Trace::start_tracing ()
#  define GAMS_DISABLE_TRACE() ACE_Trace::stop_tracing ()
#  undef DLINFO // Make log messages indent with tracing.
#  define DLINFO ACE_TEXT("%I(%P|%t) [%M] - %T - ")
#  include "ace/Trace.h"
#endif /* GAMS_NTRACE */

#if defined (GAMS_NLOGGING)
# define GAMS_ERROR(L, X) do {} while (0)
# define GAMS_DEBUG(L, X) do {} while (0)
#define GAMS_ERROR_RETURN(L, X, Y) return (Y)
#define GAMS_ERROR_BREAK(L, X) { break; }
#else
# if !defined (GAMS_ERROR)
#  define GAMS_ERROR(L, X) \
  do { \
    if (GAMS_debug_level >= L) \
      { \
        int const __ace_error = ACE_Log_Msg::last_error_adapter (); \
        ACE_Log_Msg *ace___ = ACE_Log_Msg::instance ();               \
        ace___->conditional_set (__FILE__, __LINE__, -1, __ace_error); \
        ace___->log X; \
      } \
  } while (0)
#  endif
# if !defined (GAMS_DEBUG)
#  define GAMS_DEBUG(L, X) \
  do { \
    if (GAMS_debug_level >= L) \
      { \
        int const __ace_error = ACE_Log_Msg::last_error_adapter (); \
        ACE_Log_Msg *ace___ = ACE_Log_Msg::instance (); \
        ace___->conditional_set (__FILE__, __LINE__, 0, __ace_error); \
        ace___->log X; \
      } \
  } while (0)
# endif
# if !defined (GAMS_LOG_TRACE)
#  define GAMS_LOG_TRACE(L, X) \
  do { \
    if (GAMS_debug_level >= L) \
      { \
        int const __ace_error = ACE_Log_Msg::last_error_adapter (); \
        ACE_Log_Msg *ace___ = ACE_Log_Msg::instance (); \
        ace___->conditional_set (__FILE__, __LINE__, 0, __ace_error); \
        ace___->log X; \
      } \
  } while (0)
# endif
# if !defined (GAMS_ERROR_RETURN)
#  define GAMS_ERROR_RETURN(L, X, Y) \
  do { \
    if (GAMS_debug_level >= L) \
      { \
        int const __ace_error = ACE_Log_Msg::last_error_adapter (); \
        ACE_Log_Msg *ace___ = ACE_Log_Msg::instance (); \
        ace___->conditional_set (__FILE__, __LINE__, Y, __ace_error); \
        ace___->log X; \
        return Y; \
      } \
  } while (0)
# endif
# if !defined (GAMS_ERROR_BREAK)
#  define GAMS_ERROR_BREAK(L, X) { GAMS_ERROR (L, X); break; }
# endif
#endif

#endif // _GAMS_UTILITY_LOGGING_H_
