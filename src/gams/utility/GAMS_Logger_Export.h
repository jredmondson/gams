
// -*- C++ -*-
// $Id$
// Definition for Win32 Export directives.
// This file is generated automatically by generate_export_file.pl GAMS_Logger
// ------------------------------
#ifndef GAMS_LOGGER_EXPORT_H
#define GAMS_LOGGER_EXPORT_H

#include "ace/config-all.h"

#if defined (ACE_AS_STATIC_LIBS) && !defined (GAMS_LOGGER_HAS_DLL)
#  define GAMS_LOGGER_HAS_DLL 0
#endif /* ACE_AS_STATIC_LIBS && GAMS_LOGGER_HAS_DLL */

#if !defined (GAMS_LOGGER_HAS_DLL)
#  define GAMS_LOGGER_HAS_DLL 1
#endif /* ! GAMS_LOGGER_HAS_DLL */

#if defined (GAMS_LOGGER_HAS_DLL) && (GAMS_LOGGER_HAS_DLL == 1)
#  if defined (GAMS_LOGGER_BUILD_DLL)
#    define GAMS_Logger_Export ACE_Proper_Export_Flag
#    define GAMS_LOGGER_SINGLETON_DECLARATION(T) ACE_EXPORT_SINGLETON_DECLARATION (T)
#    define GAMS_LOGGER_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK) ACE_EXPORT_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK)
#  else /* GAMS_LOGGER_BUILD_DLL */
#    define GAMS_Logger_Export ACE_Proper_Import_Flag
#    define GAMS_LOGGER_SINGLETON_DECLARATION(T) ACE_IMPORT_SINGLETON_DECLARATION (T)
#    define GAMS_LOGGER_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK) ACE_IMPORT_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK)
#  endif /* GAMS_LOGGER_BUILD_DLL */
#else /* GAMS_LOGGER_HAS_DLL == 1 */
#  define GAMS_Logger_Export
#  define GAMS_LOGGER_SINGLETON_DECLARATION(T)
#  define GAMS_LOGGER_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK)
#endif /* GAMS_LOGGER_HAS_DLL == 1 */

// Set GAMS_LOGGER_NTRACE = 0 to turn on library specific tracing even if
// tracing is turned off for ACE.
//#if !defined (GAMS_LOGGER_NTRACE)
//#  if (ACE_NTRACE == 1)
//#    define GAMS_LOGGER_NTRACE 1
//#  else /* (ACE_NTRACE == 1) */
//#    define GAMS_LOGGER_NTRACE 0
//#  endif /* (ACE_NTRACE == 1) */
//#endif /* !GAMS_LOGGER_NTRACE */
//
//#if (GAMS_LOGGER_NTRACE == 1)
//#  define GAMS_LOGGER_TRACE(X)
//#else /* (GAMS_LOGGER_NTRACE == 1) */
//#  if !defined (ACE_HAS_TRACE)
//#    define ACE_HAS_TRACE
//#  endif /* ACE_HAS_TRACE */
//#  define GAMS_LOGGER_TRACE(X) ACE_TRACE_IMPL(X)
//#  include "ace/Trace.h"
//#endif /* (GAMS_LOGGER_NTRACE == 1) */

#endif /* GAMS_LOGGER_EXPORT_H */

// End of auto generated file.
