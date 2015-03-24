
// -*- C++ -*-
// $Id$
// Definition for Win32 Export directives.
// This file is generated automatically by generate_export_file.pl -d GAMS
// ------------------------------
#ifndef GAMS_EXPORT_H
#define GAMS_EXPORT_H

#include "ace/config-all.h"

#if defined (ACE_AS_STATIC_LIBS) && !defined (GAMS_HAS_DLL)
#  define GAMS_HAS_DLL 0
#endif /* ACE_AS_STATIC_LIBS && GAMS_HAS_DLL */

#if !defined (GAMS_HAS_DLL)
#  define GAMS_HAS_DLL 1
#endif /* ! GAMS_HAS_DLL */

#if defined (GAMS_HAS_DLL) && (GAMS_HAS_DLL == 1)
#  if defined (GAMS_BUILD_DLL)
#    define GAMS_Export ACE_Proper_Export_Flag
#    define GAMS_SINGLETON_DECLARATION(T) ACE_EXPORT_SINGLETON_DECLARATION (T)
#    define GAMS_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK) ACE_EXPORT_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK)
#  else /* GAMS_BUILD_DLL */
#    define GAMS_Export ACE_Proper_Import_Flag
#    define GAMS_SINGLETON_DECLARATION(T) ACE_IMPORT_SINGLETON_DECLARATION (T)
#    define GAMS_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK) ACE_IMPORT_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK)
#  endif /* GAMS_BUILD_DLL */
#else /* GAMS_HAS_DLL == 1 */
#  define GAMS_Export
#  define GAMS_SINGLETON_DECLARATION(T)
#  define GAMS_SINGLETON_DECLARE(SINGLETON_TYPE, CLASS, LOCK)
#endif /* GAMS_HAS_DLL == 1 */

// Set GAMS_NTRACE = 0 to turn on library specific tracing even if
// tracing is turned off for ACE.
//#if !defined (GAMS_NTRACE)
//#  if (ACE_NTRACE == 1)
//#    define GAMS_NTRACE 1
//#  else /* (ACE_NTRACE == 1) */
//#    define GAMS_NTRACE 0
//#  endif /* (ACE_NTRACE == 1) */
//#endif /* !GAMS_NTRACE */
//
//#if (GAMS_NTRACE == 1)
//#  define GAMS_TRACE(X)
//#else /* (GAMS_NTRACE == 1) */
//#  if !defined (ACE_HAS_TRACE)
//#    define ACE_HAS_TRACE
//#  endif /* ACE_HAS_TRACE */
//#  define GAMS_TRACE(X) ACE_TRACE_IMPL(X)
//#  include "ace/Trace.h"
//#endif /* (GAMS_NTRACE == 1) */

#endif /* GAMS_EXPORT_H */

// End of auto generated file.
