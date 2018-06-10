
#ifndef GAMS_EXPORT_H
#define GAMS_EXPORT_H

#if defined  (_WIN32) && !defined (GAMS_BUILD_STATIC)
#  if !defined (GAMS_PROPER_EXPORT_FLAG)
#    define GAMS_PROPER_EXPORT_FLAG __declspec (dllexport)
#    define GAMS_PROPER_IMPORT_FLAG __declspec (dllimport)
#  endif
#else
#  if !defined (GAMS_PROPER_EXPORT_FLAG)
#    define GAMS_PROPER_EXPORT_FLAG
#    define GAMS_PROPER_IMPORT_FLAG
#  endif
#endif

#if defined (GAMS_BUILD_DLL)
#  define GAMS_EXPORT GAMS_PROPER_EXPORT_FLAG
#else
#  define GAMS_EXPORT GAMS_PROPER_IMPORT_FLAG
#endif /* GAMS_BUILD_DLL */

#endif /* GAMS_EXPORT_H */
