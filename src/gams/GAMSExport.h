
#ifndef GAMS_EXPORT_H
#define GAMS_EXPORT_H

#if defined  (_WIN32) && !defined (GAMS_BUILD_STATIC)
#  if !defined (GAMS_Proper_Export_Flag)
#    define GAMS_Proper_Export_Flag __declspec (dllexport)
#    define GAMS_Proper_Import_Flag __declspec (dllimport)
#  endif
#else
#  if !defined (GAMS_Proper_Export_Flag)
#    define GAMS_Proper_Export_Flag
#    define GAMS_Proper_Import_Flag
#  endif
#endif

#if defined (GAMS_BUILD_DLL)
#  define GAMSExport GAMS_Proper_Export_Flag
#else
#  define GAMSExport GAMS_Proper_Import_Flag
#endif /* GAMS_BUILD_DLL */

#endif /* GAMS_EXPORT_H */
