
#ifndef GAMS_EXPORT_H
#define GAMS_EXPORT_H

#if defined (GAMS_AS_STATIC_LIBS) && !defined (GAMS_HAS_DLL)
#  define GAMS_HAS_DLL 0
#endif /* GAMS_AS_STATIC_LIBS && GAMS_HAS_DLL */

#if !defined (GAMS_HAS_DLL)
#  define GAMS_HAS_DLL 1
#endif /* ! GAMS_HAS_DLL */

#if defined  (_WIN32)
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

#if defined (GAMS_HAS_DLL) && (GAMS_HAS_DLL == 1)
#  if defined (GAMS_BUILD_DLL)
#    define GAMSExport GAMS_Proper_Export_Flag
#  else /* GAMS_BUILD_DLL */
#    define GAMSExport GAMS_Proper_Import_Flag
#  endif /* GAMS_BUILD_DLL */
#else /* GAMS_HAS_DLL == 1 */
#  define GAMSExport
#endif /* GAMS_HAS_DLL == 1 */

#endif /* GAMS_EXPORT_H */
