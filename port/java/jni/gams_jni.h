
#ifndef _Included_com_gams_JNI
#define _Included_com_gams_JNI

#include <jni.h>
#include "gams/GAMSExport.h"

GAMSExport jint JNICALL JNI_OnLoad (JavaVM * vm, void * reserved);
GAMSExport void JNICALL JNI_OnUnload (JavaVM * vm, void * reserved);

GAMSExport bool gams_jni_is_attached ();
GAMSExport JNIEnv * gams_jni_get_env ();

GAMSExport JavaVM * gams_jni_jvm ();

GAMSExport JNIEnv * jni_attach ();
GAMSExport void jni_detach ();

namespace gams
{
  namespace utility
  {
    namespace java
    {
      /**
      * Android-safe, Thread-safe find class method
      * @param  env   Java environment
      * @param  name  name of the class to find
      **/
      jclass GAMSExport find_class (JNIEnv * env, const char * name);
    }
  }
}

#endif