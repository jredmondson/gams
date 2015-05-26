
#ifndef _Included_com_gams_JNI
#define _Included_com_gams_JNI

#include <jni.h>
#include "gams/GAMS_Export.h"

GAMS_Export jint JNICALL JNI_OnLoad (JavaVM *vm, void *reserved);
GAMS_Export void JNICALL JNI_OnUnload (JavaVM* vm, void* reserved);

GAMS_Export bool gams_jni_is_attached ();
GAMS_Export JNIEnv* gams_jni_get_env ();

GAMS_Export JavaVM* gams_jni_jvm ();

GAMS_Export JNIEnv* jni_attach ();
GAMS_Export void jni_detach ();

#endif