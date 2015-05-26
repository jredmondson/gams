
#include "gams_jni.h"

static JavaVM* gams_JVM = NULL;

jint JNICALL JNI_OnLoad (JavaVM* vm, void* reserved)
{
  JNIEnv * env;
  if (vm->GetEnv ( (void**)&env, JNI_VERSION_1_8) != JNI_OK)
  {
    return JNI_ERR;
  }

  gams_JVM = vm;

  return env->GetVersion ();
}

void JNICALL JNI_OnUnload (JavaVM* vm, void* reserved)
{
  JNIEnv * env;
  vm->GetEnv ( (void**)&env, JNI_VERSION_1_8);
  gams_JVM = 0;
}

bool gams_jni_is_attached ()
{
  JNIEnv * env (0);

  if (gams_JVM)
  {
    gams_JVM->GetEnv ( (void**)&env, JNI_VERSION_1_8);
  }

  return env != 0;
}

JNIEnv* gams_jni_get_env ()
{
  JNIEnv * env (0);

  if (gams_JVM)
  {
    gams_JVM->GetEnv ( (void**)&env, JNI_VERSION_1_8);
    if (env == 0)
    {
      //Thread is not attached
#ifndef _USING_OPENJDK_
      gams_JVM->AttachCurrentThread ( (void **)&env, NULL);
#else
      gams_JVM->AttachCurrentThread (&env, NULL);
#endif
    }
  }
  return env;
}

JavaVM* gams_jni_jvm ()
{
  return gams_JVM;
}

JNIEnv* jni_attach ()
{
  return gams_jni_get_env ();
}

void jni_detach ()
{
  if (gams_JVM)
  {
    gams_JVM->DetachCurrentThread ();
  }
}
