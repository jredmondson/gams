#include "ai_gams_platforms_BasePlatform.h"
#include "gams/platforms/java/JavaPlatform.h"
#include "gams_jni.h"

namespace platforms = gams::platforms;

/*
 * Class:     ai_gams_platforms_BasePlatform
 * Method:    jni_getKnowledgeBase
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_platforms_BasePlatform_jni_1getKnowledgeBase
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result (0);

  platforms::BasePlatform * current = (platforms::BasePlatform *)cptr;
  if (current)
  {
    result = (jlong) current->get_knowledge_base ();
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BasePlatform::getKnowledgeBase: "
      "BasePlatform object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_platforms_BasePlatform
 * Method:    jni_getSelf
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_platforms_BasePlatform_jni_1getSelf
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result (0);

  platforms::BasePlatform * current = (platforms::BasePlatform *)cptr;
  if (current)
  {
    result = (jlong) current->get_self ();
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BasePlatform::getSelf: "
      "BasePlatform object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_platforms_BasePlatform
 * Method:    jni_getPlatformStatus
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_platforms_BasePlatform_jni_1getPlatformStatus
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result (0);

  platforms::BasePlatform * current = (platforms::BasePlatform *)cptr;
  if (current)
  {
    result = (jlong) current->get_platform_status ();
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BasePlatform::getPlatformStatus: "
      "BasePlatform object is released already");
  }

  return result;
}

