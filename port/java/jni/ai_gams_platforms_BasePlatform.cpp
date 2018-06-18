#include "ai_gams_platforms_BasePlatform.h"
#include "gams/platforms/java/JavaPlatform.h"

namespace platforms = gams::platforms;

/*
 * Class:     ai_gams_platforms_BasePlatform
 * Method:    jni_getKnowledgeBase
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_platforms_BasePlatform_jni_1getKnowledgeBase
  (JNIEnv * , jobject, jlong cptr)
{
  jlong result (0);

  platforms::BasePlatform * current = (platforms::BasePlatform *)cptr;
  if (current)
  {
    result = (jlong) current->get_knowledge_base ();
  }

  return result;
}

/*
 * Class:     ai_gams_platforms_BasePlatform
 * Method:    jni_getSelf
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_platforms_BasePlatform_jni_1getSelf
  (JNIEnv * , jobject, jlong cptr)
{
  jlong result (0);

  platforms::BasePlatform * current = (platforms::BasePlatform *)cptr;
  if (current)
  {
    result = (jlong) current->get_self ();
  }

  return result;
}

/*
 * Class:     ai_gams_platforms_BasePlatform
 * Method:    jni_getPlatformStatus
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_platforms_BasePlatform_jni_1getPlatformStatus
  (JNIEnv * , jobject, jlong cptr)
{
  jlong result (0);

  platforms::BasePlatform * current = (platforms::BasePlatform *)cptr;
  if (current)
  {
    result = (jlong) current->get_platform_status ();
  }

  return result;
}

