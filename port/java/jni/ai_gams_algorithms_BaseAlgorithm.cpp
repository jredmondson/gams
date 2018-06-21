#include "ai_gams_algorithms_BaseAlgorithm.h"
#include "gams/algorithms/java/JavaAlgorithm.h"
#include "gams/platforms/java/JavaPlatform.h"
#include "gams_jni.h"

namespace algorithms = gams::algorithms;

/*
 * Class:     ai_gams_algorithms_BaseAlgorithm
 * Method:    jni_getKnowledgeBase
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_algorithms_BaseAlgorithm_jni_1getKnowledgeBase
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result (0);

  algorithms::BaseAlgorithm * current = (algorithms::BaseAlgorithm *)cptr;
  if (current)
  {
    result = (jlong) current->get_knowledge_base ();
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BaseAlgorithm::getKnowledgeBase: "
      "BaseAlgorithm object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_algorithms_BaseAlgorithm
 * Method:    jni_getSelf
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_algorithms_BaseAlgorithm_jni_1getSelf
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result (0);

  algorithms::BaseAlgorithm * current = (algorithms::BaseAlgorithm *)cptr;
  if (current)
  {
    result = (jlong) current->get_self ();
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BaseAlgorithm::getSelf: "
      "BaseAlgorithm object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_algorithms_BaseAlgorithm
 * Method:    jni_getPlatformObject
 * Signature: (J)Ljava/lang/Object;
 */
jobject JNICALL Java_ai_gams_algorithms_BaseAlgorithm_jni_1getPlatformObject
  (JNIEnv * env, jobject, jlong cptr)
{
  jobject result (0);

  algorithms::BaseAlgorithm * current = (algorithms::BaseAlgorithm *)cptr;
  if (current)
  {
    gams::platforms::JavaPlatform * platform =
      dynamic_cast <gams::platforms::JavaPlatform *> (
        current->get_platform ());

    if (platform)
    {
      result = platform->get_java_instance ();
    }
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BaseAlgorithm::getPlatformObject: "
      "BaseAlgorithm object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_algorithms_BaseAlgorithm
 * Method:    jni_getAlgorithmStatus
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_algorithms_BaseAlgorithm_jni_1getAlgorithmStatus
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result (0);

  algorithms::BaseAlgorithm * current = (algorithms::BaseAlgorithm *)cptr;
  if (current)
  {
    result = (jlong) current->get_algorithm_status ();
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BaseAlgorithm::getAlgorithmStatus: "
      "BaseAlgorithm object is released already");
  }

  return result;
}
