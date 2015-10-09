#include "com_gams_algorithms_BaseAlgorithm.h"
#include "gams/algorithms/java/JavaAlgorithm.h"
#include "gams/platforms/java/JavaPlatform.h"

namespace algorithms = gams::algorithms;

/*
 * Class:     com_gams_algorithms_BaseAlgorithm
 * Method:    jni_getKnowledgeBase
 * Signature: (J)J
 */
GAMSExport jlong JNICALL Java_com_gams_algorithms_BaseAlgorithm_jni_1getKnowledgeBase
  (JNIEnv * , jobject, jlong cptr)
{
  jlong result (0);

  algorithms::BaseAlgorithm * current = (algorithms::BaseAlgorithm *)cptr;
  if (current)
  {
    result = (jlong) current->get_knowledge_base ();
  }

  return result;
}

/*
 * Class:     com_gams_algorithms_BaseAlgorithm
 * Method:    jni_getSelf
 * Signature: (J)J
 */
GAMSExport jlong JNICALL Java_com_gams_algorithms_BaseAlgorithm_jni_1getSelf
  (JNIEnv * , jobject, jlong cptr)
{
  jlong result (0);

  algorithms::BaseAlgorithm * current = (algorithms::BaseAlgorithm *)cptr;
  if (current)
  {
    result = (jlong) current->get_self ();
  }

  return result;
}

/*
 * Class:     com_gams_algorithms_BaseAlgorithm
 * Method:    jni_getPlatformObject
 * Signature: (J)Ljava/lang/Object;
 */
GAMSExport jobject JNICALL Java_com_gams_algorithms_BaseAlgorithm_jni_1getPlatformObject
  (JNIEnv *, jobject, jlong cptr)
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

  return result;
}

/*
 * Class:     com_gams_algorithms_BaseAlgorithm
 * Method:    jni_getAlgorithmStatus
 * Signature: (J)J
 */
GAMSExport jlong JNICALL Java_com_gams_algorithms_BaseAlgorithm_jni_1getAlgorithmStatus
  (JNIEnv * , jobject, jlong cptr)
{
  jlong result (0);

  algorithms::BaseAlgorithm * current = (algorithms::BaseAlgorithm *)cptr;
  if (current)
  {
    result = (jlong) current->get_algorithm_status ();
  }

  return result;
}
