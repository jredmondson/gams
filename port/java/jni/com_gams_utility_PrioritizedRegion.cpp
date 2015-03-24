#include "com_gams_utility_PrioritizedRegion.h"
#include "gams/utility/Prioritized_Region.h"

namespace containers = Madara::Knowledge_Engine::Containers;
namespace engine = Madara::Knowledge_Engine;
namespace utility = gams::utility;

/*
 * Class:     com_gams_utility_PrioritizedRegion
 * Method:    jni_PrioritizedRegion
 * Signature: ()J
 */
jlong JNICALL Java_com_gams_utility_PrioritizedRegion_jni_1PrioritizedRegion
  (JNIEnv *, jobject)
{
  return (jlong) new utility::Prioritized_Region ();
}

/*
 * Class:     com_gams_utility_PrioritizedRegion
 * Method:    jni_freePrioritizedRegion
 * Signature: (J)V
 */
void JNICALL Java_com_gams_utility_PrioritizedRegion_jni_1freePrioritizedRegion
  (JNIEnv *, jclass, jlong cptr)
{
  delete (utility::Prioritized_Region *) cptr;
}

/*
 * Class:     com_gams_utility_PrioritizedRegion
 * Method:    jni_getPriority
 * Signature: (J)I
 */
jlong JNICALL Java_com_gams_utility_PrioritizedRegion_jni_1getPriority
  (JNIEnv *, jobject, jlong cptr)
{
  utility::Prioritized_Region * current = (utility::Prioritized_Region *) cptr;

  return (jlong) current;
}

/*
 * Class:     com_gams_utility_PrioritizedRegion
 * Method:    jni_setPriority
 * Signature: (JI)V
 */
void JNICALL Java_com_gams_utility_PrioritizedRegion_jni_1setPriority
  (JNIEnv *, jobject, jlong cptr, jlong value)
{
  utility::Prioritized_Region * current = (utility::Prioritized_Region *) cptr;
  current->priority = value;
}
