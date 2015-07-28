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
 * Method:    jni_toString
 * Signature: (J)Ljava/lang/String;
 */
GAMS_Export jstring JNICALL Java_com_gams_utility_PrioritizedRegion_jni_1toString
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring ret_val;

  utility::Prioritized_Region * current = (utility::Prioritized_Region *) cptr;
  if (current)
  {
    std::string result = current->to_string();
    ret_val = env->NewStringUTF(result.c_str());
  }

  return ret_val;
}

/*
 * Class:     com_gams_utility_PrioritizedRegion
 * Method:    jni_fromContainer
 * Signature: (JLjava/lang/String;J)V
 */
GAMS_Export void JNICALL Java_com_gams_utility_PrioritizedRegion_jni_1fromContainer
  (JNIEnv * env, jobject, jlong cptr, jstring name, jlong kb_ptr)
{
  utility::Prioritized_Region * current = (utility::Prioritized_Region *) cptr;
  const char * str_name = env->GetStringUTFChars (name, 0);
  engine::Knowledge_Base * kb = (engine::Knowledge_Base *) kb_ptr;

  if (current && str_name && kb)
  {
    current->from_container (str_name, *kb);
  }

  env->ReleaseStringUTFChars (name, str_name);
}

/*
 * Class:     com_gams_utility_PrioritizedRegion
 * Method:    jni_toContainer
 * Signature: (JLjava/lang/String;J)V
 */
GAMS_Export void JNICALL Java_com_gams_utility_PrioritizedRegion_jni_1toContainer
  (JNIEnv * env, jobject, jlong cptr, jstring name, jlong kb_ptr)
{
  utility::Prioritized_Region * current = (utility::Prioritized_Region *) cptr;
  const char * str_name = env->GetStringUTFChars (name, 0);
  engine::Knowledge_Base * kb = (engine::Knowledge_Base *) kb_ptr;

  if (current && str_name && kb)
  {
    current->to_container (str_name, *kb);
  }

  env->ReleaseStringUTFChars (name, str_name);
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
