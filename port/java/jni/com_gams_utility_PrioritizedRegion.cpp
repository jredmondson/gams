#include "com_gams_utility_PrioritizedRegion.h"
#include "gams/pose/PrioritizedRegion.h"

namespace containers = madara::knowledge::containers;
namespace engine = madara::knowledge;
namespace pose = gams::pose;

/*
 * Class:     com_gams_utility_PrioritizedRegion
 * Method:    jni_PrioritizedRegion
 * Signature: ()J
 */
jlong JNICALL Java_com_gams_utility_PrioritizedRegion_jni_1PrioritizedRegion
  (JNIEnv *, jobject)
{
  return (jlong) new pose::PrioritizedRegion ();
}

/*
 * Class:     com_gams_utility_PrioritizedRegion
 * Method:    jni_toString
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_com_gams_utility_PrioritizedRegion_jni_1toString
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring ret_val;

  pose::PrioritizedRegion * current = (pose::PrioritizedRegion *) cptr;
  if (current)
  {
    std::string result = current->to_string();
    ret_val = env->NewStringUTF(result.c_str());
  }
  else
  {
    ret_val = env->NewStringUTF ("");
  }

  return ret_val;
}

/*
 * Class:     com_gams_utility_PrioritizedRegion
 * Method:    jni_fromContainer
 * Signature: (JJLjava/lang/String;)V
 */
void JNICALL Java_com_gams_utility_PrioritizedRegion_jni_1fromContainer
  (JNIEnv * env, jobject, jlong cptr, jlong kb_ptr, jstring name)
{
  pose::PrioritizedRegion * current = (pose::PrioritizedRegion *) cptr;
  const char * str_name = env->GetStringUTFChars (name, 0);
  engine::KnowledgeBase * kb = (engine::KnowledgeBase *) kb_ptr;

  if (current && str_name && kb)
  {
    current->from_container (*kb, str_name);
  }

  env->ReleaseStringUTFChars (name, str_name);
}

/*
 * Class:     com_gams_utility_PrioritizedRegion
 * Method:    jni_toContainer
 * Signature: (JJLjava/lang/String;)V
 */
void JNICALL Java_com_gams_utility_PrioritizedRegion_jni_1toContainer
  (JNIEnv * env, jobject, jlong cptr, jlong kb_ptr, jstring name)
{
  pose::PrioritizedRegion * current = (pose::PrioritizedRegion *) cptr;
  const char * str_name = env->GetStringUTFChars (name, 0);
  engine::KnowledgeBase * kb = (engine::KnowledgeBase *) kb_ptr;

  if (current && str_name && kb)
  {
    current->to_container (*kb, str_name);
  }

  env->ReleaseStringUTFChars (name, str_name);
}

/*
 * Class:     com_gams_utility_PrioritizedRegion
 * Method:    jni_modify
 * Signature: (J)V
 */
void JNICALL Java_com_gams_utility_PrioritizedRegion_jni_1modify
  (JNIEnv *, jobject, jlong cptr)
{
  pose::PrioritizedRegion * current = (pose::PrioritizedRegion *) cptr;

  if (current)
  {
    current->modify();
  }
}

/*
 * Class:     com_gams_utility_PrioritizedRegion
 * Method:    jni_freePrioritizedRegion
 * Signature: (J)V
 */
void JNICALL Java_com_gams_utility_PrioritizedRegion_jni_1freePrioritizedRegion
  (JNIEnv *, jclass, jlong cptr)
{
  delete (pose::PrioritizedRegion *) cptr;
}

/*
 * Class:     com_gams_utility_PrioritizedRegion
 * Method:    jni_getPriority
 * Signature: (J)I
 */
jlong JNICALL Java_com_gams_utility_PrioritizedRegion_jni_1getPriority
  (JNIEnv *, jobject, jlong cptr)
{
  pose::PrioritizedRegion * current = (pose::PrioritizedRegion *) cptr;

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
  pose::PrioritizedRegion * current = (pose::PrioritizedRegion *) cptr;
  current->priority = value;
}
