#include "ai_gams_utility_PrioritizedRegion.h"
#include "gams/pose/PrioritizedRegion.h"
#include "gams_jni.h"

namespace containers = madara::knowledge::containers;
namespace engine = madara::knowledge;
namespace pose = gams::pose;

/*
 * Class:     ai_gams_utility_PrioritizedRegion
 * Method:    jni_PrioritizedRegion
 * Signature: ()J
 */
jlong JNICALL Java_ai_gams_utility_PrioritizedRegion_jni_1PrioritizedRegion
  (JNIEnv *, jobject)
{
  return (jlong) new pose::PrioritizedRegion ();
}

/*
 * Class:     ai_gams_utility_PrioritizedRegion
 * Method:    jni_toString
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_ai_gams_utility_PrioritizedRegion_jni_1toString
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring ret_val = 0;

  pose::PrioritizedRegion * current = (pose::PrioritizedRegion *) cptr;
  if (current)
  {
    std::string result = current->to_string();
    ret_val = env->NewStringUTF(result.c_str());
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "PrioritizedRegion::toString: "
      "PrioritizedRegion object is released already");
  }

  return ret_val;
}

/*
 * Class:     ai_gams_utility_PrioritizedRegion
 * Method:    jni_fromContainer
 * Signature: (JJLjava/lang/String;)V
 */
void JNICALL Java_ai_gams_utility_PrioritizedRegion_jni_1fromContainer
  (JNIEnv * env, jobject, jlong cptr, jlong kb_ptr, jstring name)
{
  pose::PrioritizedRegion * current = (pose::PrioritizedRegion *) cptr;
  const char * str_name = env->GetStringUTFChars (name, 0);
  engine::KnowledgeBase * kb = (engine::KnowledgeBase *) kb_ptr;

  if (current && str_name && kb)
  {
    current->from_container (*kb, str_name);
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    env->ReleaseStringUTFChars (name, str_name);
    
    gams::utility::java::throw_dead_obj_exception(env,
      "PrioritizedRegion::fromContainer: "
      "PrioritizedRegion, name, or KB objects are released already");
  }

  env->ReleaseStringUTFChars (name, str_name);
}

/*
 * Class:     ai_gams_utility_PrioritizedRegion
 * Method:    jni_toContainer
 * Signature: (JJLjava/lang/String;)V
 */
void JNICALL Java_ai_gams_utility_PrioritizedRegion_jni_1toContainer
  (JNIEnv * env, jobject, jlong cptr, jlong kb_ptr, jstring name)
{
  pose::PrioritizedRegion * current = (pose::PrioritizedRegion *) cptr;
  const char * str_name = env->GetStringUTFChars (name, 0);
  engine::KnowledgeBase * kb = (engine::KnowledgeBase *) kb_ptr;

  if (current && str_name && kb)
  {
    current->to_container (*kb, str_name);
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    env->ReleaseStringUTFChars (name, str_name);
    
    gams::utility::java::throw_dead_obj_exception(env,
      "PrioritizedRegion::toContainer: "
      "PrioritizedRegion, name, or KB objects are released already");
  }

  env->ReleaseStringUTFChars (name, str_name);
}

/*
 * Class:     ai_gams_utility_PrioritizedRegion
 * Method:    jni_modify
 * Signature: (J)V
 */
void JNICALL Java_ai_gams_utility_PrioritizedRegion_jni_1modify
  (JNIEnv * env, jobject, jlong cptr)
{
  pose::PrioritizedRegion * current = (pose::PrioritizedRegion *) cptr;

  if (current)
  {
    current->modify();
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "PrioritizedRegion::modify: "
      "PrioritizedRegion object is released already");
  }
}

/*
 * Class:     ai_gams_utility_PrioritizedRegion
 * Method:    jni_freePrioritizedRegion
 * Signature: (J)V
 */
void JNICALL Java_ai_gams_utility_PrioritizedRegion_jni_1freePrioritizedRegion
  (JNIEnv *, jclass, jlong cptr)
{
  delete (pose::PrioritizedRegion *) cptr;
}

/*
 * Class:     ai_gams_utility_PrioritizedRegion
 * Method:    jni_getPriority
 * Signature: (J)I
 */
jlong JNICALL Java_ai_gams_utility_PrioritizedRegion_jni_1getPriority
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  pose::PrioritizedRegion * current = (pose::PrioritizedRegion *) cptr;

  if (current)
  {
    result = (jlong) current->priority;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "PrioritizedRegion::getPriority: "
      "PrioritizedRegion object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_PrioritizedRegion
 * Method:    jni_setPriority
 * Signature: (JI)V
 */
void JNICALL Java_ai_gams_utility_PrioritizedRegion_jni_1setPriority
  (JNIEnv * env, jobject, jlong cptr, jlong value)
{
  pose::PrioritizedRegion * current = (pose::PrioritizedRegion *) cptr;

  if (current)
  {
    current->priority = value;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "PrioritizedRegion::setPriority: "
      "PrioritizedRegion object is released already");
  }
}
