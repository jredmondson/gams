
#include "ai_gams_variables_AlgorithmStatus.h"
#include "gams/variables/AlgorithmStatus.h"
#include "gams_jni.h"

namespace containers = madara::knowledge::containers;
namespace engine = madara::knowledge;
namespace variables = gams::variables;

/*
 * Class:     ai_gams_variables_AlgorithmStatus
 * Method:    jni_AlgorithmStatus
 * Signature: ()J
 */
jlong JNICALL Java_ai_gams_variables_AlgorithmStatus_jni_1AlgorithmStatus__
  (JNIEnv *, jobject)
{
  return (jlong) new variables::AlgorithmStatus ();
}

/*
 * Class:     ai_gams_variables_AlgorithmStatus
 * Method:    jni_AlgorithmStatus
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_AlgorithmStatus_jni_1AlgorithmStatus__J
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::AlgorithmStatus * current = (variables::AlgorithmStatus *) cptr;
  if (current)
  {
    result = (jlong) new variables::AlgorithmStatus (*current);
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "AlgorithmStatus::copyConstructor: "
      "AlgorithmStatus object is released already");
  }
  
  return result;
}

/*
 * Class:     ai_gams_variables_AlgorithmStatus
 * Method:    jni_freeAlgorithmStatus
 * Signature: (J)V
 */
void JNICALL Java_ai_gams_variables_AlgorithmStatus_jni_1freeAlgorithmStatus
  (JNIEnv *, jclass, jlong cptr)
{
  delete (variables::AlgorithmStatus *) cptr;
}

/*
 * Class:     ai_gams_variables_AlgorithmStatus
 * Method:    jni_getName
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_ai_gams_variables_AlgorithmStatus_jni_1getName
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result = 0;

  variables::AlgorithmStatus * current = (variables::AlgorithmStatus *) cptr;
  if (current)
  {
    result = env->NewStringUTF(current->name.c_str ());
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "AlgorithmStatus::getName: "
      "AlgorithmStatus object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_AlgorithmStatus
 * Method:    jni_init
 * Signature: (JJJLjava/lang/String;I)V
 */
void JNICALL Java_ai_gams_variables_AlgorithmStatus_jni_1init
  (JNIEnv * env, jobject, jlong cptr, jlong type, jlong context, jstring name, jint i)
{
  variables::AlgorithmStatus * current = (variables::AlgorithmStatus *) cptr;

  if (current)
  {
    const char * str_name = env->GetStringUTFChars(name, 0);

    if (type == 0)
    {
      engine::KnowledgeBase * kb = (engine::KnowledgeBase *) context;
      current->init_vars (*kb, str_name, (int)i);
    }
    else if (type == 1)
    {
      engine::Variables * vars = (engine::Variables *) context;
      current->init_vars (*vars, str_name, (int)i);
    }

    env->ReleaseStringUTFChars(name, str_name);
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "AlgorithmStatus::init: "
      "AlgorithmStatus object is released already");
  }
}

/*
 * Class:     ai_gams_variables_AlgorithmStatus
 * Method:    jni_toString
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_ai_gams_variables_AlgorithmStatus_jni_1toString
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result = 0;

  variables::AlgorithmStatus * current = (variables::AlgorithmStatus *) cptr;
  if (current)
  {
    result = env->NewStringUTF(current->name.c_str ());
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "AlgorithmStatus::toString: "
      "AlgorithmStatus object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_AlgorithmStatus
 * Method:    jni_getDeadlocked
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_AlgorithmStatus_jni_1getDeadlocked
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::AlgorithmStatus * current = (variables::AlgorithmStatus *) cptr;

  if (current)
  {
    result = (jlong) &current->deadlocked;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "AlgorithmStatus::getAlgorithm: "
      "AlgorithmStatus object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_AlgorithmStatus
 * Method:    jni_getFailed
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_AlgorithmStatus_jni_1getFailed
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::AlgorithmStatus * current = (variables::AlgorithmStatus *) cptr;

  if (current)
  {
    result = (jlong) &current->failed;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "AlgorithmStatus::getFailed: "
      "AlgorithmStatus object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_AlgorithmStatus
 * Method:    jni_getOk
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_AlgorithmStatus_jni_1getOk
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::AlgorithmStatus * current = (variables::AlgorithmStatus *) cptr;

  if (current)
  {
    result = (jlong) &current->ok;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "AlgorithmStatus::getOk: "
      "AlgorithmStatus object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_AlgorithmStatus
 * Method:    jni_getPaused
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_AlgorithmStatus_jni_1getPaused
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::AlgorithmStatus * current = (variables::AlgorithmStatus *) cptr;

  if (current)
  {
    result = (jlong) &current->paused;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "AlgorithmStatus::getPaused: "
      "AlgorithmStatus object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_AlgorithmStatus
 * Method:    jni_getUnknown
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_AlgorithmStatus_jni_1getUnknown
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::AlgorithmStatus * current = (variables::AlgorithmStatus *) cptr;

  if (current)
  {
    result = (jlong) &current->unknown;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "AlgorithmStatus::getUnknown: "
      "AlgorithmStatus object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_AlgorithmStatus
 * Method:    jni_getWaiting
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_AlgorithmStatus_jni_1getWaiting
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::AlgorithmStatus * current = (variables::AlgorithmStatus *) cptr;

  if (current)
  {
    result = (jlong) &current->waiting;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "AlgorithmStatus::getWaiting: "
      "AlgorithmStatus object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_AlgorithmStatus
 * Method:    jni_getFinished
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_AlgorithmStatus_jni_1getFinished
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::AlgorithmStatus * current = (variables::AlgorithmStatus *) cptr;

  if (current)
  {
    result = (jlong) &current->finished;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "AlgorithmStatus::getFinished: "
      "AlgorithmStatus object is released already");
  }

  return result;
}
