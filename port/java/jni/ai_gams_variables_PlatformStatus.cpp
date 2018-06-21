
#include "ai_gams_variables_PlatformStatus.h"
#include "gams/variables/PlatformStatus.h"
#include "gams_jni.h"

namespace containers = madara::knowledge::containers;
namespace engine = madara::knowledge;
namespace variables = gams::variables;

/*
 * Class:     ai_gams_variables_PlatformStatus
 * Method:    jni_PlatformStatus
 * Signature: ()J
 */
jlong JNICALL Java_ai_gams_variables_PlatformStatus_jni_1PlatformStatus__
  (JNIEnv *, jobject)
{
  return (jlong) new variables::PlatformStatus ();
}

/*
 * Class:     ai_gams_variables_PlatformStatus
 * Method:    jni_PlatformStatus
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_PlatformStatus_jni_1PlatformStatus__J
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::PlatformStatus * current = (variables::PlatformStatus *) cptr;
  if (current)
  {
    result = (jlong) new variables::PlatformStatus (*current);
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "PlatformStatus::copyConstructor: "
      "PlatformStatus object is released already");
  }
  
  return result;
}

/*
 * Class:     ai_gams_variables_PlatformStatus
 * Method:    jni_freePlatformStatus
 * Signature: (J)V
 */
void JNICALL Java_ai_gams_variables_PlatformStatus_jni_1freePlatformStatus
  (JNIEnv *, jclass, jlong cptr)
{
  delete (variables::PlatformStatus *) cptr;
}

/*
 * Class:     ai_gams_variables_PlatformStatus
 * Method:    jni_getName
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_ai_gams_variables_PlatformStatus_jni_1getName
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result = 0;

  variables::PlatformStatus * current = (variables::PlatformStatus *) cptr;
  if (current)
  {
    result = env->NewStringUTF(current->name.c_str ());
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "PlatformStatus::getName: "
      "PlatformStatus object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_PlatformStatus
 * Method:    jni_init
 * Signature: (JJJLjava/lang/String;)V
 */
void JNICALL Java_ai_gams_variables_PlatformStatus_jni_1init
  (JNIEnv * env, jobject, jlong cptr, jlong type, jlong context, jstring name)
{
  variables::PlatformStatus * current = (variables::PlatformStatus *) cptr;

  if (current)
  {
    const char * str_name = env->GetStringUTFChars(name, 0);

    if (type == 0)
    {
      engine::KnowledgeBase * kb = (engine::KnowledgeBase *) context;
      current->init_vars (*kb, str_name);
    }
    else if (type == 1)
    {
      engine::Variables * vars = (engine::Variables *) context;
      current->init_vars (*vars, str_name);
    }

    env->ReleaseStringUTFChars(name, str_name);
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "PlatformStatus::init: "
      "PlatformStatus object is released already");
  }
}

/*
 * Class:     ai_gams_variables_PlatformStatus
 * Method:    jni_toString
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_ai_gams_variables_PlatformStatus_jni_1toString
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result = 0;

  variables::PlatformStatus * current = (variables::PlatformStatus *) cptr;
  if (current)
  {
    result = env->NewStringUTF(current->name.c_str ());
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "PlatformStatus::toString: "
      "PlatformStatus object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_PlatformStatus
 * Method:    jni_getCommunicationAvailable
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_PlatformStatus_jni_1getCommunicationAvailable
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::PlatformStatus * current = (variables::PlatformStatus *) cptr;

  if (current)
  {
    result = (jlong) &current->communication_available;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "PlatformStatus::getCommunicationAvailable: "
      "PlatformStatus object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_PlatformStatus
 * Method:    jni_getDeadlocked
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_PlatformStatus_jni_1getDeadlocked
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::PlatformStatus * current = (variables::PlatformStatus *) cptr;

  if (current)
  {
    result = (jlong) &current->deadlocked;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "PlatformStatus::getDeadlocked: "
      "PlatformStatus object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_PlatformStatus
 * Method:    jni_getFailed
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_PlatformStatus_jni_1getFailed
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::PlatformStatus * current = (variables::PlatformStatus *) cptr;

  if (current)
  {
    result = (jlong) &current->failed;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "PlatformStatus::getFailed: "
      "PlatformStatus object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_PlatformStatus
 * Method:    jni_getGpsSpoofed
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_PlatformStatus_jni_1getGpsSpoofed
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::PlatformStatus * current = (variables::PlatformStatus *) cptr;

  if (current)
  {
    result = (jlong) &current->gps_spoofed;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "PlatformStatus::getGpsSpoofed: "
      "PlatformStatus object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_PlatformStatus
 * Method:    jni_getMovementAvailable
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_PlatformStatus_jni_1getMovementAvailable
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::PlatformStatus * current = (variables::PlatformStatus *) cptr;

  if (current)
  {
    result = (jlong) &current->movement_available;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "PlatformStatus::getMovementAvailable: "
      "PlatformStatus object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_PlatformStatus
 * Method:    jni_getMoving
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_PlatformStatus_jni_1getMoving
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::PlatformStatus * current = (variables::PlatformStatus *) cptr;

  if (current)
  {
    result = (jlong) &current->moving;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "PlatformStatus::getMoving: "
      "PlatformStatus object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_PlatformStatus
 * Method:    jni_getOk
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_PlatformStatus_jni_1getOk
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::PlatformStatus * current = (variables::PlatformStatus *) cptr;

  if (current)
  {
    result = (jlong) &current->ok;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "PlatformStatus::getOk: "
      "PlatformStatus object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_PlatformStatus
 * Method:    jni_getPausedMoving
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_PlatformStatus_jni_1getPausedMoving
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::PlatformStatus * current = (variables::PlatformStatus *) cptr;

  if (current)
  {
    result = (jlong) &current->paused_moving;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "PlatformStatus::getPausedMoving: "
      "PlatformStatus object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_PlatformStatus
 * Method:    jni_getReducedSensing
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_PlatformStatus_jni_1getReducedSensing
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::PlatformStatus * current = (variables::PlatformStatus *) cptr;

  if (current)
  {
    result = (jlong) &current->reduced_sensing;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "PlatformStatus::getReducedSensing: "
      "PlatformStatus object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_PlatformStatus
 * Method:    jni_getReducedMovement
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_PlatformStatus_jni_1getReducedMovement
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::PlatformStatus * current = (variables::PlatformStatus *) cptr;

  if (current)
  {
    result = (jlong) &current->reduced_movement;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "PlatformStatus::getReducedMovement: "
      "PlatformStatus object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_PlatformStatus
 * Method:    jni_getSensorsAvailable
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_PlatformStatus_jni_1getSensorsAvailable
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::PlatformStatus * current = (variables::PlatformStatus *) cptr;

  if (current)
  {
    result = (jlong) &current->sensors_available;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "PlatformStatus::getSensorsAvailable: "
      "PlatformStatus object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_PlatformStatus
 * Method:    jni_getWaiting
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_PlatformStatus_jni_1getWaiting
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::PlatformStatus * current = (variables::PlatformStatus *) cptr;

  if (current)
  {
    result = (jlong) &current->waiting;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "PlatformStatus::getWaiting: "
      "PlatformStatus object is released already");
  }

  return result;
}
