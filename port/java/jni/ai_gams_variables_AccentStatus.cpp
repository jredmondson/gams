
#include "ai_gams_variables_AccentStatus.h"
#include "gams/variables/AccentStatus.h"
#include "gams_jni.h"

namespace containers = madara::knowledge::containers;
namespace engine = madara::knowledge;
namespace variables = gams::variables;

/*
 * Class:     ai_gams_variables_AccentStatus
 * Method:    jni_AccentStatus
 * Signature: ()J
 */
jlong JNICALL Java_ai_gams_variables_AccentStatus_jni_1AccentStatus__
  (JNIEnv *, jobject)
{
  return (jlong) new variables::AccentStatus ();
}

/*
 * Class:     ai_gams_variables_AccentStatus
 * Method:    jni_AccentStatus
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_AccentStatus_jni_1AccentStatus__J
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::AccentStatus * current = (variables::AccentStatus *) cptr;
  if (current)
  {
    result = (jlong) new variables::AccentStatus (*current);
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "AccentStatus::copyConstructor: "
      "AccentStatus object is released already");
  }
  
  return result;
}

/*
 * Class:     ai_gams_variables_AccentStatus
 * Method:    jni_freeAccentStatus
 * Signature: (J)V
 */
void JNICALL Java_ai_gams_variables_AccentStatus_jni_1freeAccentStatus
  (JNIEnv *, jclass, jlong cptr)
{
  delete (variables::AccentStatus *) cptr;
}

/*
 * Class:     ai_gams_variables_AccentStatus
 * Method:    jni_getName
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_ai_gams_variables_AccentStatus_jni_1getName
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result = 0;

  variables::AccentStatus * current = (variables::AccentStatus *) cptr;
  if (current)
  {
    result = env->NewStringUTF(current->command.get_name ().c_str ());
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "AccentStatus::getName: "
      "AccentStatus object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_AccentStatus
 * Method:    jni_init
 * Signature: (JJJLjava/lang/String;)V
 */
void JNICALL Java_ai_gams_variables_AccentStatus_jni_1init
  (JNIEnv * env, jobject, jlong cptr, jlong type, jlong context, jstring name)
{
  variables::AccentStatus * current = (variables::AccentStatus *) cptr;

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
      "AccentStatus::init: "
      "AccentStatus object is released already");
  }
}

/*
 * Class:     ai_gams_variables_AccentStatus
 * Method:    jni_toString
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_ai_gams_variables_AccentStatus_jni_1toString
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result = 0;

  variables::AccentStatus * current = (variables::AccentStatus *) cptr;
  if (current)
  {
    result = env->NewStringUTF(current->command.get_name ().c_str ());
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "AccentStatus::toString: "
      "AccentStatus object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_AccentStatus
 * Method:    jni_getArgs
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_AccentStatus_jni_1getArgs
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;
  variables::AccentStatus * current = (variables::AccentStatus *) cptr;

  if (current)
  {
    result = (jlong) &current->command_args;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "AccentStatus::getArgs: "
      "AccentStatus object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_AccentStatus
 * Method:    jni_getCommand
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_AccentStatus_jni_1getCommand
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;
  variables::AccentStatus * current = (variables::AccentStatus *) cptr;

  if (current)
  {
    result = (jlong) &current->command;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "AccentStatus::getCommand: "
      "AccentStatus object is released already");
  }

  return result;
}
