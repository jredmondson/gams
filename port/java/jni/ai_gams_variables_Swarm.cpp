
#include "ai_gams_variables_Swarm.h"
#include "gams/variables/Swarm.h"
#include "gams_jni.h"

namespace containers = madara::knowledge::containers;
namespace engine = madara::knowledge;
namespace variables = gams::variables;

/*
 * Class:     ai_gams_variables_Swarm
 * Method:    jni_Swarm
 * Signature: ()J
 */
jlong JNICALL Java_ai_gams_variables_Swarm_jni_1Swarm__
  (JNIEnv *, jobject)
{
  return (jlong) new variables::Swarm ();
}

/*
 * Class:     ai_gams_variables_Swarm
 * Method:    jni_Swarm
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_Swarm_jni_1Swarm__J
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::Swarm * current = (variables::Swarm *) cptr;
  if (current)
  {
    result = (jlong) new variables::Swarm (*current);
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Swarm::copyConstructor: "
      "Swarm object is released already");
  }
  
  return result;
}

/*
 * Class:     ai_gams_variables_Swarm
 * Method:    jni_freeSwarm
 * Signature: (J)V
 */
void JNICALL Java_ai_gams_variables_Swarm_jni_1freeSwarm
  (JNIEnv *, jclass, jlong cptr)
{
  delete (variables::Swarm *) cptr;
}

/*
 * Class:     ai_gams_variables_Swarm
 * Method:    jni_init
 * Signature: (JJJLjava/lang/String;)V
 */
void JNICALL Java_ai_gams_variables_Swarm_jni_1init
  (JNIEnv * env, jobject, jlong cptr, jlong type, jlong context, jstring name)
{
  variables::Swarm * current = (variables::Swarm *) cptr;

  if (current)
  {
    const char * str_name = env->GetStringUTFChars(name, 0);

    if (type == 0)
    {
      engine::KnowledgeBase * kb = (engine::KnowledgeBase *) context;
      current->init_vars (*kb);
    }

    env->ReleaseStringUTFChars(name, str_name);
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Swarm::init: "
      "Swarm object is released already");
  }
}

/*
 * Class:     ai_gams_variables_Swarm
 * Method:    jni_toString
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_ai_gams_variables_Swarm_jni_1toString
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result = 0;

  variables::Swarm * current = (variables::Swarm *) cptr;
  if (current)
  {
    result = env->NewStringUTF("Swarm");
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Swarm::toString: "
      "Swarm object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_Swarm
 * Method:    jni_getCommand
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_Swarm_jni_1getCommand
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::Swarm * current = (variables::Swarm *) cptr;

  if (current)
  {
    result = (jlong) &current->algorithm;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Swarm::getCommand: "
      "Swarm object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_Swarm
 * Method:    jni_getArgs
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_Swarm_jni_1getArgs
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::Swarm * current = (variables::Swarm *) cptr;

  if (current)
  {
    result = (jlong) &current->algorithm_args;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Swarm::getArgs: "
      "Swarm object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_Swarm
 * Method:    jni_getMinAlt
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_Swarm_jni_1getMinAlt
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::Swarm * current = (variables::Swarm *) cptr;

  if (current)
  {
    result = (jlong) &current->min_alt;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Swarm::getMinAlt: "
      "Swarm object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_Swarm
 * Method:    jni_getSize
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_Swarm_jni_1getSize
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::Swarm * current = (variables::Swarm *) cptr;

  if (current)
  {
    result = (jlong) &current->size;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Swarm::getSize: "
      "Swarm object is released already");
  }

  return result;
}
