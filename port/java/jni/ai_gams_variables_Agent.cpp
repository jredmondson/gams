
#include "ai_gams_variables_Agent.h"
#include "gams/variables/Agent.h"
#include "gams_jni.h"

namespace containers = madara::knowledge::containers;
namespace engine = madara::knowledge;
namespace variables = gams::variables;

/*
 * Class:     ai_gams_variables_Agent
 * Method:    jni_Agent
 * Signature: ()J
 */
jlong JNICALL Java_ai_gams_variables_Agent_jni_1Agent__
  (JNIEnv *, jobject)
{
  return (jlong) new variables::Agent ();
}

/*
 * Class:     ai_gams_variables_Agent
 * Method:    jni_Agent
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_Agent_jni_1Agent__J
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::Agent * current = (variables::Agent *) cptr;
  if (current)
  {
    result = (jlong) new variables::Agent (*current);
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Agent::copyConstructor: "
      "Agent object is released already");
  }
  
  return result;
}

/*
 * Class:     ai_gams_variables_Agent
 * Method:    jni_freeAgent
 * Signature: (J)V
 */
void JNICALL Java_ai_gams_variables_Agent_jni_1freeAgent
  (JNIEnv *, jclass, jlong cptr)
{
  delete (variables::Agent *) cptr;
}

/*
 * Class:     ai_gams_variables_Agent
 * Method:    jni_init
 * Signature: (JJJLjava/lang/String;)V
 */
void JNICALL Java_ai_gams_variables_Agent_jni_1init
  (JNIEnv * env, jobject, jlong cptr, jlong type, jlong context, jlong id)
{
  variables::Agent * current = (variables::Agent *) cptr;

  if (current)
  {
    if (type == 0)
    {
      engine::KnowledgeBase * kb = (engine::KnowledgeBase *) context;
      current->init_vars (*kb, id);
    }
    else if (type == 1)
    {
      engine::Variables * vars = (engine::Variables *) context;
      current->init_vars (*vars, id);
    }
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Agent::init: "
      "Agent object is released already");
  }
}

/*
 * Class:     ai_gams_variables_Agent
 * Method:    jni_toString
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_ai_gams_variables_Agent_jni_1toString
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result = 0;

  variables::Agent * current = (variables::Agent *) cptr;
  if (current)
  {
    result = env->NewStringUTF("Agent");
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Agent::toString: "
      "Agent object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_Agent
 * Method:    jni_getBatteryRemaining
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_Agent_jni_1getBatteryRemaining
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::Agent * current = (variables::Agent *) cptr;

  if (current)
  {
    result = (jlong) &current->battery_remaining;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Agent::getBatteryRemaining: "
      "Agent object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_Agent
 * Method:    jni_getBridgeId
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_Agent_jni_1getBridgeId
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::Agent * current = (variables::Agent *) cptr;

  if (current)
  {
    result = (jlong) &current->bridge_id;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Agent::getBridgeId: "
      "Agent object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_Agent
 * Method:    jni_getCommand
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_Agent_jni_1getAlgorithm
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::Agent * current = (variables::Agent *) cptr;

  if (current)
  {
    result = (jlong) &current->algorithm;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Agent::getAlgorithm: "
      "Agent object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_Agent
 * Method:    jni_getArgs
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_Agent_jni_1getAlgorithmArgs
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::Agent * current = (variables::Agent *) cptr;

  if (current)
  {
    result = (jlong) &current->algorithm_args;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Agent::getArgs: "
      "Agent object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_Agent
 * Method:    jni_getCoverageType
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_Agent_jni_1getCoverageType
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::Agent * current = (variables::Agent *) cptr;

  if (current)
  {
    result = (jlong) &current->coverage_type;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Agent::getCoverageType: "
      "Agent object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_Agent
 * Method:    jni_getDest
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_Agent_jni_1getDest
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::Agent * current = (variables::Agent *) cptr;

  if (current)
  {
    result = (jlong) &current->dest;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Agent::getDest: "
      "Agent object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_Agent
 * Method:    jni_getHome
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_Agent_jni_1getHome
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::Agent * current = (variables::Agent *) cptr;

  if (current)
  {
    result = (jlong) &current->home;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Agent::getHome: "
      "Agent object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_Agent
 * Method:    jni_getIsMobile
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_Agent_jni_1getIsMobile
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::Agent * current = (variables::Agent *) cptr;

  if (current)
  {
    result = (jlong) &current->is_mobile;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Agent::getIsMobile: "
      "Agent object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_Agent
 * Method:    jni_getLocation
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_Agent_jni_1getLocation
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::Agent * current = (variables::Agent *) cptr;

  if (current)
  {
    result = (jlong) &current->location;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Agent::getLocation: "
      "Agent object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_Agent
 * Method:    jni_getMinAlt
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_Agent_jni_1getMinAlt
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::Agent * current = (variables::Agent *) cptr;

  if (current)
  {
    result = (jlong) &current->min_alt;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Agent::getMinAlt: "
      "Agent object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_Agent
 * Method:    jni_getNextCoverageType
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_Agent_jni_1getNextCoverageType
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::Agent * current = (variables::Agent *) cptr;

  if (current)
  {
    result = (jlong) &current->next_coverage_type;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Agent::getNextCoverageType: "
      "Agent object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_Agent
 * Method:    jni_getSearchAreaId
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_Agent_jni_1getSearchAreaId
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::Agent * current = (variables::Agent *) cptr;

  if (current)
  {
    result = (jlong) &current->search_area_id;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Agent::getSearchAreaId: "
      "Agent object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_Agent
 * Method:    jni_getSource
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_Agent_jni_1getSource
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::Agent * current = (variables::Agent *) cptr;

  if (current)
  {
    result = (jlong) &current->source;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Agent::getSource: "
      "Agent object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_Agent
 * Method:    jni_getTemperature
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_Agent_jni_1getTemperature
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::Agent * current = (variables::Agent *) cptr;

  if (current)
  {
    result = (jlong) &current->temperature;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Agent::getTemperature: "
      "Agent object is released already");
  }

  return result;
}
