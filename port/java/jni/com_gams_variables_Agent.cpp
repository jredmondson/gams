
#include "com_gams_variables_Agent.h"
#include "gams/variables/Agent.h"

namespace containers = madara::knowledge::containers;
namespace engine = madara::knowledge;
namespace variables = gams::variables;

/*
 * Class:     com_gams_variables_Agent
 * Method:    jni_Agent
 * Signature: ()J
 */
jlong JNICALL Java_com_gams_variables_Agent_jni_1Agent__
  (JNIEnv * , jobject)
{
  return (jlong) new variables::Agent ();
}

/*
 * Class:     com_gams_variables_Agent
 * Method:    jni_Agent
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Agent_jni_1Agent__J
  (JNIEnv * , jobject, jlong cptr)
{
  return (jlong) new variables::Agent (*(variables::Agent *)cptr);
}

/*
 * Class:     com_gams_variables_Agent
 * Method:    jni_freeAgent
 * Signature: (J)V
 */
void JNICALL Java_com_gams_variables_Agent_jni_1freeAgent
  (JNIEnv * , jclass, jlong cptr)
{
  delete (variables::Agent *) cptr;
}

/*
 * Class:     com_gams_variables_Agent
 * Method:    jni_init
 * Signature: (JJJLjava/lang/String;)V
 */
void JNICALL Java_com_gams_variables_Agent_jni_1init
  (JNIEnv * , jobject, jlong cptr, jlong type, jlong context, jlong id)
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
}

/*
 * Class:     com_gams_variables_Agent
 * Method:    jni_toString
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_com_gams_variables_Agent_jni_1toString
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result;

  variables::Agent * current = (variables::Agent *) cptr;
  if (current)
  {
    result = env->NewStringUTF("Agent");
  }
  else
  {
    result = env->NewStringUTF ("");
  }

  return result;
}

/*
 * Class:     com_gams_variables_Agent
 * Method:    jni_getBatteryRemaining
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Agent_jni_1getBatteryRemaining
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Agent * current = (variables::Agent *) cptr;

  return (jlong) &current->battery_remaining;
}

/*
 * Class:     com_gams_variables_Agent
 * Method:    jni_getBridgeId
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Agent_jni_1getBridgeId
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Agent * current = (variables::Agent *) cptr;

  return (jlong) &current->bridge_id;
}

/*
 * Class:     com_gams_variables_Agent
 * Method:    jni_getCommand
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Agent_jni_1getAlgorithm
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Agent * current = (variables::Agent *) cptr;

  return (jlong) &current->algorithm;
}

/*
 * Class:     com_gams_variables_Agent
 * Method:    jni_getArgs
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Agent_jni_1getAlgorithmArgs
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Agent * current = (variables::Agent *) cptr;

  return (jlong) &current->algorithm_args;
}

/*
 * Class:     com_gams_variables_Agent
 * Method:    jni_getCoverageType
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Agent_jni_1getCoverageType
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Agent * current = (variables::Agent *) cptr;

  return (jlong) &current->coverage_type;
}

/*
 * Class:     com_gams_variables_Agent
 * Method:    jni_getDest
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Agent_jni_1getDest
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Agent * current = (variables::Agent *) cptr;

  return (jlong) &current->dest;
}

/*
 * Class:     com_gams_variables_Agent
 * Method:    jni_getHome
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Agent_jni_1getHome
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Agent * current = (variables::Agent *) cptr;

  return (jlong) &current->home;
}

/*
 * Class:     com_gams_variables_Agent
 * Method:    jni_getIsMobile
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Agent_jni_1getIsMobile
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Agent * current = (variables::Agent *) cptr;

  return (jlong) &current->is_mobile;
}

/*
 * Class:     com_gams_variables_Agent
 * Method:    jni_getLocation
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Agent_jni_1getLocation
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Agent * current = (variables::Agent *) cptr;

  return (jlong) &current->location;
}

/*
 * Class:     com_gams_variables_Agent
 * Method:    jni_getMinAlt
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Agent_jni_1getMinAlt
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Agent * current = (variables::Agent *) cptr;

  return (jlong) &current->min_alt;
}

/*
 * Class:     com_gams_variables_Agent
 * Method:    jni_getNextCoverageType
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Agent_jni_1getNextCoverageType
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Agent * current = (variables::Agent *) cptr;

  return (jlong) &current->next_coverage_type;
}

/*
 * Class:     com_gams_variables_Agent
 * Method:    jni_getSearchAreaId
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Agent_jni_1getSearchAreaId
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Agent * current = (variables::Agent *) cptr;

  return (jlong) &current->search_area_id;
}

/*
 * Class:     com_gams_variables_Agent
 * Method:    jni_getSource
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Agent_jni_1getSource
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Agent * current = (variables::Agent *) cptr;

  return (jlong) &current->source;
}

/*
 * Class:     com_gams_variables_Agent
 * Method:    jni_getTemperature
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Agent_jni_1getTemperature
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Agent * current = (variables::Agent *) cptr;

  return (jlong) &current->temperature;
}
