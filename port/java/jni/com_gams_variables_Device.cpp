
#include "com_gams_variables_Device.h"
#include "gams/variables/Device.h"

namespace containers = madara::knowledge::containers;
namespace engine = madara::knowledge;
namespace variables = gams::variables;

/*
 * Class:     com_gams_variables_Device
 * Method:    jni_Device
 * Signature: ()J
 */
jlong JNICALL Java_com_gams_variables_Device_jni_1Device__
  (JNIEnv * , jobject)
{
  return (jlong) new variables::Device ();
}

/*
 * Class:     com_gams_variables_Device
 * Method:    jni_Device
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Device_jni_1Device__J
  (JNIEnv * , jobject, jlong cptr)
{
  return (jlong) new variables::Device (*(variables::Device *)cptr);
}

/*
 * Class:     com_gams_variables_Device
 * Method:    jni_freeDevice
 * Signature: (J)V
 */
void JNICALL Java_com_gams_variables_Device_jni_1freeDevice
  (JNIEnv * , jclass, jlong cptr)
{
  delete (variables::Device *) cptr;
}

/*
 * Class:     com_gams_variables_Device
 * Method:    jni_init
 * Signature: (JJJLjava/lang/String;)V
 */
void JNICALL Java_com_gams_variables_Device_jni_1init
  (JNIEnv * , jobject, jlong cptr, jlong type, jlong context, jlong id)
{
  variables::Device * current = (variables::Device *) cptr;

  if (current)
  {
    if (type == 0)
    {
      engine::Knowledge_Base * kb = (engine::Knowledge_Base *) context;
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
 * Class:     com_gams_variables_Device
 * Method:    jni_toString
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_com_gams_variables_Device_jni_1toString
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result;

  variables::Device * current = (variables::Device *) cptr;
  if (current)
    result = env->NewStringUTF("Device");

  return result;
}

/*
 * Class:     com_gams_variables_Device
 * Method:    jni_getBatteryRemaining
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Device_jni_1getBatteryRemaining
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Device * current = (variables::Device *) cptr;

  return (jlong) &current->battery_remaining;
}

/*
 * Class:     com_gams_variables_Device
 * Method:    jni_getBridgeId
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Device_jni_1getBridgeId
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Device * current = (variables::Device *) cptr;

  return (jlong) &current->bridge_id;
}

/*
 * Class:     com_gams_variables_Device
 * Method:    jni_getCommand
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Device_jni_1getCommand
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Device * current = (variables::Device *) cptr;

  return (jlong) &current->command;
}

/*
 * Class:     com_gams_variables_Device
 * Method:    jni_getArgs
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Device_jni_1getArgs
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Device * current = (variables::Device *) cptr;

  return (jlong) &current->command_args;
}

/*
 * Class:     com_gams_variables_Device
 * Method:    jni_getCoverageType
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Device_jni_1getCoverageType
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Device * current = (variables::Device *) cptr;

  return (jlong) &current->coverage_type;
}

/*
 * Class:     com_gams_variables_Device
 * Method:    jni_getDest
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Device_jni_1getDest
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Device * current = (variables::Device *) cptr;

  return (jlong) &current->dest;
}

/*
 * Class:     com_gams_variables_Device
 * Method:    jni_getHome
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Device_jni_1getHome
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Device * current = (variables::Device *) cptr;

  return (jlong) &current->home;
}

/*
 * Class:     com_gams_variables_Device
 * Method:    jni_getIsMobile
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Device_jni_1getIsMobile
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Device * current = (variables::Device *) cptr;

  return (jlong) &current->is_mobile;
}

/*
 * Class:     com_gams_variables_Device
 * Method:    jni_getLocation
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Device_jni_1getLocation
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Device * current = (variables::Device *) cptr;

  return (jlong) &current->location;
}

/*
 * Class:     com_gams_variables_Device
 * Method:    jni_getMinAlt
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Device_jni_1getMinAlt
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Device * current = (variables::Device *) cptr;

  return (jlong) &current->min_alt;
}

/*
 * Class:     com_gams_variables_Device
 * Method:    jni_getNextCoverageType
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Device_jni_1getNextCoverageType
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Device * current = (variables::Device *) cptr;

  return (jlong) &current->next_coverage_type;
}

/*
 * Class:     com_gams_variables_Device
 * Method:    jni_getSearchAreaId
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Device_jni_1getSearchAreaId
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Device * current = (variables::Device *) cptr;

  return (jlong) &current->search_area_id;
}

/*
 * Class:     com_gams_variables_Device
 * Method:    jni_getSource
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Device_jni_1getSource
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Device * current = (variables::Device *) cptr;

  return (jlong) &current->source;
}

/*
 * Class:     com_gams_variables_Device
 * Method:    jni_getTemperature
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Device_jni_1getTemperature
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Device * current = (variables::Device *) cptr;

  return (jlong) &current->temperature;
}
