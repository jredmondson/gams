
#include "ai_gams_variables_SensorMap.h"
#include "ai_gams_variables_Sensor.h"
#include "gams_jni.h"


/*
 * Class:     ai_gams_variables_SensorMap
 * Method:    jni_freeSensorMap
 * Signature: ([JI)V
 */
void JNICALL Java_ai_gams_variables_SensorMap_jni_1freeSensorMap
  (JNIEnv *env, jobject obj, jlongArray records, jint length)
{
  jboolean jniNoCopy = JNI_FALSE;
  jlong* nativeRecords = env->GetLongArrayElements(records, &jniNoCopy);
  jclass cls = env->GetObjectClass(obj);
  for (int x = 0; x < length; x++)
  {
    Java_ai_gams_variables_Sensor_jni_1freeSensor(env, cls, nativeRecords[x]);
  }
  env->DeleteLocalRef (cls);
}

