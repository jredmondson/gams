
#include "com_gams_variables_SensorMap.h"
#include "com_gams_variables_Sensor.h"


/*
 * Class:     com_gams_variables_SensorMap
 * Method:    jni_freeSensorMap
 * Signature: ([JI)V
 */
GAMS_Export void JNICALL Java_com_gams_variables_SensorMap_jni_1freeSensorMap
  (JNIEnv *env, jobject obj, jlongArray records, jint length)
{
  jboolean jniNoCopy = JNI_FALSE;
  jlong* nativeRecords = env->GetLongArrayElements(records, &jniNoCopy);
  jclass cls = env->GetObjectClass(obj);
  for (int x = 0; x < length; x++)
  {
    Java_com_gams_variables_Sensor_jni_1freeSensor(env, cls, nativeRecords[x]);
  }
}

