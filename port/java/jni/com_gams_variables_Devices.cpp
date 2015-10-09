#include "com_gams_variables_Devices.h"
#include "com_gams_variables_Device.h"

#include "gams/variables/Device.h"

GAMSExport void JNICALL Java_com_gams_variables_Devices_jni_1freeDevices
  (JNIEnv * env, jobject , jlongArray devices, jint length)
{
  jlong* array = env->GetLongArrayElements (devices, 
    NULL /* (jboolean*) don't care if the values were copied */);
  for (jint i = 0; i < length; ++i)
  {
    delete (gams::variables::Device *) array[i];
  }
  env->ReleaseLongArrayElements (devices, array, JNI_COMMIT);
}


