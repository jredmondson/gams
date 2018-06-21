#include "ai_gams_variables_Agents.h"
#include "ai_gams_variables_Agent.h"

#include "gams/variables/Agent.h"
#include "gams_jni.h"

void JNICALL Java_ai_gams_variables_Agents_jni_1freeAgents
  (JNIEnv * env, jobject , jlongArray agents, jint length)
{
  jlong* array = env->GetLongArrayElements (agents, 
    NULL /* (jboolean*) don't care if the values were copied */);
  for (jint i = 0; i < length; ++i)
  {
    delete (gams::variables::Agent *) array[i];
  }
  env->ReleaseLongArrayElements (agents, array, JNI_COMMIT);
}


