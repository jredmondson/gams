
#include "com_gams_variables_AlgorithmStatus.h"
#include "gams/variables/AlgorithmStatus.h"

namespace containers = madara::knowledge::containers;
namespace engine = madara::knowledge;
namespace variables = gams::variables;

/*
 * Class:     com_gams_variables_AlgorithmStatus
 * Method:    jni_AlgorithmStatus
 * Signature: ()J
 */
jlong JNICALL Java_com_gams_variables_AlgorithmStatus_jni_1AlgorithmStatus__
  (JNIEnv * , jobject)
{
  return (jlong) new variables::AlgorithmStatus ();
}

/*
 * Class:     com_gams_variables_AlgorithmStatus
 * Method:    jni_AlgorithmStatus
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_AlgorithmStatus_jni_1AlgorithmStatus__J
  (JNIEnv * , jobject, jlong cptr)
{
  return (jlong) new variables::AlgorithmStatus (
    *(variables::AlgorithmStatus *)cptr);
}

/*
 * Class:     com_gams_variables_AlgorithmStatus
 * Method:    jni_freeAlgorithmStatus
 * Signature: (J)V
 */
void JNICALL Java_com_gams_variables_AlgorithmStatus_jni_1freeAlgorithmStatus
  (JNIEnv * , jclass, jlong cptr)
{
  delete (variables::AlgorithmStatus *) cptr;
}

/*
 * Class:     com_gams_variables_AlgorithmStatus
 * Method:    jni_getName
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_com_gams_variables_AlgorithmStatus_jni_1getName
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result;

  variables::AlgorithmStatus * current = (variables::AlgorithmStatus *) cptr;
  if (current)
    result = env->NewStringUTF(current->name.c_str ());

  return result;
}

/*
 * Class:     com_gams_variables_AlgorithmStatus
 * Method:    jni_init
 * Signature: (JJJLjava/lang/String;I)V
 */
void JNICALL Java_com_gams_variables_AlgorithmStatus_jni_1init
  (JNIEnv * env, jobject, jlong cptr, jlong type, jlong context, jstring name, jint i)
{
  variables::AlgorithmStatus * current = (variables::AlgorithmStatus *) cptr;

  if (current)
  {
    const char * str_name = env->GetStringUTFChars(name, 0);

    if (type == 0)
    {
      engine::KnowledgeBase * kb = (engine::KnowledgeBase *) context;
      current->init_vars (*kb, str_name, (int)i);
    }
    else if (type == 1)
    {
      engine::Variables * vars = (engine::Variables *) context;
      current->init_vars (*vars, str_name, (int)i);
    }

    env->ReleaseStringUTFChars(name, str_name);
  }
}

/*
 * Class:     com_gams_variables_AlgorithmStatus
 * Method:    jni_toString
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_com_gams_variables_AlgorithmStatus_jni_1toString
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result;

  variables::AlgorithmStatus * current = (variables::AlgorithmStatus *) cptr;
  if (current)
    result = env->NewStringUTF(current->name.c_str ());

  return result;
}

/*
 * Class:     com_gams_variables_AlgorithmStatus
 * Method:    jni_getDeadlocked
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_AlgorithmStatus_jni_1getDeadlocked
  (JNIEnv * , jobject, jlong cptr)
{
  variables::AlgorithmStatus * current = (variables::AlgorithmStatus *) cptr;

  return (jlong) &current->deadlocked;
}

/*
 * Class:     com_gams_variables_AlgorithmStatus
 * Method:    jni_getFailed
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_AlgorithmStatus_jni_1getFailed
  (JNIEnv * , jobject, jlong cptr)
{
  variables::AlgorithmStatus * current = (variables::AlgorithmStatus *) cptr;

  return (jlong) &current->failed;
}

/*
 * Class:     com_gams_variables_AlgorithmStatus
 * Method:    jni_getOk
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_AlgorithmStatus_jni_1getOk
  (JNIEnv * , jobject, jlong cptr)
{
  variables::AlgorithmStatus * current = (variables::AlgorithmStatus *) cptr;

  return (jlong) &current->ok;
}

/*
 * Class:     com_gams_variables_AlgorithmStatus
 * Method:    jni_getPaused
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_AlgorithmStatus_jni_1getPaused
  (JNIEnv * , jobject, jlong cptr)
{
  variables::AlgorithmStatus * current = (variables::AlgorithmStatus *) cptr;

  return (jlong) &current->paused;
}

/*
 * Class:     com_gams_variables_AlgorithmStatus
 * Method:    jni_getUnknown
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_AlgorithmStatus_jni_1getUnknown
  (JNIEnv * , jobject, jlong cptr)
{
  variables::AlgorithmStatus * current = (variables::AlgorithmStatus *) cptr;

  return (jlong) &current->unknown;
}

/*
 * Class:     com_gams_variables_AlgorithmStatus
 * Method:    jni_getWaiting
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_AlgorithmStatus_jni_1getWaiting
  (JNIEnv * , jobject, jlong cptr)
{
  variables::AlgorithmStatus * current = (variables::AlgorithmStatus *) cptr;

  return (jlong) &current->waiting;
}

/*
 * Class:     com_gams_variables_AlgorithmStatus
 * Method:    jni_getFinished
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_AlgorithmStatus_jni_1getFinished
  (JNIEnv *, jobject, jlong cptr)
{
  variables::AlgorithmStatus * current = (variables::AlgorithmStatus *) cptr;

  return (jlong) &current->finished;
}
