
#include "com_gams_variables_Algorithm.h"
#include "gams/variables/Algorithm.h"

namespace containers = Madara::Knowledge_Engine::Containers;
namespace engine = Madara::Knowledge_Engine;
namespace variables = gams::variables;

/*
 * Class:     com_gams_variables_Algorithm
 * Method:    jni_Algorithm
 * Signature: ()J
 */
jlong JNICALL Java_com_gams_variables_Algorithm_jni_1Algorithm__
  (JNIEnv * env, jobject)
{
  return (jlong) new variables::Algorithm ();
}

/*
 * Class:     com_gams_variables_Algorithm
 * Method:    jni_Algorithm
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Algorithm_jni_1Algorithm__J
  (JNIEnv * env, jobject, jlong cptr)
{
  return (jlong) new variables::Algorithm (*(variables::Algorithm *)cptr);
}

/*
 * Class:     com_gams_variables_Algorithm
 * Method:    jni_freeAlgorithm
 * Signature: (J)V
 */
void JNICALL Java_com_gams_variables_Algorithm_jni_1freeAlgorithm
  (JNIEnv * env, jclass, jlong cptr)
{
  delete (variables::Algorithm *) cptr;
}

/*
 * Class:     com_gams_variables_Algorithm
 * Method:    jni_getName
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_com_gams_variables_Algorithm_jni_1getName
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result;

  variables::Algorithm * current = (variables::Algorithm *) cptr;
  if (current)
    result = env->NewStringUTF(current->name.c_str ());

  return result;
}

/*
 * Class:     com_gams_variables_Algorithm
 * Method:    jni_init
 * Signature: (JJJLjava/lang/String;)V
 */
void JNICALL Java_com_gams_variables_Algorithm_jni_1init
  (JNIEnv * env, jobject, jlong cptr, jlong type, jlong context, jstring name)
{
  variables::Algorithm * current = (variables::Algorithm *) cptr;

  if (current)
  {
    const char * str_name = env->GetStringUTFChars(name, 0);

    if (type == 0)
    {
      engine::Knowledge_Base * kb = (engine::Knowledge_Base *) context;
      current->init_vars (*kb, str_name);
    }
    else if (type == 1)
    {
      engine::Variables * vars = (engine::Variables *) context;
      current->init_vars (*vars, str_name);
    }

    env->ReleaseStringUTFChars(name, str_name);
  }
}

/*
 * Class:     com_gams_variables_Algorithm
 * Method:    jni_toString
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_com_gams_variables_Algorithm_jni_1toString
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result;

  variables::Algorithm * current = (variables::Algorithm *) cptr;
  if (current)
    result = env->NewStringUTF(current->name.c_str ());

  return result;
}

/*
 * Class:     com_gams_variables_Algorithm
 * Method:    jni_getDeadlocked
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Algorithm_jni_1getDeadlocked
  (JNIEnv * env, jobject, jlong cptr)
{
  variables::Algorithm * current = (variables::Algorithm *) cptr;

  return (jlong) &current->deadlocked;
}

/*
 * Class:     com_gams_variables_Algorithm
 * Method:    jni_getFailed
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Algorithm_jni_1getFailed
  (JNIEnv * env, jobject, jlong cptr)
{
  variables::Algorithm * current = (variables::Algorithm *) cptr;

  return (jlong) &current->failed;
}

/*
 * Class:     com_gams_variables_Algorithm
 * Method:    jni_getOk
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Algorithm_jni_1getOk
  (JNIEnv * env, jobject, jlong cptr)
{
  variables::Algorithm * current = (variables::Algorithm *) cptr;

  return (jlong) &current->ok;
}

/*
 * Class:     com_gams_variables_Algorithm
 * Method:    jni_getPaused
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Algorithm_jni_1getPaused
  (JNIEnv * env, jobject, jlong cptr)
{
  variables::Algorithm * current = (variables::Algorithm *) cptr;

  return (jlong) &current->paused;
}

/*
 * Class:     com_gams_variables_Algorithm
 * Method:    jni_getUnknown
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Algorithm_jni_1getUnknown
  (JNIEnv * env, jobject, jlong cptr)
{
  variables::Algorithm * current = (variables::Algorithm *) cptr;

  return (jlong) &current->unknown;
}

/*
 * Class:     com_gams_variables_Algorithm
 * Method:    jni_getWaiting
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Algorithm_jni_1getWaiting
  (JNIEnv * env, jobject, jlong cptr)
{
  variables::Algorithm * current = (variables::Algorithm *) cptr;

  return (jlong) &current->waiting;
}