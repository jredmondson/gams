
#include "com_gams_variables_AccentStatus.h"
#include "gams/variables/Accent_Status.h"

namespace containers = madara::knowledge::containers;
namespace engine = madara::knowledge;
namespace variables = gams::variables;

/*
 * Class:     com_gams_variables_AccentStatus
 * Method:    jni_AccentStatus
 * Signature: ()J
 */
jlong JNICALL Java_com_gams_variables_AccentStatus_jni_1AccentStatus__
  (JNIEnv * , jobject)
{
  return (jlong) new variables::Accent_Status ();
}

/*
 * Class:     com_gams_variables_AccentStatus
 * Method:    jni_AccentStatus
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_AccentStatus_jni_1AccentStatus__J
  (JNIEnv * , jobject, jlong cptr)
{
  return (jlong) new variables::Accent_Status (
    *(variables::Accent_Status *)cptr);
}

/*
 * Class:     com_gams_variables_AccentStatus
 * Method:    jni_freeAccentStatus
 * Signature: (J)V
 */
void JNICALL Java_com_gams_variables_AccentStatus_jni_1freeAccentStatus
  (JNIEnv * , jclass, jlong cptr)
{
  delete (variables::Accent_Status *) cptr;
}

/*
 * Class:     com_gams_variables_AccentStatus
 * Method:    jni_getName
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_com_gams_variables_AccentStatus_jni_1getName
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result;

  variables::Accent_Status * current = (variables::Accent_Status *) cptr;
  if (current)
    result = env->NewStringUTF(current->command.get_name ().c_str ());

  return result;
}

/*
 * Class:     com_gams_variables_AccentStatus
 * Method:    jni_init
 * Signature: (JJJLjava/lang/String;)V
 */
void JNICALL Java_com_gams_variables_AccentStatus_jni_1init
  (JNIEnv * env, jobject, jlong cptr, jlong type, jlong context, jstring name)
{
  variables::Accent_Status * current = (variables::Accent_Status *) cptr;

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
 * Class:     com_gams_variables_AccentStatus
 * Method:    jni_toString
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_com_gams_variables_AccentStatus_jni_1toString
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result;

  variables::Accent_Status * current = (variables::Accent_Status *) cptr;
  if (current)
    result = env->NewStringUTF(current->command.get_name ().c_str ());

  return result;
}

/*
 * Class:     com_gams_variables_AccentStatus
 * Method:    jni_getArgs
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_AccentStatus_jni_1getArgs
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Accent_Status * current = (variables::Accent_Status *) cptr;

  return (jlong) &current->command_args;
}

/*
 * Class:     com_gams_variables_AccentStatus
 * Method:    jni_getCommand
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_AccentStatus_jni_1getCommand
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Accent_Status * current = (variables::Accent_Status *) cptr;

  return (jlong) &current->command;
}
