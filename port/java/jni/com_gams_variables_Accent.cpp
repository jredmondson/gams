
#include "com_gams_variables_Accent.h"
#include "gams/variables/Accent.h"

namespace containers = Madara::Knowledge_Engine::Containers;
namespace engine = Madara::Knowledge_Engine;
namespace variables = gams::variables;

/*
 * Class:     com_gams_variables_Accent
 * Method:    jni_Accent
 * Signature: ()J
 */
jlong JNICALL Java_com_gams_variables_Accent_jni_1Accent__
  (JNIEnv * env, jobject)
{
  return (jlong) new variables::Accent ();
}

/*
 * Class:     com_gams_variables_Accent
 * Method:    jni_Accent
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Accent_jni_1Accent__J
  (JNIEnv * env, jobject, jlong cptr)
{
  return (jlong) new variables::Accent (*(variables::Accent *)cptr);
}

/*
 * Class:     com_gams_variables_Accent
 * Method:    jni_freeAccent
 * Signature: (J)V
 */
void JNICALL Java_com_gams_variables_Accent_jni_1freeAccent
  (JNIEnv * env, jclass, jlong cptr)
{
  delete (variables::Accent *) cptr;
}

/*
 * Class:     com_gams_variables_Accent
 * Method:    jni_getName
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_com_gams_variables_Accent_jni_1getName
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result;

  variables::Accent * current = (variables::Accent *) cptr;
  if (current)
    result = env->NewStringUTF(current->command.get_name ().c_str ());

  return result;
}

/*
 * Class:     com_gams_variables_Accent
 * Method:    jni_init
 * Signature: (JJJLjava/lang/String;)V
 */
void JNICALL Java_com_gams_variables_Accent_jni_1init
  (JNIEnv * env, jobject, jlong cptr, jlong type, jlong context, jstring name)
{
  variables::Accent * current = (variables::Accent *) cptr;

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
 * Class:     com_gams_variables_Accent
 * Method:    jni_toString
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_com_gams_variables_Accent_jni_1toString
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result;

  variables::Accent * current = (variables::Accent *) cptr;
  if (current)
    result = env->NewStringUTF(current->command.get_name ().c_str ());

  return result;
}

/*
 * Class:     com_gams_variables_Accent
 * Method:    jni_getArgs
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Accent_jni_1getArgs
  (JNIEnv * env, jobject, jlong cptr)
{
  variables::Accent * current = (variables::Accent *) cptr;

  return (jlong) &current->command_args;
}

/*
 * Class:     com_gams_variables_Accent
 * Method:    jni_getCommand
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Accent_jni_1getCommand
  (JNIEnv * env, jobject, jlong cptr)
{
  variables::Accent * current = (variables::Accent *) cptr;

  return (jlong) &current->command;
}