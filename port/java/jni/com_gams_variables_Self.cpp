
#include <sstream>
#include "com_gams_variables_Self.h"
#include "gams/variables/Self.h"

namespace containers = Madara::Knowledge_Engine::Containers;
namespace engine = Madara::Knowledge_Engine;
namespace variables = gams::variables;

/*
 * Class:     com_gams_variables_Self
 * Method:    jni_Self
 * Signature: ()J
 */
jlong JNICALL Java_com_gams_variables_Self_jni_1Self__
  (JNIEnv * env, jobject)
{
  return (jlong) new variables::Self ();
}

/*
 * Class:     com_gams_variables_Self
 * Method:    jni_Self
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Self_jni_1Self__J
  (JNIEnv * env, jobject, jlong cptr)
{
  return (jlong) new variables::Self (*(variables::Self *)cptr);
}

/*
 * Class:     com_gams_variables_Self
 * Method:    jni_freeSelf
 * Signature: (J)V
 */
void JNICALL Java_com_gams_variables_Self_jni_1freeSelf
  (JNIEnv * env, jclass, jlong cptr)
{
  delete (variables::Self *) cptr;
}

/*
 * Class:     com_gams_variables_Self
 * Method:    jni_init
 * Signature: (JJJLjava/lang/String;)V
 */
void JNICALL Java_com_gams_variables_Self_jni_1init
  (JNIEnv * env, jobject, jlong cptr, jlong type, jlong context, jlong id)
{
  variables::Self * current = (variables::Self *) cptr;

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
 * Class:     com_gams_variables_Self
 * Method:    jni_toString
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_com_gams_variables_Self_jni_1toString
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result;

  variables::Self * current = (variables::Self *) cptr;
  if (current)
  {
    std::stringstream buffer;
    buffer << "Device";
    buffer << *(current->id);
    result = env->NewStringUTF(buffer.str ().c_str ());
  }

  return result;
}

/*
 * Class:     com_gams_variables_Self
 * Method:    jni_getId
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Self_jni_1getId
  (JNIEnv * env, jobject, jlong cptr)
{
  variables::Self * current = (variables::Self *) cptr;

  return (jlong) &current->id;
}

/*
 * Class:     com_gams_variables_Self
 * Method:    jni_getDevice
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Self_jni_1getDevice
  (JNIEnv * env, jobject, jlong cptr)
{
  variables::Self * current = (variables::Self *) cptr;

  return (jlong) &current->device;
}
