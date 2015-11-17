
#include <sstream>
#include "com_gams_variables_Self.h"
#include "gams/variables/Self.h"

namespace containers = madara::knowledge::containers;
namespace engine = madara::knowledge;
namespace variables = gams::variables;

/*
 * Class:     com_gams_variables_Self
 * Method:    jni_Self
 * Signature: ()J
 */
jlong JNICALL Java_com_gams_variables_Self_jni_1Self__
  (JNIEnv * , jobject)
{
  return (jlong) new variables::Self ();
}

/*
 * Class:     com_gams_variables_Self
 * Method:    jni_Self
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Self_jni_1Self__J
  (JNIEnv * , jobject, jlong cptr)
{
  return (jlong) new variables::Self (*(variables::Self *)cptr);
}

/*
 * Class:     com_gams_variables_Self
 * Method:    jni_freeSelf
 * Signature: (J)V
 */
void JNICALL Java_com_gams_variables_Self_jni_1freeSelf
  (JNIEnv * , jclass, jlong cptr)
{
  delete (variables::Self *) cptr;
}

/*
 * Class:     com_gams_variables_Self
 * Method:    jni_init
 * Signature: (JJJLjava/lang/String;)V
 */
void JNICALL Java_com_gams_variables_Self_jni_1init
  (JNIEnv * , jobject, jlong cptr, jlong type, jlong context, jlong id)
{
  variables::Self * current = (variables::Self *) cptr;

  if (current)
  {
    if (type == 0)
    {
      engine::KnowledgeBase * kb = (engine::KnowledgeBase *) context;
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
    buffer << "Agent";
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
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Self * current = (variables::Self *) cptr;

  return (jlong) &current->id;
}

/*
 * Class:     com_gams_variables_Self
 * Method:    jni_getAgent
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_variables_Self_jni_1getAgent
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Self * current = (variables::Self *) cptr;

  return (jlong) &current->agent;
}
