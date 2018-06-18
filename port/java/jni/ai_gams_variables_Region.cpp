
#include "ai_gams_variables_Region.h"
#include "gams/variables/Region.h"

namespace containers = madara::knowledge::containers;
namespace engine = madara::knowledge;
namespace variables = gams::variables;

/*
 * Class:     ai_gams_variables_Region
 * Method:    jni_Region
 * Signature: ()J
 */
jlong JNICALL Java_ai_gams_variables_Region_jni_1Region__
  (JNIEnv * , jobject)
{
  return (jlong) new variables::Region ();
}

/*
 * Class:     ai_gams_variables_Region
 * Method:    jni_Region
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_Region_jni_1Region__J
  (JNIEnv * , jobject, jlong cptr)
{
  return (jlong) new variables::Region (*(variables::Region *)cptr);
}

/*
 * Class:     ai_gams_variables_Region
 * Method:    jni_freeRegion
 * Signature: (J)V
 */
void JNICALL Java_ai_gams_variables_Region_jni_1freeRegion
  (JNIEnv * , jclass, jlong cptr)
{
  delete (variables::Region *) cptr;
}

/*
 * Class:     ai_gams_variables_Region
 * Method:    jni_getName
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_ai_gams_variables_Region_jni_1getName
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result;

  variables::Region * current = (variables::Region *) cptr;
  if (current)
  {
    result = env->NewStringUTF(current->name.c_str ());
  }
  else
  {
    result = env->NewStringUTF ("");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_Region
 * Method:    jni_init
 * Signature: (JJJLjava/lang/String;)V
 */
void JNICALL Java_ai_gams_variables_Region_jni_1init
  (JNIEnv * env, jobject, jlong cptr, jlong type, jlong context, jstring name)
{
  variables::Region * current = (variables::Region *) cptr;

  if (current)
  {
    const char * str_name = env->GetStringUTFChars(name, 0);

    if (type == 0)
    {
      engine::KnowledgeBase * kb = (engine::KnowledgeBase *) context;
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
 * Class:     ai_gams_variables_Region
 * Method:    jni_toString
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_ai_gams_variables_Region_jni_1toString
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result;

  variables::Region * current = (variables::Region *) cptr;
  if (current)
  {
    result = env->NewStringUTF(current->name.c_str ());
  }
  else
  {
    result = env->NewStringUTF ("");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_Region
 * Method:    jni_getVertices
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_Region_jni_1getVertices
  (JNIEnv * , jobject, jlong cptr)
{
  variables::Region * current = (variables::Region *) cptr;

  return (jlong) &current->vertices;
}
