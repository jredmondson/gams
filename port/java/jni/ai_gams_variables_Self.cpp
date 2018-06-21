
#include <sstream>
#include "ai_gams_variables_Self.h"
#include "gams/variables/Self.h"
#include "gams_jni.h"

namespace containers = madara::knowledge::containers;
namespace engine = madara::knowledge;
namespace variables = gams::variables;

/*
 * Class:     ai_gams_variables_Self
 * Method:    jni_Self
 * Signature: ()J
 */
jlong JNICALL Java_ai_gams_variables_Self_jni_1Self__
  (JNIEnv *, jobject)
{
  return (jlong) new variables::Self ();
}

/*
 * Class:     ai_gams_variables_Self
 * Method:    jni_Self
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_Self_jni_1Self__J
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::Self * current = (variables::Self *) cptr;
  if (current)
  {
    result = (jlong) new variables::Self (*current);
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Self::copyConstructor: "
      "Self object is released already");
  }
  
  return result;
}

/*
 * Class:     ai_gams_variables_Self
 * Method:    jni_freeSelf
 * Signature: (J)V
 */
void JNICALL Java_ai_gams_variables_Self_jni_1freeSelf
  (JNIEnv *, jclass, jlong cptr)
{
  delete (variables::Self *) cptr;
}

/*
 * Class:     ai_gams_variables_Self
 * Method:    jni_init
 * Signature: (JJJLjava/lang/String;)V
 */
void JNICALL Java_ai_gams_variables_Self_jni_1init
  (JNIEnv * env, jobject, jlong cptr, jlong type, jlong context, jlong id)
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
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Self::init: "
      "Self object is released already");
  }
}

/*
 * Class:     ai_gams_variables_Self
 * Method:    jni_toString
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_ai_gams_variables_Self_jni_1toString
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result = 0;

  variables::Self * current = (variables::Self *) cptr;
  if (current)
  {
    std::stringstream buffer;
    buffer << "Agent";
    buffer << *(current->id);
    result = env->NewStringUTF(buffer.str ().c_str ());
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Self::toString: "
      "Self object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_Self
 * Method:    jni_getId
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_Self_jni_1getId
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::Self * current = (variables::Self *) cptr;

  if (current)
  {
    result = (jlong) &current->id;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Self::getId: "
      "Self object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_variables_Self
 * Method:    jni_getAgent
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_variables_Self_jni_1getAgent
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  variables::Self * current = (variables::Self *) cptr;

  if (current)
  {
    result = (jlong) &current->agent;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Self::getAgent: "
      "Self object is released already");
  }

  return result;
}
