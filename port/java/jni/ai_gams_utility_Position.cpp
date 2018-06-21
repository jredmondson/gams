
#include "ai_gams_utility_Position.h"
#include "gams/utility/Position.h"
#include "gams_jni.h"

namespace containers = madara::knowledge::containers;
namespace engine = madara::knowledge;
namespace utility = gams::utility;

/*
 * Class:     ai_gams_utility_Position
 * Method:    jni_Position
 * Signature: ()J
 */
jlong JNICALL Java_ai_gams_utility_Position_jni_1Position__
  (JNIEnv *, jobject)
{
  return (jlong) new utility::Position ();
}

/*
 * Class:     ai_gams_utility_Position
 * Method:    jni_Position
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_utility_Position_jni_1Position__J
  (JNIEnv *, jobject, jlong cptr)
{
  return (jlong) new utility::Position (*(utility::Position *)cptr);
}

/*
 * Class:     ai_gams_utility_Position
 * Method:    jni_Position
 * Signature: (DDD)J
 */
jlong JNICALL Java_ai_gams_utility_Position_jni_1Position__DDD
  (JNIEnv *, jobject, jdouble x, jdouble y, jdouble z)
{
  return (jlong) new utility::Position (x, y, z);
}

/*
 * Class:     ai_gams_utility_Position
 * Method:    jni_freePosition
 * Signature: (J)V
 */
void JNICALL Java_ai_gams_utility_Position_jni_1freePosition
  (JNIEnv *, jclass, jlong cptr)
{
  delete (utility::Position *) cptr;
}

/*
 * Class:     ai_gams_utility_Position
 * Method:    jni_toString
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_ai_gams_utility_Position_jni_1toString
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result = 0;

  utility::Position * current = (utility::Position *) cptr;
  if (current)
  {
    result = env->NewStringUTF(current->to_string ().c_str ());
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Position::toString: "
      "Position object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_Position
 * Method:    jni_getX
 * Signature: (J)D
 */
jdouble JNICALL Java_ai_gams_utility_Position_jni_1getX
  (JNIEnv * env, jobject, jlong cptr)
{
  jdouble result = 0.0;

  utility::Position * current = (utility::Position *) cptr;

  if (current)
  {
    result = current->x;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Position::getX: "
      "Position object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_Position
 * Method:    jni_getY
 * Signature: (J)D
 */
jdouble JNICALL Java_ai_gams_utility_Position_jni_1getY
  (JNIEnv * env, jobject, jlong cptr)
{
  jdouble result = 0.0;

  utility::Position * current = (utility::Position *) cptr;

  if (current)
  {
    result = current->y;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Position::getY: "
      "Position object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_Position
 * Method:    jni_getZ
 * Signature: (J)D
 */
jdouble JNICALL Java_ai_gams_utility_Position_jni_1getZ
  (JNIEnv * env, jobject, jlong cptr)
{
  jdouble result = 0.0;

  utility::Position * current = (utility::Position *) cptr;

  if (current)
  {
    result = current->z;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Position::getZ: "
      "Position object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_Position
 * Method:    jni_setX
 * Signature: (JD)V
 */
void JNICALL Java_ai_gams_utility_Position_jni_1setX
  (JNIEnv * env, jobject, jlong cptr, jdouble input)
{
  utility::Position * current = (utility::Position *) cptr;

  if (current)
  {
    current->x = input;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Position::setX: "
      "Position object is released already");
  }
}

/*
 * Class:     ai_gams_utility_Position
 * Method:    jni_setY
 * Signature: (JD)V
 */
void JNICALL Java_ai_gams_utility_Position_jni_1setY
  (JNIEnv * env, jobject, jlong cptr, jdouble input)
{
  utility::Position * current = (utility::Position *) cptr;

  if (current)
  {
    current->x = input;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Position::setY: "
      "Position object is released already");
  }
}

/*
 * Class:     ai_gams_utility_Position
 * Method:    jni_setZ
 * Signature: (JD)V
 */
void JNICALL Java_ai_gams_utility_Position_jni_1setZ
  (JNIEnv * env, jobject, jlong cptr, jdouble input)
{
  utility::Position * current = (utility::Position *) cptr;

  if (current)
  {
    current->z = input;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Position::setZ: "
      "Position object is released already");
  }
}

