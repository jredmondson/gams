
#include "com_gams_utility_Position.h"
#include "gams/utility/Position.h"

namespace containers = Madara::Knowledge_Engine::Containers;
namespace engine = Madara::Knowledge_Engine;
namespace utility = gams::utility;

/*
 * Class:     com_gams_utility_Position
 * Method:    jni_Position
 * Signature: ()J
 */
jlong JNICALL Java_com_gams_utility_Position_jni_1Position__
  (JNIEnv * env, jobject)
{
  return (jlong) new utility::Position ();
}

/*
 * Class:     com_gams_utility_Position
 * Method:    jni_Position
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_utility_Position_jni_1Position__J
  (JNIEnv * env, jobject, jlong cptr)
{
  return (jlong) new utility::Position (*(utility::Position *)cptr);
}

/*
 * Class:     com_gams_utility_Position
 * Method:    jni_Position
 * Signature: (DDD)J
 */
GAMS_Export jlong JNICALL Java_com_gams_utility_Position_jni_1Position__DDD
  (JNIEnv * env, jobject, jdouble x, jdouble y, jdouble z)
{
  return (jlong) new utility::Position (x, y, z);
}

/*
 * Class:     com_gams_utility_Position
 * Method:    jni_freePosition
 * Signature: (J)V
 */
void JNICALL Java_com_gams_utility_Position_jni_1freePosition
  (JNIEnv * env, jclass, jlong cptr)
{
  delete (utility::Position *) cptr;
}

/*
 * Class:     com_gams_utility_Position
 * Method:    jni_toString
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_com_gams_utility_Position_jni_1toString
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result;

  utility::Position * current = (utility::Position *) cptr;
  if (current)
    result = env->NewStringUTF("Position");

  return result;
}

/*
 * Class:     com_gams_utility_Position
 * Method:    jni_getX
 * Signature: (J)D
 */
jdouble JNICALL Java_com_gams_utility_Position_jni_1getX
  (JNIEnv * env, jobject, jlong cptr)
{
  utility::Position * current = (utility::Position *) cptr;

  return (jdouble) current->x;
}

/*
 * Class:     com_gams_utility_Position
 * Method:    jni_getY
 * Signature: (J)D
 */
jdouble JNICALL Java_com_gams_utility_Position_jni_1getY
  (JNIEnv * env, jobject, jlong cptr)
{
  utility::Position * current = (utility::Position *) cptr;

  return (jdouble) current->y;
}

/*
 * Class:     com_gams_utility_Position
 * Method:    jni_getZ
 * Signature: (J)D
 */
jdouble JNICALL Java_com_gams_utility_Position_jni_1getZ
  (JNIEnv * env, jobject, jlong cptr)
{
  utility::Position * current = (utility::Position *) cptr;

  return (jdouble) current->z;
}

/*
 * Class:     com_gams_utility_Position
 * Method:    jni_setX
 * Signature: (JD)V
 */
void JNICALL Java_com_gams_utility_Position_jni_1setX
  (JNIEnv * env, jobject, jlong cptr, jdouble input)
{
  utility::Position * current = (utility::Position *) cptr;

  current->x = input;
}

/*
 * Class:     com_gams_utility_Position
 * Method:    jni_setY
 * Signature: (JD)V
 */
void JNICALL Java_com_gams_utility_Position_jni_1setY
  (JNIEnv * env, jobject, jlong cptr, jdouble input)
{
  utility::Position * current = (utility::Position *) cptr;

  current->y = input;
}

/*
 * Class:     com_gams_utility_Position
 * Method:    jni_setZ
 * Signature: (JD)V
 */
void JNICALL Java_com_gams_utility_Position_jni_1setZ
  (JNIEnv * env, jobject, jlong cptr, jdouble input)
{
  utility::Position * current = (utility::Position *) cptr;

  current->z = input;
}

