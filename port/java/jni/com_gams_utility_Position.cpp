
#include "com_gams_utility_Position.h"
#include "gams/utility/Position.h"

namespace containers = madara::knowledge::containers;
namespace engine = madara::knowledge;
namespace utility = gams::utility;

/*
 * Class:     com_gams_utility_Position
 * Method:    jni_Position
 * Signature: ()J
 */
jlong JNICALL Java_com_gams_utility_Position_jni_1Position__
  (JNIEnv * , jobject)
{
  return (jlong) new utility::Position ();
}

/*
 * Class:     com_gams_utility_Position
 * Method:    jni_Position
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_utility_Position_jni_1Position__J
  (JNIEnv * , jobject, jlong cptr)
{
  return (jlong) new utility::Position (*(utility::Position *)cptr);
}

/*
 * Class:     com_gams_utility_Position
 * Method:    jni_Position
 * Signature: (DDD)J
 */
jlong JNICALL Java_com_gams_utility_Position_jni_1Position__DDD
  (JNIEnv * , jobject, jdouble x, jdouble y, jdouble z)
{
  return (jlong) new utility::Position (x, y, z);
}

/*
 * Class:     com_gams_utility_Position
 * Method:    jni_freePosition
 * Signature: (J)V
 */
void JNICALL Java_com_gams_utility_Position_jni_1freePosition
  (JNIEnv * , jclass, jlong cptr)
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
  {
    result = env->NewStringUTF(current->to_string ().c_str ());
  }
  else
  {
    result = env->NewStringUTF ("");
  }

  return result;
}

/*
 * Class:     com_gams_utility_Position
 * Method:    jni_getX
 * Signature: (J)D
 */
jdouble JNICALL Java_com_gams_utility_Position_jni_1getX
  (JNIEnv * , jobject, jlong cptr)
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
  (JNIEnv * , jobject, jlong cptr)
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
  (JNIEnv * , jobject, jlong cptr)
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
  (JNIEnv * , jobject, jlong cptr, jdouble input)
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
  (JNIEnv * , jobject, jlong cptr, jdouble input)
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
  (JNIEnv * , jobject, jlong cptr, jdouble input)
{
  utility::Position * current = (utility::Position *) cptr;

  current->z = input;
}

