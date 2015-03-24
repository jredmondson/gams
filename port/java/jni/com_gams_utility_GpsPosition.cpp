
#include "com_gams_utility_GpsPosition.h"
#include "gams/utility/GPS_Position.h"

namespace containers = Madara::Knowledge_Engine::Containers;
namespace engine = Madara::Knowledge_Engine;
namespace utility = gams::utility;

/*
 * Class:     com_gams_utility_GpsPosition
 * Method:    jni_GpsPosition
 * Signature: ()J
 */
jlong JNICALL Java_com_gams_utility_GpsPosition_jni_1GpsPosition__
  (JNIEnv * env, jobject)
{
  return (jlong) new utility::GPS_Position ();
}

/*
 * Class:     com_gams_utility_GpsPosition
 * Method:    jni_GpsPosition
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_utility_GpsPosition_jni_1GpsPosition__J
  (JNIEnv * env, jobject, jlong cptr)
{
  return (jlong) new utility::GPS_Position (*(utility::GPS_Position *)cptr);
}

/*
 * Class:     com_gams_utility_GpsPosition
 * Method:    jni_GpsPosition
 * Signature: (DDD)J
 */
GAMS_Export jlong JNICALL Java_com_gams_utility_GpsPosition_jni_1GpsPosition__DDD
  (JNIEnv * env, jobject, jdouble lat, jdouble lon, jdouble alt)
{
  return (jlong) new utility::GPS_Position (lat, lon, alt);
}

/*
 * Class:     com_gams_utility_GpsPosition
 * Method:    jni_freeGpsPosition
 * Signature: (J)V
 */
void JNICALL Java_com_gams_utility_GpsPosition_jni_1freeGpsPosition
  (JNIEnv * env, jclass, jlong cptr)
{
  delete (utility::Position *) cptr;
}

/*
 * Class:     com_gams_utility_GpsPosition
 * Method:    jni_toString
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_com_gams_utility_GpsPosition_jni_1toString
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result;

  utility::Position * current = (utility::Position *) cptr;
  if (current)
    result = env->NewStringUTF("GpsPosition");

  return result;
}

/*
 * Class:     com_gams_utility_GpsPosition
 * Method:    jni_getLatitude
 * Signature: (J)D
 */
jdouble JNICALL Java_com_gams_utility_GpsPosition_jni_1getLatitude
  (JNIEnv * env, jobject, jlong cptr)
{
  utility::Position * current = (utility::Position *) cptr;

  return (jdouble) current->x;
}

/*
 * Class:     com_gams_utility_GpsPosition
 * Method:    jni_getLongitude
 * Signature: (J)D
 */
jdouble JNICALL Java_com_gams_utility_GpsPosition_jni_1getLongitude
  (JNIEnv * env, jobject, jlong cptr)
{
  utility::Position * current = (utility::Position *) cptr;

  return (jdouble) current->y;
}

/*
 * Class:     com_gams_utility_GpsPosition
 * Method:    jni_getAltitude
 * Signature: (J)D
 */
jdouble JNICALL Java_com_gams_utility_GpsPosition_jni_1getAltitude
  (JNIEnv * env, jobject, jlong cptr)
{
  utility::Position * current = (utility::Position *) cptr;

  return (jdouble) current->z;
}

/*
 * Class:     com_gams_utility_GpsPosition
 * Method:    jni_setLatitude
 * Signature: (JD)V
 */
void JNICALL Java_com_gams_utility_GpsPosition_jni_1setLatitude
  (JNIEnv * env, jobject, jlong cptr, jdouble input)
{
  utility::Position * current = (utility::Position *) cptr;

  current->x = input;
}

/*
 * Class:     com_gams_utility_GpsPosition
 * Method:    jni_setLongitude
 * Signature: (JD)V
 */
void JNICALL Java_com_gams_utility_GpsPosition_jni_1setLongitude
  (JNIEnv * env, jobject, jlong cptr, jdouble input)
{
  utility::Position * current = (utility::Position *) cptr;

  current->y = input;
}

/*
 * Class:     com_gams_utility_GpsPosition
 * Method:    jni_setAltitude
 * Signature: (JD)V
 */
void JNICALL Java_com_gams_utility_GpsPosition_jni_1setAltitude
  (JNIEnv * env, jobject, jlong cptr, jdouble input)
{
  utility::Position * current = (utility::Position *) cptr;

  current->z = input;
}
