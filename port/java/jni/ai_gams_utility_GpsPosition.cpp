
#include "ai_gams_utility_GpsPosition.h"
#include "gams/utility/GPSPosition.h"

namespace containers = madara::knowledge::containers;
namespace engine = madara::knowledge;
namespace utility = gams::utility;

/*
 * Class:     ai_gams_utility_GpsPosition
 * Method:    jni_GpsPosition
 * Signature: ()J
 */
jlong JNICALL Java_ai_gams_utility_GpsPosition_jni_1GpsPosition__
  (JNIEnv * , jobject)
{
  return (jlong) new utility::GPSPosition ();
}

/*
 * Class:     ai_gams_utility_GpsPosition
 * Method:    jni_GpsPosition
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_utility_GpsPosition_jni_1GpsPosition__J
  (JNIEnv * , jobject, jlong cptr)
{
  return (jlong) new utility::GPSPosition (*(utility::GPSPosition *)cptr);
}

/*
 * Class:     ai_gams_utility_GpsPosition
 * Method:    jni_GpsPosition
 * Signature: (DDD)J
 */
jlong JNICALL Java_ai_gams_utility_GpsPosition_jni_1GpsPosition__DDD
  (JNIEnv * , jobject, jdouble lat, jdouble lon, jdouble alt)
{
  return (jlong) new utility::GPSPosition (lat, lon, alt);
}

/*
 * Class:     ai_gams_utility_GpsPosition
 * Method:    jni_freeGpsPosition
 * Signature: (J)V
 */
void JNICALL Java_ai_gams_utility_GpsPosition_jni_1freeGpsPosition
  (JNIEnv * , jclass, jlong cptr)
{
  delete (utility::Position *) cptr;
}

/*
 * Class:     ai_gams_utility_GpsPosition
 * Method:    jni_toString
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_ai_gams_utility_GpsPosition_jni_1toString
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result;

  utility::GPSPosition * current = (utility::GPSPosition *) cptr;
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
 * Class:     ai_gams_utility_GpsPosition
 * Method:    jni_getLatitude
 * Signature: (J)D
 */
jdouble JNICALL Java_ai_gams_utility_GpsPosition_jni_1getLatitude
  (JNIEnv * , jobject, jlong cptr)
{
  utility::Position * current = (utility::Position *) cptr;

  return (jdouble) current->x;
}

/*
 * Class:     ai_gams_utility_GpsPosition
 * Method:    jni_getLongitude
 * Signature: (J)D
 */
jdouble JNICALL Java_ai_gams_utility_GpsPosition_jni_1getLongitude
  (JNIEnv * , jobject, jlong cptr)
{
  utility::Position * current = (utility::Position *) cptr;

  return (jdouble) current->y;
}

/*
 * Class:     ai_gams_utility_GpsPosition
 * Method:    jni_getAltitude
 * Signature: (J)D
 */
jdouble JNICALL Java_ai_gams_utility_GpsPosition_jni_1getAltitude
  (JNIEnv * , jobject, jlong cptr)
{
  utility::Position * current = (utility::Position *) cptr;

  return (jdouble) current->z;
}

/*
 * Class:     ai_gams_utility_GpsPosition
 * Method:    jni_setLatitude
 * Signature: (JD)V
 */
void JNICALL Java_ai_gams_utility_GpsPosition_jni_1setLatitude
  (JNIEnv * , jobject, jlong cptr, jdouble input)
{
  utility::Position * current = (utility::Position *) cptr;

  current->x = input;
}

/*
 * Class:     ai_gams_utility_GpsPosition
 * Method:    jni_setLongitude
 * Signature: (JD)V
 */
void JNICALL Java_ai_gams_utility_GpsPosition_jni_1setLongitude
  (JNIEnv * , jobject, jlong cptr, jdouble input)
{
  utility::Position * current = (utility::Position *) cptr;

  current->y = input;
}

/*
 * Class:     ai_gams_utility_GpsPosition
 * Method:    jni_setAltitude
 * Signature: (JD)V
 */
void JNICALL Java_ai_gams_utility_GpsPosition_jni_1setAltitude
  (JNIEnv * , jobject, jlong cptr, jdouble input)
{
  utility::Position * current = (utility::Position *) cptr;

  current->z = input;
}
