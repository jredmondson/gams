
#include "ai_gams_utility_GpsPosition.h"
#include "gams/utility/GPSPosition.h"
#include "gams_jni.h"

namespace containers = madara::knowledge::containers;
namespace engine = madara::knowledge;
namespace utility = gams::utility;

/*
 * Class:     ai_gams_utility_GpsPosition
 * Method:    jni_GpsPosition
 * Signature: ()J
 */
jlong JNICALL Java_ai_gams_utility_GpsPosition_jni_1GpsPosition__
  (JNIEnv *, jobject)
{
  return (jlong) new utility::GPSPosition ();
}

/*
 * Class:     ai_gams_utility_GpsPosition
 * Method:    jni_GpsPosition
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_utility_GpsPosition_jni_1GpsPosition__J
  (JNIEnv *, jobject, jlong cptr)
{
  return (jlong) new utility::GPSPosition (*(utility::GPSPosition *)cptr);
}

/*
 * Class:     ai_gams_utility_GpsPosition
 * Method:    jni_GpsPosition
 * Signature: (DDD)J
 */
jlong JNICALL Java_ai_gams_utility_GpsPosition_jni_1GpsPosition__DDD
  (JNIEnv *, jobject, jdouble lat, jdouble lon, jdouble alt)
{
  return (jlong) new utility::GPSPosition (lat, lon, alt);
}

/*
 * Class:     ai_gams_utility_GpsPosition
 * Method:    jni_freeGpsPosition
 * Signature: (J)V
 */
void JNICALL Java_ai_gams_utility_GpsPosition_jni_1freeGpsPosition
  (JNIEnv *, jclass, jlong cptr)
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
  jstring result = 0;

  utility::GPSPosition * current = (utility::GPSPosition *) cptr;
  if (current)
  {
    result = env->NewStringUTF(current->to_string ().c_str ());
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "GpsPosition::toString: "
      "GpsPosition object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_GpsPosition
 * Method:    jni_getLatitude
 * Signature: (J)D
 */
jdouble JNICALL Java_ai_gams_utility_GpsPosition_jni_1getLatitude
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
      "GpsPosition::getLatitude: "
      "GpsPosition object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_GpsPosition
 * Method:    jni_getLongitude
 * Signature: (J)D
 */
jdouble JNICALL Java_ai_gams_utility_GpsPosition_jni_1getLongitude
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
      "GpsPosition::getLongitude: "
      "GpsPosition object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_GpsPosition
 * Method:    jni_getAltitude
 * Signature: (J)D
 */
jdouble JNICALL Java_ai_gams_utility_GpsPosition_jni_1getAltitude
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
      "GpsPosition::getAltitude: "
      "GpsPosition object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_GpsPosition
 * Method:    jni_setLatitude
 * Signature: (JD)V
 */
void JNICALL Java_ai_gams_utility_GpsPosition_jni_1setLatitude
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
      "GpsPosition::setLatitude: "
      "GpsPosition object is released already");
  }
}

/*
 * Class:     ai_gams_utility_GpsPosition
 * Method:    jni_setLongitude
 * Signature: (JD)V
 */
void JNICALL Java_ai_gams_utility_GpsPosition_jni_1setLongitude
  (JNIEnv * env, jobject, jlong cptr, jdouble input)
{
  utility::Position * current = (utility::Position *) cptr;

  if (current)
  {
    current->y = input;
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "GpsPosition::setLongitude: "
      "GpsPosition object is released already");
  }
}

/*
 * Class:     ai_gams_utility_GpsPosition
 * Method:    jni_setAltitude
 * Signature: (JD)V
 */
void JNICALL Java_ai_gams_utility_GpsPosition_jni_1setAltitude
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
      "GpsPosition::setAltitude: "
      "GpsPosition object is released already");
  }
}
