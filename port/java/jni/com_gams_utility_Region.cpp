#include "com_gams_utility_Region.h"

#include <string>

#include "gams/utility/Region.h"

namespace containers = Madara::Knowledge_Engine::Containers;
namespace engine = Madara::Knowledge_Engine;
namespace utility = gams::utility;

/*
 * Class:     com_gams_utility_Region
 * Method:    jni_Region
 * Signature: ()J
 */
jlong JNICALL Java_com_gams_utility_Region_jni_1Region
  (JNIEnv *, jobject)
{
  return (jlong) new utility::Region ();
}

/*
 * Class:     com_gams_utility_Region
 * Method:    jni_freeRegion
 * Signature: (J)V
 */
void JNICALL Java_com_gams_utility_Region_jni_1freeRegion
  (JNIEnv *, jclass, jlong cptr)
{
  delete (utility::Region *) cptr;
}

/*
 * Class:     com_gams_utility_Region
 * Method:    jni_getName
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_com_gams_utility_Region_jni_1getName
  (JNIEnv * env, jobject, jlong cptr)
{
  const utility::Region * current = (const utility::Region *) cptr;
  jstring result;

  if (current)
  {
    result = env->NewStringUTF (current->get_name ().c_str ());
  }

  return result;
}

/*
 * Class:     com_gams_utility_Region
 * Method:    jni_setName
 * Signature: (JLjava/lang/String;)V
 */
void JNICALL Java_com_gams_utility_Region_jni_1setName
  (JNIEnv * env, jobject, jlong cptr, jstring new_name)
{
  utility::Region * current = (utility::Region *) cptr;
  const char * str_name = env->GetStringUTFChars (new_name, 0);

  if (current && str_name)
  {
    current->set_name (str_name);
  }

  env->ReleaseStringUTFChars (new_name, str_name);
}

/*
 * Class:     com_gams_utility_Region
 * Method:    jni_fromContainer
 * Signature: (JJLjava/lang/String;)V
 */
void JNICALL Java_com_gams_utility_Region_jni_1fromContainer
  (JNIEnv * env, jobject, jlong cptr, jlong kb_ptr, jstring name)
{
  utility::Region * current = (utility::Region *) cptr;
  const char * str_name = env->GetStringUTFChars (name, 0);
  engine::Knowledge_Base * kb = (engine::Knowledge_Base *) kb_ptr;

  if (current && str_name && kb)
  {
    current->from_container (*kb, str_name);
  }

  env->ReleaseStringUTFChars (name, str_name);
}

/*
 * Class:     com_gams_utility_Region
 * Method:    jni_toContainer
 * Signature: (JJLjava/lang/String;)V
 */
void JNICALL Java_com_gams_utility_Region_jni_1toContainer
  (JNIEnv * env, jobject, jlong cptr, jlong kb_ptr, jstring name)
{
  utility::Region * current = (utility::Region *) cptr;
  const char * str_name = env->GetStringUTFChars (name, 0);
  engine::Knowledge_Base * kb = (engine::Knowledge_Base *) kb_ptr;

  if (current && str_name && kb)
  {
    current->to_container (*kb, str_name);
  }

  env->ReleaseStringUTFChars (name, str_name);
}

/*
 * Class:     com_gams_utility_Region
 * Method:    jni_toString
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_com_gams_utility_Region_jni_1toString
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring ret_val;

  utility::Region * current = (utility::Region *) cptr;
  if (current)
  {
    std::string result = current->to_string();
    ret_val = env->NewStringUTF(result.c_str());
  }

  return ret_val;
}

/*
 * Class:     com_gams_utility_Region
 * Method:    jni_addGpsVertex
 * Signature: (JJ)V
 */
void JNICALL Java_com_gams_utility_Region_jni_1addGpsVertex
  (JNIEnv *, jobject, jlong cptr, jlong vertex)
{
  utility::Region * current = (utility::Region *) cptr;
  if (current && vertex != 0)
    current->vertices.push_back (*(utility::GPS_Position *)vertex);
}

/*
 * Class:     com_gams_utility_Region
 * Method:    jni_getVertices
 * Signature: (J)[J
 */
jlongArray JNICALL Java_com_gams_utility_Region_jni_1getVertices
  (JNIEnv * env, jobject, jlong cptr)
{
  jlongArray result;
  utility::Region * current = (utility::Region *) cptr;

  if (current)
  {
    if (current->vertices.size () > 0)
    {
      result = env->NewLongArray ((jsize)current->vertices.size ());
      jlong * elements = env->GetLongArrayElements(result, 0);
      for (size_t i = 0; i < current->vertices.size (); ++i)
      {
        elements[i] = (jlong) new utility::GPS_Position (current->vertices[i]);
      }
      env->ReleaseLongArrayElements(result, elements, 0);
    }
  }

  return result;
}

/*
 * Class:     com_gams_utility_Region
 * Method:    jni_getArea
 * Signature: (J)D
 */
jdouble JNICALL Java_com_gams_utility_Region_jni_1getArea
  (JNIEnv *, jobject, jlong cptr)
{
  jdouble result (0.0);

  utility::Region * current = (utility::Region *) cptr;
  if (current)
    result = current->get_area ();

  return result;
}

/*
 * Class:     com_gams_utility_Region
 * Method:    jni_getBoundingBox
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_utility_Region_jni_1getBoundingBox
  (JNIEnv *, jobject, jlong cptr)
{
  jlong result (0);

  utility::Region * current = (utility::Region *) cptr;
  if (current)
    result = (jlong) new utility::Region (current->get_bounding_box ());

  return result;
}

/*
 * Class:     com_gams_utility_Region
 * Method:    jni_containsGps
 * Signature: (JJ)Z
 */
jboolean JNICALL Java_com_gams_utility_Region_jni_1containsGps
  (JNIEnv *, jobject, jlong cptr, jlong coord_ptr)
{
  jboolean result (0.0);

  utility::Region * current = (utility::Region *) cptr;
  utility::GPS_Position * coord = (utility::GPS_Position *) coord_ptr;
  if (current)
    result = current->contains (*coord);

  return result;
}

/*
 * Class:     com_gams_utility_Region
 * Method:    jni_getGpsDistance
 * Signature: (JJ)D
 */
jdouble JNICALL Java_com_gams_utility_Region_jni_1getGpsDistance
  (JNIEnv *, jobject, jlong cptr, jlong coord_ptr)
{
  jboolean result (0.0);

  utility::Region * current = (utility::Region *) cptr;
  utility::GPS_Position * coord = (utility::GPS_Position *) coord_ptr;
  if (current && coord)
    result = current->distance (*coord);

  return result;
}

/*
 * Class:     com_gams_utility_Region
 * Method:    jni_getMaxAlt
 * Signature: (J)D
 */
jdouble JNICALL Java_com_gams_utility_Region_jni_1getMaxAlt
  (JNIEnv *, jobject, jlong cptr)
{
  jdouble result (0.0);

  utility::Region * current = (utility::Region *) cptr;
  if (current)
    result = current->max_alt_;

  return result;
}
/*
 * Class:     com_gams_utility_Region
 * Method:    jni_getMinAlt
 * Signature: (J)D
 */
jdouble JNICALL Java_com_gams_utility_Region_jni_1getMinAlt
  (JNIEnv *, jobject, jlong cptr)
{
  jdouble result (0.0);

  utility::Region * current = (utility::Region *) cptr;
  if (current)
    result = current->min_alt_;

  return result;
}

/*
 * Class:     com_gams_utility_Region
 * Method:    jni_getMaxLat
 * Signature: (J)D
 */
jdouble JNICALL Java_com_gams_utility_Region_jni_1getMaxLat
  (JNIEnv *, jobject, jlong cptr)
{
  jdouble result (0.0);

  utility::Region * current = (utility::Region *) cptr;
  if (current)
    result = current->max_lat_;

  return result;
}

/*
 * Class:     com_gams_utility_Region
 * Method:    jni_getMinLat
 * Signature: (J)D
 */
jdouble JNICALL Java_com_gams_utility_Region_jni_1getMinLat
  (JNIEnv *, jobject, jlong cptr)
{
  jdouble result (0.0);

  utility::Region * current = (utility::Region *) cptr;
  if (current)
    result = current->min_lat_;

  return result;
}

/*
 * Class:     com_gams_utility_Region
 * Method:    jni_getMaxLong
 * Signature: (J)D
 */
jdouble JNICALL Java_com_gams_utility_Region_jni_1getMaxLong
  (JNIEnv *, jobject, jlong cptr)
{
  jdouble result (0.0);

  utility::Region * current = (utility::Region *) cptr;
  if (current)
    result = current->max_lon_;

  return result;
}

/*
 * Class:     com_gams_utility_Region
 * Method:    jni_getMinLong
 * Signature: (J)D
 */
jdouble JNICALL Java_com_gams_utility_Region_jni_1getMinLong
  (JNIEnv *, jobject, jlong cptr)
{
  jdouble result (0.0);

  utility::Region * current = (utility::Region *) cptr;
  if (current)
    result = current->min_lon_;

  return result;
}
