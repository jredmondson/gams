#include "ai_gams_utility_Region.h"

#include <string>

#include "gams/pose/Region.h"
#include "gams_jni.h"

namespace containers = madara::knowledge::containers;
namespace engine = madara::knowledge;
namespace pose = gams::pose;

/*
 * Class:     ai_gams_utility_Region
 * Method:    jni_Region
 * Signature: ()J
 */
jlong JNICALL Java_ai_gams_utility_Region_jni_1Region
  (JNIEnv *, jobject)
{
  return (jlong) new pose::Region ();
}

/*
 * Class:     ai_gams_utility_Region
 * Method:    jni_freeRegion
 * Signature: (J)V
 */
void JNICALL Java_ai_gams_utility_Region_jni_1freeRegion
  (JNIEnv *, jclass, jlong cptr)
{
  delete (pose::Region *) cptr;
}

/*
 * Class:     ai_gams_utility_Region
 * Method:    jni_getName
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_ai_gams_utility_Region_jni_1getName
  (JNIEnv * env, jobject, jlong cptr)
{
  const pose::Region * current = (const pose::Region *) cptr;
  jstring result = 0;

  if (current)
  {
    result = env->NewStringUTF (current->get_name ().c_str ());
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Region::getName: "
      "Region object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_Region
 * Method:    jni_setName
 * Signature: (JLjava/lang/String;)V
 */
void JNICALL Java_ai_gams_utility_Region_jni_1setName
  (JNIEnv * env, jobject, jlong cptr, jstring new_name)
{
  pose::Region * current = (pose::Region *) cptr;
  const char * str_name = env->GetStringUTFChars (new_name, 0);

  if (current && str_name)
  {
    current->set_name (str_name);
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    env->ReleaseStringUTFChars (new_name, str_name);
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Region::setName: "
      "Region or name objects are released already");
  }

  env->ReleaseStringUTFChars (new_name, str_name);
}

/*
 * Class:     ai_gams_utility_Region
 * Method:    jni_fromContainer
 * Signature: (JJLjava/lang/String;)V
 */
void JNICALL Java_ai_gams_utility_Region_jni_1fromContainer
  (JNIEnv * env, jobject, jlong cptr, jlong kb_ptr, jstring name)
{
  pose::Region * current = (pose::Region *) cptr;
  const char * str_name = env->GetStringUTFChars (name, 0);
  engine::KnowledgeBase * kb = (engine::KnowledgeBase *) kb_ptr;

  if (current && str_name && kb)
  {
    current->from_container (*kb, str_name);
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    env->ReleaseStringUTFChars (name, str_name);
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Region::fromContainer: "
      "Region, name, or KB objects are released already");
  }

  env->ReleaseStringUTFChars (name, str_name);
}

/**
 * Class:     ai_gams_utility_Region
 * Method:    jni_toContainer
 * Signature: (JJLjava/lang/String;)V
 */
void JNICALL Java_ai_gams_utility_Region_jni_1toContainer
  (JNIEnv * env, jobject, jlong cptr, jlong kb_ptr, jstring name)
{
  pose::Region * current = (pose::Region *) cptr;
  const char * str_name = env->GetStringUTFChars (name, 0);
  engine::KnowledgeBase * kb = (engine::KnowledgeBase *) kb_ptr;

  if (current && str_name && kb)
  {
    current->to_container (*kb, str_name);
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    env->ReleaseStringUTFChars (name, str_name);
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Region::toContainer: "
      "Region, name, or KB objects are released already");
  }

  env->ReleaseStringUTFChars (name, str_name);
}

/*
 * Class:     ai_gams_utility_Region
 * Method:    jni_modify
 * Signature: (J)V
 */
void JNICALL Java_ai_gams_utility_Region_jni_1modify
  (JNIEnv * env, jobject, jlong cptr)
{
  pose::Region * current = (pose::Region *) cptr;
  if (current)
  {
    current->modify();
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Region::modify: "
      "Region object is released already");
  }
}

/*
 * Class:     ai_gams_utility_Region
 * Method:    jni_toString
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_ai_gams_utility_Region_jni_1toString
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring ret_val = 0;

  pose::Region * current = (pose::Region *) cptr;
  if (current)
  {
    std::string result = current->to_string();
    ret_val = env->NewStringUTF(result.c_str());
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Region::toString: "
      "Region object is released already");
  }

  return ret_val;
}

/*
 * Class:     ai_gams_utility_Region
 * Method:    jni_addGpsVertex
 * Signature: (JJ)V
 */
void JNICALL Java_ai_gams_utility_Region_jni_1addGpsVertex
  (JNIEnv * env, jobject, jlong cptr, jlong vertex)
{
  pose::Region * current = (pose::Region *) cptr;
  if (current && vertex != 0)
    current->vertices.push_back (*(pose::Position *)vertex);
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Region::addGpsVertex: "
      "Region object is released already or vertex is 0");
  }
}

/*
 * Class:     ai_gams_utility_Region
 * Method:    jni_getVertices
 * Signature: (J)[J
 */
jlongArray JNICALL Java_ai_gams_utility_Region_jni_1getVertices
  (JNIEnv * env, jobject, jlong cptr)
{
  jlongArray result (0);
  pose::Region * current = (pose::Region *) cptr;

  if (current)
  {
    if (current->vertices.size () > 0)
    {
      result = env->NewLongArray ((jsize)current->vertices.size ());
      jlong * elements = env->GetLongArrayElements(result, 0);
      for (size_t i = 0; i < current->vertices.size (); ++i)
      {
        elements[i] = (jlong) new pose::Position (current->vertices[i]);
      }
      env->ReleaseLongArrayElements(result, elements, 0);
    }
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Region::getVertices: "
      "Region object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_Region
 * Method:    jni_getArea
 * Signature: (J)D
 */
jdouble JNICALL Java_ai_gams_utility_Region_jni_1getArea
  (JNIEnv * env, jobject, jlong cptr)
{
  jdouble result (0.0);

  pose::Region * current = (pose::Region *) cptr;
  if (current)
    result = current->get_area ();
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Region::getArea: "
      "Region object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_Region
 * Method:    jni_getBoundingBox
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_utility_Region_jni_1getBoundingBox
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result (0);

  pose::Region * current = (pose::Region *) cptr;
  if (current)
    result = (jlong) new pose::Region (current->get_bounding_box ());
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Region::getBoundingBox: "
      "Region object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_Region
 * Method:    jni_containsGps
 * Signature: (JJ)Z
 */
jboolean JNICALL Java_ai_gams_utility_Region_jni_1containsGps
  (JNIEnv * env, jobject, jlong cptr, jlong coord_ptr)
{
  jboolean result (0.0);

  pose::Region * current = (pose::Region *) cptr;
  pose::Position * coord = (pose::Position *) coord_ptr;
  if (current)
    result = current->contains (*coord);
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Region::containsGps: "
      "Region object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_Region
 * Method:    jni_getGpsDistance
 * Signature: (JJ)D
 */
jdouble JNICALL Java_ai_gams_utility_Region_jni_1getGpsDistance
  (JNIEnv * env, jobject, jlong cptr, jlong coord_ptr)
{
  jboolean result (0.0);

  pose::Region * current = (pose::Region *) cptr;
  pose::Position * coord = (pose::Position *) coord_ptr;
  if (current && coord)
    result = current->distance (*coord);
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Region::getGpsDistance: "
      "Region or coord objects are released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_Region
 * Method:    jni_getMaxAlt
 * Signature: (J)D
 */
jdouble JNICALL Java_ai_gams_utility_Region_jni_1getMaxAlt
  (JNIEnv * env, jobject, jlong cptr)
{
  jdouble result (0.0);

  pose::Region * current = (pose::Region *) cptr;
  if (current)
    result = current->max_alt_;
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Region::getMaxAlt: "
      "Region object is released already");
  }

  return result;
}
/*
 * Class:     ai_gams_utility_Region
 * Method:    jni_getMinAlt
 * Signature: (J)D
 */
jdouble JNICALL Java_ai_gams_utility_Region_jni_1getMinAlt
  (JNIEnv * env, jobject, jlong cptr)
{
  jdouble result (0.0);

  pose::Region * current = (pose::Region *) cptr;
  if (current)
    result = current->min_alt_;
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Region::getMinAlt: "
      "Region object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_Region
 * Method:    jni_getMaxLat
 * Signature: (J)D
 */
jdouble JNICALL Java_ai_gams_utility_Region_jni_1getMaxLat
  (JNIEnv * env, jobject, jlong cptr)
{
  jdouble result (0.0);

  pose::Region * current = (pose::Region *) cptr;
  if (current)
    result = current->max_lat_;
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Region::getMaxLat: "
      "Region object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_Region
 * Method:    jni_getMinLat
 * Signature: (J)D
 */
jdouble JNICALL Java_ai_gams_utility_Region_jni_1getMinLat
  (JNIEnv * env, jobject, jlong cptr)
{
  jdouble result (0.0);

  pose::Region * current = (pose::Region *) cptr;
  if (current)
    result = current->min_lat_;
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Region::getMinLat: "
      "Region object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_Region
 * Method:    jni_getMaxLong
 * Signature: (J)D
 */
jdouble JNICALL Java_ai_gams_utility_Region_jni_1getMaxLong
  (JNIEnv * env, jobject, jlong cptr)
{
  jdouble result (0.0);

  pose::Region * current = (pose::Region *) cptr;
  if (current)
    result = current->max_lon_;
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Region::getMaxLong: "
      "Region object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_Region
 * Method:    jni_getMinLong
 * Signature: (J)D
 */
jdouble JNICALL Java_ai_gams_utility_Region_jni_1getMinLong
  (JNIEnv * env, jobject, jlong cptr)
{
  jdouble result (0.0);

  pose::Region * current = (pose::Region *) cptr;
  if (current)
    result = current->min_lon_;
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "Region::getMinLong: "
      "Region object is released already");
  }

  return result;
}
