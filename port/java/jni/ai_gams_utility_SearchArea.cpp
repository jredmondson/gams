#include "ai_gams_utility_SearchArea.h"
#include "gams/pose/SearchArea.h"
#include "gams_jni.h"

namespace containers = madara::knowledge::containers;
namespace engine = madara::knowledge;
namespace pose = gams::pose;

/*
 * Class:     ai_gams_utility_SearchArea
 * Method:    jni_SearchArea
 * Signature: ()J
 */
jlong JNICALL Java_ai_gams_utility_SearchArea_jni_1SearchArea
  (JNIEnv *, jobject)
{
  return (jlong) new pose::SearchArea ();
}

/*
 * Class:     ai_gams_utility_SearchArea
 * Method:    jni_getName
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_ai_gams_utility_SearchArea_jni_1getName
  (JNIEnv * env, jobject, jlong cptr)
{
  pose::SearchArea* current = (pose::SearchArea*) cptr;
  jstring result = 0;

  if (current)
  {
    result = env->NewStringUTF (current->get_name ().c_str ());
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "SearchArea::getName: "
      "SearchArea object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_SearchArea
 * Method:    jni_setName
 * Signature: (JLjava/lang/String;)V
 */
void JNICALL Java_ai_gams_utility_SearchArea_jni_1setName
  (JNIEnv * env, jobject, jlong cptr, jstring new_name)
{
  pose::SearchArea* current = (pose::SearchArea*) cptr;
  const char * str_name = env->GetStringUTFChars (new_name, 0);

  if (current && str_name)
    current->set_name (str_name);
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "SearchArea::setName: "
      "SearchArea object is released already or name is null");
  }

  env->ReleaseStringUTFChars (new_name, str_name);
}

/*
 * Class:     ai_gams_utility_SearchArea
 * Method:    jni_fromContainer
 * Signature: (JJLjava/lang/String;)V
 */
void JNICALL Java_ai_gams_utility_SearchArea_jni_1fromContainer
  (JNIEnv * env, jobject, jlong cptr, jlong kb_ptr, jstring name)
{
  pose::SearchArea* current = (pose::SearchArea*) cptr;
  engine::KnowledgeBase* kb = (engine::KnowledgeBase*) kb_ptr;
  const char * str_name = env->GetStringUTFChars (name, 0);

  if (current && kb && str_name)
    current->from_container (*kb, str_name);
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    env->ReleaseStringUTFChars (name, str_name);
    
    gams::utility::java::throw_dead_obj_exception(env,
      "SearchArea::fromContainer: "
      "SearchArea, KB, or name objects are released already");
  }

  env->ReleaseStringUTFChars (name, str_name);
}

/*
 * Class:     ai_gams_utility_SearchArea
 * Method:    jni_toContainer
 * Signature: (JJLjava/lang/String;)V
 */
void JNICALL Java_ai_gams_utility_SearchArea_jni_1toContainer
  (JNIEnv * env, jobject, jlong cptr, jlong kb_ptr, jstring name)
{
  pose::SearchArea* current = (pose::SearchArea*) cptr;
  engine::KnowledgeBase* kb = (engine::KnowledgeBase*) kb_ptr;
  const char * str_name = env->GetStringUTFChars (name, 0);

  if (current && kb && str_name)
    current->to_container (*kb, str_name);
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    env->ReleaseStringUTFChars (name, str_name);
    
    gams::utility::java::throw_dead_obj_exception(env,
      "SearchArea::toContainer: "
      "SearchArea, KB, or name objects are released already");
  }

  env->ReleaseStringUTFChars (name, str_name);
}

/*
 * Class:     ai_gams_utility_SearchArea
 * Method:    jni_modify
 * Signature: (J)V
 */
void JNICALL Java_ai_gams_utility_SearchArea_jni_1modify
  (JNIEnv * env, jobject, jlong cptr)
{
  pose::SearchArea* current = (pose::SearchArea*) cptr;

  if (current)
  {
    current->modify();
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "SearchArea::modify: "
      "SearchArea object is released already");
  }
}

/*
 * Class:     ai_gams_utility_SearchArea
 * Method:    jni_freeSearchArea
 * Signature: (J)V
 */
void JNICALL Java_ai_gams_utility_SearchArea_jni_1freeSearchArea
  (JNIEnv *, jclass, jlong cptr)
{
  delete (pose::SearchArea *) cptr;
}

/*
 * Class:     ai_gams_utility_SearchArea
 * Method:    jni_toString
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_ai_gams_utility_SearchArea_jni_1toString
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result = 0;

  pose::Position * current = (pose::Position *) cptr;
  if (current)
  {
    result = env->NewStringUTF("SearchArea");
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "SearchArea::toString: "
      "SearchArea object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_SearchArea
 * Method:    jni_addPrioritizedRegion
 * Signature: (JJ)V
 */
void JNICALL Java_ai_gams_utility_SearchArea_jni_1addPrioritizedRegion
  (JNIEnv * env, jobject, jlong cptr, jlong region_ptr)
{
  pose::SearchArea * current = (pose::SearchArea *) cptr;
  pose::PrioritizedRegion * region = (pose::PrioritizedRegion *) region_ptr;

  if (current && region)
    current->add_prioritized_region (*region);
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "SearchArea::addPrioritizedRegion: "
      "SearchArea or region objects are released already");
  }
}

/*
 * Class:     ai_gams_utility_SearchArea
 * Method:    jni_getConvexHull
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_utility_SearchArea_jni_1getConvexHull
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result (0);

  pose::SearchArea * current = (pose::SearchArea *) cptr;
  if (current)
    result = (jlong) new pose::Region (current->get_convex_hull ());
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "SearchArea::getConvexHull: "
      "SearchArea object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_SearchArea
 * Method:    jni_containsGps
 * Signature: (JJ)Z
 */
jboolean JNICALL Java_ai_gams_utility_SearchArea_jni_1containsGps
  (JNIEnv * env, jobject, jlong cptr, jlong coord_ptr)
{
  jboolean result (false);

  pose::SearchArea * current = (pose::SearchArea *) cptr;
  pose::Position * coord = (pose::Position *) coord_ptr;
  if (current && coord)
    result = current->contains (*coord);
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "SearchArea::containsGps: "
      "SearchArea or coord objects are released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_SearchArea
 * Method:    jni_getMaxAlt
 * Signature: (J)D
 */
jdouble JNICALL Java_ai_gams_utility_SearchArea_jni_1getMaxAlt
  (JNIEnv * env, jobject, jlong cptr)
{
  jdouble result (0.0);

  pose::SearchArea * current = (pose::SearchArea *) cptr;
  if (current)
    result = current->max_alt_;
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "SearchArea::getMaxAlt: "
      "SearchArea object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_SearchArea
 * Method:    jni_getMinAlt
 * Signature: (J)D
 */
jdouble JNICALL Java_ai_gams_utility_SearchArea_jni_1getMinAlt
  (JNIEnv * env, jobject, jlong cptr)
{
  jdouble result (0.0);

  pose::SearchArea * current = (pose::SearchArea *) cptr;
  if (current)
    result = current->min_alt_;
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "SearchArea::getMinAlt: "
      "SearchArea object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_SearchArea
 * Method:    jni_getMaxLat
 * Signature: (J)D
 */
jdouble JNICALL Java_ai_gams_utility_SearchArea_jni_1getMaxLat
  (JNIEnv * env, jobject, jlong cptr)
{
  jdouble result (0.0);

  pose::SearchArea * current = (pose::SearchArea *) cptr;
  if (current)
    result = current->max_lat_;
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "SearchArea::getMaxLat: "
      "SearchArea object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_SearchArea
 * Method:    jni_getMinLat
 * Signature: (J)D
 */
jdouble JNICALL Java_ai_gams_utility_SearchArea_jni_1getMinLat
  (JNIEnv * env, jobject, jlong cptr)
{
  jdouble result (0.0);

  pose::SearchArea * current = (pose::SearchArea *) cptr;
  if (current)
    result = current->min_lat_;
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "SearchArea::getMinLat: "
      "SearchArea object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_SearchArea
 * Method:    jni_getMaxLong
 * Signature: (J)D
 */
jdouble JNICALL Java_ai_gams_utility_SearchArea_jni_1getMaxLong
  (JNIEnv * env, jobject, jlong cptr)
{
  jdouble result (0.0);

  pose::SearchArea * current = (pose::SearchArea *) cptr;
  if (current)
    result = current->max_lon_;
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "SearchArea::getMaxLong: "
      "SearchArea object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_SearchArea
 * Method:    jni_getMinLong
 * Signature: (J)D
 */
jdouble JNICALL Java_ai_gams_utility_SearchArea_jni_1getMinLong
  (JNIEnv * env, jobject, jlong cptr)
{
  jdouble result (0.0);

  pose::SearchArea * current = (pose::SearchArea *) cptr;
  if (current)
    result = current->min_lon_;
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "SearchArea::getMinLong: "
      "SearchArea object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_SearchArea
 * Method:    jni_getGpsPriority
 * Signature: (JJ)I
 */
jlong JNICALL Java_ai_gams_utility_SearchArea_jni_1getGpsPriority
  (JNIEnv * env, jobject, jlong cptr, jlong coord_ptr)
{
  jlong result (0);

  pose::SearchArea * current = (pose::SearchArea *) cptr;
  pose::Position * coord = (pose::Position *) coord_ptr;
  if (current)
    result = current->get_priority (*coord);
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "SearchArea::getGpsPriority: "
      "SearchArea object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_utility_SearchArea
 * Method:    jni_getRegions
 * Signature: (J)[J
 */
jlongArray JNICALL Java_ai_gams_utility_SearchArea_jni_1getRegions
  (JNIEnv * env, jobject, jlong cptr)
{
  jlongArray result (0);
  pose::SearchArea * current = (pose::SearchArea *) cptr;

  if (current)
  {
    std::vector <pose::PrioritizedRegion> regions = current->get_regions ();

    if (regions.size () > 0)
    {
      result = env->NewLongArray ((jsize)regions.size ());
      jlong * elements = env->GetLongArrayElements(result, 0);
      for (size_t i = 0; i < regions.size (); ++i)
      {
        elements[i] = (jlong) new pose::PrioritizedRegion (regions[i]);
      }
      env->ReleaseLongArrayElements(result, elements, 0);
    }
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "SearchArea::getRegions: "
      "SearchArea object is released already");
  }

  return result;
}
