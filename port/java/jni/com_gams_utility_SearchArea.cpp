#include "com_gams_utility_SearchArea.h"
#include "gams/utility/SearchArea.h"

namespace containers = madara::knowledge::containers;
namespace engine = madara::knowledge;
namespace utility = gams::utility;

/*
 * Class:     com_gams_utility_SearchArea
 * Method:    jni_SearchArea
 * Signature: ()J
 */
jlong JNICALL Java_com_gams_utility_SearchArea_jni_1SearchArea
  (JNIEnv *, jobject)
{
  return (jlong) new utility::SearchArea ();
}

/*
 * Class:     com_gams_utility_SearchArea
 * Method:    jni_getName
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_com_gams_utility_SearchArea_jni_1getName
  (JNIEnv * env, jobject, jlong cptr)
{
  utility::SearchArea* current = (utility::SearchArea*) cptr;
  jstring result;

  if (current)
  {
    result = env->NewStringUTF (current->get_name ().c_str ());
  }

  return result;
}

/*
 * Class:     com_gams_utility_SearchArea
 * Method:    jni_setName
 * Signature: (JLjava/lang/String;)V
 */
void JNICALL Java_com_gams_utility_SearchArea_jni_1setName
  (JNIEnv * env, jobject, jlong cptr, jstring new_name)
{
  utility::SearchArea* current = (utility::SearchArea*) cptr;
  const char * str_name = env->GetStringUTFChars (new_name, 0);

  if (current && str_name)
    current->set_name (str_name);

  env->ReleaseStringUTFChars (new_name, str_name);
}

/*
 * Class:     com_gams_utility_SearchArea
 * Method:    jni_fromContainer
 * Signature: (JJLjava/lang/String;)V
 */
void JNICALL Java_com_gams_utility_SearchArea_jni_1fromContainer
  (JNIEnv * env, jobject, jlong cptr, jlong kb_ptr, jstring name)
{
  utility::SearchArea* current = (utility::SearchArea*) cptr;
  engine::KnowledgeBase* kb = (engine::KnowledgeBase*) kb_ptr;
  const char * str_name = env->GetStringUTFChars (name, 0);

  if (current && kb && str_name)
    current->from_container (*kb, str_name);

  env->ReleaseStringUTFChars (name, str_name);
}

/*
 * Class:     com_gams_utility_SearchArea
 * Method:    jni_toContainer
 * Signature: (JJLjava/lang/String;)V
 */
void JNICALL Java_com_gams_utility_SearchArea_jni_1toContainer
  (JNIEnv * env, jobject, jlong cptr, jlong kb_ptr, jstring name)
{
  utility::SearchArea* current = (utility::SearchArea*) cptr;
  engine::KnowledgeBase* kb = (engine::KnowledgeBase*) kb_ptr;
  const char * str_name = env->GetStringUTFChars (name, 0);

  if (current && kb && str_name)
    current->to_container (*kb, str_name);

  env->ReleaseStringUTFChars (name, str_name);
}

/*
 * Class:     com_gams_utility_SearchArea
 * Method:    jni_modify
 * Signature: (J)V
 */
void JNICALL Java_com_gams_utility_SearchArea_jni_1modify
  (JNIEnv *, jobject, jlong cptr)
{
  utility::SearchArea* current = (utility::SearchArea*) cptr;

  if (current)
  {
    current->modify();
  }
}

/*
 * Class:     com_gams_utility_SearchArea
 * Method:    jni_freeSearchArea
 * Signature: (J)V
 */
void JNICALL Java_com_gams_utility_SearchArea_jni_1freeSearchArea
  (JNIEnv *, jclass, jlong cptr)
{
  delete (utility::SearchArea *) cptr;
}

/*
 * Class:     com_gams_utility_SearchArea
 * Method:    jni_toString
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_com_gams_utility_SearchArea_jni_1toString
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result;

  utility::Position * current = (utility::Position *) cptr;
  if (current)
    result = env->NewStringUTF("SearchArea");

  return result;
}

/*
 * Class:     com_gams_utility_SearchArea
 * Method:    jni_addPrioritizedRegion
 * Signature: (JJ)V
 */
void JNICALL Java_com_gams_utility_SearchArea_jni_1addPrioritizedRegion
  (JNIEnv *, jobject, jlong cptr, jlong region_ptr)
{
  utility::SearchArea * current = (utility::SearchArea *) cptr;
  utility::PrioritizedRegion * region = (utility::PrioritizedRegion *) region_ptr;

  if (current && region)
    current->add_prioritized_region (*region);
}

/*
 * Class:     com_gams_utility_SearchArea
 * Method:    jni_getConvexHull
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_utility_SearchArea_jni_1getConvexHull
  (JNIEnv *, jobject, jlong cptr)
{
  jlong result (0);

  utility::SearchArea * current = (utility::SearchArea *) cptr;
  if (current)
    result = (jlong) new utility::Region (current->get_convex_hull ());

  return result;
}

/*
 * Class:     com_gams_utility_SearchArea
 * Method:    jni_containsGps
 * Signature: (JJ)Z
 */
jboolean JNICALL Java_com_gams_utility_SearchArea_jni_1containsGps
  (JNIEnv *, jobject, jlong cptr, jlong coord_ptr)
{
  jboolean result (false);

  utility::SearchArea * current = (utility::SearchArea *) cptr;
  utility::GPSPosition * coord = (utility::GPSPosition *) coord_ptr;
  if (current && coord)
    result = current->contains (*coord);

  return result;
}

/*
 * Class:     com_gams_utility_SearchArea
 * Method:    jni_getMaxAlt
 * Signature: (J)D
 */
jdouble JNICALL Java_com_gams_utility_SearchArea_jni_1getMaxAlt
  (JNIEnv *, jobject, jlong cptr)
{
  jdouble result (0.0);

  utility::SearchArea * current = (utility::SearchArea *) cptr;
  if (current)
    result = current->max_alt_;

  return result;
}

/*
 * Class:     com_gams_utility_SearchArea
 * Method:    jni_getMinAlt
 * Signature: (J)D
 */
jdouble JNICALL Java_com_gams_utility_SearchArea_jni_1getMinAlt
  (JNIEnv *, jobject, jlong cptr)
{
  jdouble result (0.0);

  utility::SearchArea * current = (utility::SearchArea *) cptr;
  if (current)
    result = current->min_alt_;

  return result;
}

/*
 * Class:     com_gams_utility_SearchArea
 * Method:    jni_getMaxLat
 * Signature: (J)D
 */
jdouble JNICALL Java_com_gams_utility_SearchArea_jni_1getMaxLat
  (JNIEnv *, jobject, jlong cptr)
{
  jdouble result (0.0);

  utility::SearchArea * current = (utility::SearchArea *) cptr;
  if (current)
    result = current->max_lat_;

  return result;
}

/*
 * Class:     com_gams_utility_SearchArea
 * Method:    jni_getMinLat
 * Signature: (J)D
 */
jdouble JNICALL Java_com_gams_utility_SearchArea_jni_1getMinLat
  (JNIEnv *, jobject, jlong cptr)
{
  jdouble result (0.0);

  utility::SearchArea * current = (utility::SearchArea *) cptr;
  if (current)
    result = current->min_lat_;

  return result;
}

/*
 * Class:     com_gams_utility_SearchArea
 * Method:    jni_getMaxLong
 * Signature: (J)D
 */
jdouble JNICALL Java_com_gams_utility_SearchArea_jni_1getMaxLong
  (JNIEnv *, jobject, jlong cptr)
{
  jdouble result (0.0);

  utility::SearchArea * current = (utility::SearchArea *) cptr;
  if (current)
    result = current->max_lon_;

  return result;
}

/*
 * Class:     com_gams_utility_SearchArea
 * Method:    jni_getMinLong
 * Signature: (J)D
 */
jdouble JNICALL Java_com_gams_utility_SearchArea_jni_1getMinLong
  (JNIEnv *, jobject, jlong cptr)
{
  jdouble result (0.0);

  utility::SearchArea * current = (utility::SearchArea *) cptr;
  if (current)
    result = current->min_lon_;

  return result;
}

/*
 * Class:     com_gams_utility_SearchArea
 * Method:    jni_getGpsPriority
 * Signature: (JJ)I
 */
jlong JNICALL Java_com_gams_utility_SearchArea_jni_1getGpsPriority
  (JNIEnv *, jobject, jlong cptr, jlong coord_ptr)
{
  jlong result (0);

  utility::SearchArea * current = (utility::SearchArea *) cptr;
  utility::GPSPosition * coord = (utility::GPSPosition *) coord_ptr;
  if (current)
    result = current->get_priority (*coord);

  return result;
}

/*
 * Class:     com_gams_utility_SearchArea
 * Method:    jni_getRegions
 * Signature: (J)[J
 */
jlongArray JNICALL Java_com_gams_utility_SearchArea_jni_1getRegions
  (JNIEnv * env, jobject, jlong cptr)
{
  jlongArray result;
  utility::SearchArea * current = (utility::SearchArea *) cptr;

  if (current)
  {
    std::vector <utility::PrioritizedRegion> regions = current->get_regions ();

    if (regions.size () > 0)
    {
      result = env->NewLongArray ((jsize)regions.size ());
      jlong * elements = env->GetLongArrayElements(result, 0);
      for (size_t i = 0; i < regions.size (); ++i)
      {
        elements[i] = (jlong) new utility::PrioritizedRegion (regions[i]);
      }
      env->ReleaseLongArrayElements(result, elements, 0);
    }
  }

  return result;
}
