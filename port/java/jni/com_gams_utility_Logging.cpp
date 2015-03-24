#include "com_gams_utility_Logging.h"
#include "gams/utility/Logging.h"

namespace utility = gams::utility;

/*
 * Class:     com_gams_utility_Logging
 * Method:    jni_set_level
 * Signature: (I)V
 */
void JNICALL Java_com_gams_utility_Logging_jni_1set_1level
  (JNIEnv *, jclass, jint level)
{
  utility::set_log_level (level);
}

/*
 * Class:     com_gams_utility_Logging
 * Method:    jni_get_level
 * Signature: ()I
 */
jint JNICALL Java_com_gams_utility_Logging_jni_1get_1level
  (JNIEnv *, jclass)
{
  return utility::get_log_level ();
}
