#include "com_gams_utility_Logging.h"
#include "gams/loggers/GlobalLogger.h"

namespace loggers = gams::loggers;

/*
 * Class:     com_gams_utility_Logging
 * Method:    jni_set_level
 * Signature: (I)V
 */
void JNICALL Java_com_gams_utility_Logging_jni_1set_1level
  (JNIEnv *, jclass, jint level)
{
  loggers::global_logger->set_level (level);
}

/*
 * Class:     com_gams_utility_Logging
 * Method:    jni_get_level
 * Signature: ()I
 */
jint JNICALL Java_com_gams_utility_Logging_jni_1get_1level
  (JNIEnv *, jclass)
{
  return loggers::global_logger->get_level ();
}
