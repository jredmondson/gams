#include "com_gams_logger_GlobalLogger.h"
#include "gams/loggers/Global_Logger.h"

namespace logger = gams::loggers;

jlong JNICALL Java_com_gams_logger_GlobalLogger_jni_1getCPtr
(JNIEnv *, jclass)
{
  return (jlong)logger::global_logger.get ();
}

void JNICALL Java_com_gams_logger_GlobalLogger_jni_1setLevel
(JNIEnv *, jclass, jint level)
{
  logger::global_logger->set_level ((int)level);
}


jint JNICALL Java_com_gams_logger_GlobalLogger_jni_1getLevel
(JNIEnv *, jclass)
{
  return (jint)logger::global_logger->get_level ();
}


jstring JNICALL Java_com_gams_logger_GlobalLogger_jni_1getTag
(JNIEnv * env, jclass)
{
  return env->NewStringUTF (logger::global_logger->get_tag ().c_str ());
}


void JNICALL Java_com_gams_logger_GlobalLogger_jni_1setTag
(JNIEnv * env, jclass, jstring tag)
{
  const char * str_tag = env->GetStringUTFChars (tag, 0);

  logger::global_logger->set_tag (str_tag);

  env->ReleaseStringUTFChars (tag, str_tag);
}


void JNICALL Java_com_gams_logger_GlobalLogger_jni_1addTerm
(JNIEnv *, jclass)
{
  logger::global_logger->add_term ();
}


void JNICALL Java_com_gams_logger_GlobalLogger_jni_1addSyslog
(JNIEnv *, jclass)
{
  logger::global_logger->add_syslog ();
}


void JNICALL Java_com_gams_logger_GlobalLogger_jni_1clear
(JNIEnv *, jclass)
{
  logger::global_logger->clear ();
}


void JNICALL Java_com_gams_logger_GlobalLogger_jni_1addFile
(JNIEnv * env, jclass, jstring filename)
{
  const char * str_filename = env->GetStringUTFChars (filename, 0);

  logger::global_logger->add_file (str_filename);

  env->ReleaseStringUTFChars (filename, str_filename);
}

void JNICALL Java_com_gams_logger_GlobalLogger_jni_1log
(JNIEnv * env, jclass, jint level, jstring message)
{
  const char * str_message = env->GetStringUTFChars (message, 0);

  madara_logger_ptr_log (logger::global_logger.get (), (int)level, str_message);

  env->ReleaseStringUTFChars (message, str_message);
}
