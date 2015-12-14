
#include "com_gams_controllers_BaseController.h"
#include "gams/controllers/BaseController.h"
#include "gams/platforms/java/JavaPlatform.h"
#include "gams/algorithms/java/JavaAlgorithm.h"
#include "gams/loggers/GlobalLogger.h"

namespace containers = madara::knowledge::containers;
namespace engine = madara::knowledge;
namespace controllers = gams::controllers;
namespace algorithms = gams::algorithms;
namespace platforms = gams::platforms;

typedef madara::knowledge::KnowledgeRecord::Integer  Integer;

/*
 * Class:     com_gams_controllers_BaseController
 * Method:    jni_BaseControllerFromKb
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_controllers_BaseController_jni_1BaseControllerFromKb
  (JNIEnv *, jobject, jlong cptr)
{
  jlong result = 0;

  engine::KnowledgeBase * kb = (engine::KnowledgeBase *) cptr;
  if (kb)
  {
    result = (jlong) new gams::controllers::BaseController (*kb);
  }

  return result;
}

/*
 * Class:     com_gams_controllers_BaseController
 * Method:    jni_BaseController
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_controllers_BaseController_jni_1BaseController
  (JNIEnv *, jobject, jlong cptr)
{
  jlong result = 0;

  controllers::BaseController * input = (controllers::BaseController *) cptr;
  if (input)
  {
    result = (jlong) new controllers::BaseController (*input);
  }

  return result;
}

/*
 * Class:     com_gams_controllers_BaseController
 * Method:    jni_freeBaseController
 * Signature: (J)V
 */
void JNICALL Java_com_gams_controllers_BaseController_jni_1freeBaseController
  (JNIEnv *, jclass, jlong cptr)
{
  delete (controllers::BaseController *) cptr;
}

/*
 * Class:     com_gams_controllers_BaseController
 * Method:    jni_toString
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_com_gams_controllers_BaseController_jni_1toString
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result;

  controllers::BaseController * current = (controllers::BaseController *) cptr;
  if (current)
    result = env->NewStringUTF("BaseController");

  return result;
}

/*
 * Class:     com_gams_controllers_BaseController
 * Method:    jni_analyze
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_controllers_BaseController_jni_1analyze
  (JNIEnv *, jobject, jlong cptr)
{
  jlong result (0);

  controllers::BaseController * current = (controllers::BaseController *) cptr;
  if (current)
    result = current->analyze ();

  return result;
}

/*
 * Class:     com_gams_controllers_BaseController
 * Method:    jni_execute
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_controllers_BaseController_jni_1execute
  (JNIEnv *, jobject, jlong cptr)
{
  jlong result (0);

  controllers::BaseController * current = (controllers::BaseController *) cptr;
  if (current)
    result = current->execute ();

  return result;
}

void JNICALL Java_com_gams_controllers_BaseController_jni_1addAlgorithmFactory
(JNIEnv * env, jobject, jlong cptr, jstring name, jobject factory)
{
  controllers::BaseController * current = (controllers::BaseController *) cptr;

  if (current)
  {
    // get the name in C string format
    const char * str_name = env->GetStringUTFChars (name, 0);

    std::vector<std::string> aliases;
    aliases.push_back (str_name);

    algorithms::JavaAlgorithmFactory * facade =
      new algorithms::JavaAlgorithmFactory (factory);

    current->add_algorithm_factory (aliases, facade);


    // clean up the allocated elements
    env->ReleaseStringUTFChars (name, str_name);
  }
}

/*
 * Class:     com_gams_controllers_BaseController
 * Method:    jni_initAccent
 * Signature: (JLjava/lang/String;Lcom/madara/KnowledgeList;)V
 */
void JNICALL Java_com_gams_controllers_BaseController_jni_1initAccent
  (JNIEnv * env, jobject, jlong cptr, jstring name, jlongArray argslist)
{
  controllers::BaseController * current = (controllers::BaseController *) cptr;

  if (current)
  {
    // get the name in C string format
    const char * str_name = env->GetStringUTFChars(name, 0);

    // create a knowledge vector to hold the arguments
    madara::knowledge::KnowledgeVector args (env->GetArrayLength (argslist));
    jlong * elements = env->GetLongArrayElements (argslist, 0);

    // iterate through arguments and copy the knowledge record for each arg
    for (size_t i = 0; i < args.size (); ++i)
    {
      madara::knowledge::KnowledgeRecord * cur_record = (madara::knowledge::KnowledgeRecord *)elements[i];

      if (cur_record)
        args[i] = madara::knowledge::KnowledgeRecord (*cur_record);
    }
    
    // call the initialization method
    current->init_accent (str_name, args);
    
    // clean up the allocated elements
    env->ReleaseLongArrayElements(argslist, elements, 0);
    env->ReleaseStringUTFChars(name, str_name);
  }
}

/*
 * Class:     com_gams_controllers_BaseController
 * Method:    jni_initAlgorithm
 * Signature: (JLjava/lang/String;[J)V
 */
void JNICALL Java_com_gams_controllers_BaseController_jni_1initAlgorithm__JLjava_lang_String_2_3J
  (JNIEnv * env, jobject, jlong cptr, jstring name, jlongArray argslist)
{
  controllers::BaseController * current = (controllers::BaseController *) cptr;

  if (current)
  {
    // get the name in C string format
    const char * str_name = env->GetStringUTFChars(name, 0);
    madara::knowledge::KnowledgeVector args;

    if (argslist)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::controllers::BaseController::init_algorithm (java):" \
        " initializing algorithm %s with %d args\n", str_name,
        (int)env->GetArrayLength (argslist));

      args.resize (env->GetArrayLength (argslist));

      // create a knowledge vector to hold the arguments
      jlong * elements = env->GetLongArrayElements (argslist, 0);

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::controllers::BaseController::init_algorithm (java):" \
        " iterating through %d args\n",
        (int)env->GetArrayLength (argslist));

      // iterate through arguments and copy the knowledge record for each arg
      for (size_t i = 0; i < args.size (); ++i)
      {
        madara::knowledge::KnowledgeRecord * cur_record = (madara::knowledge::KnowledgeRecord *)elements[i];

        if (cur_record)
          args[i] = madara::knowledge::KnowledgeRecord (*cur_record);
      }

      env->ReleaseLongArrayElements (argslist, elements, 0);
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::controllers::BaseController::init_algorithm (java):" \
        " initializing algorithm %s with no args\n", str_name);
    }


    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::init_algorithm (java):" \
      " calling C++ init algorithm\n");

    // call the initialization method
    current->init_algorithm (str_name, args);

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::controllers::BaseController::init_algorithm (java):" \
      " Releasing args list and string name\n");

    // clean up the allocated elements
    env->ReleaseStringUTFChars(name, str_name);
  }
}

/*
 * Class:     com_gams_controllers_BaseController
 * Method:    jni_initPlatform
 * Signature: (JLjava/lang/String;)V
 */
void JNICALL Java_com_gams_controllers_BaseController_jni_1initPlatform__JLjava_lang_String_2
  (JNIEnv * env, jobject, jlong cptr, jstring plat)
{
  controllers::BaseController * current = (controllers::BaseController *) cptr;

  if (current)
  {
    // get the name in C string format
    const char * str_name = env->GetStringUTFChars(plat, 0);

    // call the initialization method
    current->init_platform (str_name);
    
    // release the string
    env->ReleaseStringUTFChars(plat, str_name);
  }
}

/*
 * Class:     com_gams_controllers_BaseController
 * Method:    jni_initPlatform
 * Signature: (JLjava/lang/String;Lcom/madara/KnowledgeList;)V
 */
void JNICALL Java_com_gams_controllers_BaseController_jni_1initPlatform__JLjava_lang_String_2_3J
  (JNIEnv * env, jobject, jlong cptr, jstring name, jlongArray argslist)
{
  controllers::BaseController * current = (controllers::BaseController *) cptr;

  if (current)
  {
    // get the name in C string format
    const char * str_name = env->GetStringUTFChars(name, 0);

    // create a knowledge vector to hold the arguments
    madara::knowledge::KnowledgeVector args (env->GetArrayLength (argslist));
    jlong * elements = env->GetLongArrayElements (argslist, 0);

    // iterate through arguments and copy the knowledge record for each arg
    for (size_t i = 0; i < args.size (); ++i)
    {
      madara::knowledge::KnowledgeRecord * cur_record = (madara::knowledge::KnowledgeRecord *)elements[i];

      if (cur_record)
        args[i] = madara::knowledge::KnowledgeRecord (*cur_record);
    }

    // call the initialization method
    current->init_platform (str_name, args);

    // clean up the allocated elements
    env->ReleaseLongArrayElements(argslist, elements, 0);
    env->ReleaseStringUTFChars(name, str_name);
  }
}

/*
 * Class:     com_gams_controllers_BaseController
 * Method:    jni_initAlgorithm
 * Signature: (JLjava/lang/Object;)V
 */
void JNICALL Java_com_gams_controllers_BaseController_jni_1initAlgorithm__JLjava_lang_Object_2
  (JNIEnv *, jobject, jlong cptr, jobject algorithm)
{
  controllers::BaseController * current = (controllers::BaseController *) cptr;

  if (current)
  {
    current->init_algorithm (algorithm);
  }
}

/*
 * Class:     com_gams_controllers_BaseController
 * Method:    jni_initPlatform
 * Signature: (JLjava/lang/Object;)V
 */
void JNICALL Java_com_gams_controllers_BaseController_jni_1initPlatform__JLjava_lang_Object_2
  (JNIEnv *, jobject, jlong cptr, jobject platform)
{
  controllers::BaseController * current = (controllers::BaseController *) cptr;

  if (current)
  {
    current->init_platform (platform);
  }
}

/*
 * Class:     com_gams_controllers_BaseController
 * Method:    jni_getPlatform
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_controllers_BaseController_jni_1getPlatform
  (JNIEnv *, jobject, jlong cptr)
{
  jlong result (0);

  controllers::BaseController * current = (controllers::BaseController *) cptr;
  if (current)
  {
    result = (jlong) current->get_platform ();
  }

  return result;
}

/*
 * Class:     com_gams_controllers_BaseController
 * Method:    jni_getAlgorithm
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_controllers_BaseController_jni_1getAlgorithm
  (JNIEnv *, jobject, jlong cptr)
{
  jlong result (0);

  controllers::BaseController * current = (controllers::BaseController *) cptr;
  if (current)
  {
    result = (jlong) current->get_algorithm ();
  }

  return result;
}

/*
 * Class:     com_gams_controllers_BaseController
 * Method:    jni_initVars
 * Signature: (JJJ)V
 */
void JNICALL Java_com_gams_controllers_BaseController_jni_1initVars
  (JNIEnv *, jobject, jlong cptr, jlong id, jlong processes)
{
  Integer tempId (id), tempProcesses (processes);
  controllers::BaseController * current = (controllers::BaseController *) cptr;
  if (current)
    current->init_vars (tempId, tempProcesses);
}

/*
 * Class:     com_gams_controllers_BaseController
 * Method:    jni_initVarsAlgorithm
 * Signature: (JJ)V
 */
void JNICALL Java_com_gams_controllers_BaseController_jni_1initVarsAlgorithm
  (JNIEnv *, jobject, jlong cptr, jlong aptr)
{
  controllers::BaseController * current = (controllers::BaseController *) cptr;
  algorithms::BaseAlgorithm * algorithm = (algorithms::BaseAlgorithm *) aptr;

  if (current && algorithm)
  {
    current->init_vars (*algorithm);
  }
}

/*
 * Class:     com_gams_controllers_BaseController
 * Method:    jni_initVarsPlatform
 * Signature: (JJ)V
 */
void JNICALL Java_com_gams_controllers_BaseController_jni_1initVarsPlatform
  (JNIEnv *, jobject, jlong cptr, jlong pptr)
{
  controllers::BaseController * current = (controllers::BaseController *) cptr;
  platforms::BasePlatform * platform = (platforms::BasePlatform *) pptr;

  if (current && platform)
  {
    current->init_vars (*platform);
  }
}

/*
 * Class:     com_gams_controllers_BaseController
 * Method:    jni_monitor
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_controllers_BaseController_jni_1monitor
  (JNIEnv *, jobject, jlong cptr)
{
  jlong result (0);

  controllers::BaseController * current = (controllers::BaseController *) cptr;
  if (current)
    result = current->monitor ();

  return result;
}

/*
 * Class:     com_gams_controllers_BaseController
 * Method:    jni_plan
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_controllers_BaseController_jni_1plan
  (JNIEnv *, jobject, jlong cptr)
{
  jlong result (0);

  controllers::BaseController * current = (controllers::BaseController *) cptr;
  if (current)
    result = current->plan ();

  return result;
}

/*
 * Class:     com_gams_controllers_BaseController
 * Method:    jni_run
 * Signature: (JDD)J
 */
jlong JNICALL Java_com_gams_controllers_BaseController_jni_1run__JDD
  (JNIEnv *, jobject, jlong cptr, jdouble loop_period, jdouble duration)
{
  jlong result (0);

  controllers::BaseController * current = (controllers::BaseController *) cptr;
  if (current)
    result = current->run (loop_period, duration);

  return result;
}

/*
 * Class:     com_gams_controllers_BaseController
 * Method:    jni_run
 * Signature: (JDDD)J
 */
jlong JNICALL Java_com_gams_controllers_BaseController_jni_1run__JDDD
  (JNIEnv *, jobject, jlong cptr, jdouble loop_period, jdouble duration, jdouble send_period)
{
  jlong result (0);

  controllers::BaseController * current = (controllers::BaseController *) cptr;
  if (current)
    result = current->run (loop_period, duration, send_period);

  return result;
}

/*
 * Class:     com_gams_controllers_BaseController
 * Method:    jni_runHz
 * Signature: (JDDD)J
 */
jlong JNICALL Java_com_gams_controllers_BaseController_jni_1runHz
  (JNIEnv *, jobject, jlong cptr, jdouble loop_hz, jdouble duration, jdouble send_hz)
{
  jlong result (0);

  controllers::BaseController * current = (controllers::BaseController *) cptr;
  if (current)
    result = current->run_hz (loop_hz, duration, send_hz);

  return result;
}

/*
 * Class:     com_gams_controllers_BaseController
 * Method:    jni_systemAnalyze
 * Signature: (J)J
 */
jlong JNICALL Java_com_gams_controllers_BaseController_jni_1systemAnalyze
  (JNIEnv *, jobject, jlong cptr)
{
  jlong result (0);

  controllers::BaseController * current = (controllers::BaseController *) cptr;
  if (current)
    result = current->system_analyze ();

  return result;
}
