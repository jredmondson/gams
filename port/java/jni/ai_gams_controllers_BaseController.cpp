
#include "ai_gams_controllers_BaseController.h"
#include "gams/controllers/BaseController.h"
#include "gams/platforms/java/JavaPlatform.h"
#include "gams/algorithms/java/JavaAlgorithm.h"
#include "gams/loggers/GlobalLogger.h"
#include "gams_jni.h"

namespace containers = madara::knowledge::containers;
namespace engine = madara::knowledge;
namespace controllers = gams::controllers;
namespace algorithms = gams::algorithms;
namespace platforms = gams::platforms;

typedef madara::knowledge::KnowledgeRecord::Integer  Integer;

/*
 * Class:     ai_gams_controllers_BaseController
 * Method:    jni_BaseControllerFromKb
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_controllers_BaseController_jni_1BaseControllerFromKb
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  engine::KnowledgeBase * kb = (engine::KnowledgeBase *) cptr;
  if (kb)
  {
    result = (jlong) new gams::controllers::BaseController (*kb);
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BaseController::BaseControllerFromKb: "
      "KB object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_controllers_BaseController
 * Method:    jni_BaseController
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_controllers_BaseController_jni_1BaseController
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result = 0;

  controllers::BaseController * input = (controllers::BaseController *) cptr;
  if (input)
  {
    result = (jlong) new controllers::BaseController (*input);
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BaseController::BaseControllerFromKb: "
      "BaseController object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_controllers_BaseController
 * Method:    jni_freeBaseController
 * Signature: (J)V
 */
void JNICALL Java_ai_gams_controllers_BaseController_jni_1freeBaseController
  (JNIEnv *, jclass, jlong cptr)
{
  delete (controllers::BaseController *) cptr;
}

/*
 * Class:     ai_gams_controllers_BaseController
 * Method:    jni_toString
 * Signature: (J)Ljava/lang/String;
 */
jstring JNICALL Java_ai_gams_controllers_BaseController_jni_1toString
  (JNIEnv * env, jobject, jlong cptr)
{
  jstring result (0);

  controllers::BaseController * current = (controllers::BaseController *) cptr;
  if (current)
    result = env->NewStringUTF("BaseController");
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BaseController::toString: "
      "BaseController object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_controllers_BaseController
 * Method:    jni_analyze
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_controllers_BaseController_jni_1analyze
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result (0);

  controllers::BaseController * current = (controllers::BaseController *) cptr;
  if (current)
    result = current->analyze ();
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BaseController::analyze: "
      "BaseController object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_controllers_BaseController
 * Method:    jni_execute
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_controllers_BaseController_jni_1execute
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result (0);

  controllers::BaseController * current = (controllers::BaseController *) cptr;
  if (current)
    result = current->execute ();
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BaseController::execute: "
      "BaseController object is released already");
  }

  return result;
}

void JNICALL Java_ai_gams_controllers_BaseController_jni_1addAlgorithmFactory
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
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BaseController::addAlgorithmFactory: "
      "BaseController object is released already");
  }
}

/*
 * Class:     ai_gams_controllers_BaseController
 * Method:    jni_initAccent
 * Signature: (JLjava/lang/String;Lai/madara/knowledge/KnowledgeList;)V
 */
void JNICALL Java_ai_gams_controllers_BaseController_jni_1initAccent
  (JNIEnv * env, jobject, jlong cptr, jstring name, jlongArray argslist)
{
  controllers::BaseController * current = (controllers::BaseController *) cptr;

  if (current)
  {
    // get the name in C string format
    const char * str_name = env->GetStringUTFChars(name, 0);

    int argsLen = env->GetArrayLength (argslist);

    // create a knowledge map to hold the arguments
    madara::knowledge::KnowledgeMap args;
    jlong * elements = env->GetLongArrayElements (argslist, 0);

    // iterate through arguments and copy the knowledge record for each arg
    for (int i = 0; i < argsLen; ++i)
    {
      madara::knowledge::KnowledgeRecord * cur_record = (madara::knowledge::KnowledgeRecord *)elements[i];

      if (cur_record)
      {
        std::stringstream s;
        s << i;
        args[s.str()] = madara::knowledge::KnowledgeRecord (*cur_record);
      }
    }
    
    // call the initialization method
    current->init_accent (str_name, args);
    
    // clean up the allocated elements
    env->ReleaseLongArrayElements(argslist, elements, 0);
    env->ReleaseStringUTFChars(name, str_name);
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BaseController::initAccent: "
      "BaseController object is released already");
  }
}

/*
 * Class:     ai_gams_controllers_BaseController
 * Method:    jni_initAlgorithm
 * Signature: (JLjava/lang/String;[J)V
 */
void JNICALL Java_ai_gams_controllers_BaseController_jni_1initAlgorithm__JLjava_lang_String_2_3J
  (JNIEnv * env, jobject, jlong cptr, jstring name, jlongArray argslist)
{
  controllers::BaseController * current = (controllers::BaseController *) cptr;

  if (current)
  {
    // get the name in C string format
    const char * str_name = env->GetStringUTFChars(name, 0);
    madara::knowledge::KnowledgeMap args;

    if (argslist)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::controllers::BaseController::init_algorithm (java):" \
        " initializing algorithm %s with %d args\n", str_name,
        (int)env->GetArrayLength (argslist));

      int argsLen = env->GetArrayLength (argslist);
      //args.resize (env->GetArrayLength (argslist));

      // create a knowledge vector to hold the arguments
      jlong * elements = env->GetLongArrayElements (argslist, 0);

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::controllers::BaseController::init_algorithm (java):" \
        " iterating through %d args\n",
        (int)env->GetArrayLength (argslist));

      // iterate through arguments and copy the knowledge record for each arg
      for (int i = 0; i < argsLen; ++i)
      {
        madara::knowledge::KnowledgeRecord * cur_record = (madara::knowledge::KnowledgeRecord *)elements[i];

        if (cur_record)
        {
          std::stringstream s;
          s << i;
          args[s.str()] = madara::knowledge::KnowledgeRecord (*cur_record);
        }
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
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BaseController::initAlgorithm: "
      "BaseController object is released already");
  }
}

/*
 * Class:     ai_gams_controllers_BaseController
 * Method:    jni_initPlatform
 * Signature: (JLjava/lang/String;)V
 */
void JNICALL Java_ai_gams_controllers_BaseController_jni_1initPlatform__JLjava_lang_String_2
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
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BaseController::initPlatform: "
      "BaseController object is released already");
  }
}

/*
 * Class:     ai_gams_controllers_BaseController
 * Method:    jni_initPlatform
 * Signature: (JLjava/lang/String;Lai/madara/knowledge/KnowledgeList;)V
 */
void JNICALL Java_ai_gams_controllers_BaseController_jni_1initPlatform__JLjava_lang_String_2_3J
  (JNIEnv * env, jobject, jlong cptr, jstring name, jlongArray argslist)
{
  controllers::BaseController * current = (controllers::BaseController *) cptr;

  if (current)
  {
    // get the name in C string format
    const char * str_name = env->GetStringUTFChars(name, 0);

    // create a knowledge vector to hold the arguments
    jlong * elements = env->GetLongArrayElements (argslist, 0);

    int argsLen = env->GetArrayLength (argslist);

    madara::knowledge::KnowledgeMap args;

    // iterate through arguments and copy the knowledge record for each arg
    for (int i = 0; i < argsLen; ++i)
    {
      madara::knowledge::KnowledgeRecord * cur_record =
        (madara::knowledge::KnowledgeRecord *)elements[i];

      if (cur_record)
      {
        std::stringstream s;
        s << i;
        args[s.str ()] = madara::knowledge::KnowledgeRecord (*cur_record);
      }
    }
    // call the initialization method
    current->init_platform (str_name, args);

    // clean up the allocated elements
    env->ReleaseLongArrayElements(argslist, elements, 0);
    env->ReleaseStringUTFChars(name, str_name);
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BaseController::initPlatform: "
      "BaseController object is released already");
  }
}

/*
 * Class:     ai_gams_controllers_BaseController
 * Method:    jni_initAlgorithm
 * Signature: (JLjava/lang/Object;)V
 */
void JNICALL Java_ai_gams_controllers_BaseController_jni_1initAlgorithm__JLjava_lang_Object_2
  (JNIEnv * env, jobject, jlong cptr, jobject algorithm)
{
  controllers::BaseController * current = (controllers::BaseController *) cptr;

  if (current)
  {
    current->init_algorithm (algorithm);
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BaseController::initAlgorithm: "
      "BaseController object is released already");
  }
}

/*
 * Class:     ai_gams_controllers_BaseController
 * Method:    jni_initPlatform
 * Signature: (JLjava/lang/Object;)V
 */
void JNICALL Java_ai_gams_controllers_BaseController_jni_1initPlatform__JLjava_lang_Object_2
  (JNIEnv * env, jobject, jlong cptr, jobject platform)
{
  controllers::BaseController * current = (controllers::BaseController *) cptr;

  if (current)
  {
    current->init_platform (platform);
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BaseController::initPlatform: "
      "BaseController object is released already");
  }
}

/*
 * Class:     ai_gams_controllers_BaseController
 * Method:    jni_getPlatform
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_controllers_BaseController_jni_1getPlatform
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result (0);

  controllers::BaseController * current = (controllers::BaseController *) cptr;
  if (current)
  {
    result = (jlong) current->get_platform ();
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BaseController::getPlatform: "
      "BaseController object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_controllers_BaseController
 * Method:    jni_getAlgorithm
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_controllers_BaseController_jni_1getAlgorithm
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result (0);

  controllers::BaseController * current = (controllers::BaseController *) cptr;
  if (current)
  {
    result = (jlong) current->get_algorithm ();
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BaseController::getAlgorithm: "
      "BaseController object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_controllers_BaseController
 * Method:    jni_initVars
 * Signature: (JJJ)V
 */
void JNICALL Java_ai_gams_controllers_BaseController_jni_1initVars
  (JNIEnv * env, jobject, jlong cptr, jlong id, jlong processes)
{
  Integer tempId (id), tempProcesses (processes);
  controllers::BaseController * current = (controllers::BaseController *) cptr;
  if (current)
    current->init_vars (tempId, tempProcesses);
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BaseController::initVars: "
      "BaseController object is released already");
  }
}

/*
 * Class:     ai_gams_controllers_BaseController
 * Method:    jni_initVarsAlgorithm
 * Signature: (JJ)V
 */
void JNICALL Java_ai_gams_controllers_BaseController_jni_1initVarsAlgorithm
  (JNIEnv * env, jobject, jlong cptr, jlong aptr)
{
  controllers::BaseController * current = (controllers::BaseController *) cptr;
  algorithms::BaseAlgorithm * algorithm = (algorithms::BaseAlgorithm *) aptr;

  if (current && algorithm)
  {
    current->init_vars (*algorithm);
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BaseController::initVarsAlgorithm: "
      "BaseController or algorithm object are released already");
  }
}

/*
 * Class:     ai_gams_controllers_BaseController
 * Method:    jni_initVarsPlatform
 * Signature: (JJ)V
 */
void JNICALL Java_ai_gams_controllers_BaseController_jni_1initVarsPlatform
  (JNIEnv * env, jobject, jlong cptr, jlong pptr)
{
  controllers::BaseController * current = (controllers::BaseController *) cptr;
  platforms::BasePlatform * platform = (platforms::BasePlatform *) pptr;

  if (current && platform)
  {
    current->init_vars (*platform);
  }
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BaseController::initVarsPlatform: "
      "BaseController or platform object are released already");
  }
}

/*
 * Class:     ai_gams_controllers_BaseController
 * Method:    jni_monitor
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_controllers_BaseController_jni_1monitor
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result (0);

  controllers::BaseController * current = (controllers::BaseController *) cptr;
  if (current)
    result = current->monitor ();
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BaseController::monitor: "
      "BaseController object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_controllers_BaseController
 * Method:    jni_plan
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_controllers_BaseController_jni_1plan
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result (0);

  controllers::BaseController * current = (controllers::BaseController *) cptr;
  if (current)
    result = current->plan ();
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BaseController::plan: "
      "BaseController object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_controllers_BaseController
 * Method:    jni_run
 * Signature: (JDD)J
 */
jlong JNICALL Java_ai_gams_controllers_BaseController_jni_1run__JDD
  (JNIEnv * env, jobject, jlong cptr, jdouble loop_period, jdouble duration)
{
  jlong result (0);

  controllers::BaseController * current = (controllers::BaseController *) cptr;
  if (current)
    result = current->run (loop_period, duration);
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BaseController::run: "
      "BaseController object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_controllers_BaseController
 * Method:    jni_run
 * Signature: (JDDD)J
 */
jlong JNICALL Java_ai_gams_controllers_BaseController_jni_1run__JDDD
  (JNIEnv * env, jobject, jlong cptr, jdouble loop_period, jdouble duration, jdouble send_period)
{
  jlong result (0);

  controllers::BaseController * current = (controllers::BaseController *) cptr;
  if (current)
    result = current->run (loop_period, duration, send_period);
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BaseController::run: "
      "BaseController object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_controllers_BaseController
 * Method:    jni_runHz
 * Signature: (JDDD)J
 */
jlong JNICALL Java_ai_gams_controllers_BaseController_jni_1runHz
  (JNIEnv * env, jobject, jlong cptr, jdouble loop_hz, jdouble duration, jdouble send_hz)
{
  jlong result (0);

  controllers::BaseController * current = (controllers::BaseController *) cptr;
  if (current)
    result = current->run_hz (loop_hz, duration, send_hz);
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BaseController::runHz: "
      "BaseController object is released already");
  }

  return result;
}

/*
 * Class:     ai_gams_controllers_BaseController
 * Method:    jni_systemAnalyze
 * Signature: (J)J
 */
jlong JNICALL Java_ai_gams_controllers_BaseController_jni_1systemAnalyze
  (JNIEnv * env, jobject, jlong cptr)
{
  jlong result (0);

  controllers::BaseController * current = (controllers::BaseController *) cptr;
  if (current)
    result = current->system_analyze ();
  else
  {
    // user has tried to use a deleted object. Clean up and throw
    
    gams::utility::java::throw_dead_obj_exception(env,
      "BaseController::systemAnalyze: "
      "BaseController object is released already");
  }

  return result;
}
