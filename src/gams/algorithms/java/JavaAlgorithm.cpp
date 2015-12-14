/**
 * Copyright (c) 2014 Carnegie Mellon University. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following acknowledgments and disclaimers.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. The names "Carnegie Mellon University," "SEI" and/or "Software
 *    Engineering Institute" shall not be used to endorse or promote products
 *    derived from this software without prior written permission. For written
 *    permission, please contact permission@sei.cmu.edu.
 * 
 * 4. Products derived from this software may not be called "SEI" nor may "SEI"
 *    appear in their names without prior written permission of
 *    permission@sei.cmu.edu.
 * 
 * 5. Redistributions of any form whatsoever must retain the following
 *    acknowledgment:
 * 
 *      This material is based upon work funded and supported by the Department
 *      of Defense under Contract No. FA8721-05-C-0003 with Carnegie Mellon
 *      University for the operation of the Software Engineering Institute, a
 *      federally funded research and development center. Any opinions,
 *      findings and conclusions or recommendations expressed in this material
 *      are those of the author(s) and do not necessarily reflect the views of
 *      the United States Department of Defense.
 * 
 *      NO WARRANTY. THIS CARNEGIE MELLON UNIVERSITY AND SOFTWARE ENGINEERING
 *      INSTITUTE MATERIAL IS FURNISHED ON AN "AS-IS" BASIS. CARNEGIE MELLON
 *      UNIVERSITY MAKES NO WARRANTIES OF ANY KIND, EITHER EXPRESSED OR
 *      IMPLIED, AS TO ANY MATTER INCLUDING, BUT NOT LIMITED TO, WARRANTY OF
 *      FITNESS FOR PURPOSE OR MERCHANTABILITY, EXCLUSIVITY, OR RESULTS
 *      OBTAINED FROM USE OF THE MATERIAL. CARNEGIE MELLON UNIVERSITY DOES
 *      NOT MAKE ANY WARRANTY OF ANY KIND WITH RESPECT TO FREEDOM FROM PATENT,
 *      TRADEMARK, OR COPYRIGHT INFRINGEMENT.
 * 
 *      This material has been approved for public release and unlimited
 *      distribution.
 **/
#include "JavaAlgorithm.h"
#include "gams/utility/java/Acquire_VM.h"
#include "gams/loggers/GlobalLogger.h"

gams::algorithms::JavaAlgorithmFactory::JavaAlgorithmFactory (
  jobject obj)
{
  gams::utility::java::Acquire_VM jvm;

  if (jvm.env)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::JavaAlgorithmFactory::constructor:" \
      " allocating global reference for object.\n");

    obj_ = (jobject)jvm.env->NewGlobalRef (obj);

    if (obj_ == 0)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "gams::algorithms::JavaAlgorithmFactory::constructor:" \
        " ERROR: object is invalid.\n");
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::algorithms::JavaAlgorithmFactory::constructor:" \
      " ERROR: unable to acquire JAVA environment.\n");
  }
}


gams::algorithms::JavaAlgorithmFactory::~JavaAlgorithmFactory ()
{
  gams::utility::java::Acquire_VM jvm;

  if (jvm.env)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::JavaAlgorithmFactory::destructor:" \
      " deleting factory object.\n");

    jvm.env->DeleteGlobalRef (obj_);
  }
}

gams::algorithms::BaseAlgorithm *
gams::algorithms::JavaAlgorithmFactory::create (
  const madara::knowledge::KnowledgeVector & args,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * /*platform*/,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Agents * agents)
{
  // Acquire the Java virtual machine
  gams::utility::java::Acquire_VM jvm;

  gams::algorithms::BaseAlgorithm * result (0);

  if (jvm.env)
  {
    // obtain class handles
    jclass kb_class = gams::utility::java::find_class (
      jvm.env, "com/madara/KnowledgeBase");
    jclass list_class = gams::utility::java::find_class (
      jvm.env, "com/madara/KnowledgeList");

    // get method call ids that we will need
    jmethodID listConstructor = jvm.env->GetMethodID (list_class,
      "<init>", "([J)V");
    jmethodID kbFromPointerCall = jvm.env->GetStaticMethodID (kb_class,
      "fromPointer", "(J)Lcom/madara/KnowledgeBase;");
    jmethodID factoryCreateCall = jvm.env->GetMethodID (kb_class,
      "create", "(J)Lcom/gams/algorithms/BaseAlgorithm;");

    if (factoryCreateCall)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::JavaAlgorithmFactory::create:"
        " Java class has a create method.\n");
      // create the list of args

      jlongArray ret = jvm.env->NewLongArray ((jsize)args.size ());
      jlong * tmp = new jlong[(jsize)args.size ()];

      for (unsigned int x = 0; x < args.size (); x++)
      {
        tmp[x] = (jlong)args[x].clone ();
      }

      jvm.env->SetLongArrayRegion (ret, 0, (jsize)args.size (), tmp);
      delete[] tmp;

      // create the KnowledgeList
      jobject list = jvm.env->NewObject (list_class, listConstructor, ret);

      // create the KnowledgeBase
      jobject jknowledge = jvm.env->CallStaticObjectMethod (kb_class,
        kbFromPointerCall, (jlong)knowledge);

      // get the factory's class
      jclass filter_class = jvm.env->GetObjectClass (obj_);

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::JavaAlgorithmFactory::create:"
        " Calling create method on Java object.\n");

      jobject obj = jvm.env->CallObjectMethod (
        obj_, factoryCreateCall, list, jknowledge);

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::JavaAlgorithmFactory::create:"
        " Creating Java algorithm instance for controller.\n");

      result = new JavaAlgorithm (obj, knowledge, 0, sensors, self, agents);

      jvm.env->DeleteLocalRef (obj);
      jvm.env->DeleteLocalRef (filter_class);
      jvm.env->DeleteLocalRef (jknowledge);
      jvm.env->DeleteLocalRef (list);
      jvm.env->DeleteLocalRef (ret);
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "gams::algorithms::JavaAlgorithmFactory::create:"
        " Java class does not have a create method.\n");
    }
    jvm.env->DeleteWeakGlobalRef (kb_class);
    jvm.env->DeleteWeakGlobalRef (list_class);
  }

  return result;
}

gams::algorithms::JavaAlgorithm::JavaAlgorithm (
  jobject obj,
  madara::knowledge::KnowledgeBase * knowledge,
  platforms::BasePlatform * platform,
  variables::Sensors * sensors,
  variables::Self * self,
  variables::Agents * agents)
  : BaseAlgorithm (knowledge, platform, sensors, self, agents)
{
  gams::utility::java::Acquire_VM jvm;

  if (jvm.env)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
       "gams::algorithms::JavaAlgorithm::constructor:" \
      " allocating global reference for object.\n");
  
    obj_ = (jobject) jvm.env->NewGlobalRef (obj);

    if (obj_)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
         "gams::algorithms::JavaAlgorithm::constructor:" \
        " allocating global reference for object's class.\n");
      class_ = (jclass) jvm.env->NewGlobalRef (jvm.env->GetObjectClass (obj_));
      if (class_)
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
           "gams::algorithms::JavaAlgorithm::constructor:" \
          " class and object obtained successfully.\n");
      }
      else
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_ERROR,
           "gams::algorithms::JavaAlgorithm::constructor:" \
          " ERROR: class object inaccessible.\n");
      }
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
         "gams::algorithms::JavaAlgorithm::constructor:" \
        " ERROR: object is invalid.\n");
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
       "gams::algorithms::JavaAlgorithm::constructor:" \
      " ERROR: unable to acquire JAVA environment.\n");
  }
}

gams::algorithms::JavaAlgorithm::~JavaAlgorithm ()
{
  gams::utility::java::Acquire_VM jvm;
  if (jvm.env)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
       "gams::algorithms::JavaAlgorithm::destructor:" \
      " Deleting global references.\n");

    jvm.env->DeleteGlobalRef (obj_);
    jvm.env->DeleteGlobalRef (class_);
  }
}

void
gams::algorithms::JavaAlgorithm::operator= (const JavaAlgorithm & rhs)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
     "gams::algorithms::JavaAlgorithm::assignment:" \
    " Checking for source not being same as dest.\n");

  if (this != &rhs && obj_ != rhs.obj_)
  {
    gams::utility::java::Acquire_VM jvm;
    algorithms::BaseAlgorithm * dest = dynamic_cast <algorithms::BaseAlgorithm *> (this);
    const algorithms::BaseAlgorithm * source =
      dynamic_cast <const algorithms::BaseAlgorithm *> (&rhs);

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
       "gams::algorithms::JavaAlgorithm::assignment:" \
      " Copying source to dest.\n");

    *dest = *source;

    if (jvm.env)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
         "gams::algorithms::JavaAlgorithm::assignment:" \
        " Deleting global references.\n");

      jvm.env->DeleteGlobalRef (obj_);
      jvm.env->DeleteGlobalRef (class_);

      obj_ = jvm.env->NewGlobalRef (rhs.obj_);
      class_ = (jclass) jvm.env->NewGlobalRef (rhs.class_);
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "gams::algorithms::JavaAlgorithm::assignment:" \
        " ERROR: unable to acquire JAVA environment.\n");
    }
  }
}
 
int
gams::algorithms::JavaAlgorithm::analyze (void)
{
  gams::utility::java::Acquire_VM jvm;
  jint result (0);

  if (jvm.env)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::JavaAlgorithm::analyze:" \
      " Obtaining user-defined analyze method.\n");

    jmethodID call = jvm.env->GetMethodID (class_, "analyze", "()I");

    if (call)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::JavaAlgorithm::analyze:" \
        " Calling user-defined analyze method.\n");
      result = jvm.env->CallIntMethod (obj_, call);
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "gams::algorithms::JavaAlgorithm::analyze:" \
        " ERROR: Unable to find user-defined analyze method.\n");
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::algorithms::JavaAlgorithm::analyze:" \
      " ERROR: unable to acquire JAVA environment.\n");
  }

  return result;
}

std::string gams::algorithms::JavaAlgorithm::get_id () const
{
  gams::utility::java::Acquire_VM jvm;
  std::string id;
  jstring result (0);

  if (jvm.env)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::JavaAlgorithm::get_id:" \
      " Obtaining user-defined getId method.\n");

    jmethodID call = jvm.env->GetMethodID (class_, "getId", "()Ljava.lang.String;");

    if (call)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::JavaAlgorithm::get_id:" \
        " Calling user-defined getId method.\n");

      result = (jstring)jvm.env->CallObjectMethod (obj_, call);
      const char * id_chars = jvm.env->GetStringUTFChars (result, 0);
      id = id_chars;
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "gams::algorithms::JavaAlgorithm::get_id:" \
        " ERROR: Unable to find user-defined getId method.\n");
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::algorithms::JavaAlgorithm::get_id:" \
      " ERROR: unable to acquire JAVA environment.\n");
  }

  return id;
}

std::string gams::algorithms::JavaAlgorithm::get_name () const
{
  gams::utility::java::Acquire_VM jvm;
  std::string name;
  jstring result (0);

  if (jvm.env)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::JavaAlgorithm::get_name:" \
      " Obtaining user-defined getName method.\n");

    jmethodID call = jvm.env->GetMethodID (class_, "getName", "()Ljava.lang.String;");

    if (call)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::JavaAlgorithm::get_name:" \
        " Calling user-defined getName method.\n");

      result = (jstring)jvm.env->CallObjectMethod (obj_, call);
      const char * name_chars = jvm.env->GetStringUTFChars (result, 0);
      name = name_chars;
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::JavaAlgorithm::get_name:" \
        " ERROR: Unable to find user-defined getName method.\n");
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::algorithms::JavaAlgorithm::get_name:" \
      " ERROR: unable to acquire JAVA environment.\n");
  }

  return name;
}

int
gams::algorithms::JavaAlgorithm::execute (void)
{
  gams::utility::java::Acquire_VM jvm;
  jint result (0);

  if (jvm.env)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::JavaAlgorithm::execute:" \
      " Obtaining user-defined execute method.\n");

    jmethodID call = jvm.env->GetMethodID (class_, "execute", "()I");

    if (call)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::JavaAlgorithm::execute:" \
        " Calling user-defined execute method.\n");

      result = jvm.env->CallIntMethod (obj_, call);
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "gams::algorithms::JavaAlgorithm::execute:" \
        " ERROR: Unable to find user-defined execute method.\n");
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::algorithms::JavaAlgorithm::execute:" \
      " ERROR: unable to acquire JAVA environment.\n");
  }

  return result;
}

int
gams::algorithms::JavaAlgorithm::plan (void)
{
  gams::utility::java::Acquire_VM jvm;
  jint result (0);

  if (jvm.env)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::JavaAlgorithm::plan:" \
      " Obtaining user-defined plan method.\n");

    jmethodID call = jvm.env->GetMethodID (class_, "plan", "()I");

    if (call)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::algorithms::JavaAlgorithm::plan:" \
        " Calling user-defined plan method.\n");

      result = jvm.env->CallIntMethod (obj_, call);
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "gams::algorithms::JavaAlgorithm::plan:" \
        " ERROR: Unable to find user-defined plan method.\n");
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::algorithms::JavaAlgorithm::plan:" \
      " ERROR: unable to acquire JAVA environment.\n");
  }

  return result;
}

jobject
gams::algorithms::JavaAlgorithm::get_java_instance (void)
{
  gams::utility::java::Acquire_VM jvm;
  jobject result (0);

  if (jvm.env)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::algorithms::JavaAlgorithm::get_java_instance:" \
      " Creating new local ref out of saved global reference.\n");

    result = jvm.env->NewLocalRef (obj_);
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::algorithms::JavaAlgorithm::get_java_instance:" \
      " ERROR: unable to acquire JAVA environment.\n");
  }

  return result;
}
