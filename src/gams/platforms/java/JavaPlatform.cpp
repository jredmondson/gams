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
#include "JavaPlatform.h"
#include "gams/utility/java/Acquire_VM.h"
#include "gams/loggers/GlobalLogger.h"


gams::platforms::JavaPlatform::JavaPlatform (
  jobject obj,
  madara::knowledge::KnowledgeBase * knowledge,
  variables::Sensors * sensors,
  variables::Platforms * platforms,
  variables::Self * self)
  : BasePlatform (knowledge, sensors, self)
{
  gams::utility::java::Acquire_VM jvm;
  
  if (jvm.env)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
       "gams::platforms::JavaPlatform::constructor:" \
      " initializing platform and status.\n");
  
    if (platforms && knowledge)
    {
      (*platforms)[get_id ()].init_vars (*knowledge, get_id ());
      status_ = (*platforms)[get_id ()];
    }

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
       "gams::platforms::JavaPlatform::constructor:" \
      " allocating global reference for object.\n");
  
    obj_ = (jobject) jvm.env->NewGlobalRef (obj);
    if (obj_)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
         "gams::platforms::JavaPlatform::constructor:" \
        " allocating global reference for object's class.\n");
      class_ = (jclass) jvm.env->NewGlobalRef (jvm.env->GetObjectClass (obj_));

      if (class_)
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MAJOR,
           "gams::platforms::JavaPlatform::constructor:" \
          " class and object obtained successfully.\n");
      }
      else
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_ERROR,
           "gams::platforms::JavaPlatform::constructor:" \
          " ERROR: class object inaccessible.\n");
      }
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
         "gams::platforms::JavaPlatform::constructor:" \
        " ERROR: object is invalid.\n");
    }
  }
}

gams::platforms::JavaPlatform::~JavaPlatform ()
{
  gams::utility::java::Acquire_VM jvm;
  if (jvm.env)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
       "gams::platforms::JavaPlatform::destructor:" \
      " Deleting global references.\n");

    jvm.env->DeleteGlobalRef (obj_);
    jvm.env->DeleteGlobalRef (class_);
  }
}

void
gams::platforms::JavaPlatform::operator= (const JavaPlatform & rhs)
{
  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_MAJOR,
     "gams::platforms::JavaPlatform::assignment:" \
    " Checking for source not being same as dest.\n");

  if (this != &rhs && obj_ != rhs.obj_)
  {
    gams::utility::java::Acquire_VM jvm;
    platforms::BasePlatform * dest = dynamic_cast <platforms::BasePlatform *> (this);
    const platforms::BasePlatform * source =
      dynamic_cast <const platforms::BasePlatform *> (&rhs);

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
       "gams::platforms::JavaPlatform::assignment:" \
      " Copying source to dest.\n");

    *dest = *source;

    if (jvm.env)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
         "gams::platforms::JavaPlatform::assignment:" \
        " Deleting global references.\n");

      jvm.env->DeleteGlobalRef (obj_);
      jvm.env->DeleteGlobalRef (class_);

      obj_ = jvm.env->NewGlobalRef (rhs.obj_);
      class_ = (jclass) jvm.env->NewGlobalRef (rhs.class_);
    }
  }
}
 
int
gams::platforms::JavaPlatform::analyze (void)
{
  gams::utility::java::Acquire_VM jvm;
  jint result (0);

  if (jvm.env)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::platforms::JavaPlatform::analyze:" \
      " Obtaining user-defined analyze method.\n");

    jmethodID call = jvm.env->GetMethodID (class_, "analyze", "()I");

    if (call)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::platforms::JavaPlatform::analyze:" \
        " Calling user-defined analyze method.\n");
      result = jvm.env->CallIntMethod (obj_, call);
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "gams::platforms::JavaPlatform::analyze:" \
        " ERROR: Unable to find user-defined analyze method.\n");
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::platforms::JavaPlatform::analyze:" \
      " ERROR: Unable to obtain JVM environment.\n");
  }

  return result;
}

double
gams::platforms::JavaPlatform::get_accuracy () const
{
  gams::utility::java::Acquire_VM jvm;
  jdouble result (0);

  if (jvm.env)
  {

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::platforms::JavaPlatform::get_accuracy:" \
      " Obtaining user-defined getAccuracy method.\n");

    jmethodID call = jvm.env->GetMethodID (class_, "getAccuracy", "()D");

    if (call)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::platforms::JavaPlatform::get_accuracy:" \
        " Calling user-defined getAccuracy method.\n");

      result = jvm.env->CallDoubleMethod (obj_, call);
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "gams::platforms::JavaPlatform::get_accuracy:" \
        " ERROR: Unable to find user-defined getAccuracy method.\n");
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::platforms::JavaPlatform::get_accuracy:" \
      " ERROR: Unable to obtain JVM environment.\n");
  }

  madara_logger_ptr_log (gams::loggers::global_logger.get (),
    gams::loggers::LOG_DETAILED,
    "gams::platforms::JavaPlatform::get_accuracy:" \
    " returning %f\n", result);

  return result;
}

std::string gams::platforms::JavaPlatform::get_id () const
{
  gams::utility::java::Acquire_VM jvm;
  std::string id;
  jstring result (0);

  if (jvm.env)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
       "gams::platforms::JavaPlatform::get_id:" \
      " Obtaining user-defined getId method.\n");

    jmethodID call = jvm.env->GetMethodID(class_, "getId", "()Ljava.lang.String;" );

    if (call)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
         "gams::platforms::JavaPlatform::get_id:" \
        " Calling user-defined getId method.\n");

      result = (jstring) jvm.env->CallObjectMethod (obj_, call);
      const char * id_chars = jvm.env->GetStringUTFChars(result, 0);
      id = id_chars;
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
         "gams::platforms::JavaPlatform::get_id:" \
        " ERROR: Unable to find user-defined getId method.\n");
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::platforms::JavaPlatform::get_id:" \
      " ERROR: Unable to obtain JVM environment.\n");
  }
  return id;
}

std::string gams::platforms::JavaPlatform::get_name () const
{
  gams::utility::java::Acquire_VM jvm;
  std::string name;
  jstring result (0);

  if (jvm.env)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
       "gams::platforms::JavaPlatform::get_name:" \
      " Obtaining user-defined getName method.\n");

    jmethodID call = jvm.env->GetMethodID(class_, "getName", "()Ljava.lang.String;" );

    if (call)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
         "gams::platforms::JavaPlatform::get_name:" \
        " Calling user-defined getName method.\n");

      result = (jstring) jvm.env->CallObjectMethod (obj_, call);
      const char * name_chars = jvm.env->GetStringUTFChars(result, 0);
      name = name_chars;
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
         "gams::platforms::JavaPlatform::get_name:" \
        " ERROR: Unable to find user-defined getName method.\n");
    }

  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::platforms::JavaPlatform::get_name:" \
      " ERROR: Unable to obtain JVM environment.\n");
  }

  return name;
}

double
gams::platforms::JavaPlatform::get_move_speed () const
{
  gams::utility::java::Acquire_VM jvm;
  jdouble result (0);

  if (jvm.env)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
      "gams::platforms::JavaPlatform::get_move_speed:" \
      " Obtaining user-defined getMoveSpeed method\n");

    jmethodID call = jvm.env->GetMethodID (class_, "getMoveSpeed", "()D");

    if (call)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
        "gams::platforms::JavaPlatform::get_move_speed:" \
        " Calling user-defined getMoveSpeed method.\n");

      result = jvm.env->CallDoubleMethod (obj_, call);
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
        "gams::platforms::JavaPlatform::get_move_speed:" \
        " ERROR: Unable to find user-defined getMoveSpeed method.\n");
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::platforms::JavaPlatform::get_move_speed:" \
      " ERROR: Unable to obtain JVM environment.\n");
  }

  return result;
}

int
gams::platforms::JavaPlatform::home (void)
{
  gams::utility::java::Acquire_VM jvm;
  jint result (0);

  if (jvm.env)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
       "gams::platforms::JavaPlatform::home:" \
      " Obtaining user-defined home method\n");

    jmethodID call = jvm.env->GetMethodID(class_, "home", "()I" );

    if (call)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
         "gams::platforms::JavaPlatform::home:" \
        " Calling user-defined home method.\n");

      result = jvm.env->CallIntMethod (obj_, call);
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
         "gams::platforms::JavaPlatform::home:" \
        " ERROR: Unable to find user-defined home method.\n");
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::platforms::JavaPlatform::home:" \
      " ERROR: Unable to obtain JVM environment.\n");
  }
  return result;
}

int
gams::platforms::JavaPlatform::land (void)
{
  gams::utility::java::Acquire_VM jvm;
  jint result (0);

  if (jvm.env)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
       "gams::platforms::JavaPlatform::land:" \
      " Obtaining user-defined land method\n");

    jmethodID call = jvm.env->GetMethodID(class_, "land", "()I" );

    if (call)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
         "gams::platforms::JavaPlatform::land:" \
        " Calling user-defined land method.\n");

      result = jvm.env->CallIntMethod (obj_, call);
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
         "gams::platforms::JavaPlatform::land:" \
        " ERROR: Unable to find user-defined land method.\n");
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::platforms::JavaPlatform::land:" \
      " ERROR: Unable to obtain JVM environment.\n");
  }

  return result;
}

int
gams::platforms::JavaPlatform::move (const pose::Position & position,
        const pose::PositionBounds &)
{
  gams::utility::java::Acquire_VM jvm;
  jint result (0);

  if (jvm.env)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
       "gams::platforms::JavaPlatform::move:" \
      " Obtaining user-defined move method\n");

    jmethodID move_call = jvm.env->GetMethodID(
      class_, "move", "(Lai/gams/utility/Position;D)I" );

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
       "gams::platforms::JavaPlatform::move:" \
      " Obtaining Position class and constructor\n");

    jclass pos_class = utility::java::find_class (jvm.env, "ai/gams/pose/Position");
    jmethodID pos_const = jvm.env->GetMethodID (pos_class, "<init>", "(DDD)V");
    jmethodID pos_free = jvm.env->GetMethodID (pos_class, "free", "()V");

    if (move_call)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MINOR,
         "gams::platforms::JavaPlatform::move:" \
        " Creating new position object.\n");

      jobject inpos = jvm.env->NewObject (
        pos_class, pos_const, position.x (), position.y (), position.z ());
      jdouble inepsilon (0.1); // TODO support bounds checking in Java

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
         "gams::platforms::JavaPlatform::move:" \
        " Calling user-defined move method.\n");

      result = jvm.env->CallIntMethod (obj_, move_call, inpos, inepsilon);

      jvm.env->CallVoidMethod (inpos, pos_free);

      jvm.env->DeleteLocalRef (inpos);
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
         "gams::platforms::JavaPlatform::move:" \
        " ERROR: Unable to find user-defined move method.\n");
    }

    jvm.env->DeleteWeakGlobalRef (pos_class);
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::platforms::JavaPlatform::analyze:" \
      " ERROR: Unable to obtain JVM environment.\n");
  }

  return result;
}

int
gams::platforms::JavaPlatform::orient (const pose::Orientation & axes,
    const pose::OrientationBounds &)
{
  gams::utility::java::Acquire_VM jvm;
  jint result (0);

  if (jvm.env)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
       "gams::platforms::JavaPlatform::rotate:" \
      " Obtaining user-defined rotate method\n");

    jmethodID move_call = jvm.env->GetMethodID (
      class_, "move", "(Lai/gams/utility/Position;D)I");

    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MINOR,
       "gams::platforms::JavaPlatform::rotate:" \
      " Obtaining Axes class and constructor\n");

    jclass axes_class = utility::java::find_class (jvm.env, "ai/gams/pose/Orientation");
    jmethodID axes_const = jvm.env->GetMethodID (axes_class, "<init>", "(DDD)V");
    jmethodID axes_free = jvm.env->GetMethodID (axes_class, "free", "()V");

    if (move_call)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MINOR,
         "gams::platforms::JavaPlatform::rotate:" \
        " Creating new axes object.\n");

      jobject inaxes = jvm.env->NewObject (
        axes_class, axes_const, axes.rx (), axes.ry (), axes.rz ());

      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
         "gams::platforms::JavaPlatform::rotate:" \
        " Calling user-defined rotate method.\n");

      result = jvm.env->CallIntMethod (obj_, move_call, inaxes);

      jvm.env->CallVoidMethod (inaxes, axes_free);
      jvm.env->DeleteLocalRef (inaxes);
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
         "gams::platforms::JavaPlatform::rotate:" \
        " ERROR: Unable to find user-defined rotate method.\n");
    }

    jvm.env->DeleteWeakGlobalRef (axes_class);
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::platforms::JavaPlatform::rotate:" \
      " ERROR: Unable to obtain JVM environment.\n");
  }

  return result;
}

int
gams::platforms::JavaPlatform::sense (void)
{
  gams::utility::java::Acquire_VM jvm;
  jint result (0);

  if (jvm.env)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
       "gams::platforms::JavaPlatform::sense:" \
      " Obtaining user-defined sense method\n");

    jmethodID call = jvm.env->GetMethodID(class_, "sense", "()I" );

    if (call)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
         "gams::platforms::JavaPlatform::sense:" \
        " Calling user-defined sense method.\n");

      result = jvm.env->CallIntMethod (obj_, call);
    }
    else
    {
      std::stringstream buffer;
      if (class_)
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_MINOR,
           "gams::platforms::JavaPlatform::sense:" \
          " Trying to acquire class name for error message\n");

        jmethodID getName = jvm.env->GetMethodID(
          class_, "getName", "()Ljava/lang/String;");
        jstring name = (jstring) jvm.env->CallObjectMethod(class_, getName);
        const char * name_chars = jvm.env->GetStringUTFChars(name, 0);

        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_ERROR,
           "gams::platforms::JavaPlatform::sense:" \
          " ERROR: No sense() method found in %s\n", name_chars);

      }
      else
      {
        madara_logger_ptr_log (gams::loggers::global_logger.get (),
          gams::loggers::LOG_ERROR,
           "gams::platforms::JavaPlatform::sense:" \
          " ERROR: Unable to acquire class from object.\n");
      }

      knowledge_->print (buffer.str ());
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::platforms::JavaPlatform::sense:" \
      " ERROR: Unable to obtain JVM environment.\n");
  }
  return result;
}

void
gams::platforms::JavaPlatform::set_move_speed (const double & speed)
{
  gams::utility::java::Acquire_VM jvm;

  if (jvm.env)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
       "gams::platforms::JavaPlatform::set_move_speed:" \
      " Obtaining user-defined setMoveSpeed method\n");

    jmethodID call = jvm.env->GetMethodID(class_, "setMoveSpeed", "(D)V" );

    if (call)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
         "gams::platforms::JavaPlatform::set_move_speed:" \
        " Calling user-defined setMoveSpeed method.\n");

      jdouble jspeed (speed);
      jvm.env->CallVoidMethod (obj_, call, jspeed);
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
         "gams::platforms::JavaPlatform::set_move_speed:" \
        " ERROR: Unable to find user-defined setMoveSpeed method.\n");
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::platforms::JavaPlatform::set_move_speed:" \
      " ERROR: Unable to obtain JVM environment.\n");
  }
}

int
gams::platforms::JavaPlatform::takeoff (void)
{
  gams::utility::java::Acquire_VM jvm;
  jint result (0);

  if (jvm.env)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
       "gams::platforms::JavaPlatform::takeoff:" \
      " Obtaining user-defined takeoff method\n");

    jmethodID call = jvm.env->GetMethodID(class_, "takeoff", "()I" );

    if (call)
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_MAJOR,
         "gams::platforms::JavaPlatform::takeoff:" \
        " Calling user-defined takeoff method.\n");

      result = jvm.env->CallIntMethod (obj_, call);
    }
    else
    {
      madara_logger_ptr_log (gams::loggers::global_logger.get (),
        gams::loggers::LOG_ERROR,
         "gams::platforms::JavaPlatform::takeoff:" \
        " ERROR: Unable to find user-defined takeoff method.\n");
    }
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::platforms::JavaPlatform::takeoff:" \
      " ERROR: Unable to obtain JVM environment.\n");
  }

  return result;
}

jobject
gams::platforms::JavaPlatform::get_java_instance (void)
{
  gams::utility::java::Acquire_VM jvm;
  jobject result (0);

  if (jvm.env)
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_MAJOR,
       "gams::platforms::JavaPlatform::get_java_instance:" \
      " Creating new local ref out of saved global reference.\n");

    result = jvm.env->NewLocalRef (obj_);
  }
  else
  {
    madara_logger_ptr_log (gams::loggers::global_logger.get (),
      gams::loggers::LOG_ERROR,
      "gams::platforms::JavaPlatform::analyze:" \
      " ERROR: Unable to obtain JVM environment.\n");
  }

  return result;
}
