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
 * 3. The names “Carnegie Mellon University,” "SEI” and/or “Software
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
 *      INSTITUTE MATERIAL IS FURNISHED ON AN “AS-IS” BASIS. CARNEGIE MELLON
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
#include "Java_Platform.h"
#include "gams/utility/java/Acquire_VM.h"
#include "gams/utility/Logging.h"


gams::platforms::Java_Platform::Java_Platform (
  jobject obj,
  Madara::Knowledge_Engine::Knowledge_Base * knowledge,
  variables::Sensors * sensors,
  variables::Platforms * platforms,
  variables::Self * self)
  : Base (knowledge, sensors, self)
{
  gams::utility::java::Acquire_VM jvm;
  
  if (jvm.env)
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::platforms::Java_Platform::constructor:" \
      " initializing platform and status.\n"));
  
    if (platforms && knowledge)
    {
      (*platforms)[get_id ()].init_vars (*knowledge, get_id ());
      status_ = (*platforms)[get_id ()];
    }
  
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::platforms::Java_Platform::constructor:" \
      " allocating global reference for object.\n"));
  
    obj_ = (jobject) jvm.env->NewGlobalRef (obj);
    if (obj_)
    {
      GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
        DLINFO "gams::platforms::Java_Platform::constructor:" \
        " allocating global reference for object's class.\n"));
      class_ = (jclass) jvm.env->NewGlobalRef (jvm.env->GetObjectClass (obj_));

      if (class_)
      {
        GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
          DLINFO "gams::platforms::Java_Platform::constructor:" \
          " class and object obtained successfully.\n"));
      }
      else
      {
        GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
          DLINFO "gams::platforms::Java_Platform::constructor:" \
          " ERROR: class object inaccessible.\n"));
      }
    }
    else
    {
      GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
        DLINFO "gams::platforms::Java_Platform::constructor:" \
        " ERROR: object is invalid.\n"));
    }
  }
}

gams::platforms::Java_Platform::~Java_Platform ()
{
  gams::utility::java::Acquire_VM jvm;
  if (jvm.env)
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::platforms::Java_Platform::destructor:" \
      " Deleting global references.\n"));

    jvm.env->DeleteGlobalRef (obj_);
    jvm.env->DeleteGlobalRef (class_);
  }
}

void
gams::platforms::Java_Platform::operator= (const Java_Platform & rhs)
{
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::platforms::Java_Platform::assignment:" \
    " Checking for source not being same as dest.\n"));

  if (this != &rhs && obj_ != rhs.obj_)
  {
    gams::utility::java::Acquire_VM jvm;
    platforms::Base * dest = dynamic_cast <platforms::Base *> (this);
    const platforms::Base * source =
      dynamic_cast <const platforms::Base *> (&rhs);
    
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::platforms::Java_Platform::assignment:" \
      " Copying source to dest.\n"));

    *dest = *source;

    if (jvm.env)
    {
      GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
        DLINFO "gams::platforms::Java_Platform::assignment:" \
        " Deleting global references.\n"));

      jvm.env->DeleteGlobalRef (obj_);
      jvm.env->DeleteGlobalRef (class_);

      obj_ = jvm.env->NewGlobalRef (rhs.obj_);
      class_ = (jclass) jvm.env->NewGlobalRef (rhs.class_);
    }
  }
}
 
int
gams::platforms::Java_Platform::analyze (void)
{
  gams::utility::java::Acquire_VM jvm;
  jint result (0);
  
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::platforms::Java_Platform::analyze:" \
    " Obtaining user-defined analyze method.\n"));

  jmethodID call = jvm.env->GetMethodID(class_, "analyze", "()I" );

  if (call)
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::platforms::Java_Platform::analyze:" \
      " Calling user-defined analyze method.\n"));
    result = jvm.env->CallIntMethod (obj_, call);
  }
  else
  {
    GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
      DLINFO "gams::platforms::Java_Platform::analyze:" \
      " ERROR: Unable to find user-defined analyze method.\n"));
  }

  return result;
}

double
gams::platforms::Java_Platform::get_accuracy () const
{
  gams::utility::java::Acquire_VM jvm;
  jdouble result (0);
  
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::platforms::Java_Platform::get_gps_accuracy:" \
    " Obtaining user-defined getGpsAccuracy method.\n"));

  jmethodID call = jvm.env->GetMethodID(class_, "getGpsAccuracy", "()D" );

  if (call)
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::platforms::Java_Platform::get_gps_accuracy:" \
      " Calling user-defined getGpsAccuracy method.\n"));

    result = jvm.env->CallDoubleMethod (obj_, call);
  }
  else
  {
    GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
      DLINFO "gams::platforms::Java_Platform::get_gps_accuracy:" \
      " ERROR: Unable to find user-defined getGpsAccuracy method.\n"));
  }

  return result;
}

std::string gams::platforms::Java_Platform::get_id () const
{
  gams::utility::java::Acquire_VM jvm;
  std::string id;
  jstring result (0);
  
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::platforms::Java_Platform::get_id:" \
    " Obtaining user-defined getId method.\n"));

  jmethodID call = jvm.env->GetMethodID(class_, "getId", "()Ljava.lang.String;" );

  if (call)
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::platforms::Java_Platform::get_id:" \
      " Calling user-defined getId method.\n"));

    result = (jstring) jvm.env->CallObjectMethod (obj_, call);
    const char * id_chars = jvm.env->GetStringUTFChars(result, 0);
    id = id_chars;
  }
  else
  {
    GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
      DLINFO "gams::platforms::Java_Platform::get_id:" \
      " ERROR: Unable to find user-defined getId method.\n"));
  }

  return id;
}

std::string gams::platforms::Java_Platform::get_name () const
{
  gams::utility::java::Acquire_VM jvm;
  std::string name;
  jstring result (0);
  
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::platforms::Java_Platform::get_name:" \
    " Obtaining user-defined getName method.\n"));

  jmethodID call = jvm.env->GetMethodID(class_, "getName", "()Ljava.lang.String;" );

  if (call)
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::platforms::Java_Platform::get_name:" \
      " Calling user-defined getName method.\n"));

    result = (jstring) jvm.env->CallObjectMethod (obj_, call);
    const char * name_chars = jvm.env->GetStringUTFChars(result, 0);
    name = name_chars;
  }
  else
  {
    GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
      DLINFO "gams::platforms::Java_Platform::get_name:" \
      " ERROR: Unable to find user-defined getName method.\n"));
  }

  return name;
}

double
gams::platforms::Java_Platform::get_move_speed () const
{
  gams::utility::java::Acquire_VM jvm;
  jdouble result (0);
  
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::platforms::Java_Platform::get_move_speed:" \
    " Obtaining user-defined getMoveSpeed method\n"));

  jmethodID call = jvm.env->GetMethodID(class_, "getMoveSpeed", "()D" );

  if (call)
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::platforms::Java_Platform::get_move_speed:" \
      " Calling user-defined getMoveSpeed method.\n"));

    result = jvm.env->CallDoubleMethod (obj_, call);
  }
  else
  {
    GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
      DLINFO "gams::platforms::Java_Platform::get_move_speed:" \
      " ERROR: Unable to find user-defined getMoveSpeed method.\n"));
  }

  return result;
}

int
gams::platforms::Java_Platform::home (void)
{
  gams::utility::java::Acquire_VM jvm;
  jint result (0);
  
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::platforms::Java_Platform::home:" \
    " Obtaining user-defined home method\n"));

  jmethodID call = jvm.env->GetMethodID(class_, "home", "()I" );

  if (call)
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::platforms::Java_Platform::home:" \
      " Calling user-defined home method.\n"));

    result = jvm.env->CallIntMethod (obj_, call);
  }
  else
  {
    GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
      DLINFO "gams::platforms::Java_Platform::home:" \
      " ERROR: Unable to find user-defined home method.\n"));
  }

  return result;
}

int
gams::platforms::Java_Platform::land (void)
{
  gams::utility::java::Acquire_VM jvm;
  jint result (0);
  
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::platforms::Java_Platform::land:" \
    " Obtaining user-defined land method\n"));

  jmethodID call = jvm.env->GetMethodID(class_, "land", "()I" );

  if (call)
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::platforms::Java_Platform::land:" \
      " Calling user-defined land method.\n"));

    result = jvm.env->CallIntMethod (obj_, call);
  }
  else
  {
    GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
      DLINFO "gams::platforms::Java_Platform::land:" \
      " ERROR: Unable to find user-defined land method.\n"));
  }

  return result;
}

int
gams::platforms::Java_Platform::move (const utility::Position & position,
  const double & epsilon)
{
  gams::utility::java::Acquire_VM jvm;
  jint result (0);
  
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::platforms::Java_Platform::move:" \
    " Obtaining user-defined move method\n"));

  jmethodID move_call = jvm.env->GetMethodID(
    class_, "move", "(Lcom.gams.utility.Position;D)I" );
  
  GAMS_DEBUG (gams::utility::LOG_MINOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::platforms::Java_Platform::move:" \
    " Obtaining Position class and constructor\n"));

  jclass pos_class = jvm.env->FindClass ("com.gams.utility.Position");
  jmethodID pos_const = jvm.env->GetMethodID(pos_class, "<init>", "(JJJ)V");

  if (move_call)
  {
    GAMS_DEBUG (gams::utility::LOG_MINOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::platforms::Java_Platform::move:" \
      " Creating new position object.\n"));

    jobject inpos = jvm.env->NewObject (
      pos_class, pos_const, position.x, position.y, position.z);
    jdouble inepsilon (epsilon);
    
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::platforms::Java_Platform::move:" \
      " Calling user-defined move method.\n"));

    result = jvm.env->CallIntMethod (obj_, move_call, inpos, inepsilon);
  }
  else
  {
    GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
      DLINFO "gams::platforms::Java_Platform::move:" \
      " ERROR: Unable to find user-defined move method.\n"));
  }

  return result;
}

int
gams::platforms::Java_Platform::sense (void)
{
  gams::utility::java::Acquire_VM jvm;
  jint result (0);

  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::platforms::Java_Platform::sense:" \
    " Obtaining user-defined sense method\n"));

  jmethodID call = jvm.env->GetMethodID(class_, "sense", "()I" );

  if (call)
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::platforms::Java_Platform::sense:" \
      " Calling user-defined sense method.\n"));

    result = jvm.env->CallIntMethod (obj_, call);
  }
  else
  {
    std::stringstream buffer;
    if (class_)
    {
      GAMS_DEBUG (gams::utility::LOG_MINOR_EVENT, (LM_DEBUG, 
        DLINFO "gams::platforms::Java_Platform::sense:" \
        " Trying to acquire class name for error message\n"));

      jmethodID getName = jvm.env->GetMethodID(
        class_, "getName", "()Ljava/lang/String;");
      jstring name = (jstring) jvm.env->CallObjectMethod(class_, getName);
      const char * name_chars = jvm.env->GetStringUTFChars(name, 0);

      GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
        DLINFO "gams::platforms::Java_Platform::sense:" \
        " ERROR: No sense() method found in %s\n", name_chars));

    }
    else
    {
      GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
        DLINFO "gams::platforms::Java_Platform::sense:" \
        " ERROR: Unable to acquire class from object.\n"));
    }

    knowledge_->print (buffer.str ());
  }

  return result;
}

void
gams::platforms::Java_Platform::set_move_speed (const double & speed)
{
  gams::utility::java::Acquire_VM jvm;
  
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::platforms::Java_Platform::set_move_speed:" \
    " Obtaining user-defined setMoveSpeed method\n"));

  jmethodID call = jvm.env->GetMethodID(class_, "setMoveSpeed", "(D)V" );

  if (call)
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::platforms::Java_Platform::set_move_speed:" \
      " Calling user-defined setMoveSpeed method.\n"));

    jdouble jspeed (speed);
    jvm.env->CallVoidMethod (obj_, call, jspeed);
  }
  else
  {
    GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
      DLINFO "gams::platforms::Java_Platform::set_move_speed:" \
      " ERROR: Unable to find user-defined setMoveSpeed method.\n"));
  }
}

int
gams::platforms::Java_Platform::takeoff (void)
{
  gams::utility::java::Acquire_VM jvm;
  jint result (0);
  
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::platforms::Java_Platform::takeoff:" \
    " Obtaining user-defined takeoff method\n"));

  jmethodID call = jvm.env->GetMethodID(class_, "takeoff", "()I" );

  if (call)
  {
    GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
      DLINFO "gams::platforms::Java_Platform::takeoff:" \
      " Calling user-defined takeoff method.\n"));

    result = jvm.env->CallIntMethod (obj_, call);
  }
  else
  {
    GAMS_DEBUG (gams::utility::LOG_EMERGENCY, (LM_DEBUG, 
      DLINFO "gams::platforms::Java_Platform::takeoff:" \
      " ERROR: Unable to find user-defined takeoff method.\n"));
  }

  return result;
}

jobject
gams::platforms::Java_Platform::get_java_instance (void)
{
  gams::utility::java::Acquire_VM jvm;
  jobject result (0);
  
  GAMS_DEBUG (gams::utility::LOG_MAJOR_EVENT, (LM_DEBUG, 
    DLINFO "gams::platforms::Java_Platform::get_java_instance:" \
    " Creating new local ref out of saved global reference.\n"));

  result = jvm.env->NewLocalRef (obj_);
  return result;
}
