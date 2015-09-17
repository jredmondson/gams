/*********************************************************************
 * Copyright (c) 2013-2015 Carnegie Mellon University. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following acknowledgments and disclaimers.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 
 * 3. The names "Carnegie Mellon University," "SEI" and/or
 * "Software Engineering Institute" shall not be used to endorse or promote
 * products derived from this software without prior written permission. For
 * written permission, please contact permission@sei.cmu.edu.
 * 
 * 4. Products derived from this software may not be called "SEI" nor may "SEI"
 * appear in their names without prior written permission of
 * permission@sei.cmu.edu.
 *
 * 5. Redistributions of any form whatsoever must retain the following
 * acknowledgment:
 *
 * This material is based upon work funded and supported by the Department of
 * Defense under Contract No. FA8721-05-C-0003 with Carnegie Mellon University
 * for the operation of the Software Engineering Institute, a federally funded
 * research and development center. Any opinions, findings and conclusions or
 * recommendations expressed in this material are those of the author(s) and
 * do not necessarily reflect the views of the United States Department of
 * Defense.
 * 
 * NO WARRANTY. THIS CARNEGIE MELLON UNIVERSITY AND SOFTWARE ENGINEERING
 * INSTITUTE MATERIAL IS FURNISHED ON AN "AS-IS" BASIS. CARNEGIE MELLON
 * UNIVERSITY MAKES NO WARRANTIES OF ANY KIND, EITHER EXPRESSED OR IMPLIED,
 * AS TO ANY MATTER INCLUDING, BUT NOT LIMITED TO, WARRANTY OF FITNESS FOR
 * PURPOSE OR MERCHANTABILITY, EXCLUSIVITY, OR RESULTS OBTAINED FROM USE OF THE
 * MATERIAL. CARNEGIE MELLON UNIVERSITY DOES NOT MAKE ANY WARRANTY OF ANY KIND
 * WITH RESPECT TO FREEDOM FROM PATENT, TRADEMARK, OR COPYRIGHT INFRINGEMENT.
 * 
 * This material has been approved for public release and unlimited
 * distribution.
 * 
 * @author James Edmondson <jedmondson@gmail.com>
 *********************************************************************/
package com.gams.logger;

import com.gams.GamsJNI;
import com.madara.logger.Logger;

/**
 * A facade for the GAMS logging service
 **/

public class GlobalLogger extends GamsJNI
{	
  private static native long jni_getCPtr();
  private static native void jni_setLevel(int level);
  private static native int jni_getLevel();
  private static native java.lang.String jni_getTag();
  private static native void jni_setTag(java.lang.String tag);
  private static native void jni_addTerm();
  private static native void jni_addSyslog();
  private static native void jni_clear();
  private static native void jni_addFile(java.lang.String filename);
  private static native void jni_log(int level, java.lang.String message);
  private static native void jni_setTimestampFormat(java.lang.String format);

  /**
   * Default constructor
   **/
  public GlobalLogger()
  {
    setCPtr(jni_getCPtr());
  }
  
  /**
   * Gets the tag used in system logging
   *
   * @return   current tag for system logging
   */
  public static java.lang.String getTag()
  {
    return jni_getTag();
  }

  /**
   * Gets the logging level, e.g., where 0 is EMERGENCY and 6 is DETAILED
   *
   * @return  the current logging level
   */
  public static int getLevel()
  {
    return jni_getLevel();
  }

  /**
   * Gets the logging level, e.g., where 0 is EMERGENCY and 6 is DETAILED
   *
   * @return  the current logging level
   */
  public static Logger toLogger()
  {
    return Logger.fromPointer(jni_getCPtr(), false);
  }

  /**
   * Sets the tag used for system logging
   * @param  tag   the tag to be used for system logging
   */
  public static void setTag(java.lang.String tag)
  {
    jni_setTag(tag);
  }

  /**
   * Sets the logging level
   *
   * @param  level   level of message severity to print to log targets
   */
  public static void setLevel(int level)
  {
    jni_setLevel(level);
  }

  /**
   * Clears all logging targets
   */
  public static void clear()
  {
    jni_clear();
  }

  /**
   * Adds the terminal to logging outputs (turned on by default)
   */
  public static void addTerm()
  {
    jni_addTerm();
  }

  /**
   * Adds the system logger to logging outputs
   */
  public static void addSyslog()
  {
    jni_addSyslog();
  }

  /**
   * Adds a file to logging outputs
   * @param  filename   the name of a file to add to logging targets
   */
  public static void addFile(java.lang.String filename)
  {
    jni_addFile(filename);
  }
  
  /**
   * Logs a message at the specified level
   * @param  level   the logging severity level (0 is high)
   * @param  message the message to send to all logging targets
   */
  public static void log(int level, java.lang.String message)
  {
    jni_log(level, message);
  }
  
  /**
   * Sets the timestamp format for future messages
   * @param  format the format to print timestamps for logging
   */
  public static void setTimestampFormat(java.lang.String format)
  {
    jni_setTimestampFormat(format);
  }
}

