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
package ai.gams.variables;

import ai.gams.GamsJNI;
import ai.madara.knowledge.KnowledgeBase;
import ai.madara.knowledge.Variables;
import ai.madara.knowledge.containers.Integer;

public class PlatformStatus extends GamsJNI
{	
  private native long jni_PlatformStatus();
  private native long jni_PlatformStatus(long cptr);
  private static native void jni_freePlatformStatus(long cptr);
  private native java.lang.String jni_getName(long cptr);
  private native void jni_init(long cptr, long type, long kb, java.lang.String name);
  private native java.lang.String jni_toString(long cptr);
  private native long jni_getCommunicationAvailable(long cptr);
  private native long jni_getDeadlocked(long cptr);
  private native long jni_getFailed(long cptr);
  private native long jni_getGpsSpoofed(long cptr);
  private native long jni_getMovementAvailable(long cptr);
  private native long jni_getMoving(long cptr);
  private native long jni_getOk(long cptr);
  private native long jni_getPausedMoving(long cptr);
  private native long jni_getReducedSensing(long cptr);
  private native long jni_getReducedMovement(long cptr);
  private native long jni_getSensorsAvailable(long cptr);
  private native long jni_getWaiting(long cptr);

  private boolean manageMemory = true;

  /**
   * Default constructor
   **/
  public PlatformStatus()
  {
    setCPtr(jni_PlatformStatus());
    init();
  }

  /**
   * Copy constructor
   * @param  input the platform to copy
   **/
  public PlatformStatus(PlatformStatus input)
  {
    setCPtr(jni_PlatformStatus(input.getCPtr()));
    init();
  }

  /**
   * Gets the name of the variable
   *
   * @return  name of the variable within the context
   */
  public java.lang.String getName()
  {
    return jni_getName(getCPtr());
  }

  /**
   * Creates a java object instance from a C/C++ pointer
   *
   * @param cptr C pointer to the object
   * @return a new java instance of the underlying pointer
   */
  public static PlatformStatus fromPointer(long cptr)
  {
    PlatformStatus ret = new PlatformStatus();
    ret.manageMemory = true;
    ret.setCPtr(cptr);
    ret.init();
    return ret;
  }

  /**
   * Creates a java object instance from a C/C++ pointer
   *
   * @param cptr C pointer to the object
   * @param shouldManage  if true, manage the pointer
   * @return a new java instance of the underlying pointer
   */
  public static PlatformStatus fromPointer(long cptr, boolean shouldManage)
  {
    PlatformStatus ret = new PlatformStatus();
    ret.manageMemory=shouldManage;
    ret.setCPtr(cptr);
    ret.init();
    return ret;
  }

  /**
   * Initializes the member variables
   **/
  public void init()
  {
    communicationAvailable = Integer.fromPointer (
      jni_getCommunicationAvailable (getCPtr ()),false);
    deadlocked = Integer.fromPointer (
      jni_getDeadlocked (getCPtr ()),false);
    failed = Integer.fromPointer (
      jni_getFailed (getCPtr ()),false);
    gpsSpoofed = Integer.fromPointer (
      jni_getGpsSpoofed (getCPtr ()),false);
    movementAvailable = Integer.fromPointer (
      jni_getMovementAvailable (getCPtr ()),false);
    moving = Integer.fromPointer (
      jni_getMoving (getCPtr ()),false);
    ok = Integer.fromPointer (
      jni_getOk (getCPtr ()),false);
    pausedMoving = Integer.fromPointer (
      jni_getPausedMoving (getCPtr ()),false);
    reducedSensing = Integer.fromPointer (
      jni_getReducedSensing (getCPtr ()),false);
    reducedMovement = Integer.fromPointer (
      jni_getReducedMovement (getCPtr ()),false);
    sensorsAvailable = Integer.fromPointer (
      jni_getSensorsAvailable (getCPtr ()),false);
    waiting = Integer.fromPointer (
      jni_getWaiting (getCPtr ()),false);
  }
  
  /**
   * Sets the name and knowledge base being referred to
   *
   * @param  kb      the knowledge base that contains the name
   * @param  name    the variable name
   */
  public void init(KnowledgeBase kb, java.lang.String name)
  {
    jni_init(getCPtr(), 0, kb.getCPtr (), name);
    init();
  }

  /**
   * Sets the name and knowledge base being referred to
   *
   * @param  vars    the variables facade that contains the name
   * @param  name    the variable name
   */
  public void init(Variables vars, java.lang.String name)
  {
    jni_init(getCPtr(), 1, vars.getCPtr (), name);
    init();
  }

  /**
   * Flag for whether the algorithm is deadlocked or not
   */
  public Integer communicationAvailable;

  /**
   * Flag for whether the algorithm is deadlocked or not
   */
  public Integer deadlocked;

  /**
   * Flag for whether the algorithm is failed or not
   */
  public Integer failed;

  /**
   * Flag for whether the algorithm is failed or not
   */
  public Integer gpsSpoofed;

  /**
   * Flag for whether the algorithm is failed or not
   */
  public Integer movementAvailable;

  /**
   * Flag for whether the algorithm is failed or not
   */
  public Integer moving;

  /**
   * Flag for whether the algorithm is ok or not
   */
  public Integer ok;

  /**
   * Flag for whether the algorithm is paused or not
   */
  public Integer pausedMoving;

  /**
   * Flag for whether the algorithm is paused or not
   */
  public Integer reducedSensing;

  /**
   * Flag for whether the algorithm is paused or not
   */
  public Integer reducedMovement;

  /**
   * Flag for whether the algorithm is paused or not
   */
  public Integer sensorsAvailable;

  /**
   * Flag for whether the algorithm is in a waiting state or not
   */
  public Integer waiting;

  /**
   * Converts the value to a string
   *
   * @return current string value
   */
  public java.lang.String toString()
  {
    return jni_toString(getCPtr());
  }

  /**
   * Deletes the C instantiation. To prevent memory leaks, this <b>must</b> be
   * called before an instance gets garbage collected
   */
  public void free()
  {
    if (manageMemory)
    {
      jni_freePlatformStatus(getCPtr());
    }
  }
  
  /**
   * Cleans up underlying C resources
   * @throws Throwable necessary for override but unused
   */
  @Override
  protected void finalize() throws Throwable
  {
    try {
      free();
    } catch (Throwable t) {
      throw t;
    } finally {
      super.finalize();
    }
  }
}

