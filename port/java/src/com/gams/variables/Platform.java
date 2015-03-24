/*********************************************************************
 * Usage of this software requires acceptance of the GAMS-CMU License,
 * which can be found at the following URL:
 *
 * https://code.google.com/p/gams-cmu/wiki/License
 *********************************************************************/
package com.gams.variables;

import com.gams.GamsJNI;
import com.madara.KnowledgeBase;
import com.madara.Variables;
import com.madara.containers.Integer;

public class Platform extends GamsJNI
{	
  private native long jni_Platform();
  private native long jni_Platform(long cptr);
  private static native void jni_freePlatform(long cptr);
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
  public Platform()
  {
    setCPtr(jni_Platform());
    init();
  }

  /**
   * Copy constructor
   * @param  input the platform to copy
   **/
  public Platform(Platform input)
  {
    setCPtr(jni_Platform(input.getCPtr()));
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
  public static Platform fromPointer(long cptr)
  {
    Platform ret = new Platform();
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
  public static Platform fromPointer(long cptr, boolean shouldManage)
  {
    Platform ret = new Platform();
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
      jni_freePlatform(getCPtr());
    }
  }
}

