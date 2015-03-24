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

public class Algorithm extends GamsJNI
{	
  private native long jni_Algorithm();
  private native long jni_Algorithm(long cptr);
  private static native void jni_freeAlgorithm(long cptr);
  private native java.lang.String jni_getName(long cptr);
  private native void jni_init(long cptr, long type, long kb, java.lang.String name);
  private native java.lang.String jni_toString(long cptr);
  private native long jni_getDeadlocked(long cptr);
  private native long jni_getFailed(long cptr);
  private native long jni_getOk(long cptr);
  private native long jni_getPaused(long cptr);
  private native long jni_getUnknown(long cptr);
  private native long jni_getWaiting(long cptr);

  private boolean manageMemory = true;

  /**
   * Default constructor
   **/
  public Algorithm()
  {
    setCPtr(jni_Algorithm());
    init();
  }

  /**
   * Copy constructor
   * @param input the algorithm to copy
   **/
  public Algorithm(Algorithm input)
  {
    setCPtr(jni_Algorithm(input.getCPtr()));
    init();
  }

  /**
   * Creates a java object instance from a C/C++ pointer
   *
   * @param cptr C pointer to the object
   * @return a new java instance of the underlying pointer
   */
  public static Algorithm fromPointer(long cptr)
  {
    Algorithm ret = new Algorithm();
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
  public static Algorithm fromPointer(long cptr, boolean shouldManage)
  {
    Algorithm ret = new Algorithm();
    ret.manageMemory=shouldManage;
    ret.setCPtr(cptr);
    ret.init();
    return ret;
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
   * Initializes the member variables
   **/
  public void init()
  {
    deadlocked = Integer.fromPointer (jni_getDeadlocked (getCPtr ()),false);
    failed = Integer.fromPointer (jni_getFailed (getCPtr ()),false);
    ok = Integer.fromPointer (jni_getOk (getCPtr ()),false);
    paused = Integer.fromPointer (jni_getPaused (getCPtr ()),false);
    unknown = Integer.fromPointer (jni_getUnknown (getCPtr ()),false);
    waiting = Integer.fromPointer (jni_getWaiting (getCPtr ()),false);
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
  public com.madara.containers.Integer deadlocked;

  /**
   * Flag for whether the algorithm is failed or not
   */
  public com.madara.containers.Integer failed;

  /**
   * Flag for whether the algorithm is ok or not
   */
  public com.madara.containers.Integer ok;

  /**
   * Flag for whether the algorithm is paused or not
   */
  public com.madara.containers.Integer paused;

  /**
   * Flag for whether the algorithm is in an unknown state or not
   */
  public com.madara.containers.Integer unknown;

  /**
   * Flag for whether the algorithm is in a waiting state or not
   */
  public com.madara.containers.Integer waiting;

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
      jni_freeAlgorithm(getCPtr());
    }
  }
}

