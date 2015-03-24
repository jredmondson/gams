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
import com.madara.containers.Double;
import com.madara.containers.Integer;
import com.madara.containers.String;
import com.madara.containers.Vector;
import com.madara.containers.NativeDoubleVector;

public class Swarm extends GamsJNI
{	
  private native long jni_Swarm();
  private native long jni_Swarm(long cptr);
  private static native void jni_freeSwarm(long cptr);
  private native void jni_init(long cptr, long type, long kb, java.lang.String name);
  private native java.lang.String jni_toString(long cptr);
  private native long jni_getCommand(long cptr);
  private native long jni_getArgs(long cptr);
  private native long jni_getMinAlt(long cptr);
  private native long jni_getSize(long cptr);

  private boolean manageMemory = true;

  /**
   * Default constructor
   **/
  public Swarm()
  {
    setCPtr(jni_Swarm());
    init();
  }

  /**
   * Copy constructor
   * @param input the swarm object to copy
   **/
  public Swarm(Swarm input)
  {
    setCPtr(jni_Swarm(input.getCPtr()));
    init();
  }

  /**
   * Creates a java object instance from a C/C++ pointer
   *
   * @param cptr C pointer to the object
   * @return a new java instance of the underlying pointer
   */
  public static Swarm fromPointer(long cptr)
  {
    Swarm ret = new Swarm();
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
  public static Swarm fromPointer(long cptr, boolean shouldManage)
  {
    Swarm ret = new Swarm();
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
    command = com.madara.containers.String.fromPointer (
      jni_getCommand (getCPtr ()),false);
    args = com.madara.containers.Vector.fromPointer (
      jni_getArgs (getCPtr ()),false);
    minAlt = com.madara.containers.Double.fromPointer (
      jni_getMinAlt (getCPtr ()),false);
    size = com.madara.containers.Integer.fromPointer (
      jni_getSize (getCPtr ()),false);
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
   * The current swarm command
   */
  public com.madara.containers.String command;

  /**
   * The current swarm command args
   */
  public com.madara.containers.Vector args;

  /**
   * The current swarm minimum altitude
   */
  public com.madara.containers.Double minAlt;

  /**
   * The current swarm size
   */
  public com.madara.containers.Integer size;

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
      jni_freeSwarm(getCPtr());
    }
  }
}

