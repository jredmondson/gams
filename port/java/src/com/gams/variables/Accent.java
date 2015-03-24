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
import com.madara.containers.String;
import com.madara.containers.Vector;

public class Accent extends GamsJNI
{	
  private native long jni_Accent();
  private native long jni_Accent(long cptr);
  private static native void jni_freeAccent(long cptr);
  private native java.lang.String jni_getName(long cptr);
  private native void jni_init(long cptr, long type, long kb, java.lang.String name);
  private native java.lang.String jni_toString(long cptr);
  private native long jni_getCommand(long cptr);
  private native long jni_getArgs(long cptr);

  /**
   * Default constructor
   **/
  public Accent()
  {
    setCPtr(jni_Accent());
    init();
  }

  /**
   * Copy constructor
   * @param input  the accent to copy
   **/
  public Accent(Accent input)
  {
    setCPtr(jni_Accent(input.getCPtr()));
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
   * Initializes the member variables
   **/
  public void init()
  {
    command = com.madara.containers.String.fromPointer (
      jni_getCommand (getCPtr ()),false);
    args = com.madara.containers.Vector.fromPointer (
      jni_getArgs (getCPtr ()),false);
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
   * The current accent command
   */
  public com.madara.containers.String command;

  /**
   * The current accent command arguments
   */
  public com.madara.containers.Vector args;

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
    jni_freeAccent(getCPtr());
  }
}

