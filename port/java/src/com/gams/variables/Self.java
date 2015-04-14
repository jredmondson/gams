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
import com.madara.containers.String;
import com.madara.containers.Vector;

public class Self extends GamsJNI
{	
  private native long jni_Self();
  private native long jni_Self(long cptr);
  private static native void jni_freeSelf(long cptr);
  private native void jni_init(long cptr, long type, long kb, long name);
  private native java.lang.String jni_toString(long cptr);
  private native long jni_getId(long cptr);
  private native long jni_getDevice(long cptr);

  private boolean manageMemory = true;

  /**
   * Default constructor
   **/
  public Self()
  {
    setCPtr(jni_Self());
    init();
  }

  /**
   * Copy constructor
   * @param input the Self variable to copy
   **/
  public Self(Self input)
  {
    setCPtr(jni_Self(input.getCPtr()));
    init();
  }

  /**
   * Creates a java object instance from a C/C++ pointer
   *
   * @param cptr C pointer to the object
   * @return a new java instance of the underlying pointer
   */
  public static Self fromPointer(long cptr)
  {
    Self ret = new Self();
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
  public static Self fromPointer(long cptr, boolean shouldManage)
  {
    Self ret = new Self();
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
    id = Integer.fromPointer (jni_getId (getCPtr ()),false);
    device = Device.fromPointer (jni_getDevice (getCPtr ()),false);
  }
  
  /**
   * Initializes the id and device containers within Self
   *
   * @param  kb      the knowledge base that contains the device info
   * @param  id      the device id (0->n-1)
   */
  public void init(KnowledgeBase kb, long id)
  {
    jni_init(getCPtr(), 0, kb.getCPtr (), id);
    init();
  }

  /**
   * Initializes the id and device containers within Self
   *
   * @param  vars    the variables facade that contains the device info
   * @param  id      the device id (0->n-1)
   */
  public void init(Variables vars, long id)
  {
    jni_init(getCPtr(), 1, vars.getCPtr (), id);
    init();
  }

  /**
   * The device id
   */
  public com.madara.containers.Integer id;

  /**
   * The device-specific variables
   */
  public Device device;
  
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
      jni_freeSelf(getCPtr());
    }
  }
}

