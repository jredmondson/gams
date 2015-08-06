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
   * @param  id      the device id (0 to n - 1, inclusively)
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
   * @param  id      the device id (0 to n - 1, inclusively)
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
      setCPtr(0);
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

