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

public class AlgorithmStatus extends GamsJNI
{	
  private native long jni_AlgorithmStatus();
  private native long jni_AlgorithmStatus(long cptr);
  private static native void jni_freeAlgorithmStatus(long cptr);
  private native java.lang.String jni_getName(long cptr);
  private native void jni_init(long cptr, long type, long kb, java.lang.String name, int id);
  private native java.lang.String jni_toString(long cptr);
  private native long jni_getDeadlocked(long cptr);
  private native long jni_getFailed(long cptr);
  private native long jni_getOk(long cptr);
  private native long jni_getPaused(long cptr);
  private native long jni_getUnknown(long cptr);
  private native long jni_getWaiting(long cptr);
  private native long jni_getFinished(long cptr);

  private boolean manageMemory = true;

  /**
   * Default constructor
   **/
  public AlgorithmStatus()
  {
    setCPtr(jni_AlgorithmStatus());
    init();
  }

  /**
   * Copy constructor
   * @param input the algorithm to copy
   **/
  public AlgorithmStatus(AlgorithmStatus input)
  {
    setCPtr(jni_AlgorithmStatus(input.getCPtr()));
    init();
  }

  /**
   * Creates a java object instance from a C/C++ pointer
   *
   * @param cptr C pointer to the object
   * @return a new java instance of the underlying pointer
   */
  public static AlgorithmStatus fromPointer(long cptr)
  {
    AlgorithmStatus ret = new AlgorithmStatus();
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
  public static AlgorithmStatus fromPointer(long cptr, boolean shouldManage)
  {
    AlgorithmStatus ret = new AlgorithmStatus();
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
    deadlocked = Integer.fromPointer (jni_getDeadlocked (getCPtr ()), false);
    failed = Integer.fromPointer (jni_getFailed (getCPtr ()), false);
    ok = Integer.fromPointer (jni_getOk (getCPtr ()), false);
    paused = Integer.fromPointer (jni_getPaused (getCPtr ()), false);
    unknown = Integer.fromPointer (jni_getUnknown (getCPtr ()), false);
    waiting = Integer.fromPointer (jni_getWaiting (getCPtr ()), false);
    finished = Integer.fromPointer (jni_getFinished (getCPtr ()), false);
  }
  
  /**
   * Sets the name and knowledge base being referred to
   *
   * @param  kb      the knowledge base that contains the name
   * @param  name    the variable name
   */
  public void init(KnowledgeBase kb, java.lang.String name, int id)
  {
    jni_init(getCPtr(), 0, kb.getCPtr (), name, id);
    init();
  }

  /**
   * Sets the name and knowledge base being referred to
   *
   * @param  vars    the variables facade that contains the name
   * @param  name    the variable name
   */
  public void init(Variables vars, java.lang.String name, int id)
  {
    jni_init(getCPtr(), 1, vars.getCPtr (), name, id);
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
   * Flag for whether the algorithm is finished
   */
  public com.madara.containers.Integer finished;

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
      jni_freeAlgorithmStatus(getCPtr());
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

