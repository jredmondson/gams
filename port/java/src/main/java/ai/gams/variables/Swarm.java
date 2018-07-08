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
import ai.gams.exceptions.GamsDeadObjectException;
import ai.madara.knowledge.KnowledgeBase;
import ai.madara.knowledge.Variables;

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
  public Swarm() throws GamsDeadObjectException
  {
    setCPtr(jni_Swarm());
    init();
  }

  /**
   * Copy constructor
   * @param input the swarm object to copy
   **/
  public Swarm(Swarm input) throws GamsDeadObjectException
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
  public static Swarm fromPointer(long cptr) throws GamsDeadObjectException
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
  public static Swarm fromPointer(long cptr, boolean shouldManage) throws GamsDeadObjectException
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
  public void init() throws GamsDeadObjectException
  {
    command = ai.madara.knowledge.containers.String.fromPointer (
      jni_getCommand (getCPtr ()),false);
    args = ai.madara.knowledge.containers.Vector.fromPointer (
      jni_getArgs (getCPtr ()),false);
    minAlt = ai.madara.knowledge.containers.Double.fromPointer (
      jni_getMinAlt (getCPtr ()),false);
    size = ai.madara.knowledge.containers.Integer.fromPointer (
      jni_getSize (getCPtr ()),false);
  }

  /**
   * Sets the name and knowledge base being referred to
   *
   * @param  kb      the knowledge base that contains the name
   * @param  name    the variable name
   */
  public void init(KnowledgeBase kb, java.lang.String name) throws GamsDeadObjectException
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
  public void init(Variables vars, java.lang.String name) throws GamsDeadObjectException
  {
    jni_init(getCPtr(), 1, vars.getCPtr (), name);
    init();
  }

  /**
   * The current swarm command
   */
  public ai.madara.knowledge.containers.String command;

  /**
   * The current swarm command args
   */
  public ai.madara.knowledge.containers.Vector args;

  /**
   * The current swarm minimum altitude
   */
  public ai.madara.knowledge.containers.Double minAlt;

  /**
   * The current swarm size
   */
  public ai.madara.knowledge.containers.Integer size;

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

