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

public class Agent extends GamsJNI
{
  private native long jni_Agent();
  private native long jni_Agent(long cptr);
  private static native void jni_freeAgent(long cptr);
  private native void jni_init(long cptr, long type, long kb, java.lang.String name);
  private native java.lang.String jni_toString(long cptr);
  private native long jni_getBatteryRemaining(long cptr);
  private native long jni_getBridgeId(long cptr);
  private native long jni_getAlgorithm(long cptr);
  private native long jni_getAlgorithmArgs(long cptr);
  private native long jni_getCoverageType(long cptr);
  private native long jni_getDest(long cptr);
  private native long jni_getHome(long cptr);
  private native long jni_getIsMobile(long cptr);
  private native long jni_getLocation(long cptr);
  private native long jni_getMinAlt(long cptr);
  private native long jni_getNextCoverageType(long cptr);
  private native long jni_getSearchAreaId(long cptr);
  private native long jni_getSource(long cptr);
  private native long jni_getTemperature(long cptr);

  private boolean manageMemory = true;

  /**
   * Default constructor
   **/
  public Agent() throws GamsDeadObjectException
  {
    setCPtr(jni_Agent());
    init();
  }

  /**
   * Copy constructor
   * @param input  the device to copy
   **/
  public Agent(Agent input) throws GamsDeadObjectException
  {
    setCPtr(jni_Agent(input.getCPtr()));
    init();
  }

  /**
   * Creates a java object instance from a C/C++ pointer
   *
   * @param cptr C pointer to the object
   * @return a new java instance of the underlying pointer
   */
  public static Agent fromPointer(long cptr) throws GamsDeadObjectException
  {
    Agent ret = new Agent();
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
  public static Agent fromPointer(long cptr, boolean shouldManage) throws GamsDeadObjectException
  {
    Agent ret = new Agent();
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
    batteryRemaining = ai.madara.knowledge.containers.Integer.fromPointer (
      jni_getBatteryRemaining (getCPtr ()),false);
    bridgeId = ai.madara.knowledge.containers.Integer.fromPointer (
      jni_getBridgeId (getCPtr ()),false);
    algorithm = ai.madara.knowledge.containers.String.fromPointer (
      jni_getAlgorithm (getCPtr ()),false);
    algorithmArgs = ai.madara.knowledge.containers.Map.fromPointer (
      jni_getAlgorithmArgs (getCPtr ()),false);
    coverageType = ai.madara.knowledge.containers.String.fromPointer (
      jni_getCoverageType (getCPtr ()),false);
    dest = ai.madara.knowledge.containers.NativeDoubleVector.fromPointer (
      jni_getDest (getCPtr ()),false);
    home = ai.madara.knowledge.containers.NativeDoubleVector.fromPointer (
      jni_getHome (getCPtr ()),false);
    isMobile = ai.madara.knowledge.containers.Integer.fromPointer (
      jni_getIsMobile (getCPtr ()),false);
    location = ai.madara.knowledge.containers.NativeDoubleVector.fromPointer (
      jni_getLocation (getCPtr ()),false);
    minAlt = ai.madara.knowledge.containers.Double.fromPointer (
      jni_getMinAlt (getCPtr ()),false);
    nextCoverageType = ai.madara.knowledge.containers.String.fromPointer (
      jni_getNextCoverageType (getCPtr ()),false);
    searchAreaId = ai.madara.knowledge.containers.Integer.fromPointer (
      jni_getSearchAreaId (getCPtr ()),false);
    source = ai.madara.knowledge.containers.NativeDoubleVector.fromPointer (
      jni_getSource (getCPtr ()),false);
    temperature = ai.madara.knowledge.containers.Double.fromPointer (
      jni_getTemperature (getCPtr ()),false);
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
   * The current device command
   */
  public ai.madara.knowledge.containers.Integer batteryRemaining;

  /**
   * The current device bridge id
   */
  public ai.madara.knowledge.containers.Integer bridgeId;

  /**
   * The current device command
   */
  public ai.madara.knowledge.containers.String algorithm;

  /**
   * The current device command args
   */
  public ai.madara.knowledge.containers.Map algorithmArgs;

  /**
   * The current device coverage type
   */
  public ai.madara.knowledge.containers.String coverageType;

  /**
   * The current device destination location
   */
  public ai.madara.knowledge.containers.NativeDoubleVector dest;

  /**
   * The current device home location
   */
  public ai.madara.knowledge.containers.NativeDoubleVector home;

  /**
   * Flag for if current device is mobile
   */
  public ai.madara.knowledge.containers.Integer isMobile;

  /**
   * The current device current location
   */
  public ai.madara.knowledge.containers.NativeDoubleVector location;

  /**
   * Flag for if current device is mobile
   */
  public ai.madara.knowledge.containers.Double minAlt;

  /**
   * The current device's next coverage type
   */
  public ai.madara.knowledge.containers.String nextCoverageType;

  /**
   * The current device search area id
   */
  public ai.madara.knowledge.containers.Integer searchAreaId;

  /**
   * The current device home source location
   */
  public ai.madara.knowledge.containers.NativeDoubleVector source;

  /**
   * The current device temperature
   */
  public ai.madara.knowledge.containers.Double temperature;

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
      jni_freeAgent(getCPtr());
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

