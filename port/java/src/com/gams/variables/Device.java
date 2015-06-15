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
import com.madara.containers.Double;
import com.madara.containers.Integer;
import com.madara.containers.String;
import com.madara.containers.Vector;
import com.madara.containers.NativeDoubleVector;

public class Device extends GamsJNI
{	
  private native long jni_Device();
  private native long jni_Device(long cptr);
  private static native void jni_freeDevice(long cptr);
  private native void jni_init(long cptr, long type, long kb, java.lang.String name);
  private native java.lang.String jni_toString(long cptr);
  private native long jni_getBatteryRemaining(long cptr);
  private native long jni_getBridgeId(long cptr);
  private native long jni_getCommand(long cptr);
  private native long jni_getArgs(long cptr);
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
  public Device()
  {
    setCPtr(jni_Device());
    init();
  }

  /**
   * Copy constructor
   * @param input  the device to copy
   **/
  public Device(Device input)
  {
    setCPtr(jni_Device(input.getCPtr()));
    init();
  }

  /**
   * Creates a java object instance from a C/C++ pointer
   *
   * @param cptr C pointer to the object
   * @return a new java instance of the underlying pointer
   */
  public static Device fromPointer(long cptr)
  {
    Device ret = new Device();
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
  public static Device fromPointer(long cptr, boolean shouldManage)
  {
    Device ret = new Device();
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
    batteryRemaining = com.madara.containers.Integer.fromPointer (
      jni_getBatteryRemaining (getCPtr ()),false);
    bridgeId = com.madara.containers.Integer.fromPointer (
      jni_getBridgeId (getCPtr ()),false);
    command = com.madara.containers.String.fromPointer (
      jni_getCommand (getCPtr ()),false);
    args = com.madara.containers.Vector.fromPointer (
      jni_getArgs (getCPtr ()),false);
    coverageType = com.madara.containers.String.fromPointer (
      jni_getCoverageType (getCPtr ()),false);
    dest = com.madara.containers.NativeDoubleVector.fromPointer (
      jni_getDest (getCPtr ()),false);
    home = com.madara.containers.NativeDoubleVector.fromPointer (
      jni_getHome (getCPtr ()),false);
    isMobile = com.madara.containers.Integer.fromPointer (
      jni_getIsMobile (getCPtr ()),false);
    location = com.madara.containers.NativeDoubleVector.fromPointer (
      jni_getLocation (getCPtr ()),false);
    minAlt = com.madara.containers.Double.fromPointer (
      jni_getMinAlt (getCPtr ()),false);
    nextCoverageType = com.madara.containers.String.fromPointer (
      jni_getNextCoverageType (getCPtr ()),false);
    searchAreaId = com.madara.containers.Integer.fromPointer (
      jni_getSearchAreaId (getCPtr ()),false);
    source = com.madara.containers.NativeDoubleVector.fromPointer (
      jni_getSource (getCPtr ()),false);
    temperature = com.madara.containers.Double.fromPointer (
      jni_getTemperature (getCPtr ()),false);
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
   * The current device command
   */
  public com.madara.containers.Integer batteryRemaining;

  /**
   * The current device bridge id
   */
  public com.madara.containers.Integer bridgeId;

  /**
   * The current device command
   */
  public com.madara.containers.String command;

  /**
   * The current device command args
   */
  public com.madara.containers.Vector args;

  /**
   * The current device coverage type
   */
  public com.madara.containers.String coverageType;

  /**
   * The current device destination location
   */
  public com.madara.containers.NativeDoubleVector dest;

  /**
   * The current device home location
   */
  public com.madara.containers.NativeDoubleVector home;

  /**
   * Flag for if current device is mobile 
   */
  public com.madara.containers.Integer isMobile;

  /**
   * The current device current location
   */
  public com.madara.containers.NativeDoubleVector location;

  /**
   * Flag for if current device is mobile 
   */
  public com.madara.containers.Double minAlt;

  /**
   * The current device's next coverage type
   */
  public com.madara.containers.String nextCoverageType;

  /**
   * The current device search area id 
   */
  public com.madara.containers.Integer searchAreaId;

  /**
   * The current device home source location
   */
  public com.madara.containers.NativeDoubleVector source;

  /**
   * The current device temperature
   */
  public com.madara.containers.Double temperature;

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
      jni_freeDevice(getCPtr());
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

