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
package ai.gams.utility;

import ai.gams.GamsJNI;

/**
 * Axes of a device/agent
 **/
public class Axes extends GamsJNI
{
  private native long jni_Axes();
  private native long jni_Axes(long cptr);
  private native long jni_Axes(double inx, double iny, double inz);
  private static native void jni_freeAxes(long cptr);
  private native java.lang.String jni_toString(long cptr);
  private native double jni_getX(long cptr);
  private native double jni_getY(long cptr);
  private native double jni_getZ(long cptr);
  private native void jni_setX(long cptr, double input);
  private native void jni_setY(long cptr, double input);
  private native void jni_setZ(long cptr, double input);

  private boolean manageMemory = true;

  /**
   * Default constructor
   **/
  public Axes()
  {
    setCPtr(jni_Axes());
  }

  /**
   * Constructor for a provided x,y,z coordinate
   * @param inx the x coordinate
   * @param iny the y coordinate
   * @param inz the z coordinate
   **/
  public Axes(double inx, double iny, double inz)                                   
  {
    setCPtr(jni_Axes(inx,iny,inz));
  }

  /**
   * Copy constructor
   * @param input the Axes to copy
   **/
  public Axes(Axes input)
  {
    setCPtr(jni_Axes(input.getCPtr()));
  }

  /**
   * Checks two instances for equality
   * @param other  other Axes to check against
   * @return true if equal, false otherwise
   **/
  public boolean equals (Axes other)
  {
    return getX() == other.getX() &&
           getY() == other.getY() &&
           getZ() == other.getZ();
  }
  
  /**
   * Gets a unique hashcode for this instance
   * @return hashcode for this object instance
   **/
  @Override
  public int hashCode()
  {
    return (int)getCPtr();
  }
  
  /**
   * Converts the Axes into a string
   * @return Axes as a string
   **/
  public java.lang.String toString()
  {
    String result = "";
    result += getX();
    result += ",";
    result += getY();
    result += ",";
    result += getZ();
    
    return result;
  }
  
  /**
   * Returns the latitude
   * @return latitude
   **/
  public double getX()
  {
    return jni_getX(getCPtr());
  }
  
  /**
   * Returns the longitude
   * @return longitude
   **/
  public double getY()
  {
    return jni_getY(getCPtr());
  }

  /**
   * Returns the altitude
   * @return altitude
   **/
  public double getZ()
  {
    return jni_getZ(getCPtr());
  }
  
  /**
   * Sets the latitude/x coord
   * @param  input  the new coord
   **/
  public void setX(double input)
  {
    jni_setX(getCPtr(),input);
  }
  
  /**
   * Sets the longitude/y coord
   * @param  input  the new coord
   **/
  public void setY(double input)
  {
    jni_setY(getCPtr(),input);
  }

  /**
   * Sets the altitude/z coord
   * @param  input  the new coord
   **/
  public void setZ(double input)
  {
    jni_setZ(getCPtr(),input);
  }
      
  /**
   * Creates a java object instance from a C/C++ pointer
   *
   * @param cptr C pointer to the object
   * @return a new java instance of the underlying pointer
   */
  public static Axes fromPointer(long cptr)
  {
    Axes ret = new Axes();
    ret.manageMemory = true;
    ret.setCPtr(cptr);
    return ret;
  }
  
  /**
   * Creates a java object instance from a C/C++ pointer
   *
   * @param cptr C pointer to the object
   * @param shouldManage  if true, manage the pointer
   * @return a new java instance of the underlying pointer
   */
  public static Axes fromPointer(long cptr, boolean shouldManage)
  {
    Axes ret = new Axes();
    ret.manageMemory=shouldManage;
    ret.setCPtr(cptr);
    return ret;
  }

  /**
   * Deletes the C instantiation. To prevent memory leaks, this <b>must</b> be
   * called before an instance of WaitSettings gets garbage collected
   */
  public void free()
  {
    if(manageMemory)
    {
      jni_freeAxes(getCPtr());
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

