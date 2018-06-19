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

public class GpsPosition extends Position
{	
  private native long jni_GpsPosition();
  private native long jni_GpsPosition(long cptr);
  private native long jni_GpsPosition(double lat, double lon, double alt);
  private static native void jni_freeGpsPosition(long cptr);
  private native java.lang.String jni_toString(long cptr);
  private native double jni_getLatitude(long cptr);
  private native double jni_getLongitude(long cptr);
  private native double jni_getAltitude(long cptr);
  private native void jni_setLatitude(long cptr, double input);
  private native void jni_setLongitude(long cptr, double input);
  private native void jni_setAltitude(long cptr, double input);

  private boolean manageMemory = true;

  public GpsPosition()
  {
    setCPtr(jni_GpsPosition());
  }

  public GpsPosition(double lat, double lon, double alt)
  {
    setCPtr(jni_GpsPosition(lat,lon,alt));
  }

  public GpsPosition(Position input)
  {
    setCPtr(jni_GpsPosition(input.getCPtr()));
  }

  /**
   * Converts the position into a string
   * @return position as a string
   **/
  public java.lang.String toString()
  {
    String result = "";
    result += getLatitude();
    result += ",";
    result += getLongitude();
    result += ",";
    result += getAltitude();
    
    return result;
  }
  
  /**
   * Returns the latitude
   * @return latitude
   **/
  public double getLatitude()
  {
    return jni_getLatitude(getCPtr());
  }
  
  /**
   * Returns the longitude
   * @return longitude
   **/
  public double getLongitude()
  {
    return jni_getLongitude(getCPtr());
  }

  /**
   * Returns the altitude
   * @return altitude
   **/
  public double getAltitude()
  {
    return jni_getAltitude(getCPtr());
  }
  
  /**
   * Sets the latitude
   * @param input  the new latitude
   **/
  public void setLatitude(double input)
  {
    jni_setLatitude(getCPtr(),input);
  }
  
  /**
   * Sets the longitude
   * @param input  the new longitude
   **/
  public void setLongitude(double input)
  {
    jni_setLongitude(getCPtr(),input);
  }

  /**
   * Sets the altitude
   * @param input  the new altitude
   **/
  public void setAltitude(double input)
  {
    jni_setAltitude(getCPtr(),input);
  }
    
  /**
   * Creates a java object instance from a C/C++ pointer
   *
   * @param cptr C pointer to the object
   * @return a new java instance of the underlying pointer
   */
  public static GpsPosition fromPointer(long cptr)
  {
    GpsPosition ret = new GpsPosition();
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
  public static GpsPosition fromPointer(long cptr, boolean shouldManage)
  {
    GpsPosition ret = new GpsPosition();
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
      jni_freeGpsPosition(getCPtr());
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

