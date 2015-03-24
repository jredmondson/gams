/*********************************************************************
 * Usage of this software requires acceptance of the GAMS-CMU License,
 * which can be found at the following URL:
 *
 * https://code.google.com/p/gams-cmu/wiki/License
 *********************************************************************/
package com.gams.utility;

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
    }
  }                
}

