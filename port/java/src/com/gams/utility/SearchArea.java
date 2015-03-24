/*********************************************************************
 * Usage of this software requires acceptance of the GAMS-CMU License,
 * which can be found at the following URL:
 *
 * https://code.google.com/p/gams-cmu/wiki/License
 *********************************************************************/
package com.gams.utility;

import com.gams.GamsJNI;

/**
 * A utility class that acts as a facade for a region
 **/
public class SearchArea extends GamsJNI
{
  private native long jni_SearchArea();
  private static native void jni_freeSearchArea(long cptr);
  private native java.lang.String jni_toString(long cptr);
  private native void jni_addPrioritizedRegion(long cptr, long region); 
  private native long jni_getConvexHull(long cptr);
  private native boolean jni_containsGps(long cptr, long coord);  
  private native double jni_getMaxAlt(long cptr); 
  private native double jni_getMinAlt(long cptr); 
  private native double jni_getMaxLat(long cptr); 
  private native double jni_getMinLat(long cptr); 
  private native double jni_getMaxLong(long cptr); 
  private native double jni_getMinLong(long cptr); 
  private native long jni_getGpsPriority(long cptr, long coord); 
  private native long[] jni_getRegions(long cptr); 
  
  private boolean manageMemory = true;

  public SearchArea()
  {
    setCPtr(jni_SearchArea());
  }

  /**
   * Converts the position into a string
   * @return position as a string
   **/
  public java.lang.String toString()
  {
    return jni_toString(getCPtr());
  }

  /**
   * Adds a region to the search area
   * @param  region  the region to add to the search area 
   **/
  public void add(PrioritizedRegion region)
  {
    jni_addPrioritizedRegion(getCPtr(),region.getCPtr()); 
  }
  
  /**
   * Gets bounding box
   * @return Region object corresponding to bounding box
   **/
  public Region getConvexHull()
  {
    return Region.fromPointer(jni_getConvexHull(getCPtr()));
  }
  
  /**
   * Checks to see if the point is contained in this region
   * @param   point     point to check
   * @return 0 if in region, otherwise distance from region
   **/
  public boolean contains(GpsPosition point)
  {
    return jni_containsGps(getCPtr(),point.getCPtr());
  } 
   
  /**
   * Gets priority of a gps position
   * @param   point     point to check
   * @return  the priority at the point
   **/
  public long getPriority(GpsPosition point)
  {
    return jni_getGpsPriority(getCPtr(),point.getCPtr());
  }
  
  /**
   * Gets the maximum altitude
   * @return maximum altitude in the region
   **/
  public double getMaxAlt()
  {
    return jni_getMaxAlt(getCPtr());
  }
  
  /**
   * Gets the minimum altitude
   * @return minimum altitude in the region
   **/
  public double getMinAlt()
  {
    return jni_getMinAlt(getCPtr());
  }
    
  /**
   * Gets the maximum latitude
   * @return maximum latitude in the region
   **/
  public double getMaxLat()
  {
    return jni_getMaxLat(getCPtr());
  }
      
  /**
   * Gets the minimum latitude
   * @return minimum latitude in the region
   **/
  public double getMinLat()
  {
    return jni_getMinLat(getCPtr());
  }
      
  /**
   * Gets the maximum longitude
   * @return maximum longitude in the region
   **/
  public double jni_getMaxLong()
  {
    return jni_getMaxLong(getCPtr());
  }
      
  /**
   * Gets the minimum longitude
   * @return minimum longitude in the region
   **/
  public double getMinLong()
  {
    return jni_getMinLong(getCPtr());
  }
  
  /**
   * Gets the regions within the search area
   * @return vertices that form the region boundary
   **/
  public PrioritizedRegion[] getRegions()
  {
    long[] vertices = jni_getRegions(getCPtr());
    PrioritizedRegion[] result = new PrioritizedRegion[vertices.length];
    
    if(vertices.length > 0)
    {
      result = new PrioritizedRegion[vertices.length];
      for (int i = 0; i < vertices.length; ++i)
        result[i].fromPointer(vertices[i]);
    }
    return result;
  }
    
  /**
   * Creates a java object instance from a C/C++ pointer
   *
   * @param cptr C pointer to the object
   * @return a new java instance of the underlying pointer
   */
  public static SearchArea fromPointer(long cptr)
  {
    SearchArea ret = new SearchArea();
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
  public static SearchArea fromPointer(long cptr, boolean shouldManage)
  {
    SearchArea ret = new SearchArea();
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
      jni_freeSearchArea(getCPtr());
    }
  }                
}

