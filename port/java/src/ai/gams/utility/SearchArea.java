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
 * A utility class that acts as a facade for a SearchArea
 **/
public class SearchArea extends GamsJNI
{
  private native long jni_SearchArea();
  private native String jni_getName(long cptr);
  private native void jni_setName(long cptr, String name);
  private native void jni_fromContainer(long cptr, long kb, String name);
  private native void jni_toContainer(long cptr, long kb, String name);
  private native void jni_modify(long cptr);
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
   * Get name of the SearchArea
   * @return name of the SearchArea
   **/
  public String getName()
  {
    return jni_getName(getCPtr());
  }

  /**
   * Set name of the SearchArea
   * @param n   new name for SearchArea
   **/
  public void setName(String n)
  {
    jni_setName(getCPtr(), n);
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
   * Helper function for copying values from a MADARA knowledge base
   * @param kb      KnowledgeBase to copy SearchArea information to
   **/
  public void fromContainer(ai.madara.knowledge.KnowledgeBase kb)
  {
    fromContainer (kb, getName());
  }

  /**
   * Helper function for copying values from a MADARA knowledge base
   * @param kb      KnowledgeBase to copy SearchArea information to
   * @param name    name of the SearchArea
   **/
  public void fromContainer(ai.madara.knowledge.KnowledgeBase kb, String name)
  {
    jni_fromContainer(getCPtr(), kb.getCPtr(), name);
  }

  /**
   * Helper function for copying values to a MADARA knowledge base
   * @param kb      KnowledgeBase with SearchArea information
   **/
  public void toContainer(ai.madara.knowledge.KnowledgeBase kb)
  {
    toContainer(kb, getName());
  }

  /**
   * Helper function for copying values to a MADARA knowledge base
   * @param kb      KnowledgeBase with SearchArea information
   * @param name    name of the SearchArea
   **/
  public void toContainer(ai.madara.knowledge.KnowledgeBase kb, String name)
  {
    jni_toContainer(getCPtr(), kb.getCPtr(), name);
  }

  /**
   * Resend SearchArea over KnowledgeBase
   **/
  public void modify()
  {
    jni_modify(getCPtr());
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

