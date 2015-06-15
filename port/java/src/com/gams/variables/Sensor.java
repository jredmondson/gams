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

import java.util.HashSet;

import com.gams.GamsJNI;
import com.madara.KnowledgeBase;
import com.madara.containers.String;
import com.madara.containers.Vector;
import com.gams.utility.Position;
import com.gams.utility.GpsPosition;

public class Sensor extends GamsJNI
{	
  private native long jni_Sensor();
  private native long jni_Sensor(long cptr);
  private static native void jni_freeSensor(long cptr);
  private native java.lang.String jni_getName(long cptr);
  private native void jni_init(long cptr, long type, long kb, java.lang.String name, double range);
  private native java.lang.String jni_toString(long cptr);
  private native double jni_getPositionValue(long cptr, long pos);
  private native double jni_getGpsValue(long cptr, long pos);
  private native long jni_getOrigin(long cptr);
  private native double jni_getRange(long cptr);
  private native void jni_setOrigin(long cptr, long origin);
  private native void jni_setRange(long cptr, double range);
  private native void jni_setPositionValue(long cptr, long pos, double value);
  private native void jni_setGpsValue(long cptr, long pos, double value);
  private native long jni_getGpsFromIndex(long cptr, long index);
  private native long jni_getIndexFromGps(long cptr, long position);
  private native double jni_getDiscretization(long cptr);
  private native long[] jni_discretizeRegion(long cptr, long region);
  private native long[] jni_discretizeSearchArea(long cptr, long area);

  private boolean manageMemory = true;

  /**
   * Default constructor
   **/
  public Sensor()
  {
    setCPtr(jni_Sensor());
  }

  /**
   * Copy constructor
   * @param input the sensor value to copy
   **/
  public Sensor(Sensor input)
  {
    setCPtr(jni_Sensor(input.getCPtr()));
  }

  /**
   * Gets the sensor range in meters.
   * @return range in meters
   **/
  public GpsPosition getOrigin()
  {
    return GpsPosition.fromPointer (jni_getOrigin(getCPtr()));
  }
  
  /**
   * Gets the sensor range in meters.
   * @return range in meters
   **/
  public double getRange()
  {
    return jni_getRange(getCPtr());
  }
  
  /**
   * Gets a GPS coordinate from an index
   * @param index the cartesian position
   * @return the position at the specified index
   **/
  public GpsPosition getGpsFromIndex(Position index)
  {
    return GpsPosition.fromPointer(
      jni_getGpsFromIndex(getCPtr(),index.getCPtr()));
  }
    
  /**
   * Gets a GPS coordinate from an index
   * @param coord coordinate to convert to an index 
   * @return the position at the specified index
   **/
  public Position getIndexFromGps(GpsPosition coord)
  {
    return Position.fromPointer(
      jni_getIndexFromGps(getCPtr(),coord.getCPtr()));
  }
  
  /**
   * Gets the value stored at a particular location
   * @param  position  the location to check
   * @return value at the location
   **/
  public double getValue(GpsPosition position)
  {
    return jni_getGpsValue(getCPtr(), position.getCPtr());
  }
    
  /**
   * Gets the value stored at a particular location
   * @param  position  the location to check
   * @return value at the location
   **/
  public double getValue(Position position)
  {
    return jni_getPositionValue(getCPtr(), position.getCPtr());
  }
       
  /**
   * Gets the discretization value for the sensor
   * @return the discretization value for the sensor
   **/
  public double getDiscretization()
  {
    return jni_getDiscretization(getCPtr());
  }
        
  /**
   * Sets the range of the sensor
   * @param  range  the range of the sensor in meters
   **/
  public void setRange(double range)
  {
    jni_setRange(getCPtr(), range);
  }
          
  /**
   * Sets the origin for coverage information and discretization
   * @param  origin  the origin for coverage information
   **/
  public void setOrigin(GpsPosition origin)
  {
    jni_setOrigin(getCPtr(), origin.getCPtr());
  }
         
  /**
   * Sets a value at a location
   * @param  location  the position the value will be at
   * @param  value     the value to set
   **/
  public void setValue(Position location, double value)
  {
    jni_setPositionValue(getCPtr(), location.getCPtr(), value);
  }
          
  /**
   * Sets a value at a location
   * @param  location  the position the value will be at
   * @param  value     the value to set
   **/
  public void setValue(GpsPosition location, double value)
  {
    jni_setPositionValue(getCPtr(), location.getCPtr(), value);
  }
            
  /**
   * Discretizes the region into individual positions
   * @param  region  the region to discretize
   * @return individual positions within the region
   **/
  public HashSet<Position> discretize(com.gams.utility.Region region)
  {
    long[] indices = jni_discretizeRegion(getCPtr(), region.getCPtr());

    HashSet<Position> hash = new HashSet<Position> ();
    for (int i = 0; i < indices.length; ++i)
      hash.add(Position.fromPointer(indices[i]));

    return hash;
  }
              
  /**
   * Discretizes the area into individual positions
   * @param  area  the area to discretize
   * @return individual positions within the area
   **/
  public HashSet<Position> discretize(com.gams.utility.SearchArea area)
  {
    long[] indices = jni_discretizeSearchArea(getCPtr(), area.getCPtr());

    HashSet<Position> hash = new HashSet<Position> ();
    for (int i = 0; i < indices.length; ++i)
      hash.add(Position.fromPointer(indices[i]));

    return hash;
  }
  
  /**
   * Creates a java object instance from a C/C++ pointer
   *
   * @param cptr C pointer to the object
   * @return a new java instance of the underlying pointer
   */
  public static Sensor fromPointer(long cptr)
  {
    Sensor ret = new Sensor();
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
  public static Sensor fromPointer(long cptr, boolean shouldManage)
  {
    Sensor ret = new Sensor();
    ret.manageMemory=shouldManage;
    ret.setCPtr(cptr);
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
   * Sets the name and knowledge base being referred to
   *
   * @param  kb      the knowledge base that contains the name
   * @param  name    the variable name
   * @param  range   the range of the sensor in meters or appropriate unit
   */
  public void init(KnowledgeBase kb, java.lang.String name, double range)
  {
    jni_init(getCPtr(), 0, kb.getCPtr (), name, range);
  }

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
      jni_freeSensor(getCPtr());
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

