/*********************************************************************
 * Usage of this software requires acceptance of the GAMS-CMU License,
 * which can be found at the following URL:
 *
 * https://code.google.com/p/gams-cmu/wiki/License
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
}

