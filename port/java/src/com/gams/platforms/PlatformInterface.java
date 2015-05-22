/*********************************************************************
 * Usage of this software requires acceptance of the GAMS-CMU License,
 * which can be found at the following URL:
 *
 * https://code.google.com/p/gams-cmu/wiki/License
 *********************************************************************/
package com.gams.platforms;

import com.gams.utility.Axes;
import com.gams.utility.Position;
import com.madara.KnowledgeBase;
import com.gams.variables.Self;
import com.gams.variables.Platform;

/**
 * Interface for defining a platform to be used by GAMS. Care must be taken
 * to make all methods non-blocking, to prevent locking up the underlying
 * MADARA context.
 */
public interface PlatformInterface
{
  /**
   * Analyzes the platform. This should be
   * a non-blocking call.
   * @return   status information (@see Status)
   **/
  public int analyze ();
  
  /**
   * Returns the GPS accuracy in meters
   * @return GPS accuracy
   **/
  public double getGpsAccuracy ();
    
  /**
   * Returns the position accuracy in meters
   * @return position accuracy
   **/
  public double getPositionAccuracy ();

  /**
   * Returns the current position 
   * @return the current position of the device/agent
   **/
  public Position getPosition ();
  
  /**
   * Returns to the home location. This should be
   * a non-blocking call.
   * @return   status information (@see Status)
   **/
  public int home ();
  
  /**
   * Requests the platform to land. This should be
   * a non-blocking call.
   * @return   status information (@see Status)
   **/
  public int land ();
  
  /**
   * Initializes a move to the target position. This should be
   * a non-blocking call.
   * @param   target     the new position to move to
   * @param   proximity  the minimum distance between current position
   *                   and target position that terminates the move.
   * @return  status information (@see Status)
   **/
  public int move (Position target, double proximity);
   
  /**
   * Initializes a rotation around 3D axes. This should be
   * a non-blocking call.
   * @param   axes       parameters for rotation along x, y, z axes
   * @param   proximity  the minimum distance between current position
   *                   and target position that terminates the move.
   * @return  status information (@see Status)
   **/
  public int rotate (Axes axes);
   
  /**
   * Get sensor radius
   * @return minimum radius of all available sensors for this platform
   */
  public double getMinSensorRange ();

  /**
   * Gets the movement speed
   * @return movement speed
   **/
  public double getMoveSpeed ();

  /**
   * Gets the unique id of the platform. This should be an alphanumeric
   * id that can be part of a MADARA variable name. Specifically, this
   * is used in the variable expansion of .platform.{yourid}.*
   * @return the id of the platform (alphanumeric only: no spaces!)
   **/
  public java.lang.String getId ();

  /**
   * Gets the name of the platform
   * @return the name of the platform
   **/
  public java.lang.String getName ();

  /**
   * Gets results from the platform's sensors. This should be
   * a non-blocking call.
   * @return   1 if moving, 2 if arrived, 0 if an error occurred
   **/
  public int sense ();
  
  /**
   * Sets move speed
   * @param speed new speed in meters/second
   **/
  public void setMoveSpeed (double speed);
      
  /**
   * Takes off. This should be
   * a non-blocking call.
   * @return   status information (@see Status)
   **/
  public int takeoff ();
  
  /**
   * Stops moving
   **/
  public void stopMove ();
}

