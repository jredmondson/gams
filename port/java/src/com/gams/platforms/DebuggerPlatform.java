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
package com.gams.platforms;

import com.gams.utility.Axes;
import com.gams.utility.Position;
import com.madara.EvalSettings;

/**
 * Interface for defining a platform to be used by GAMS. Care must be taken
 * to make all methods non-blocking, to prevent locking up the underlying
 * MADARA context.
 */
public class DebuggerPlatform extends BasePlatform
{
  /**
   * Default constructor
   **/
  public DebuggerPlatform()
  {
    executions = new com.madara.containers.Integer();
  }
   
  /**
   * Analyzes the platform.
   * @return   status information(@see Status)
   **/
  public int analyze()
  {
    System.out.println(self.id.get() + ":" + executions.get () +
      ":Platform.analyze called");
    return Status.OK.value();
  }
  
  /**
   * Returns the GPS accuracy in meters
   * @return GPS accuracy
   **/
  public double getGpsAccuracy()
  {
      System.out.println(self.id.get() + ":" + executions.get () +
        ":  Platform.getGpsAccuracy called");
      return 0.0;
  }

  /**
   * Returns the position accuracy in meters
   * @return position accuracy
   **/
  public double getPositionAccuracy()
  {
    System.out.println(self.id.get() + ":" + executions.get () +
      ":  Platform.getPositionAccuracy called");
    return 0.0;
  }

  /**
   * Returns the current GPS position 
   **/
  public Position getPosition()
  {
    Position position = new Position(0.0, 0.0, 0.0);
    System.out.println(self.id.get() + ":" + executions.get () +
      ":  Platform.getPosition called");
    return position;
  }
  
  /**
   * Returns to the home location. This should be
   * a non-blocking call.
   * @return   status information(@see Status)
   **/
  public int home()
  {
    System.out.println(self.id.get() + ":" + executions.get () +
      ":  Platform.home called");
    return Status.OK.value();
  }
  
  /**
   * Requests the platform to land. This should be
   * a non-blocking call.
   * @return   status information(@see Status)
   **/
  public int land()
  {
    System.out.println(self.id.get() + ":" + executions.get () +
      ":  Platform.land called");
    return Status.OK.value();
  }
  
  /**
   * Initializes a move to the target position. This should be
   * a non-blocking call.
   * @param   target     the new position to move to
   * @param   proximity  the minimum distance between current position
   *                   and target position that terminates the move.
   * @return  status information(@see Status)
   **/
  public int move(Position target, double proximity)
  {
    System.out.println(self.id.get() + ":" + executions.get () +
      ":  Platform.move called");
    return Status.OK.value();
  }
   
  /**
   * Initializes a rotate along x, y, z axes. This should be
   * a non-blocking call.
   * @param   target     the new position to move to
   * @param   proximity  the minimum distance between current position
   *                   and target position that terminates the move.
   * @return  status information(@see Status)
   **/
  public int rotate(Axes target)
  {
    System.out.println(self.id.get() + ":" + executions.get () +
      ":  Platform.rotate called");
    return Status.OK.value();
  }
   
  /**
   * Get sensor radius
   * @return minimum radius of all available sensors for this platform
   */
  public double getMinSensorRange()
  {
    System.out.println(self.id.get() + ":  Platform.getMinSensorRange called");
    return 0.0;
  }

  /**
   * Gets the movement speed
   * @return movement speed
   **/
  public double getMoveSpeed()
  {
    System.out.println(self.id.get() + ":" + executions.get () +
      ":  Platform.getMoveSpeed called");
    return 0.0;
  }

  /**
   * Gets the unique id of the platform. This should be an alphanumeric
   * id that can be part of a MADARA variable name. Specifically, this
   * is used in the variable expansion of .platform.{yourid}.*
   * @return the id of the platform(alphanumeric only: no spaces!)
   **/
  public java.lang.String getId()
  {
    return "java_debugger";
  }

  /**
   * Gets the name of the platform
   * @return the name of the platform
   **/
  public java.lang.String getName()
  {
    return "Java Debugger";
  }

  /**
   * Gets results from the platform's sensors. This should be
   * a non-blocking call.
   * @return   1 if moving, 2 if arrived, 0 if an error occurred
   **/
  public int sense()
  {
    executions.setName(knowledge, ".executions");
    System.out.println(self.id.get() + ":" + executions.get () +
      ":Platform.sense called");
    
    Position position = getPosition();
    
    self.device.location.set(0,position.getX());
    self.device.location.set(1,position.getY());
    self.device.location.set(2,position.getZ());
    
    return Status.OK.value();
  }
  
  /**
   * Sets move speed
   * @param speed new speed in meters/second
   **/
  public void setMoveSpeed(double speed)
  {
    System.out.println(self.id.get() + ":" + executions.get () +
      ":  Platform.setMoveSpeed called with " + speed);
  }
      
  /**
   * Takes off. This should be
   * a non-blocking call.
   * @return   status information(@see Status)
   **/
  public int takeoff()
  {
    System.out.println(self.id.get() + ":" + executions.get () +
      ":  Platform.takeoff called");
    return Status.OK.value();
  }
  
  /**
   * Stops moving
   **/
  public void stopMove()
  {
    System.out.println(self.id.get() + ":" + executions.get () +
      ":  Platform.stopMove called");
  }
  
  private com.madara.containers.Integer executions;
}

