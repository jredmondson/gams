/*********************************************************************
 * Usage of this software requires acceptance of the GAMS-CMU License,
 * which can be found at the following URL:
 *
 * https://code.google.com/p/gams-cmu/wiki/License
 *********************************************************************/
package com.gams.algorithms;

/**
 * Algorithm settings debugger class
 */
public class DebuggerAlgorithm extends BaseAlgorithm
{
  /**
   * Default constructor
   **/
  public DebuggerAlgorithm()
  {
  }

  /**
   * Analyzes the state of the algorithm
   **/
  public int analyze ()
  {
    System.out.println("Algorithm.analyze called");
    
    if(platform.getCPtr() != 0)
    {
      System.out.println("  platform_id: " + platform.getId() +
      	", platform_name: " + platform.getName());
      platform.getPositionAccuracy();
      platform.getPosition();
      platform.getMinSensorRange();
      platform.getMoveSpeed();
    }
    else
    {
      System.out.println("  platform is null, so no operations called");
    }
    
    return Status.OK.value ();
  }
  
  /**
   * Plans the next stage of the algorithm
   **/
  public int plan ()
  {
    System.out.println("Algorithm.plan called");
    
    return Status.OK.value ();
  }
  
  /**
   * Executes the next stage of the algorithm
   **/
  public int execute ()
  {
    System.out.println("Algorithm.execute called");
    
    if(platform.getCPtr() != 0)
    {
      platform.takeoff();
      platform.home();
      platform.land();
      platform.move(platform.getPosition(), 2.0);
      platform.setMoveSpeed(3.0);
      platform.stopMove();
    }
    else
    {
      System.out.println("  platform is null, so no operations called");
    }
    
    return Status.OK.value ();
  }
}

