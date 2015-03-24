/*********************************************************************
 * Usage of this software requires acceptance of the GAMS-CMU License,
 * which can be found at the following URL:
 *
 * https://code.google.com/p/gams-cmu/wiki/License
 *********************************************************************/
package com.gams.algorithms;

import com.madara.KnowledgeBase;
import com.gams.platforms.PlatformInterface;
import com.gams.variables.Self;
import com.gams.variables.Algorithm;

/**
 * Interface for defining an algorithm to be used by GAMS. Care must be taken
 * to make all methods non-blocking, to prevent locking up the underlying
 * MADARA context.
 */
public interface AlgorithmInterface
{
  /**
   * Analyzes the algorithm for new status information. This should be
   * a non-blocking call.
   * @return  status information (@see Status)
   **/
  public int analyze ();
  
  /**
   * Plans next steps in the algorithm. This should be
   * a non-blocking call.
   * @return  status information (@see Status)
   **/
  public int plan ();
  
  /**
   * Executes next step in the algorithm. This should be
   * a non-blocking call.
   * @return  status information (@see Status)
   **/
  public int execute ();
}

