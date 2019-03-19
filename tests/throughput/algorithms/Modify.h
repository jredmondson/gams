
#ifndef   _ALGORITHM_MODIFY_H_
#define   _ALGORITHM_MODIFY_H_

#include <string>

#include "madara/knowledge/containers/Integer.h"

#include "gams/variables/Sensor.h"
#include "gams/platforms/BasePlatform.h"
#include "gams/variables/AlgorithmStatus.h"
#include "gams/variables/Self.h"
#include "gams/algorithms/BaseAlgorithm.h"
#include "gams/algorithms/AlgorithmFactory.h"

namespace algorithms
{
  /**
  * An algorithm that modifies common agent variables
  **/
  class Modify : public gams::algorithms::BaseAlgorithm
  {
  public:
    /**
     * Constructor
     * @param  knowledge    the context containing variables and values
     * @param  platform     the underlying platform the algorithm will use
     * @param  sensors      map of sensor names to sensor information
     * @param  self         self-referencing variables
     * @param payload_size  the size of the payload that should be
     *                      generated and delivered
     **/
    Modify(
      madara::knowledge::KnowledgeBase * knowledge = 0,
      gams::platforms::BasePlatform * platform = 0,
      gams::variables::Sensors * sensors = 0,
      gams::variables::Self * self = 0,
      gams::variables::Agents * agents = 0,
      int payload_size = 0);

    /**
     * Destructor
     **/
    virtual ~Modify();

    /**
     * Analyzes environment, platform, or other information
     * @return bitmask status of the platform. @see Status.
     **/
    virtual int analyze(void);
      
    /**
     * Plans the next execution of the algorithm
     * @return bitmask status of the platform. @see Status.
     **/
    virtual int execute(void);

    /**
     * Plans the next execution of the algorithm
     * @return bitmask status of the platform. @see Status.
     **/
    virtual int plan(void);

  protected:
    /// file payload
    madara::knowledge::VariableReference payload_;

    /// file payload size
    int payload_size_;

    // executions in the knowledge base
    madara::knowledge::containers::Integer kb_executions_;
  };

  /**
   * A factory class for creating Debug Algorithms
   **/
  class ModifyFactory : public gams::algorithms::AlgorithmFactory
  {
  public:
     /**
     * Creates a Modify algorithm.
     * @param   args      the args passed with the 
     * @param   knowledge the knowledge base to use
     * @param   platform  the platform. This will be set by the
     *                     controller in init_vars.
     * @param   sensors   the sensor info. This will be set by the
     *                     controller in init_vars.
     * @param   self      self-referencing variables. This will be
     *                     set by the controller in init_vars
     * @param   agents    the list of agents, which is dictated by
     *                     init_vars when a number of processes is set. This
     *                     will be set by the controller in init_vars
     **/
    virtual gams::algorithms::BaseAlgorithm * create(
      const madara::knowledge::KnowledgeMap & args,
      madara::knowledge::KnowledgeBase * knowledge,
      gams::platforms::BasePlatform * platform,
      gams::variables::Sensors * sensors,
      gams::variables::Self * self,
      gams::variables::Agents * agents);
  };
} // end algorithms namespace

#endif // _ALGORITHM_MODIFY_H_
