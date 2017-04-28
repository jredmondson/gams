
#ifndef   _FILTER_RCV_COUNT_H_
#define   _FILTER_RCV_COUNT_H_

#include <string>
#include <map>
#include "madara/knowledge/containers/Integer.h"

#include "madara/filters/AggregateFilter.h"

#include "../utility/MovingAverage.h"

namespace filters
{
  // convenience typedef for originators map
  typedef   std::map <std::string, int>   Originators;

  /**
  * A filter for keeping track of statistics on receive
  **/
  class RcvCount : public madara::filters::AggregateFilter
  {
  public:
    /**
     * Constructor
     * @param averages  a moving average calculator to use with on messages
     * @param no_apply  if true, clear the records so they are not applied
     **/
    RcvCount (utility::MovingAverage & latencies, bool no_apply = false);

    /**
     * Destructor
     **/
    virtual ~RcvCount ();

    /**
     * The method that filters incoming or outgoing 
     * @param   records           the aggregated packet being evaluated
     * @param   transport_context context for querying transport state
     * @param   vars              context for querying current program state
     **/
    virtual void filter (madara::knowledge::KnowledgeMap & records,
      const madara::transport::TransportContext & transport_context,
      madara::knowledge::Variables & vars);

    /**
     * Retrieves the number of unique originators
     **/
    size_t num_originators (void);

    /**
    * Retrieves the number of unique originators
    **/
    Originators originators (void);

  protected:
    /**
     * counters
     **/
    madara::knowledge::containers::Integer  count_;

    /**
    * number of filtered messages
    **/
    madara::knowledge::containers::Integer  filtered_messages_;

    /// checks for initialization
    bool initialized_;

    /// if enabled, deletes all received data before applying to KB
    bool no_apply_;

    /// moving averages calculator that can be accessed from caller
    utility::MovingAverage & latencies_;

    /// keeps track of the unique originators
    Originators originators_;
  };

} // end filters namespace

#endif // _FILTER_RcvCount_H_
