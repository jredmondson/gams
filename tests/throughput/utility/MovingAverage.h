#ifndef _UTILITY_MOVING_AVERAGE_
#define _UTILITY_MOVING_AVERAGE_

#include <vector>
#include <stdlib.h>

namespace utility
{
  /**
   * Calculates a moving average of the last size doubles
   **/
  class MovingAverage
  {
  public:
    /**
     * Constructor
     * @param  size   the number of values allowed in the moving average
     **/
    MovingAverage (size_t size = 100);

    /**
    * Destructor
    **/
    ~MovingAverage ();

    /**
    * adds a latency to the average
    **/
    void add (double latency);

    /**
     * returns an average
     **/
    double average (void);

    /**
    * dumps the latency list to a vector
    **/
    void dump (std::vector <double> & values);

    /**
    * returns the maximum value ever seen
    * @return the minimum value
    **/
    double get_max (void);

    /**
    * returns the minimum value ever seen
    * @return the minimum value
    **/
    double get_min (void);

    /**
     * returns the number of values
     * @return the number of values currently in the moving average
     **/
    size_t size (void);

  private:

    /// the list of values
    std::vector <double> values_;

    /// maximum value seen
    double max_;

    /// minimum value seen
    double min_;

    /// indicates if min and max are valid
    bool has_values;

    /// index into the circular value array
    size_t index_;

    /// number of elements actually stored in array
    size_t count_;
  };
}

#endif // _UTILITY_MOVING_AVERAGE_
