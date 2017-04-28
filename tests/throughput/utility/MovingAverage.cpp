#include "MovingAverage.h"

utility::MovingAverage::MovingAverage (size_t size)
  : values_ (size), max_ (0.0), min_ (0.0), has_values (false),
    index_ (0), count_ (0)
{
  values_.resize (size);
}


utility::MovingAverage::~MovingAverage ()
{

}


double utility::MovingAverage::average (void)
{
  double total (0.0);

  for (size_t i = 0; i < count_; ++i)
  {
    total += values_[i];
  }

  if (count_ > 0)
  {
    total = total / (double)count_;
  }

  return total;
}


double utility::MovingAverage::get_max (void)
{
  return max_;
}


double utility::MovingAverage::get_min (void)
{
  return min_;
}


void utility::MovingAverage::add (double latency)
{
  if (!has_values || latency < min_)
    min_ = latency;

  if (!has_values || latency > max_)
    max_ = latency;

  // set latency and increment within circular buffer
  values_[index_] = latency;
  ++index_ == values_.size () ? index_ = 0 : index_;

  
  
  // increment count if we have not filled the buffer
  if (count_ < values_.size ())
    ++count_;

  if (!has_values)
    has_values = true;
}

void utility::MovingAverage::dump (std::vector <double> & values)
{
  values = values_;
}

size_t utility::MovingAverage::size (void)
{
  return count_;
}
