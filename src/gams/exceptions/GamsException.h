/* -*- C++ -*- */
#ifndef _GAMS_EXCEPTIONS_GAMSEXCEPTION_H_
#define _GAMS_EXCEPTIONS_GAMSEXCEPTION_H_

#include <string>
#include <stdexcept>

namespace gams
{
  namespace exceptions
  {
    /**
     * Base exception class for all custom Gams exceptions
     **/
    class GamsException : public std::runtime_error
    {
    public:
      using std::runtime_error::runtime_error;
    };
  }
}

#endif /* _GAMS_EXCEPTIONS_GAMSEXCEPTION_H_ */
