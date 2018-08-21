/* -*- C++ -*- */
#ifndef _GAMS_EXCEPTIONS_REFERENCEFRAMEEXCEPTION_H_
#define _GAMS_EXCEPTIONS_REFERENCEFRAMEEXCEPTION_H_

#include <string>
#include "GamsException.h"

namespace gams
{
  namespace exceptions
  {
    /**
     * An exception class for ReferenceFrame errors
     **/
    class ReferenceFrameException : public GamsException
    {
    public:
      using GamsException::GamsException;
    };
  }
}

#endif /* _GAMS_EXCEPTIONS_REFERENCEFRAMEEXCEPTION_H_ */
