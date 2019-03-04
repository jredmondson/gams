#ifndef  _GAMS_UTILITY_LINUXJOYSTICK_H
#define  _GAMS_UTILITY_LINUXJOYSTICK_H

/**
 * @file LinuxJoystick.h
 * @author James Edmondson <jedmondson@gmail.com>
 *
 * This file contains a class that abstracts the Linux Joystick interface
 **/

#include <string>
#include <stdio.h>
#include "gams/loggers/GlobalLogger.h"

#ifndef __WIN32__

#include <linux/input.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h> 

namespace gams { namespace utility {

class Joystick
{
  private:

  // handle to the device
  int file_ = 0;

  public:

  inline bool close_handle(void)
  {
    close(file_);

    // clean up just in case user calls close 2x
    file_ = 0;
  }

  inline bool open_handle(const std::string & handle)
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
      "gams::utility::Joystick::open: " \
      "attempting close on existing file %d\n",
      file_);

    close_handle();

    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
      "gams::utility::Joystick::open: " \
      "calling fopen on %s\n",
      handle.c_str());

    file_ = open(handle.c_str(), O_RDONLY);// | O_NONBLOCK);

    return file_ > 0;
  }

  Joystick(const std::string & handle)
  : file_(0)
  {
    if(handle != "")
    {
      open_handle(handle);
    }
  }

  Joystick()
  : file_(0)
  {

  }

  ~Joystick()
  {
    close_handle();
  }

  inline bool get(input_event & event)
  {
    int bytes = 0;

    if (file_)
    {
      bytes = read(file_, &event, sizeof(event));
    }

    return bytes == sizeof(input_event);
  }

  inline bool is_axis(const input_event & event) const
  {
    return event.type & 0x02 != 0;
  }

  inline bool is_button(const input_event & event) const
  {
    return event.type & 0x01 != 0;
  }

  inline bool is_init(const input_event & event) const
  {
    return event.type & 0x80 != 0;
  }

  inline double to_double(const input_event & event) const
  {
    double result = event.value;
    result /= 32768;

    // note that we're cheating here because positives go up to 32767
    // so, max positive axis will be 0.999969482

    return result;
  }
};

}}

#endif // __WIN32__

#endif // _GAMS_UTILITY_LINUXJOYSTICK_H
