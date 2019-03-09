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

namespace gams { namespace utility {

class JoystickEvent
{
  public:
    uint32_t time;
    int16_t value;
    unsigned char type;
    unsigned char number;
};

class Joystick
{
  private:

  // handle to the device
  FILE * file_ = 0;

  public:

  inline bool close_handle(void)
  {
    fclose(file_);

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

    file_ = fopen(handle.c_str(), "rb");

    return file_ != 0;
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

  inline bool get(JoystickEvent & event)
  {
    int bytes = 0;

    if (file_)
    {
      bytes = fread(&event, sizeof(event), 1, file_);
    }

    return bytes == sizeof(event);
  }

  inline bool is_axis(const JoystickEvent & event) const
  {
    return event.type & 0x02 != 0;
  }

  inline bool is_button(const JoystickEvent & event) const
  {
    return event.type & 0x01 != 0;
  }

  inline bool is_init(const JoystickEvent & event) const
  {
    return event.type & 0x80 != 0;
  }

  inline double to_double(const JoystickEvent & event) const
  {
    double result = event.value;
    result /= 32768;

    // note that we're cheating here because positives go up to 32767
    // so, max positive axis will be 0.999969482

    return result;
  }
};

}}

#endif // _GAMS_UTILITY_LINUXJOYSTICK_H
