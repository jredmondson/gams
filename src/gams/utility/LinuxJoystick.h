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
    if (file_ != 0)
    {
      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_MAJOR,
        "gams::utility::Joystick::open: " \
        "attempting close on existing file %x\n",
        file_);

      fclose(file_);

      // clean up just in case user calls close 2x
      file_ = 0;

      return true;
    }
    else
    {
      madara_logger_ptr_log(gams::loggers::global_logger.get(),
        gams::loggers::LOG_MAJOR,
        "gams::utility::Joystick::open: " \
        "file is unset\n");
    }

    return false;
  }

  inline bool open_handle(const std::string & handle)
  {
    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
      "gams::utility::Joystick::open: " \
      "attempting close on file %x\n",
      file_);

    close_handle();

    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
      "gams::utility::Joystick::open: " \
      "calling fopen on %s\n",
      handle.c_str());

    file_ = fopen(handle.c_str(), "rb");

    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
      "gams::utility::Joystick::open: " \
      "return of fopen is %x\n",
      file_);

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
    size_t events = 0;

    if (file_ != 0)
    {
      events = fread(&event, sizeof(event), 1, file_);
    }

    madara_logger_ptr_log(gams::loggers::global_logger.get(),
      gams::loggers::LOG_MAJOR,
      "gams::utility::Joystick::open: " \
      "fread returned %zu events\n",
      events);

    return events == 1;
  }

  inline bool is_axis(const JoystickEvent & event) const
  {
    // we don't want inits
    return (event.type & 0x02) != 0;
  }

  inline bool is_button(const JoystickEvent & event) const
  {
    // we don't want inits
    return (event.type & 0x01) != 0;
  }

  inline bool is_x_move(const JoystickEvent & event) const
  {
    return event.number == 0;
  }

  inline bool is_y_move(const JoystickEvent & event) const
  {
    return event.number == 1;
  }

  inline bool is_rotate(const JoystickEvent & event) const
  {
    return event.number == 3;
  }

  inline bool is_z_move(const JoystickEvent & event) const
  {
    return event.number == 4;
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
