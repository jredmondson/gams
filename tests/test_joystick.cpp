#include "gams/utility/LinuxJoystick.h"
#include <iostream>

int gams_errors = 0;

int main(int, char**)
{
#ifndef __WIN32__
  gams::utility::Joystick joystick;
  
  if (joystick.open_handle ("/dev/input/js0"))
  {
    while(true)
    {
      gams::utility::JoystickEvent event;
      if (joystick.get(event))
      {
        if (joystick.is_axis(event))
        {
          std::cerr << "axis: " <<
            "code=" << event.number <<
            "type=" << event.type <<
            "value=" << event.value <<
            "double()=" << joystick.to_double(event) << "\n";
        }
        else if (joystick.is_button(event))
        {
          std::cerr << "button: " <<
            "code=" << event.number <<
            "type=" << event.type <<
            "value=" << event.value <<
            "double()=" << joystick.to_double(event) << "\n";
        }
        else if (joystick.is_init(event))
        {
          std::cerr << "init: " <<
            "code=" << event.number <<
            "type=" << event.type <<
            "value=" << event.value <<
            "double()=" << joystick.to_double(event) << "\n";
        }
      }
      else
      {
        std::cerr << "Nothing to read...\n";
      }
    }
  }
  else
  {
    std::cerr << "Unable to open /dev/input/js0 for reading...\n";
  }
  

#endif
  return 0;
}
