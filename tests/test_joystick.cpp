#include "gams/utility/LinuxJoystick.h"
#include <iostream>

int gams_errors = 0;

int main(int, char**)
{
  gams::utility::Joystick joystick;
  
  gams::loggers::global_logger->set_level(4);

  if (joystick.open_handle ("/dev/input/js0"))
  {
    while(true)
    {
      gams::utility::JoystickEvent event;
      if (joystick.get(event))
      {
        if (joystick.is_axis(event))
        {
          std::cerr << "AXIS: " <<
            "code=" << event.number <<
            ",type=" << event.type <<
            ",value=" << event.value <<
            ",double()=" << joystick.to_double(event) << "\n";
        }
        else if (joystick.is_button(event))
        {
          std::cerr << "BUTTON: " <<
            "code=" << event.number <<
            ",type=" << event.type <<
            ",value=" << event.value <<
            ",double()=" << joystick.to_double(event) << "\n";
        }
        else if (joystick.is_init(event))
        {
          std::cerr << "INIT: " <<
            "code=" << event.number <<
            ",type=" << event.type <<
            ",value=" << event.value <<
            ",double()=" << joystick.to_double(event) << "\n";
        }
        else
        {
          std::cerr << "UNKNOWN: " <<
            "code=" << event.number <<
            ",type=" << event.type <<
            ",value=" << event.value <<
            ",double()=" << joystick.to_double(event) << "\n";
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
  
  return 0;
}
