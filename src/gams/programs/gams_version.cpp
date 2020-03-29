#include "gams/utility/Version.h"
#include <iostream>

int main(int, char**)
{
  std::cout << "GAMS libraries are at version "
            << gams::utility::get_version() << ".\n";

  return 0;
}