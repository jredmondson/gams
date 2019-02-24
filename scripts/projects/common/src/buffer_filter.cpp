#include "MyFilter.h"

#include "madara/utility/Utility.h"

#include "madara/logger/GlobalLogger.h"

filters::MyFilter::MyFilter()
{
}

filters::MyFilter::~MyFilter()
{
}

int filters::MyFilter::encode(
    char* source, int size, int max_size) const
{
  int encoded_size = 0;

  return encoded_size;
}

int filters::MyFilter::decode(
    char* source, int size, int max_size) const
{
  int decoded_size = 0;

  return decoded_size;
}

std::string filters::MyFilter::get_id(void)
{
  return "MyFilter";
}

/**
 * Gets the version of the filter. @see madara::utility::get_uint_version
 * for one way to get this from a string version
 **/
uint32_t filters::MyFilter::get_version(void)
{
  return madara::utility::get_uint_version("1.0.0");
}
