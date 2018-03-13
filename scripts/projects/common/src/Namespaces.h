#ifndef _RISLAB_NAMESPACES_H_
#define _RISLAB_NAMESPACES_H_

/**
* @file Namespaces.h
* @author James Edmondson <jedmondson@gmail.com>
*
* This file contains namespace documentation. There is no reason to include
* this file in any project. It is purely for Doxygen documentation.
**/

/**
 * @namespace algorithms
 * Contains algorithms that can be changed at runtime in GAMS
 **/
namespace algorithms
{
}

/**
 * @namespace containers
 * Contains managed C++ containers that map between C++ variables and knowledge variables
 **/
namespace containers
{
}

/**
 * @namespace filters
 * Contains filters that may shape traffic on-send or on-receive
 **/
namespace filters
{
}

/**
 * @namespace platforms
 * Contains platform drivers and platform threads
 **/
namespace platforms
{
  /**
   * @namespace platforms::threads
   * Contains threads that will be managed by platform drivers
   **/
  namespace threads
  {
  }
}

/**
 * @namespace threads
 * Contains threads that will be managed by an agent controller
 **/
namespace threads
{
}

#endif

