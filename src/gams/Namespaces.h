#ifndef _GAMS_NAMESPACES_H_
#define _GAMS_NAMESPACES_H_

/**
* @file Namespaces.h
* @author James Edmondson <jedmondson@gmail.com>
*
* This file contains namespace documentation. There is no reason to include
* this file in any project. It is purely for Doxygen documentation.
**/

/**
 * @namespace gams
 * Contains all GAMS-related tools, classes and code
 **/
namespace gams
{
  /**
  * @namespace gams::algorithms
  * Provides algorithms and infrastructure for autonomy
  **/
  namespace algorithms
  {
    /**
    * @namespace gams::algorithms::area_coverage
    * Provides algorithms specialized in area coverage
    **/
    namespace area_coverage
    {
    }
    
    /**
    * @namespace gams::algorithms::java
    * Provides Java algorithms support 
    **/
    namespace area_coverage
    {
    }
  }

  /**
  * @namespace gams::auctions
  * Provides mechanisms for distributed auctions
  **/
  namespace auctions
  {
  }

  /**
  * @namespace gams::controllers
  * Provides controllers for managing autonomy and execution
  **/
  namespace controllers
  {
  }

  /**
  * @namespace gams::elections
  * Provides voting mechanisms for distributed elections
  **/
  namespace elections
  {
  }

  /**
  * @namespace gams::formations
  * Provides convenience classes for formations
  **/
  namespace formations
  {
  }

  /**
  * @namespace gams::groups
  * Provides infrastructure for managing distributed groups
  **/
  namespace groups
  {
  }

  /**
  * @namespace gams::loggers
  * Provides knowledge logging services to files and terminals
  **/
  namespace loggers
  {
  }

  /**
  * @namespace gams::maps
  * Provides map-related functions
  **/
  namespace maps
  {
  }

  /**
  * @namespace gams::platforms
  * Provides platform abstractions for hardware and simulations
  **/
  namespace platforms
  {
    /**
    * @namespace gams::platforms::dronerk
    * Provides bindings to Drone-RK (no longer supported)
    **/
    namespace dronerk
    {
    }
    
    /**
    * @namespace gams::platforms::java
    * Provides infrastructure for Java-based platforms
    **/
    namespace java
    {
    }
    
    /**
    * @namespace gams::platforms::ros
    * Provides infrastructure for ROS-based platforms
    **/
    namespace ros
    {
    }
    
    /**
    * @namespace gams::platforms::vrep
    * Provides infrastructure for VREP-based platforms
    **/
    namespace vrep
    {
    }
  }

  /**
  * @namespace gams::pose
  * Provides position and orientation primitives
  **/
  namespace pose
  {
    /**
    * @namespace gams::pose::detail
    * Provides underlying pose classes that provide infrastructure for
    * higher level pose-related functions
    **/
    namespace detail
    {
    }

    /**
    * @namespace gams::pose::euler
    * Provides support for Euler tools for positions and orientations
    **/
    namespace euler
    {
    }

    /**
    * @namespace gams::pose::order
    * Provides support for ordering reference-frame inputss
    **/
    namespace order
    {
    }
  }

  /**
  * @namespace gams::programs
  * Provides source for certain non-test-related programs
  **/
  namespace programs
  {
  }

  /**
  * @namespace gams::time
  * Provides time-related tools and mechanisms
  **/
  namespace time
  {
  }
  
  /**
  * @namespace gams::utility
  * Provides utility functions and classes for common tasks and needs
  **/
  namespace utility
  {
    /**
    * @namespace gams::utility::java
    * Provides utility functions and classes for common Java tasks and needs
    **/
    namespace java
    {
    }

    /**
    * @namespace gams::utility::euler
    * DEPRECATED: Provides classes here have been moved to gams::pose::euler
    **/
    namespace euler
    {
    }

    /**
    * @namespace gams::utility::order
    * DEPRECATED: Provides classes here have been moved to gams::pose::order
    **/
    namespace order
    {
    }
  }

  
  /**
  * @namespace gams::variables
  * Provides MADARA containers for GAMS-related entities and information
  **/
  namespace variables
  {
  }

}


#endif // end _GAMS_NAMESPACES_H_
