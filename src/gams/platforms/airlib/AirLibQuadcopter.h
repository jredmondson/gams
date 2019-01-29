/**
 * @file AirLibQuadcopter.h
 * @author Devon Ash <noobaca2@gmail.com>
 *
 * This file contains the declaration of the AirLibQuadcopter interface to AirSim's quadcopter. AirLib communicates to AirSim over RPC.
 **/

#ifndef   _GAMS_PLATFORM_AIRLIB_QUADCOPTER_H_
#define   _GAMS_PLATFORM_AIRLIB_QUADCOPTER_H_

#include "gams/platforms/PlatformFactory.h"
#include "gams/platforms/airlib/AirLibBase.h"

#include "gams/variables/Self.h"
#include "gams/variables/Sensor.h"
#include "gams/variables/PlatformStatus.h"
#include "madara/knowledge/KnowledgeBase.h"

#ifdef _GAMS_AIRLIB_

namespace gams
{
  namespace platforms
  {
    /**
    * A AIRLIB platform for an autonomous surface Quadcopter robotic system
    **/
    class GAMS_EXPORT AirLibQuadcopter : public AirLibBase
    {

    public:

      /**
       * Default Quadcopter model
       */
      const static std::string DEFAULT_Quadcopter_MODEL;

      /**
       * Constructor
       * @param  file         file of model to load
       * @param  client_side  0 if model is server side, 1 if client side
       * @param  knowledge    knowledge base
       * @param  sensors      map of sensor names to sensor information
       * @param  platforms    map of platform names to platform information
       * @param  self         agent variables that describe self state
       **/
      AirLibQuadcopter (
        const std::string& file, 
        madara::knowledge::KnowledgeBase * knowledge,
        variables::Sensors * sensors,
        variables::Platforms * platforms,
        variables::Self * self
      );
      
      /**
       * Gets the unique identifier of the platform. This should be an
       * alphanumeric identifier that can be used as part of a MADARA
       * variable (e.g. AIRLIB_ant, autonomous_snake, etc.)
       **/
      virtual std::string get_id () const;

      /**
       * Gets the name of the platform
       **/
      virtual std::string get_name () const;

      /**
       * Get the position accuracy in meters
       * @return position accuracy
       **/
      virtual double get_accuracy () const;

    protected:

      /**
       * Get target handle
       */
      virtual void get_target_handle ();
  
      /**
       * Set initial position
       */
      virtual double get_initial_z () const;
    }; // class AirLibQuadcopter

    /**
     * A factory class for creating AIRLIB Quadcopter platforms
     **/
    class GAMS_EXPORT AirLibQuadcopterFactory : public PlatformFactory
    {
    public:

      /**
       * Creates a AIRLIB ant platform.
       * @param   args      no arguments are necessary for this platform
       * @param   knowledge the knowledge base. This will be set by the
       *                    controller in init_vars.
       * @param   sensors   the sensor info. This will be set by the
       *                    controller in init_vars.
       * @param   platforms status inform for all known agents. This
       *                    will be set by the controller in init_vars
       * @param   self      self-referencing variables. This will be
       *                    set by the controller in init_vars
       **/
      virtual BasePlatform * create (
        const madara::knowledge::KnowledgeMap & args,
        madara::knowledge::KnowledgeBase * knowledge,
        variables::Sensors * sensors,
        variables::Platforms * platforms,
        variables::Self * self);
    };
  } // namespace platform
} // namespace gams

#endif // _GAMS_AIRLIB_

#endif // _GAMS_PLATFORM_AIRLIB_QUADCOPTER_H_
