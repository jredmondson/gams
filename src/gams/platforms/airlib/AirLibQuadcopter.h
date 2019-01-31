/**
 * @file AirLibQuadcopter.h
 * @authors Alex Rozgo, Devon Ash <noobaca2@gmail.com>
 *
 * This file contains the declaration of the AirLibQuadcopter interface to AirSim's quadcopter. AirLib communicates to AirSim over RPC.
 **/

#ifndef   _GAMS_PLATFORM_AIRLIB_QUADCOPTER_H_
#define   _GAMS_PLATFORM_AIRLIB_QUADCOPTER_H_

#include "gams/platforms/PlatformFactory.h"
#include "gams/platforms/airlib/AirLibBase.h"

#include "gams/variables/Self.h"
#include "gams/variables/Sensor.h"
#include "gams/pose/GPSFrame.h"
#include "gams/variables/PlatformStatus.h"
#include "gams/pose/CartesianFrame.h"

#include "madara/knowledge/KnowledgeBase.h"
#include "madara/threads/Threader.h"

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
       * Constructor
       * @param  client_side  0 if model is server side, 1 if client side
       * @param  knowledge    knowledge base
       * @param  sensors      map of sensor names to sensor information
       * @param  platforms    map of platform names to platform information
       * @param  self         agent variables that describe self state
       **/
      AirLibQuadcopter (
        madara::knowledge::KnowledgeBase * knowledge,
        variables::Sensors * sensors,
        variables::Self * self
      );

      /**
       * Destructor
       */
      virtual ~AirLibQuadcopter ();
      
      /**
       * Gets the unique identifier of the platform. This should be an
       * alphanumeric identifier that can be used as part of a MADARA
       * variable (e.g. AIRLIB_ant, autonomous_snake, etc.)
       **/
      virtual std::string get_id () const override;

      /**
       * Gets the name of the platform
       **/
      virtual std::string get_name () const override;

      virtual int sense () override;

      virtual int home () override;

      virtual int takeoff () override;

      virtual int land () override;

      virtual int move (const gams::pose::Position & location, const gams::platforms::PositionBounds & bounds) override;

      virtual const gams::pose::ReferenceFrame& get_frame () const override;

      int rotate (const gams::pose::Orientation &target, double epsilon);

    private:

      msr::airlib::MultirotorRpcLibClient* client_;

      std::string vehicle_name_;

      madara::threads::Threader threader_;  

    }; // class AirLibQuadcopter

    /**
     * A factory class for creating VREP Boat platforms
     **/
    class GAMS_EXPORT AirLibQuadcopterFactory : public PlatformFactory
    {
    public:

      /**
       * Creates a AirLibQuadcopter platform.
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
      virtual ~AirLibQuadcopterFactory (){};

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
