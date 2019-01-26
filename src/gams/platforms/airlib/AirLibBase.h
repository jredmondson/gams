/**
 * @file AirLibBase.h
 * @author Devon Ash <noobaca2@gmail.com>
 *
 * This file contains the definition of the AirLibBase abstract class
 **/

#ifndef   _GAMS_PLATFORM_AIRLIB_BASE_H_
#define   _GAMS_PLATFORM_AIRLIB_BASE_H_

#ifdef _GAMS_AIRLIB_


#include "gams/variables/Self.h"
#include "gams/variables/Sensor.h"
#include "gams/variables/PlatformStatus.h"
#include "gams/platforms/BasePlatform.h"
#include "gams/pose/GPSFrame.h"
#include "gams/pose/CartesianFrame.h"
#include "madara/knowledge/KnowledgeBase.h"

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

namespace gams
{
  namespace platforms
  {
    class GAMS_EXPORT AirLibBase : public BasePlatform
    {
    public:
      /**
       * Constructor
       * @param  knowledge  knowledge base
       * @param  sensors    map of sensor names to sensor information
       * @param  platforms  map of platform names to platform information
       * @param  self       device variables that describe self state
       **/
      AirLibBase (
        madara::knowledge::KnowledgeBase * knowledge,
        variables::Sensors * sensors,
        variables::Self * self);

      /**
       * Polls the sensor environment for useful information
       * @return number of sensors updated/used
       **/
      virtual int sense (void) override;

      /**
       * Analyzes platform information
       * @return bitmask status of the platform. @see Status.
       **/
      virtual int analyze (void) override;

      /**
       * Get the position accuracy in meters
       * @return position accuracy
       **/
      virtual double get_accuracy () const override;

      /**
       * Instructs the platform to land
       * @return 1 if moving, 2 if arrived, 0 if error
       **/
      virtual int land (void) override;

      /**
       * Moves the platform to a position
       * @param   position  the coordinate to move to
       * @param   epsilon   approximation value
       * @return 1 if moving, 2 if arrived, 0 if error
       **/
      int move (const pose::Position & position,
        const PositionBounds &bounds) override;

      // inherit BasePlatform's move overloads
      using BasePlatform::move;
      
      /**
       * Set move speed
       * @param speed new speed in meters/loop execution
       **/
      virtual void set_move_speed (const double& speed) override;

      /**
       * Instructs the platform to take off
       * @return 1 if moving, 2 if arrived, 0 if error
       **/
      virtual int takeoff (void) override;

      /**
       * Returns the reference frame for the platform (usually GPS)
       * @return the platform's reference frame for positions
       **/
      virtual const pose::ReferenceFrame & get_frame (void) const override;


    protected:
      /// the current frame (can theoretically be switched between options)
      pose::ReferenceFrame * frame_;

      /**
       * wait for go signal from controller
       */
      void wait_for_go () const;

      /// flag for simulated robot ready to receive instruction
      bool ready_;
    }; // class AirLibBase
  } // namespace platform
} // namespace gams

#endif // _GAMS_AIRLIB_

#endif // _GAMS_PLATFORM_AIRLIB_BASE_H_
