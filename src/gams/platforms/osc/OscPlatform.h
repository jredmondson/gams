
#ifndef   _GAMS_PLATFORM_OSCPLATFORM_H_
#define   _GAMS_PLATFORM_OSCPLATFORM_H_

#include "madara/knowledge/KnowledgeBase.h"

#include "gams/variables/Self.h"
#include "gams/variables/Sensor.h"
#include "gams/variables/PlatformStatus.h"
#include "gams/platforms/BasePlatform.h"
#include "gams/platforms/PlatformFactory.h"
#include "gams/pose/CartesianFrame.h"

#include "gams/utility/OscUdp.h"

namespace gams { namespace platforms
{        
  /**
  * A low fidelity platform driver for issuing Open State Protocol
  * bundles to control agents in UnrealGams
  **/
  class GAMS_EXPORT OscPlatform : public BasePlatform
  {
  public:

    enum MovementTypes
    {
      MOVEMENT_STOP_ON_ARRIVAL = 0,
      MOVEMENT_PASSTHROUGH = 1
    };

    /**
     * Constructor
     * @param  knowledge  context containing variables and values
     * @param  sensors    map of sensor names to sensor information
     * @param  self       self referencing variables for the agent
     **/
    OscPlatform (
      madara::knowledge::KnowledgeBase * knowledge = 0,
      gams::variables::Sensors * sensors = 0,
      gams::variables::Self * self = 0);

    /**
     * Destructor
     **/
    virtual ~OscPlatform ();

    /**
     * Polls the sensor environment for useful information. Required.
     * @return number of sensors updated/used
     **/
    virtual int sense (void) override;

    /**
     * Analyzes platform information. Required.
     * @return bitmask status of the platform. @see PlatformAnalyzeStatus.
     **/
    virtual int analyze (void) override;

    /**
     * Gets the name of the platform. Required.
     **/
    virtual std::string get_name () const override;

    /**
     * Gets the unique identifier of the platform. This should be an
     * alphanumeric identifier that can be used as part of a MADARA
     * variable (e.g. vrep_ant, autonomous_snake, etc.) Required.
     * @return the id of the platform to use in factory methods
     **/
    virtual std::string get_id () const override;

    /**
     * Gets the position accuracy in meters. Optional.
     * @return position accuracy
     **/
    virtual double get_accuracy (void) const override;

    /**
     * Gets sensor radius. Optional.
     * @return minimum radius of all available sensors for this platform
     **/
    virtual double get_min_sensor_range (void) const override;

    /**
     * Gets move speed. Optional.
     * @return speed in meters per second
     **/
    virtual double get_move_speed (void) const override;

    /**
     * Instructs the agent to return home. Optional.
     * @return the status of the home operation, @see PlatformReturnValues
     **/
    virtual int home (void) override;

    /**
     * Instructs the agent to land. Optional.
     * @return the status of the land operation, @see PlatformReturnValues
     **/
    virtual int land (void) override;

    /**
     * Moves the platform to a location
     * @param   target    the coordinates to move to
     * @param   bounds   approximation value
     * @return the status of the move operation, @see PlatformReturnValues
     **/
    int move (const pose::Position & location,
      const PositionBounds &bounds) override;

    using BasePlatform::move;

    /**
     * Rotates the platform to a specified Orientation
     * @param   target    the coordinates to move to
     * @param   bounds   approximation value, in radians
     * @return the status of the orient operation, @see PlatformReturnValues
     **/
    int orient (const pose::Orientation & location,
    const OrientationBounds &bounds) override;

    using BasePlatform::orient;

    /**
     * Pauses movement, keeps source and dest at current values. Optional.
     **/
    virtual void pause_move (void) override;

    /**
     * Set move speed. Optional.
     * @param speed new speed in meters/second
     **/
    virtual void set_move_speed (const double& speed) override;

    /**
     * Stops movement, resetting source and dest to current location.
     * Optional.
     **/
    virtual void stop_move (void) override;

    /**
     * Instructs the agent to take off. Optional.
     * @return the status of the takeoff, @see PlatformReturnValues
     **/
    virtual int takeoff (void) override;
    
    /**
     * Returns the world reference frame for the platform (e.g. GPS or cartesian)
     **/
    virtual const gams::pose::ReferenceFrame & get_frame (void) const override;
    
  private:
    /**
     * Builds prefixes
     **/
    void build_prefixes(void);

    /**
     * Calculate velocity/thrust necessary to send in OSC
     * @param current  current location
     * @param target   target location
     * @param type     0 (try to stop at target), 1 (keep on trucking)
     * @return  thrust vector to get from current to target according to
     *          movement profile
     **/
    std::vector<double> calculate_thrust(
      const pose::Position & current, const pose::Position & target,
      int type = MOVEMENT_STOP_ON_ARRIVAL);

    /// handle to OSC UDP utility class
    gams::utility::OscUdp osc_;

    /// transport settings for OSC
    madara::transport::QoSTransportSettings settings_;

    /// holds xy velocity prefix
    std::string xy_velocity_prefix_;

    /// holds z velocity prefix
    std::string z_velocity_prefix_;

    /// holds xy velocity prefix
    std::string xyz_position_prefix_;

    /// holds z velocity prefix
    std::string xyz_quaternion_prefix_;
  }; // end OscPlatform class
    

  /**
   * A factory class for creating OscPlatform platforms
   **/
  class GAMS_EXPORT OscPlatformFactory : public PlatformFactory
  {
  public:

    /**
     * Destructor. Shouldn't be necessary but trying to find vtable issue
     **/
    virtual ~OscPlatformFactory();

    /**
     * Creates a OscPlatform platform.
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
    
} // end platforms namespace
} // end gams namespace

#endif // _GAMS_PLATFORM_OSCPLATFORM_H_