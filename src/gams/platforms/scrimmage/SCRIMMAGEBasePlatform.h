#ifndef _GAMS_PLATFORM_SCRIMMAGE_BASEPLATFORM_H_
#define _GAMS_PLATFORM_SCRIMMAGE_BASEPLATFORM_H_

#include "gams/variables/Self.h"
#include "gams/variables/Sensor.h"
#include "gams/variables/PlatformStatus.h"
#include "gams/platforms/BasePlatform.h"
#include "gams/platforms/PlatformFactory.h"
#include "madara/knowledge/KnowledgeBase.h"
#include "gams/pose/CartesianFrame.h"
#include "gams/pose/Position.h"
#include "gams/pose/Orientation.h"
#include "gams/pose/GPSFrame.h"
#include "madara/threads/Threader.h"
#include "madara/threads/BaseThread.h"
#include "madara/LockType.h"
#include "madara/knowledge/containers/Integer.h"



#include "gams/loggers/GlobalLogger.h"

#include <scrimmage/simcontrol/SimControl.h>

namespace gams
{
  namespace platforms 
  {
     class SCRIMMAGEBasePlatform : BasePlatform
     {
     public:
       SCRIMMAGEBasePlatform(
        scrimmage::SimControl& simcontrol,
        madara::knowledge::KnowledgeBase * knowledge,
        variables::Sensors * sensors,
        variables::Self * self
       );
     
       virtual ~SCRIMMAGEBasePlatform();
       
       void operator=(const SCRIMMAGEBasePlatform & rhs);
     
       /*
          Required implementations
       */
       virtual int sense(void) override;
       virtual int analyze(void) override;
       virtual std::string get_name() const override;
       virtual std::string get_id() const override;

       /*
          Platform actions required for Spell, Zone Coverage, Move.
       */
       virtual double get_accuracy() const override;
       virtual int move(const pose::Position & target, const pose::PositionBounds &bounds) override;
       virtual const gams::pose::ReferenceFrame & get_frame(void) const override;
       
       private:
       
         scrimmage::SimControl simcontrol;
     };
     
     
  /**
   * A factory class for creating OscJoystickPlatform platforms
   **/
  class GAMS_EXPORT SCRIMMAGEBasePlatformFactory : public PlatformFactory
  {
  public:

    /**
     * Constructor
     * @param type   the type of robotics system to simulate(quadcopter,
     *               satellite)
     **/
    SCRIMMAGEBasePlatformFactory(const std::string & type = "car");

    /**
     * Destructor. Shouldn't be necessary but trying to find vtable issue
     **/
    virtual ~SCRIMMAGEBasePlatformFactory();

    /**
     * Creates a OscJoystickPlatform platform.
     * @param   args      no arguments are necessary for this platform
     * @param   knowledge the knowledge base. This will be set by the
     *                    controller in init_vars.
     * @param   sensors   the sensor info. This will be set by the
     *                    controller in init_vars.
     * @param   platforms status inform for all known agents. This
     *                    will be set by the controller in init_vars
     * @param   self      self-referencing variables. This will be
     *                    set by the controller in init_vars
     * @param   type      self-referencing variables. This will be
     *                    set by the controller in init_vars
     **/
    virtual BasePlatform * create(
      const madara::knowledge::KnowledgeMap & args,
      madara::knowledge::KnowledgeBase * knowledge,
      variables::Sensors * sensors,
      variables::Platforms * platforms,
      variables::Self * self);

    /// the type of the factory/platform
    std::string type_;
  };


  } // ns platforms end
} // ns gams end

#endif
