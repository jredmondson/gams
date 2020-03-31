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
#include "madara/knowledge/containers/NativeDoubleVector.h"

#include "gams/loggers/GlobalLogger.h"

#include <scrimmage/simcontrol/SimControl.h>

namespace gams
{
  namespace platforms 
  {
     class SCRIMMAGEBasePlatform : public BasePlatform
     {
     public:
       SCRIMMAGEBasePlatform(
        madara::knowledge::KnowledgeBase * kb_,
        gams::variables::Sensors * sensors_,
        gams::variables::Self * self_
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
       
       void spawn_entity(void);

       /*
          Platform actions required for Spell, Zone Coverage, Move.
       */
       virtual double get_accuracy() const override;
       virtual int move(const gams::pose::Position & target, const gams::pose::PositionBounds &bounds) override;
       virtual int orient(const gams::pose::Orientation & target, const gams::pose::OrientationBounds &bounds) override;
       virtual const gams::pose::ReferenceFrame & get_frame(void) const override;
       
       /*
          Scrimmage specific functions
       */
       
       static scrimmage::SimControl * get_simcontrol_instance()
       { 
          return simcontrol;
       }
       
       static bool simcontrol_threaded()
       {
          return running_threaded;
       }

       scrimmage::EntityPtr get_entity();
       scrimmage::EntityPtr this_ent_;
       
       
       madara::knowledge::KnowledgeRecord self_id;
       int scrimmage_access_id;
       
       std::string tag;
       
       madara::threads::Threader threader_;  
       
       
       //scrimmage::SimControl * simcontrol;
       static int num_agents;
       static scrimmage::SimControl * simcontrol;
       static bool running_threaded;
       static std::string world_file;
       
     };
     
     
  /**
   * A factory class for creating SCRIMMAGEBasePlatform platforms
   **/
  class GAMS_EXPORT SCRIMMAGEBasePlatformFactory : public PlatformFactory
  {
  public:
  
    SCRIMMAGEBasePlatformFactory();
  
    virtual ~SCRIMMAGEBasePlatformFactory();

    /**
     * Creates a SCRIMMAGEBasePlatform platform.
     * @param   args      no arguments are necessary for this platform
     * @param   knowledge the knowledge base. This will be set by the
     *                    controller in init_vars.s
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
