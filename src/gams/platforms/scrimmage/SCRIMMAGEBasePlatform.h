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
       
       /*
          Reads the state from the agent that this platform is controlling
       */
       virtual int sense(void) override;
       virtual int analyze(void) override;
       virtual std::string get_name() const override;
       
       /*
          Returns the ID of the GAMS agent (SCRIMMAGEBasePlatform) used by controllers/algorithms
       */
       virtual std::string get_id() const override;

       /*
          Platform actions required for Spell, Zone Coverage, Move.
       */
       
       /*
          \@desc Spawns an entity in the SCRIMMAGE simulator via a protobuf msg on the GenerateEntity topic presented by SimControl.cpp. At present, this spawns a copy of the entity at position 0 in the <entity> list provided by the default world. 
       */
       void spawn_entity(void);

       /*
          \@desc Used in determining how close to a waypoint/goal the robot needs to be before moving to the next point
       */
       virtual double get_accuracy() const override;
       
       /*
          \@desc 
       */
       virtual int move(const gams::pose::Position & target, const gams::pose::PositionBounds &bounds) override;
       
       /*
          \@desc 
       */
       virtual int orient(const gams::pose::Orientation & target, const gams::pose::OrientationBounds &bounds) override;
       virtual const gams::pose::ReferenceFrame & get_frame(void) const override;
       
       /*
          \@desc Returns the pointer to the SimControl object. One SimControl instance for all of the SCRIMMAGEBasePlatforms (singleton pattern)
       */
       static scrimmage::SimControl * get_simcontrol_instance()
       { 
          return simcontrol;
       }
       
       /*
          \@desc Returns whether simcontrol was started with the run_threaded() option 
       */
       static bool simcontrol_threaded()
       {
          return running_threaded;
       }

       /*
          \@desc Stored access of this agents ID, which is different from the ID used to access the SCRIMMAGE agent associated with this specific instance. SCRIMMAGE starts its non-spawned entity (the one specified in the world XML file) at 0, and all other spawned copies then start at 1 moving forward. To access the SCRIMMAGE agent, it will be self_id + 1 indexing, or, just use scrimmage_access_id below
       */
       madara::knowledge::KnowledgeRecord self_id;
       
       /*
          \@desc The ID to access a SCRIMMAGE agent 
       */
       int scrimmage_access_id;
       
       madara::threads::Threader threader_;  
       
       /*
          \@desc Static pointer to the SimControl object needed to interface with agents in the SCRIMMAGE simulator
       */
       static scrimmage::SimControl * simcontrol;
       
       /*
          \@desc Specifies whether or not SimControl is running threaded
       */
       static bool running_threaded;
       
       /*
          \@desc XML file the SimControl object loads from. Loaded into the KB at "worldfile"
       */
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
