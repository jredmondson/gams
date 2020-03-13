#ifndef _GAMS_PLATFORM_SCRIMMAGE_BASEPLATFORM_H_
#define _GAMS_PLATFORM_SCRIMMAGE_BASEPLATFORM_H_

#include "gams/variables/Self.h"
#include "gams/variables/Sensor.h"
#include "gams/variables/PlatformStatus.h"
#include "gams/platforms/BasePlatform.h"
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
        SimControl& simcontrol,
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
       virtual const pose::ReferenceFrame & get_frame(void) const override;
       
       private:
       
         SimControl simcontrol;
     };


  } // ns platforms end
} // ns gams end

#endif
