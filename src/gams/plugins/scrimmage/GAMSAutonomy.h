#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_GAMSAUTONOMY_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_GAMSAUTONOMY_H_

#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/math/State.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/proto/State.pb.h>
#include <scrimmage/msgs/Event.pb.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <string>
#include <map>

namespace scrimmage {
namespace autonomy {

class GAMSAutonomy : public scrimmage::Autonomy {

  public:
    GAMSAutonomy();
    void init(std::map<std::string, std::string> &params) override;
    bool step_autonomy(double t, double dt) override;
    /*
       GAMS Algorithms move() will call this to set the state, 
       the autonomy function step() will read it and propogate
       through SCRIMMAGE
    */
    
  protected:
    uint8_t position_x_idx_ = 0;
    uint8_t position_y_idx_ = 0;
    uint8_t position_z_idx_ = 0;
    
    scrimmage::PublisherPtr pub_gen_ents_;

}; // end class GAMSAutonomy
}  // ns scrimmage
}  // ns autonomy

#endif
