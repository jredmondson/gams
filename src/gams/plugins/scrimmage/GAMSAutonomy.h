#ifndef INCLUDE_SCRIMMAGE_PLUGINS_GAMS_AUTONOMY_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_GAMS_AUTONOMY_H_

#include <scrimmage/autonomy/Autonomy.h>
#include <string>
#include <map>

namespace scrimmage {
namespace autonomy {

class GAMSAutonomy : public scrimmage::Autonomy {

  public:
    GAMSAutonomy();
    void init(std::map<std::string, std::string> &params) override;
    bool step_autonomy(double t, double dt) override;
    
  protected:
    int desired_alt_idx_ = 0;
    int desired_speed_idx_ = 0;
    int desired_heading_idx_ = 0;

}; // end class GAMSAutonomy
}  // ns scrimmage
}  // ns autonomy

#endif
