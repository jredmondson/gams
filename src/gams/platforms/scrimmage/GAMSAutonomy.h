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


}; // end class GAMSAutonomy
}  // ns scrimmage
}  // ns autonomy
