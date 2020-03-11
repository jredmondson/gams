
// SCRIMMAGE INCLUDES
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>

// GAMS INCLUDES
#include <gams/platforms/scrimmage/GAMSAutonomy.h>

// CAR CONTROLLER PLUGIN INCLUDES
#include <scrimmage/common/VariableIO.h>

REGISTER_PLUGIN(scrimmage::Autonomy, scrimmage::autonomy::GAMSAutonomy, GAMSAutonomy_plugin)

namespace scrimmage {
namespace autonomy {

GAMSAutonomy::GAMSAutonomy() {
}

/*
   @param params Stores xml params used to initialize plugin, found in gams/platforms/scrimmage/*.xml

*/
void GAMSAutonomy::init(std::map<std::string, std::string> &params) {
    desired_heading_idx_ = vars_.declare("desired_heading", scrimmage::VariableIO::Direction::Out);
    desired_speed_idx_ = vars_.declare("desired_speed", scrimmage::VariableIO::Direction::Out);


}

/*
   @param t  time
   @param dt time interval
   @effect Changes the entities current state and desired state
*/
bool GAMSAutonomy::step_autonomy(double t, double dt) {
   vars_.output(desired_heading_idx_, value_to_output);
   vars_.output(desired_speed_idx_, value_to_output);


   return true;
}

} // ns scrimmage
} // ns autonomy
