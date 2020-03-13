#include "gams/platforms/scrimmage/SCRIMMAGEBasePlatform.h"

gams::platforms::SCRIMMAGEBasePlatform(
  SimControl& simcontrol,
  madara::knowledge::KnowledgeBase * knowledge,
  variables::Sensors * sensors,
  variables::Self * self
) : BasePlatform(knowledge, sensors, self) 
{

}

gams::platforms::SCRIMMAGEBasePlatform::~SCRIMMAGEBasePlatform()
{

}

void
gams::platforms::SCRIMMAGEBasePlatform::operator=(const SCRIMMAGEBasePlatform & rhs)
{
  if (this != &rhs)
  {
  
  }
}

int
gams::platforms::SCRIMMAGEBasePlatform::sense(void)
{

   return 1;
}

int
gams::platforms::SCRIMMAGEBasePlatform::analyze(void)
{

   return 0;
}

std::string
gams::platforms::SCRIMMAGEBasePlatform::get_name(void)
{

   return std::string("get_name");
}

std::string
gams::platforms::SCRIMMAGEBasePlatform::get_id(void)
{

  return std::string("get_id");
}

double
gams::platforms::SCRIMMAGEBasePlatform::get_accuracy(void)
{

  return 1.0;
}

int
gams::platforms::SCRIMMAGEBasePlatform::move(const pose::Position & target, const pose::PositionBounds &bounds)
{

  return 1;
}

int
gams::platforms::SCRIMMAGEBasePlatform::get_frame(void)
{

  return 1;
}
