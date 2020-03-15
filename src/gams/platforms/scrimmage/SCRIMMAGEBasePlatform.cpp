#include "gams/platforms/scrimmage/SCRIMMAGEBasePlatform.h"

gams::platforms::SCRIMMAGEBasePlatform::SCRIMMAGEBasePlatform(
  scrimmage::SimControl& simcontrol,
  madara::knowledge::KnowledgeBase * knowledge,
  variables::Sensors * sensors,
  variables::Self * self
) 
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
gams::platforms::SCRIMMAGEBasePlatform::get_name(void) const
{

   return std::string("get_name");
}

std::string
gams::platforms::SCRIMMAGEBasePlatform::get_id(void) const 
{

  return std::string("get_id");
}

double
gams::platforms::SCRIMMAGEBasePlatform::get_accuracy(void) const 
{

  return 1.0;
}

int
gams::platforms::SCRIMMAGEBasePlatform::move(const pose::Position & target, const pose::PositionBounds &bounds)
{

  return 1;
}

const gams::pose::ReferenceFrame & gams::platforms::SCRIMMAGEBasePlatform::get_frame(void) const
{

  gams::pose::ReferenceFrame r;
  return r;
}

gams::platforms::SCRIMMAGEBasePlatformFactory::SCRIMMAGEBasePlatformFactory(const std::string & type)
{

}

gams::platforms::SCRIMMAGEBasePlatformFactory::~SCRIMMAGEBasePlatformFactory()
{

}

gams::platforms::BasePlatform *
gams::platforms::SCRIMMAGEBasePlatformFactory::create(
      const madara::knowledge::KnowledgeMap & args,
      madara::knowledge::KnowledgeBase * knowledge,
      variables::Sensors * sensors,
      variables::Platforms * platforms,
      variables::Self * self
)
{

}
