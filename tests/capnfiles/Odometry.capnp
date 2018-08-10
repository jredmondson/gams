@0xbebf4ee71f581bf1;
using Header = import "Header.capnp";
using PoseWithCovariance = import "PoseWithCovariance.capnp";
using TwistWithCovariance = import "TwistWithCovariance.capnp";

struct Odometry {
  header @0 :Header.Header;
  childframeid @1: Text;
  pose @2 : PoseWithCovariance.PoseWithCovariance;
  twist @3 : TwistWithCovariance.TwistWithCovariance;
}