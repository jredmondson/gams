@0x990dc56caaedcdfc;
using Header = import "Header.capnp";
using Pose = import "Pose.capnp";

struct PoseStamped {
  header @0 :Header.Header;
  pose @1: Pose.Pose;
}