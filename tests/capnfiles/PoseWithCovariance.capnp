@0x8394dd54e6fff3bd;

using Pose = import "Pose.capnp";


struct PoseWithCovariance
{
  pose @0: Pose.Pose;
  covariance @1: List(Float32);
}