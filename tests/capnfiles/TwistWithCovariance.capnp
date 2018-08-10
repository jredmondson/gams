@0x83cf22b7849690aa;

using Twist = import "Twist.capnp";


struct TwistWithCovariance {
  twist @0: Twist.Twist;
  covariance @1: List(Float64);
}