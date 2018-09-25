@0xcf7419e96f91ec54;

# Namespace setup
using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("gams::types");

using import "PointXYZ.capnp".PointXYZ;

# TODO use capnproto generics (Templates) to unduplicate this and PointCloudXYZI

# Type definition
struct PointCloudXYZ {
  tov @0 :UInt64;
  frameId @1 :Text;
  width @2 :UInt32;
  height @3 :UInt32;
  isDense @4 :Bool;
  points @5 :List(PointXYZ);

}
