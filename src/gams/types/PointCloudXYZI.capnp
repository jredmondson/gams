@0xa62f4365c6b2a67c;

# Namespace setup
using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("gams::types");

using import "PointXYZI.capnp".PointXYZI;

# TODO use capnproto generics (Templates) to unduplicate this and PointCloudXYZ

# Type definition
struct PointCloudXYZI {
  tov @0 :UInt64;
  frameId @1 :Text;
  width @2 :UInt32;
  height @3 :UInt32;
  isDense @4 :Bool;
  points @5 :List(PointXYZI);

}

