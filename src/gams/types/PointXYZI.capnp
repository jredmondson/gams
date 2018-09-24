@0xc94eebdb0ddcfeb7;

# Namespace setup
using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("gams::types");

# Type definition
struct PointXYZI {
   y @0 :Float32;
   x @1 :Float32;
   z @2 :Float32;
   intensity @3 :Float32;
   
}
