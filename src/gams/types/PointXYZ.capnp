@0xb8c9b16fb51cc7cc;

# Namespace setup
using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("gams::types");

# Capnfile Imports

# Type definition
struct PointXYZ {
   y @0: Float32;
   x @1: Float32;
   z @2: Float32;

}
