# Generated by ./generate_schemas.py. This file should not be modified by hand.
@0xc47cfd11ad22a50f;

# Namespace setup
using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("gams::types");

# Capnfile Imports

# Type definition
struct ChannelFloat32 {
   values @0: List(Float32);
   name @1: Text;
   
}
