# Generated by ./generate_schemas.py. This file should not be modified by hand.
@0xdd215ff0cd219f8c;

# Namespace setup
using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("gams::types");

# Capnfile Imports
using import "Header.capnp".Header;
using import "RegionOfInterest.capnp".RegionOfInterest;

# Type definition
struct CameraInfo {
   roi @0: RegionOfInterest;
   d @1: List(Float64);
   p @2: List(Float64);
   width @3: UInt32;
   k @4: List(Float64);
   height @5: UInt32;
   header @6: Header;
   r @7: List(Float64);
   binningY @8: UInt32;
   binningX @9: UInt32;
   distortionModel @10: Text;
   
}
