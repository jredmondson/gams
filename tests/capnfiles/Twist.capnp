@0xceb106692fc07276;

using Vec3 = import "Vector3.capnp";


struct Twist {
  linear @0: Vec3.Vector3;
  angular @1: Vec3.Vector3;
}
