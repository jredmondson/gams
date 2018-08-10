@0xc0b2c1572d185015;

using Point = import "Point.capnp";
using Quaternion = import "Quaternion.capnp";

struct Pose
{
  position @0: Point.Point;
  orientation @1: Quaternion.Quaternion;
}