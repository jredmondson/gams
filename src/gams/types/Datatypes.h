

#ifndef __GAMS_TYPES_H__
#define __GAMS_TYPES_H__

#include "madara/knowledge/Any.h"

#include "DisparityImage.capnp.h"
#include "GridCells.capnp.h"
#include "OccupancyGrid.capnp.h"
#include "Odometry.capnp.h"
#include "Path.capnp.h"
#include "PidState.capnp.h"
#include "VoxelGrid.capnp.h"
#include "Duration.capnp.h"
#include "Header.capnp.h"
#include "Time.capnp.h"
#include "BatteryState.capnp.h"
#include "CameraInfo.capnp.h"
#include "CompressedImage.capnp.h"
#include "FluidPressure.capnp.h"
#include "Illuminance.capnp.h"
#include "Image.capnp.h"
#include "Imu.capnp.h"
#include "JointState.capnp.h"
#include "Joy.capnp.h"
#include "LaserScan.capnp.h"
#include "MagneticField.capnp.h"
#include "PointCloud.capnp.h"
#include "PointCloud2.capnp.h"
#include "PointField.capnp.h"
#include "Temperature.capnp.h"
#include "TimeReference.capnp.h"
#include "RegionOfInterest.capnp.h"
#include "Point.capnp.h"
#include "MapMetaData.capnp.h"
#include "Pose.capnp.h"
#include "Quaternion.capnp.h"
#include "PoseWithCovariance.capnp.h"
#include "TwistWithCovariance.capnp.h"
#include "Twist.capnp.h"
#include "Vector3.capnp.h"
#include "PoseStamped.capnp.h"
#include "Point32.capnp.h"
#include "ChannelFloat32.capnp.h"

namespace mk = madara::knowledge;

namespace gams
{
  
  namespace types 
  {

    void register_all_datatypes()
    {
      mk::Any::register_type<mk::CapnObject<DisparityImage> >("DisparityImage");
      mk::Any::register_type<mk::CapnObject<GridCells> >("GridCells");
      mk::Any::register_type<mk::CapnObject<OccupancyGrid> >("OccupancyGrid");
      mk::Any::register_type<mk::CapnObject<Odometry> >("Odometry");
      mk::Any::register_type<mk::CapnObject<Path> >("Path");
      mk::Any::register_type<mk::CapnObject<PidState> >("PidState");
      mk::Any::register_type<mk::CapnObject<VoxelGrid> >("VoxelGrid");
      mk::Any::register_type<mk::CapnObject<Duration> >("Duration");
      mk::Any::register_type<mk::CapnObject<Header> >("Header");
      mk::Any::register_type<mk::CapnObject<Time> >("Time");
      mk::Any::register_type<mk::CapnObject<BatteryState> >("BatteryState");
      mk::Any::register_type<mk::CapnObject<CameraInfo> >("CameraInfo");
      mk::Any::register_type<mk::CapnObject<CompressedImage> >("CompressedImage");
      mk::Any::register_type<mk::CapnObject<FluidPressure> >("FluidPressure");
      mk::Any::register_type<mk::CapnObject<Illuminance> >("Illuminance");
      mk::Any::register_type<mk::CapnObject<Image> >("Image");
      mk::Any::register_type<mk::CapnObject<Imu> >("Imu");
      mk::Any::register_type<mk::CapnObject<JointState> >("JointState");
      mk::Any::register_type<mk::CapnObject<Joy> >("Joy");
      mk::Any::register_type<mk::CapnObject<LaserScan> >("LaserScan");
      mk::Any::register_type<mk::CapnObject<MagneticField> >("MagneticField");
      mk::Any::register_type<mk::CapnObject<PointCloud> >("PointCloud");
      mk::Any::register_type<mk::CapnObject<PointCloud2> >("PointCloud2");
      mk::Any::register_type<mk::CapnObject<PointField> >("PointField");
      mk::Any::register_type<mk::CapnObject<Temperature> >("Temperature");
      mk::Any::register_type<mk::CapnObject<TimeReference> >("TimeReference");
      mk::Any::register_type<mk::CapnObject<RegionOfInterest> >("RegionOfInterest");
      mk::Any::register_type<mk::CapnObject<Point> >("Point");
      mk::Any::register_type<mk::CapnObject<MapMetaData> >("MapMetaData");
      mk::Any::register_type<mk::CapnObject<Pose> >("Pose");
      mk::Any::register_type<mk::CapnObject<Quaternion> >("Quaternion");
      mk::Any::register_type<mk::CapnObject<PoseWithCovariance> >("PoseWithCovariance");
      mk::Any::register_type<mk::CapnObject<TwistWithCovariance> >("TwistWithCovariance");
      mk::Any::register_type<mk::CapnObject<Twist> >("Twist");
      mk::Any::register_type<mk::CapnObject<Vector3> >("Vector3");
      mk::Any::register_type<mk::CapnObject<PoseStamped> >("PoseStamped");
      mk::Any::register_type<mk::CapnObject<Point32> >("Point32");
      mk::Any::register_type<mk::CapnObject<ChannelFloat32> >("ChannelFloat32");
           
    }

  }
}

#endif
