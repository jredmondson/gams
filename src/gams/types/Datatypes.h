

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

namespace gams
{
  
  namespace types 
  {

    void register_all_datatypes()
    {
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<DisparityImage> >("DisparityImage");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<GridCells> >("GridCells");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<OccupancyGrid> >("OccupancyGrid");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<Odometry> >("Odometry");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<Path> >("Path");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<PidState> >("PidState");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<VoxelGrid> >("VoxelGrid");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<Duration> >("Duration");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<Header> >("Header");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<Time> >("Time");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<BatteryState> >("BatteryState");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<CameraInfo> >("CameraInfo");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<CompressedImage> >("CompressedImage");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<FluidPressure> >("FluidPressure");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<Illuminance> >("Illuminance");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<Image> >("Image");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<Imu> >("Imu");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<JointState> >("JointState");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<Joy> >("Joy");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<LaserScan> >("LaserScan");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<MagneticField> >("MagneticField");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<PointCloud> >("PointCloud");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<PointCloud2> >("PointCloud2");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<PointField> >("PointField");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<Temperature> >("Temperature");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<TimeReference> >("TimeReference");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<RegionOfInterest> >("RegionOfInterest");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<Point> >("Point");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<MapMetaData> >("MapMetaData");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<Pose> >("Pose");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<Quaternion> >("Quaternion");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<PoseWithCovariance> >("PoseWithCovariance");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<TwistWithCovariance> >("TwistWithCovariance");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<Twist> >("Twist");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<Vector3> >("Vector3");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<PoseStamped> >("PoseStamped");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<Point32> >("Point32");
      madara::knowledge::Any::register_type<madara::knowledge::CapnObject<ChannelFloat32> >("ChannelFloat32"); 
    }

  }
}

#endif
