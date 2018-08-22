

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


using namespace madara::knowledge;

namespace gams
{
  
  namespace types 
  {

    void register_all_datatypes()
    {
        Any::register_type<CapnObject<DisparityImage> >("DisparityImage");
      Any::register_type<CapnObject<GridCells> >("GridCells");
      Any::register_type<CapnObject<OccupancyGrid> >("OccupancyGrid");
      Any::register_type<CapnObject<Odometry> >("Odometry");
      Any::register_type<CapnObject<Path> >("Path");
      Any::register_type<CapnObject<PidState> >("PidState");
      Any::register_type<CapnObject<VoxelGrid> >("VoxelGrid");
      Any::register_type<CapnObject<Duration> >("Duration");
      Any::register_type<CapnObject<Header> >("Header");
      Any::register_type<CapnObject<Time> >("Time");
      Any::register_type<CapnObject<BatteryState> >("BatteryState");
      Any::register_type<CapnObject<CameraInfo> >("CameraInfo");
      Any::register_type<CapnObject<CompressedImage> >("CompressedImage");
      Any::register_type<CapnObject<FluidPressure> >("FluidPressure");
      Any::register_type<CapnObject<Illuminance> >("Illuminance");
      Any::register_type<CapnObject<Image> >("Image");
      Any::register_type<CapnObject<Imu> >("Imu");
      Any::register_type<CapnObject<JointState> >("JointState");
      Any::register_type<CapnObject<Joy> >("Joy");
      Any::register_type<CapnObject<LaserScan> >("LaserScan");
      Any::register_type<CapnObject<MagneticField> >("MagneticField");
      Any::register_type<CapnObject<PointCloud> >("PointCloud");
      Any::register_type<CapnObject<PointCloud2> >("PointCloud2");
      Any::register_type<CapnObject<PointField> >("PointField");
      Any::register_type<CapnObject<Temperature> >("Temperature");
      Any::register_type<CapnObject<TimeReference> >("TimeReference");
      Any::register_type<CapnObject<RegionOfInterest> >("RegionOfInterest");
      Any::register_type<CapnObject<Point> >("Point");
      Any::register_type<CapnObject<MapMetaData> >("MapMetaData");
      Any::register_type<CapnObject<Pose> >("Pose");
      Any::register_type<CapnObject<Quaternion> >("Quaternion");
      Any::register_type<CapnObject<PoseWithCovariance> >("PoseWithCovariance");
      Any::register_type<CapnObject<TwistWithCovariance> >("TwistWithCovariance");
      Any::register_type<CapnObject<Twist> >("Twist");
      Any::register_type<CapnObject<Vector3> >("Vector3");
      Any::register_type<CapnObject<PoseStamped> >("PoseStamped");
      Any::register_type<CapnObject<Point32> >("Point32");
      Any::register_type<CapnObject<ChannelFloat32> >("ChannelFloat32");
           
    }

  }
}

#endif
