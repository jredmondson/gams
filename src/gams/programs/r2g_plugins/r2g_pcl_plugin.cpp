#include <iostream>
#include "madara/knowledge/KnowledgeBase.h"
#include "madara/knowledge/Any.h"
#include "gams/types/PointCloudXYZ.capnp.h"


#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wreorder"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#endif

#include "ros/ros.h"
#include "ros/time.h"
#include "ros/serialization.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "rosbag/message_instance.h"
#include <ros_type_introspection/ros_introspection.hpp>
#include <topic_tools/shape_shifter.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>

#include "sensor_msgs/PointCloud2.h"

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

extern "C" void parse(const rosbag::MessageInstance* m,
  madara::knowledge::KnowledgeBase * kb, 
  std::string container_name) {
  // THIS METHOD IS HARDCODED TO RUN WITH PCL BASED SCHEMAS

  sensor_msgs::PointCloud2 *pointcloud =
    m->instantiate<sensor_msgs::PointCloud2> ().get ();

  // Load PCL schema
  std::string pointcloud_schema_name = "PointCloudXYZ";
  capnp::MallocMessageBuilder buffer;
  gams::types::PointCloudXYZ::Builder pointcloud_builder = 
    buffer.initRoot<gams::types::PointCloudXYZ> ();

  // Convert from ROS PointCloud2 to a PCL PointCloud
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL (*pointcloud, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2 (pcl_pc2, *temp_cloud);

  //Fill the list of points with data
  auto point_list = pointcloud_builder.initPoints((*temp_cloud).size ());
  int point_counter = 0;
  for (pcl::PointXYZ point : *temp_cloud)
  {
    point_list[point_counter].setX(point.x);
    point_list[point_counter].setY(point.y);
    point_list[point_counter].setZ(point.z);
    point_counter++;
  }

  // Now fill the other fields
  pointcloud_builder.setTov (
    ((unsigned long)pointcloud->header.stamp.sec * 1e9) +
    (unsigned long)pointcloud->header.stamp.nsec);
  pointcloud_builder.setFrameId (pointcloud->header.frame_id.c_str());
  pointcloud_builder.setWidth (pointcloud->width);
  pointcloud_builder.setHeight (pointcloud->height);
  pointcloud_builder.setIsDense((bool)pointcloud->is_dense);

  // Store in the knowledgebase
  madara::knowledge::GenericCapnObject any(pointcloud_schema_name.c_str(),
    buffer);
  kb->set_any(container_name, any);

  std::cout << "WE DID IT!!" << std::endl;
}
