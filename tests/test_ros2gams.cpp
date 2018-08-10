#define TEST_ROS2GAMS

#include <string>
#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "madara/knowledge/Any.h"
#include "madara/knowledge/KnowledgeRecord.h"
#include "madara/knowledge/KnowledgeBase.h"
#include "madara/utility/SupportTest.h"
#include "madara/utility/Utility.h"

#include "kj/io.h"
#include "capnp/serialize.h"
#include "capnp/schema-loader.h"
#include "capnfiles/Odometry.capnp.h"
#include "capnfiles/Point.capnp.h"


#include "gams/utility/ros/RosParser.cpp"
#include "std_msgs/Header.h"
#include "nav_msgs/Odometry.h"


const double TEST_epsilon = 0.0001;
int gams_fails = 0;

double round_nearest(double in)
{
  return floor(in + 0.5);
}

#define LOG(expr) \
  std::cout << #expr << " == " << (expr) << std::endl

#define TEST(expr, expect) \
  do {\
    double bv = (expr); \
    double v = round_nearest((bv) * 1024)/1024; \
    double e = round_nearest((expect) * 1024)/1024; \
    bool ok = \
      e >= 0 ? (v >= e * (1 - TEST_epsilon) && v <= e * (1 + TEST_epsilon)) \
             : (v >= e * (1 + TEST_epsilon) && v <= e * (1 - TEST_epsilon)); \
    if(ok) \
    { \
      std::cout << #expr << " ?= " << e << "  SUCCESS! got " << bv << std::endl; \
    } \
    else \
    { \
      std::cout << #expr << " ?= " << e << "  FAIL! got " << bv << " instead" << std::endl; \
			++gams_fails; \
    } \
  } while(0)


std::map<std::string, std::string> capnp_types;


void test_pose()
{
	std::cout << std::endl << "test geometry_msgs::Pose" << std::endl;

	knowledge::KnowledgeBase knowledge;
	gams::utility::ros::RosParser parser (&knowledge, "", "", capnp_types);

	geometry_msgs::Pose p;
	p.position.x = 1.0;
	p.position.y = 2.0;
	p.position.z = 3.0;
	// 90 degree roation around x axis
	p.orientation.x = 0.707;
	p.orientation.y = 0.0;
	p.orientation.z = 0.0;
	p.orientation.w = 0.707;

	std::string container_name = "pose_test";
	parser.parse_pose(&p, container_name);

	containers::NativeDoubleVector cont(container_name, knowledge, 6);
	gams::pose::Pose pose;
	pose.from_container(cont);
	TEST(pose.get(0), p.position.x);
	TEST(pose.get(1), p.position.y);
	TEST(pose.get(2), p.position.z);

	TEST(pose.get(3), M_PI/2);
	TEST(pose.get(4), 0.0);
	TEST(pose.get(5), 0.0);

}

void test_tf_tree()
{
	std::cout << std::endl << "test tf2_msgs::TFMessage" << std::endl;

	knowledge::KnowledgeBase knowledge;
	gams::utility::ros::RosParser parser (&knowledge, "world", "frame1",
		capnp_types);

	geometry_msgs::Transform transform;
	geometry_msgs::Vector3 t;
	t.x = 1.0;
	t.y = 2.0;
	t.z = 3.0;
	transform.translation = t;
	geometry_msgs::Quaternion q;
	q.x = 0.707;
	q.y = 0.0;
	q.z = 0.0;
	q.w = 0.707;
	transform.rotation = q;
	geometry_msgs::TransformStamped st;
	st.transform = transform;
	st.child_frame_id = "frame1";

	std_msgs::Header h;
	h.seq = 0;
	h.stamp = ros::Time(0);
	h.frame_id = "world";
	st.header = h;

	tf2_msgs::TFMessage m;
	m.transforms.push_back(st);

	parser.parse_tf_message(&m);

	gams::pose::ReferenceFrame ref_frame = gams::pose::ReferenceFrame::load(knowledge, st.child_frame_id);
	TEST(ref_frame.valid(), true);
	gams::pose::Pose origin = ref_frame.origin();

	TEST(origin.get(0), t.x);
	TEST(origin.get(1), t.y);
	TEST(origin.get(2), t.z);

	TEST(origin.get(3), M_PI/2);
	TEST(origin.get(4), 0.0);
	TEST(origin.get(5), 0.0);
}

template <typename T>
void serialize_message_to_array (
	const T& msg, std::vector<uint8_t>& destination_buffer)
{
  const uint32_t length = ros::serialization::serializationLength(msg);
  destination_buffer.resize( length );
  //copy into your own buffer
  ros::serialization::OStream stream(destination_buffer.data(), length);
  ros::serialization::serialize(stream, msg);
}

void test_any_point()
{
	// This tests configures the parser so that Odometry messages are stored
	// into capnproto any types.

	std::cout << std::endl << std::endl << "test_any_point" << std::endl << std::endl;

	std::string container_name = "thisisatest";

	geometry_msgs::Point point;

	double x = 1.0;
	double y = 2.0;
	double z = 3.0;
	

	//set the position
  point.x = x;
  point.y = y;
  point.z = z;

	// transform to shapeshifter object
	topic_tools::ShapeShifter shape_shifter;
  shape_shifter.morph(
      ros::message_traits::MD5Sum<geometry_msgs::Point>::value(),
      ros::message_traits::DataType<geometry_msgs::Point>::value(),
      ros::message_traits::Definition<geometry_msgs::Point>::value(),
      "" );

  std::vector<uint8_t> buffer;
	serialize_message_to_array(point, buffer);
  ros::serialization::OStream stream( buffer.data(), buffer.size() );
  shape_shifter.read( stream );


	// The parser
	std::map<std::string, std::string> typemap;
	typemap["geometry_msgs/Point"] = "tests/capnfiles/Point.capnp:Point";

	knowledge::KnowledgeBase knowledge;
	gams::utility::ros::RosParser parser (&knowledge, "", "", typemap);

  // register the type using the topic_name as identifier. This allows us to
	// use RosIntrospection
  const std::string  topic_name =  "point";
  const std::string  datatype   =  shape_shifter.getDataType();
  const std::string  definition =  shape_shifter.getMessageDefinition();
  parser.registerMessageDefinition (topic_name,
      RosIntrospection::ROSType (datatype), definition);

	// Load the schemas from disk
	auto point_schema_path = madara::utility::expand_envs(
		"$(GAMS_ROOT)/tests/capnfiles/Point.capnp.bin");
	parser.load_capn_schema(point_schema_path);

	// parse to capnp format
	parser.parse_any(topic_name, shape_shifter, container_name);

	knowledge.print();

	madara::knowledge::GenericCapnObject any = knowledge.get(container_name).to_any<madara::knowledge::GenericCapnObject>();

	int fd = open(point_schema_path.c_str(), 0, O_RDONLY);
  capnp::StreamFdMessageReader schema_message_reader(fd);
  auto schema_reader = schema_message_reader.getRoot<capnp::schema::CodeGeneratorRequest>();
  capnp::SchemaLoader loader;
  auto schema = loader.load(schema_reader.getNodes()[0]);
	auto capn_point = any.reader(schema.asStruct());

	TEST(capn_point.get("x").as<float>(), x);
	TEST(capn_point.get("y").as<float>(), y);
	TEST(capn_point.get("z").as<float>(), z);
}

void test_any_odom()
{
	// This tests configures the parser so that Odometry messages are stored
	// into capnproto any types.

	std::cout << std::endl << std::endl << "test_any_odom" << std::endl << std::endl;

	std::string container_name = "thisisatest";

	nav_msgs::Odometry odom;
	//odom.header.stamp = ros::Time::now();
	odom.header.frame_id = "odom";

	double x = 1.0;
	double y = 2.0;
	double z = 3.0;
	double vx = 0.1;
	double vy = 0.2;
	double vz = 0.3;
	double cov = 0.123;
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(10.0);


	//set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = z;
  odom.pose.pose.orientation = odom_quat;
	odom.pose.covariance[17] = cov;

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vz;

	// transform to shapeshifter object
	topic_tools::ShapeShifter shape_shifter;
  shape_shifter.morph(
      ros::message_traits::MD5Sum<nav_msgs::Odometry>::value(),
      ros::message_traits::DataType<nav_msgs::Odometry>::value(),
      ros::message_traits::Definition<nav_msgs::Odometry>::value(),
      "" );

  std::vector<uint8_t> buffer;
	serialize_message_to_array(odom, buffer);
  ros::serialization::OStream stream( buffer.data(), buffer.size() );
  shape_shifter.read( stream );


	// The parser
	std::map<std::string, std::string> typemap;
	typemap["nav_msgs/Odometry"] = "tests/capnfiles/Odometry.capnp:Odometry";

	knowledge::KnowledgeBase knowledge;
	gams::utility::ros::RosParser parser (&knowledge, "", "", typemap);

  // register the type using the topic_name as identifier. This allows us to
	// use RosIntrospection
  const std::string  topic_name =  "odom";
  const std::string  datatype   =  shape_shifter.getDataType();
  const std::string  definition =  shape_shifter.getMessageDefinition();
  parser.registerMessageDefinition (topic_name,
      RosIntrospection::ROSType (datatype), definition);

	// Load the schemas from disk
	auto path = madara::utility::expand_envs(
		"$(GAMS_ROOT)/tests/capnfiles/Odometry.capnp.bin");
	parser.load_capn_schema(path);

	// parse to capnp format
	parser.parse_any(topic_name, shape_shifter, container_name);

	knowledge.print();

	madara::knowledge::GenericCapnObject any = knowledge.get(container_name).to_any<madara::knowledge::GenericCapnObject>();

	int fd = open(path.c_str(), 0, O_RDONLY);
  capnp::StreamFdMessageReader schema_message_reader(fd);
  auto schema_reader = schema_message_reader.getRoot<capnp::schema::CodeGeneratorRequest>();
  capnp::SchemaLoader loader;
	
	std::map<std::string, capnp::Schema> schemas;

  for (auto schema : schema_reader.getNodes()) {
    schemas[schema.getDisplayName()] = loader.load(schema);
  }
	auto capn_odom = any.reader(schemas[typemap["nav_msgs/Odometry"]].asStruct());

	std::string cfd = capn_odom.get("childframeid").as<capnp::Text>().cStr();
	if (cfd != odom.child_frame_id)
	{
		std::cout <<  "childframeid ?= " << odom.child_frame_id << "  FAIL! got " << cfd << " instead" << std::endl;
			++gams_fails;
	}


	auto pose_with_cov = capn_odom.get("pose").as<capnp::DynamicStruct>();
	auto pose = pose_with_cov.get("pose").as<capnp::DynamicStruct>();
	auto position = pose.get("position").as<capnp::DynamicStruct>();	
	auto orientation = pose.get("orientation").as<capnp::DynamicStruct>();	

	TEST(position.get("x").as<double>(), x);
	TEST(position.get("y").as<double>(), y);
	TEST(position.get("z").as<double>(), z);

	TEST(orientation.get("x").as<double>(), odom_quat.x);
	TEST(orientation.get("y").as<double>(), odom_quat.y);
	TEST(orientation.get("z").as<double>(), odom_quat.z);
	TEST(orientation.get("w").as<double>(), odom_quat.w);

	auto pose_covariance = pose_with_cov.get("covariance").as<capnp::DynamicList>();
	TEST(pose_covariance[0].as<double>(), 0);
	TEST(pose_covariance[17].as<double>(), cov);
}

int main (int, char **)
{
	std::cout << "Testing ros2gams" << std::endl;
	//test_pose();
	//test_tf_tree();		
	test_any_point();
	test_any_odom();

  if (gams_fails > 0)
  {
    std::cerr << "OVERALL: FAIL. " << gams_fails << " tests failed.\n";
  }
  else
  {
    std::cerr << "OVERALL: SUCCESS.\n";
  }

  return gams_fails;
}


