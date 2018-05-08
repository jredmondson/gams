#define TEST_ROS2GAMS
#include <gams/programs/ros2gams.cpp>
#include <std_msgs/Header.h>


const double TEST_epsilon = 0.0001;

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
    } \
  } while(0)


void test_pose()
{
	std::cout << std::endl << "test geometry_msgs::Pose" << std::endl;

	knowledge::KnowledgeBase knowledge;


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
	parse_pose(&p, &knowledge, container_name);

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

	parse_tf_message(&m, &knowledge);

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

int main (int, char **)
{
	std::cout << "Testing ros2gams" << std::endl;
	test_pose();
	test_tf_tree();
	return 0;
}