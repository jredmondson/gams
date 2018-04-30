ROS2GAMS is a tool to convert rosbag files to checkpoints.

Build ROS2GAMS:
You need an installed version of ROS (tested with ROS Kinetic).
After calling base_build.sh you can compile ros2gams by calling:

make -f GNUmakefile.ros2madara ros=1



Commandline-Parameters:

  [-r|--rosbag]                   Path to the rosbag file
  [-rp|--ros-robot-prefix]        Topic prefix of each robot if multiple robots are used in the bagfile (default: '/robot_')
  [-c|--checkpoint-prefix]        Prefix for the madara checkpoint files (default: 'checkpoint_')



Example:

./ros2gams -r test.bag -c mycheckpoint

will read the file test.bag and create several checkpoints named mycheckpoint_1.kb, mycheckpoint_2.kb, etc.