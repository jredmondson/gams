ROS2GAMS is a tool to convert rosbag files to checkpoints.

Build ROS2GAMS:
You need an installed version of ROS (tested with ROS Kinetic).
After calling base_build.sh you can compile ros2gams by calling:

make -f GNUmakefile.ros2madara ros=1



Commandline-Parameters:

  [-r|--rosbag]                        Path to the rosbag file
  [-rp|--ros-robot-prefix]             Topic prefix of each robot if multiple robots are used in the bagfile (default: '/robot_')
  [-scp|--save-checkpoint-prefix prfx] prefix of knowledge to save in checkpoint
  [-sb|--save-binary]                  save the resulting knowledge base as a binary checkpoint

Example:

./ros2gams -r test.bag -scp mycheckpoint -sb

will read the file test.bag and create several checkpoints named mycheckpoint_1.kb, mycheckpoint_2.kb, etc. in the binary format.
Without the option -sb all files are stored in karl format.