ROS2GAMS is a tool to convert rosbag files to checkpoints.

Build ROS2GAMS:
You need an installed version of ROS (tested with ROS Kinetic).
After calling base_build.sh you can compile ros2gams by calling:

make -f GNUmakefile.ros2madara ros=1



Commandline-Parameters:

  [-r|--rosbag]                        Path to the rosbag file
  [-rp|--ros-robot-prefix prfx]        Topic prefix of each robot if multiple robots are used in the bagfile (default: '/robot_')
  [-scp|--save-checkpoint-prefix prfx] prefix of knowledge to save in checkpoint
  [-sb|--save-binary]                  save the resulting knowledge base as a binary checkpoint
  [-m|--map-file file]                 File with filter and mapping information
Example:

./ros2gams -r test.bag -scp mycheckpoint -sb

will read the file test.bag and create several checkpoints named mycheckpoint_1.kb, mycheckpoint_2.kb, etc. in the binary format.
Without the option -sb all files are stored in karl format.



Mapfile format:

The file consits of a list of mapping entries. Each line is a new entry.
Example:

imu/data_raw agent.0.raw_imu
imu/data agent.0.filtered_imu

will map the topic imu/data_raw to the madara variable agent.0.raw_imu and the topic imu/data to the variable agent.0.filtered_imu

All other topics in the bagfile will be ignored if there is a mapfile given.
If no mapfile is given all the parseable topics are converted.
Please keep in mind that the transform tree is the only topic which starts with a leading slash.
So to convert the tree you need to add for example:

/tf