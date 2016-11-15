# ros_kinfu_solo
a minimal package for running kinfu-large-scale on data from ros topics

installation:

1, git clone pcl with gpu module(1.8 for example, 1.7 doesn't have it)

2, set PCL_GPU_DIR to the path to pcl/gpu. default path is $HOME/workspace/pcl/gpu

3, catkin_make install, possibly with -D PCL_GPU_DIR=...
