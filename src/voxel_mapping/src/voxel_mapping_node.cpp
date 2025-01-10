#include "voxel_mapping_ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "voxel_mapping");
    ros::NodeHandle private_nh("~");
    VoxelMappingROS voxel_mapping_ros(private_nh);
    ros::spin();
    return 0;
}