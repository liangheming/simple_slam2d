#include "map_generator_ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_generator");
    ros::NodeHandle nh_private("~");
    MapGeneratorROS map_generator_ros(nh_private);
    ros::spin();
    return 0;
}