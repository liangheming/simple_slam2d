#include "localizer_ros.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localizer");
    ros::NodeHandle private_nh("~");
    LocalizerROS localizer(private_nh);
    ros::spin();
    return 0;
}