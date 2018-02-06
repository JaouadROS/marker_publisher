
#include "marker_publisher/marker_pub.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "marker_publisher");

  MarkerPosePublisher cam_image_map;

  ros::spin();
  return 0;
}
