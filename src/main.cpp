#include "grvc_ef_tracker/trackerWrapper.h"

int main(int argc, char* argv[]){

  ros::init(argc, argv, "grvc_ef_tracker");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  
  ETWrapper efT (nh, nh_private);
  ros::spin();

  return 0;
}
