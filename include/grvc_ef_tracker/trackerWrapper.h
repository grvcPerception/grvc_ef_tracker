#include "grvc_ef_tracker/tracker.h"
#include <dvs_msgs/EventArray.h>
#include <dvs_msgs/Event.h>

class ETWrapper{
    public:

    ETWrapper(ros::NodeHandle & nh, ros::NodeHandle nh_private);

    private:
    ros::Publisher tracker_pub_;
    ros::NodeHandle nh_;
    ros::Subscriber ef_sub_;
    
    void eventFeatureCallback(const dvs_msgs::EventArray::ConstPtr &msg);
  
    EventFeatureTracker tracker_;
    bool timeFlag_;
};
