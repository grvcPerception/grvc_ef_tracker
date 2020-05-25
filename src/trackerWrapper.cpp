#include "grvc_ef_tracker/trackerWrapper.h"

ETWrapper::ETWrapper(ros::NodeHandle & nh, ros::NodeHandle nh_private): nh_(nh){
    
    ef_sub_ = nh_.subscribe("eventFeatureTopic", 1, &ETWrapper::eventFeatureCallback, this);
    tracker_pub_ = nh.advertise<dvs_msgs::EventArray>("trackerPublisher", 1);

    int windowSearchSize = 10, maxBufferSize = 200, minNeighboursFilter = 2, neighboursFilterSize = 3;
    // By default for the davis 240 
    int im_width = 240, im_height = 180; 
   
    
    nh_private.getParam("windowSearchSize", windowSearchSize);
    nh_private.getParam("minNeighboursFilter", minNeighboursFilter);
    nh_private.getParam("neighboursFilterSize", neighboursFilterSize);
    nh_private.getParam("bufferSize", maxBufferSize);

    timeFlag_ = true;
    tracker_ =  EventFeatureTracker(im_width, im_height, windowSearchSize, minNeighboursFilter, neighboursFilterSize, maxBufferSize);

}

void ETWrapper::eventFeatureCallback(const dvs_msgs::EventArray::ConstPtr &msg){

    const int n_event = msg->events.size();
    if (n_event == 0) 
        return;

    dvs_msgs::EventArray tracker_msg;
    tracker_msg.header = msg->header;
    tracker_msg.width = msg->width;
    tracker_msg.height = msg->height;

    for (const auto& e : msg->events) {
        // Set the timeZero reference for tracking (Just once)
        if (timeFlag_){
            tracker_.setTimeZero(e.ts.toSec());
            tracker_.setCameraParameters(msg->width,msg->height);
            timeFlag_ = false;
        }
        tracker_.trackerUpdate(e.ts.toSec(), e.x, e.y, e.ts);
    }

    std::vector <EventFeatureTracker::efTrack> trackerList_ = tracker_.getTracker();
    for (const auto& ef : trackerList_) {
        dvs_msgs::Event ef_msg;
        ef_msg.ts = ef.ef_gt_;
        ef_msg.x = ef.u_;
        ef_msg.y = ef.v_;
        tracker_msg.events.push_back(ef_msg);
    }
    tracker_pub_.publish(tracker_msg);

}

