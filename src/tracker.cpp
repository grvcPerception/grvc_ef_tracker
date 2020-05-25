#include "grvc_ef_tracker/tracker.h"

EventFeatureTracker::EventFeatureTracker(){
    // DVS 240 dimensions
    setCameraParameters(240, 180);

    neighbourWindow_ = 10;
    deltaWindow_ = floor(neighbourWindow_/2);

    minNeighboursFilter_ = 2; 
    forgetWindow_ = 3;
    deltaForgetWindow_ = floor(forgetWindow_/2);

    maxBufferSize_ = 200;
}

EventFeatureTracker::EventFeatureTracker(int sensorWidth, int sensorHeight, int neighbourSeachWindow, int minNeighboursFilter, int neighboursFilterSize, int maxBufferSize){
    setCameraParameters(sensorWidth, sensorHeight);

    neighbourWindow_ = neighbourSeachWindow;
    deltaWindow_ = floor(neighbourWindow_/2);

    minNeighboursFilter_ = minNeighboursFilter; 
    forgetWindow_ = neighboursFilterSize;
    deltaForgetWindow_ = floor(forgetWindow_/2);

    maxBufferSize_ = maxBufferSize;
}

int EventFeatureTracker::findMinID(std::vector <int> list){
    int minIndex = 0;
    int minIndexList = list[0];
    for (int i = 0; i< list.size(); ++i){        
        if (list[i] < minIndexList ){
            minIndex = i;
            minIndexList = list[i];
        }
    }
    return minIndex;
}

int EventFeatureTracker::nonZeroElements(const double ts, const int x, const int y){

    Eigen::MatrixXd saeWindow = saeTracking_.block(y-deltaForgetWindow_,x-deltaForgetWindow_,forgetWindow_,forgetWindow_);
    int zeroElements = -1;
    
    for (int i = 0; i<forgetWindow_ ; ++i){
        for (int j =0; j<forgetWindow_ ; ++j){
            if (bufferSae_.back().ts_ < saeWindow(j,i))
                zeroElements++;
        }
    }
    return zeroElements;
}


std::vector <int> EventFeatureTracker::searchNeighbours(const int x, const int y){

    std::vector <int> trackToDelete;

    if(tracker_.size()> neighbourWindow_*neighbourWindow_){
        for (int i=x-deltaWindow_; i<=x+deltaWindow_; i++) {
            for (int j=y-deltaWindow_; j<=y+deltaWindow_; j++) {
                for (int k=0; k<tracker_.size();k++){
                    if (tracker_[k].u_ == i && tracker_[k].v_ == j)
                        trackToDelete.push_back(k);
                }
            }
        }
    }
    else{
        for (int k=0; k<tracker_.size();k++){
            double d = (tracker_[k].u_-x)*(tracker_[k].u_-x)+(tracker_[k].v_-y)*(tracker_[k].v_-y);
            if (d <= deltaWindow_*deltaWindow_)              
                trackToDelete.push_back(k);
        }
    }
    return trackToDelete;
}

void EventFeatureTracker::cleanTracker(std::vector <int > trackToDelete){
    int counter = 0;
    int deleteCounter = 0;
    std::vector <efTrack> newTracker;

    for (int i = 0; i<tracker_.size(); ++i){

        if (trackToDelete.size()>0) {
            bool flag = true;
            for (int j=0; j<trackToDelete.size(); ++j){
                if (i == trackToDelete[j]){
                    flag = false;
                    trackToDelete.erase(trackToDelete.begin()+ j);
                    break; 
                }
            }
            if (flag == true) 
                newTracker.push_back(tracker_[i]);
        }
        else
            newTracker.push_back(tracker_[i]);
    }
    tracker_ = newTracker;
}

void EventFeatureTracker::cleanTrackerByTime(double currentTime){
    std::vector <efTrack> newTracker;
    std::vector <int> newTrackIdsList;
    for (int i = 0; i<tracker_.size(); ++i){
        if (tracker_[i].ts_ > bufferSae_.back().ts_)
            newTracker.push_back(tracker_[i]);
    }

    tracker_ = newTracker;  
}

void EventFeatureTracker::trackerUpdate(double et, int ex, int ey, ros::Time egt){

    const double ts = et-timeZero_;
    const int x = ex;
    const int y = ey;
    ros::Time eGlobalTime = egt;
    
    // Update sae
    saeTracking_(y,x) = ts;
    // Update the bufferSae_
    bufferSae_.push_front(efTrack(ts, x, y, eGlobalTime));

    // Limit the number of samples in the Buffer
    if (bufferSae_.size() > maxBufferSize_)
        bufferSae_.pop_back();

    // Avoid the access to non valid positions
    if (x-deltaForgetWindow_<0  || x+deltaForgetWindow_> width_-1 || y-deltaForgetWindow_ < 0  || y+deltaForgetWindow_ >  height_-1)
        return;

    if (nonZeroElements(ts,x,y) > minNeighboursFilter_) {

        bool newFeature = false;
        if (tracker_.size() > 0) {   // The tracker is not empty
            // Find possible track candidates
            std::vector <int> trackToDelete = searchNeighbours(x, y);

            if(trackToDelete.size()<1)
                newFeature = true;

            if(newFeature ==  false){    // Replace the oldest feature track with the current feature
                int index = findMinID(trackToDelete);
                efTrack newTrack = efTrack(ts, x, y, eGlobalTime);
                // Update the list in the index position
                tracker_[trackToDelete[index]] = newTrack;
                // Delete the index element from trackToDelete (The list of tracks to delete)
                trackToDelete.erase(trackToDelete.begin()+ index);

                if (trackToDelete.size() > 0)
                    cleanTracker(trackToDelete);
            }
            else // Add a new feature to the list of tracks
                tracker_.push_back(efTrack(ts, x, y, eGlobalTime));
        }
        else  // The list of tracks is empty
            tracker_.push_back(efTrack(ts, x, y, eGlobalTime));
        // Delete old tracks
        cleanTrackerByTime(ts);
    }
}


void EventFeatureTracker::setTimeZero(double initialTime){
    timeZero_ = initialTime;
}

void EventFeatureTracker::setCameraParameters(int sensorWidth, int sensorHeight){
    height_ = sensorHeight; 
    width_ = sensorWidth;
    saeTracking_ = Eigen::MatrixXd::Zero(height_,width_);
}

std::vector <EventFeatureTracker::efTrack> EventFeatureTracker::getTracker(){ 
    return  tracker_;
}
