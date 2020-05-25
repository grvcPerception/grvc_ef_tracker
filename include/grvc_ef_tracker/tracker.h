#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <Eigen/Dense>
#include <vector>
#include <deque>
#include <iostream>

class EventFeatureTracker{
    public:
    EventFeatureTracker();
    EventFeatureTracker(int sensorWidth, int sensorHeight, int neighbourWindow, int minNeighboursFilter, int neighboursFilterSize, int maxBufferSize);
      
    public:

        // Methods
        void trackerUpdate(double et, int ex, int ey, ros::Time egt);

        // Struct
        struct efTrack{
            double ts_;
            int u_;
            int v_;
            ros::Time ef_gt_;
            efTrack(double ts, int u, int v, ros::Time ef_gt)
            : ts_(ts), u_(u), v_(v), ef_gt_(ef_gt)
            {}
        };

        void setTimeZero(double initialTime);
        void setCameraParameters(int width, int height);
        std::vector <efTrack> getTracker();

    private:
        //  Sensor Parameters
        int width_, height_;
        
        // Tracker Parameters
        int  neighbourWindow_, deltaWindow_, deltaWindowCounter_,  minNeighboursFilter_, forgetWindow_, deltaForgetWindow_, maxBufferSize_; 

        double timeZero_;

        // Sae
        Eigen::MatrixXd saeTracking_;

        // Vectors
        std::vector <efTrack> tracker_;
        std::deque<efTrack> bufferSae_;

        // Methods
        std::vector <int> searchNeighbours(const int x, const int y);    
        int findMinID(std::vector <int> list);
        int nonZeroElements(const double ts, const int x, const int y);
        void cleanTracker(std::vector <int> trackToDelte);
        void cleanTrackerByTime(double currentTime);
        
};