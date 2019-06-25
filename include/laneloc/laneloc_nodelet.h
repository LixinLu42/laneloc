#ifndef LANELOC_NODELET_H
#define LANELOC_NODELET_H

#include <nodelet/nodelet.h>
#include <iostream>	
#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp>  
#include <cv_bridge/cv_bridge.h>  
#include <opencv2/opencv.hpp>
#include <math.h>

#include <vector>
#include <list>
#include <algorithm>
#include <laneloc/encode.h>

#include <pluginlib/class_list_macros.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include "laneloc/LaneLoc.h"
#include <visualization_msgs/Marker.h>


using namespace std;
using namespace cv;

namespace laneloc
{
    class LaneLoc;
    class laneloc_nodeletclass : public nodelet::Nodelet
    {
        public:
		    Mat src1;
			ros::NodeHandle nh;
			ros::Publisher pub;
            ros::Publisher marker_pub; 
			image_transport::Subscriber sub;
            boost::shared_ptr<boost::thread> spinThread_;

        public:
            virtual void onInit();
            void Process();
            void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    };
}
#endif 
