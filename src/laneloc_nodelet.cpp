// this should really be in the implementation (.cpp file)
#include <ros/ros.h>
#include <iostream>	
#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>  
#include <math.h>

#include <vector>
#include <algorithm>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include "laneloc/laneloc_nodelet.h"
#include "laneloc/encode.h"
#include "laneloc/LaneLoc.h"
#include "laneloc/LineProcess.h"
#include "laneloc/ImageSeg.h"


using namespace std;
using namespace cv;

namespace laneloc
{
    void laneloc_nodeletclass::Process()  
    {  

		list<vector<double> > history_l1;
		list<vector<double> > history_l2;
		list<double> history_theta1;
		list<double> history_theta2;

		LaneLoc l; 

        while(1)
        {
        	l.laneloc(src1, pub, marker_pub, history_l1, history_l2, history_theta1, history_theta2);	
        	waitKey(10);
        }
    }


	void laneloc_nodeletclass::imageCallback(const sensor_msgs::ImageConstPtr& msg)
	{
		//sensor_msgs::ImageConstPtr是ROS中image传递的消息形式
		try{ 

			src1 = cv_bridge::toCvShare(msg, "bgr8")->image;
		}
		catch (cv_bridge::Exception& e){
			ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str()); 
		}
	}


   void laneloc_nodeletclass::onInit()
   {

        image_transport::ImageTransport it(nh);  
		pub = nh.advertise<laneloc::encode>("/theta_l", 1000);
		marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);


		sub = it.subscribe("/camera/color/image_raw", 1, &laneloc_nodeletclass::imageCallback,this);
		cout<<"dingyue"<<endl;

        spinThread_ = boost::shared_ptr< boost::thread > (new boost::thread(boost::bind(&laneloc_nodeletclass::Process, this)));
    }

} //namespace


PLUGINLIB_DECLARE_CLASS(laneloc, laneloc_nodeletclass, laneloc::laneloc_nodeletclass, nodelet::Nodelet)
