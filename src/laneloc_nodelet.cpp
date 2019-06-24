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

		LaneLoc l; 
        l.lanel(src1,pub);								
        



        /***
		double PI = 3.1415926535;
		list<vector<double> > history_l1;
		list<vector<double> > history_l2;
		list<double> history_theta1;
		list<double> history_theta2;

		Mat gray_src, dst,imgHSV, pic, imgThresholded;

        while(1)
        {
            if(src1.empty()){
                continue;
            }
            else{
                ImageSeg imageseg;
                LineProcess lineprocess;

				double secs =ros::Time::now().toSec();
				int secs1 = (int)secs;

				stringstream ss;
				ss<<secs1;
				string s1 = ss.str();

				cv::imshow("view", src1);
				cvtColor(src1, imgHSV, COLOR_BGR2HSV);//转为HSV
		 
                cout<< "channels"<<endl;
				vector<Mat> channels;
				cv::split(imgHSV, channels);

				Mat img_H = channels.at(0);
				Mat img_S = channels.at(1);
				Mat img_V = channels.at(2);

                cout<< "lines"<<endl;
				vector<Vec4f> lines;

				double mean_x_start1=0, mean_y_start1=0, mean_x_end1=0, mean_y_end1=0;
				double mean_x_start2=0, mean_y_start2=0, mean_x_end2=0, mean_y_end2=0;
				double num_pic = 5;

				cout<< "history_l1.size(): " << history_l1.size() <<endl;
				cout<< "history_l2.size(): " << history_l2.size() <<endl;

				Mat imgTh(480,640,CV_8UC1,Scalar(0));

				if(history_l1.size()>num_pic){
                    imageseg.Process_right_image(history_l1,  mean_x_start1, mean_y_start1, mean_x_end1, mean_y_end1, num_pic, 
                                                    img_S, img_V, img_H, imgTh, imgThresholded, dst, lines);
				}

				else if(history_l2.size() > num_pic){
                    imageseg.Process_left_image(history_l2,  mean_x_start2, mean_y_start2, mean_x_end2, mean_y_end2, num_pic, 
                                                    img_S, img_V, img_H, imgTh, imgThresholded, dst, lines);
				}

				else{
                    imageseg.initialize(imgHSV, imgThresholded, dst, imgTh, lines);
				}


				if(lines.size() > 0){

                    lineprocess.Process_all_imageline(lines, imgThresholded, dst, history_theta1, history_theta2, num_pic, pub, 
                                                      PI, history_l1, history_l2);
				} 
				imshow("output_line_pic", dst);
		    }
        waitKey(10);
        }
        ***/
    }


	  void laneloc_nodeletclass::imageCallback(const sensor_msgs::ImageConstPtr& msg)
	  {
	   	   //sensor_msgs::ImageConstPtr是ROS中image传递的消息形式
		   try{ 

		       src1 = cv_bridge::toCvShare(msg, "bgr8")->image;
				imshow("src1s",src1);
           		waitKey(35);

		   }
           catch (cv_bridge::Exception& e){
                 ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str()); 
		   }
	  }


   void laneloc_nodeletclass::onInit()
   {

        image_transport::ImageTransport it(nh);  
		pub = nh.advertise<laneloc::encode>("/theta_l", 1000);

		sub = it.subscribe("/camera/color/image_raw", 1, &laneloc_nodeletclass::imageCallback,this);
		cout<<"dingyue"<<endl;

        spinThread_ = boost::shared_ptr< boost::thread > (new boost::thread(boost::bind(&laneloc_nodeletclass::Process, this)));
    }

} //namespace


PLUGINLIB_DECLARE_CLASS(laneloc, laneloc_nodeletclass, laneloc::laneloc_nodeletclass, nodelet::Nodelet)
