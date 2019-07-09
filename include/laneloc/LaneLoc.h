#ifndef LANELOC_H
#define LANELOC_H
#include "laneloc/LineProcess.h"
#include "laneloc/ImageSeg.h"
#include <ros/ros.h>  


using namespace std;
using namespace cv;

namespace laneloc
{

    class LaneLoc 
    {
        public:
    		void laneloc(Mat src,ros::Publisher pub, ros::Publisher marker_pub, list<vector<double> > &history_l1, list<vector<double> > &history_l2,
                    list<double> &history_theta1, list<double> &history_theta2);
            void laneloc_fisheye(Mat src, ros::Publisher pub, ros::Publisher marker_pub, list<vector<double> > &history_l1, 
			list<vector<double> > &history_l2, list<double> &history_theta1, list<double> &history_theta2);
           
        private:


			int iLowH = 75;
			int iHighH = 90;  
			int iLowS = 80;   
			int iHighS = 255;  
			int iLowV = 63;  
			int iHighV = 163;  


			/*** 
			int iLowH = 75;
			int iHighH = 90;  
			int iLowS = 80;   
			int iHighS = 255;  
			int iLowV = 63;  
			int iHighV = 163; 


			int iLowH = 50;
			int iHighH = 95;  
			int iLowS = 60;   
			int iHighS = 255;  
			int iLowV = 50;  
			int iHighV = 255; 
			***/
    };
}

#endif 
