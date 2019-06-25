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
    		void laneloc(Mat src1,ros::Publisher pub, ros::Publisher marker_pub, list<vector<double> > &history_l1, list<vector<double> > &history_l2,
                    list<double> &history_theta1, list<double> &history_theta2);
           
    };
}

#endif 
