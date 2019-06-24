#ifndef LANELOC_H
#define LANELOC_H
#include "laneloc/LineProcess.h"
#include "laneloc/ImageSeg.h"
#include <ros/ros.h>  


using namespace std;
using namespace cv;

namespace laneloc
{
    //class ImageSeg;
    //class LineProcess;
    class LaneLoc 
    {
        public:
			void lanel(Mat src1, ros::Publisher pub);



            
    };
}

#endif 
