#ifndef RVIZMARKER_H
#define RVIZMARKER_H

#include <ros/ros.h>  
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace cv;

namespace laneloc
{

    class RvizMarker 
    {
        public:
    		void markerline(ros::Publisher marker_pub, double x_start, double z_start, 
                                    double x_end, double z_end);

            
    };
}

#endif 