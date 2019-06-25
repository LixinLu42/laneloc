#include <ros/ros.h>  
#include <visualization_msgs/Marker.h>
 
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>	

#include "laneloc/RvizMarker.h"
 
using namespace std;
using namespace cv;

namespace laneloc
{	
    void RvizMarker::markerline(ros::Publisher marker_pub, double x_start, double z_start, 
                                    double x_end, double z_end){
        visualization_msgs::Marker line_list;

         line_list.header.frame_id = "line";
         line_list.header.stamp = ros::Time::now(); 
         line_list.ns = "lines"; 
         line_list.action = visualization_msgs::Marker::ADD; 
         line_list.pose.orientation.w = 1.0; 
         line_list.id = 2; 
         line_list.type = visualization_msgs::Marker::LINE_LIST; 
         // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width 
         line_list.scale.x = 0.1; 
         // Line list is red 
         line_list.color.r = 1.0; 
         line_list.color.a = 1.0; 
         // Create the vertices for the points and lines 
         geometry_msgs::Point p; 
         p.x = x_start / 100.0; 
         p.y = z_start / 100.0; 
         p.z = 0; 

        // The line list needs two points for each line 
        line_list.points.push_back(p); 
        p.x = x_end / 100.0; 
        p.y = z_end / 100.0;
        line_list.points.push_back(p); 


         cout << "publish line_list" <<endl;
         marker_pub.publish(line_list); 

        
    }
} //namespace