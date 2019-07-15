#ifndef LINEPROCESS_H
#define LINEPROCESS_H
#include <laneloc/laneloc_nodelet.h>
using namespace std;
using namespace cv;
namespace laneloc
{

    class LineProcess
    {
        public:
            double get_XZ(double h, double theta, double y, double f, double PI);
			double get_P_to_L(double x, double y, double x_start, double z_start, double K);
			float get_P_to_P(float x, float y, float x1, float y1);
    		double get_K(double mean_x_start, double mean_y_start, double mean_x_end, double mean_y_end);


    		void get_theta_l(Mat &dst, vector< vector<double> > line_ifo, int best_line, double result[], double PI,
                                    list<vector<double> > &history_l1, list<vector<double> > &history_l2,
									ros::Publisher marker_pub);

    		void get_thetaAndL_fisheye(Mat &dst, vector< vector<double> > line_ifo, int best_line, 
												double result[], double PI, list<vector<double> > &history_l1,
												 list<vector<double> > &history_l2, ros::Publisher marker_pub,
												 int width_birdimage, int height_birdimage);

    		void get_line_ifo_one(Mat imgThresholded, Vec4f plines, double theta, double P_to_L, vector< vector<double> > &line_ifo, 
                          			double &con_max, int &best_line,double delta_x, double delta_y, int &is_line);

            void get_line_meanvalue(list<vector<double> > history_l, double &mean_x_start, double &mean_y_start, 
                                   double &mean_x_end, double &mean_y_end, double num_pic);

    		void get_theta_mean(list<double> history_theta, double &mean_theta, double num);

            void get_cross_point_left(double &cross_up_x_left, double &cross_up_y_left, double &cross_down_x_left, 
								double &cross_down_y_left,double mean_x_start, double mean_y_start, double K_right,
								int width_birdimage, int height_birdimage);

            void get_cross_point_right(double &cross_up_x_right, double &cross_up_y_right, double &cross_down_x_right,
							double &cross_down_y_right, double mean_x_start, double mean_y_start, double K_right,
							int width_birdimage, int height_birdimage);

	        void get_Unitstep(double &minstep, double &xUnitstep, double &yUnitstep, double cross_up_x, 
								double cross_up_y, double cross_down_x, double cross_down_y, int &x, int &y);

    		void Process_half_imageline(list<double> history_theta, double num_pic, Mat imgThresholded, Vec4f plines,
										double theta, double P_to_L, vector<vector<double> > &line_ifo, double con_max, int &best_line, 
										double delta_x, double delta_y, int &is_line);

			void pub_theta_l(int is_line1, int is_line2, vector<vector<double> > line_ifo_1, vector<vector<double> > line_ifo_2, 
                                     double PI, int best_line1, int best_line2, list<vector<double> > &history_l1, 
                                     list<vector<double> > &history_l2, Mat &dst, ros::Publisher pub, ros::Publisher marker_pub,
									 int width_birdimage, int height_birdimage);

			void Process_all_imageline(vector<Vec4f> lines, Mat imgThresholded, Mat dst, list<double> &history_theta1, 
                 				list<double> &history_theta2, double num_pic, ros::Publisher pub, ros::Publisher marker_pub, double PI, 
                				list< vector<double> > &history_l1, list<vector<double> > &history_l2,
								int width_birdimage,int height_birdimage);

    };


}

#endif 
