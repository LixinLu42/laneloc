// this should really be in the implementation (.cpp file)
#include <iostream>	
#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp> 
#include <cv_bridge/cv_bridge.h>  
#include <opencv2/opencv.hpp>
#include <math.h>

#include <vector>
#include <algorithm>

#include "laneloc/encode.h"
#include "laneloc/LineProcess.h"
#include "laneloc/laneloc_nodelet.h"
#include "laneloc/RvizMarker.h"

using namespace std;
using namespace cv;

namespace laneloc
{

	RvizMarker rvizmarker;

    double LineProcess::get_XZ(double h, double theta, double y, double f, double PI){
		double res;
		if(y > 0){
		    res = (h/sinf(theta)) * (cosf(theta) - double(y / (f*sinf(theta)+y*cosf(theta))));
		}
		else{
		    y = fabs(y);
		    double artan_yf = atan(double(y/f))* 180 / PI;
		    double theta1 = theta*180/PI - artan_yf;
		    cout<< "y: " << y << endl;
		    cout<< "(h/sinf(theta)): " << (h/sinf(theta)) <<endl;
		    cout<< "cosf(theta): " << cosf(theta) <<endl;
		    cout<< "sinf(theta - atan(double(y/f))* 180 / PI): "<< sinf(theta1 * PI /180) << endl;
		    cout<< "double(y / (f*sinf(theta1))): "<< double(y / (f*sinf(theta1 * PI /180)))<<endl;
		    cout << "atan(double(y/f))* 180 / PI: "<< atan(double(y/f))* 180 / PI <<endl;
		    cout << "theta: " << theta << endl;
		    cout << "theta1: " << theta1  << endl;
		    res = (h/sinf(theta)) * (cosf(theta) + double(y / (f*sinf(theta1 * PI /180))));
		}
		return res;
    }


    double LineProcess::get_P_to_L(double x, double y, double x_start, double z_start, double K){
		double  n = pow(K, 2) + 1;
		return (fabs(y - K * (x - x_start) - z_start) / pow(n,0.5));
  	}

    

    void LineProcess::get_theta_l(Mat &dst, vector< vector<double> > line_ifo, int best_line, double result[],
									double PI, list<vector<double> > &history_l1, list<vector<double> > &history_l2, 
									ros::Publisher marker_pub){

		vector<double> line_best_ifo;
		line_best_ifo.push_back(line_ifo[best_line][0]);
		line_best_ifo.push_back(line_ifo[best_line][1]);
		line_best_ifo.push_back(line_ifo[best_line][2]);
		line_best_ifo.push_back(line_ifo[best_line][3]);
		line_best_ifo.push_back(line_ifo[best_line][4]);
		line_best_ifo.push_back(line_ifo[best_line][5]);
		line_best_ifo.push_back(line_ifo[best_line][6]);
		line_best_ifo.push_back(line_ifo[best_line][7]);



		int point_start_x = line_ifo[best_line][0];
		int point_start_y = line_ifo[best_line][1];
		int point_end_x = line_ifo[best_line][2]; 	
		int point_end_y = line_ifo[best_line][3];
		double theta = line_ifo[best_line][4];

				
		cout<< "best_line " << best_line << endl;
		cout<< "line_ifo_size(): " << line_ifo.size() << endl;
		cout<< "(x1,y1): " << line_ifo[best_line][0] << ","<< line_ifo[best_line][1] <<endl;
		cout<< "(x2,y2): " << line_ifo[best_line][2] << "," << line_ifo[best_line][3] <<endl;
		cout<< "theta_best_l: " << line_ifo[best_line][4] <<endl;
		cout<< "P_to_L: " << line_ifo[best_line][5] <<endl;
		cout<< "mid_point: " << line_ifo[best_line][6] <<endl;
		cout<< "Confidence:" << line_ifo[best_line][7] << endl;
		cout<< "max_size()" << line_ifo.max_size() <<endl;
		
		line(dst,Point(point_start_x, point_start_y),Point(point_end_x, point_end_y),Scalar(0,255,0),2,LINE_AA);

		line(dst,Point(0,0),Point(100,100),Scalar(0,255,0),3,LINE_AA);
		line(dst,Point(0,0),Point(0,100), Scalar(0,255,0),3,LINE_AA);
		   
		double f=617.2258911132812, h=280, J=15 * PI /180;
		double delta_start_y =  point_start_y - dst.rows/2;
		double delta_start_x =  abs(point_start_x - dst.cols/2);

		cout<< "sinf(theta) " << sinf(J) << " cosf(theta) " 	<< cosf(J) <<endl;
		double z_start = get_XZ(h, J, delta_start_y, f, PI);
		double x_start = delta_start_x * z_start / f;

		double delta_end_y = point_end_y - dst.rows/2;
		double delta_end_x = abs(point_end_x - dst.cols/2);
		double z_end = get_XZ(h, J, delta_end_y, f, PI);
		double x_end = delta_end_x * z_end / f;

		cout <<"delta_end_x: " << delta_end_x << " delta_end_y: " << delta_end_y
			 << " delta_start_x: " <<  delta_start_x << " delta_start_y: " <<  delta_start_y <<endl;

		double K = ((z_end - z_start) / (x_end - x_start));
		result[0] = atan(1 / K) * 180 / PI - 4.9;
		result[1] = get_P_to_L(0,0,x_start, z_start, K) + 44;

		Mat image(720,1280,CV_8UC3,Scalar(255,255,255));  //创建一个高200，宽100的灰度图
		if(0<theta){
		  cout<<"theta > 0"<<endl;
		  cout<<theta<<endl;
		  line(image,Point(x_start/3 + 640, 720 - z_start/3), Point(x_end/3 + 640, 720 - z_end/3), Scalar(0,255,0),1,LINE_AA);
		  history_l1.push_back(line_best_ifo);

		  rvizmarker.markerline(marker_pub, x_start, z_start, x_end, z_end);

		}
		else{
		  cout<<"theta < 0"<<endl;
		  cout<<theta<<endl;
		  line(image,Point(640 - x_start/3, 720 - z_start/3), Point(640 - x_end/3, 720 - z_end/3), Scalar(0,255,0),1,LINE_AA);
		  history_l2.push_back(line_best_ifo);

		  rvizmarker.markerline(marker_pub, (-1) * x_start, z_start, (-1) * x_end, z_end);

		}
		line(image,Point(640, 700), Point(300, 715), Scalar(0,255,0),1,LINE_AA);
		line(image,Point(640, 700), Point(340, 715), Scalar(0,255,0),1,LINE_AA);
		line(image,Point(300, 715), Point(340, 715), Scalar(0,255,0),1,LINE_AA);
		imshow("fushi", image);


		cout <<"z_start: " << z_start << " x_start: " << x_start << " z_end: " <<  z_end << " x_end: " <<  x_end<<endl;
		cout <<"K: " << K <<" theta_real: " << result[0] << " l_real: " << result[1] << " sin(90 * PI /180): " <<endl;
		cout << endl;
    }


    void LineProcess::get_thetaAndL_fisheye(Mat &dst, vector< vector<double> > line_ifo, int best_line, double result[],
									double PI, list<vector<double> > &history_l1, list<vector<double> > &history_l2, 
									ros::Publisher marker_pub, int width_birdimage, int height_birdimage){

		vector<double> line_best_ifo;
		line_best_ifo.push_back(line_ifo[best_line][0]);
		line_best_ifo.push_back(line_ifo[best_line][1]);
		line_best_ifo.push_back(line_ifo[best_line][2]);
		line_best_ifo.push_back(line_ifo[best_line][3]);
		line_best_ifo.push_back(line_ifo[best_line][4]);
		line_best_ifo.push_back(line_ifo[best_line][5]);
		line_best_ifo.push_back(line_ifo[best_line][6]);
		line_best_ifo.push_back(line_ifo[best_line][7]);

		int point_start_x = line_ifo[best_line][0];
		int point_start_y = line_ifo[best_line][1];
		int point_end_x = line_ifo[best_line][2]; 	
		int point_end_y = line_ifo[best_line][3];
		double theta = line_ifo[best_line][4];

				
		cout<< "best_line " << best_line << endl;
		cout<< "line_ifo_size(): " << line_ifo.size() << endl;
		cout<< "(x1,y1): " << line_ifo[best_line][0] << ","<< line_ifo[best_line][1] <<endl;
		cout<< "(x2,y2): " << line_ifo[best_line][2] << "," << line_ifo[best_line][3] <<endl;
		cout<< "theta_best_l: " << line_ifo[best_line][4] <<endl;
		cout<< "P_to_L: " << line_ifo[best_line][5] <<endl;
		cout<< "mid_point: " << line_ifo[best_line][6] <<endl;
		cout<< "Confidence:" << line_ifo[best_line][7] << endl;
		cout<< "max_size()" << line_ifo.max_size() <<endl;
		
		line(dst,Point(point_start_x,point_start_y),Point(point_end_x, point_end_y),Scalar(0,255,0),2,LINE_AA);

		line(dst,Point(0,0),Point(100,100),Scalar(0,255,0),3,LINE_AA);
		line(dst,Point(0,0),Point(0,100), Scalar(0,255,0),3,LINE_AA);

		double K;
		if(point_end_x != point_start_x){
			K = ((point_end_y - point_start_y) / (point_end_x - point_start_x));
			result[0] = atan(1 / K) * 180 / PI;
			result[1] = get_P_to_L(0,0,point_start_x, point_start_y, K);
		}
		else{
			K = 1000000;
			result[0] = 0;
			result[1] = get_P_to_L(0,0,point_start_x, point_start_y, K);
		}

		if(point_end_x  > width_birdimage/2 ){
		  cout<<"theta > 0: "<< theta << endl;
		  history_l1.push_back(line_best_ifo);

		  rvizmarker.markerline(marker_pub, point_start_x, point_start_y, point_end_x, point_end_y);
		}
		else{
		  cout<<"theta <= 0: "<< theta << endl;
		  history_l2.push_back(line_best_ifo);

		  rvizmarker.markerline(marker_pub, (-1) * point_start_x, point_start_y, (-1) * point_end_x, point_end_y);
		}

		cout <<"point_start_x: " << point_start_x << " point_start_y: " << point_start_y << " K: " << K <<
		" point_end_x: " <<  point_end_x << " point_end_y: " <<  point_end_y << " theta_real: " << result[0] << " l_real: " << result[1] << " sin(90 * PI /180): " <<endl;
		cout << endl;




		 /***  
		double f=617.2258911132812, h=280, J=15 * PI /180;
		double delta_start_y =  point_start_y - dst.rows/2;
		double delta_start_x =  abs(point_start_x - dst.cols/2);

		cout<< "sinf(theta) " << sinf(J) << " cosf(theta) " 	<< cosf(J) <<endl;
		double z_start = get_XZ(h, J, delta_start_y, f, PI);
		double x_start = delta_start_x * z_start / f;

		double delta_end_y = point_end_y - dst.rows/2;
		double delta_end_x = abs(point_end_x - dst.cols/2);
		double z_end = get_XZ(h, J, delta_end_y, f, PI);
		double x_end = delta_end_x * z_end / f;

		cout <<"delta_end_x: " << delta_end_x << " delta_end_y: " << delta_end_y
			 << " delta_start_x: " <<  delta_start_x << " delta_start_y: " <<  delta_start_y <<endl;

		double K = ((z_end - z_start) / (x_end - x_start));
		result[0] = atan(1 / K) * 180 / PI - 4.9;
		result[1] = get_P_to_L(0,0,x_start, z_start, K) + 44;

		Mat image(720,1280,CV_8UC3,Scalar(255,255,255));  //创建一个高200，宽100的灰度图
		if(0<theta){
		  cout<<"theta > 0"<<endl;
		  cout<<theta<<endl;
		  line(image,Point(x_start/3 + 640, 720 - z_start/3), Point(x_end/3 + 640, 720 - z_end/3), Scalar(0,255,0),1,LINE_AA);
		  history_l1.push_back(line_best_ifo);

		  rvizmarker.markerline(marker_pub, x_start, z_start, x_end, z_end);

		}
		else{
		  cout<<"theta < 0"<<endl;
		  cout<<theta<<endl;
		  line(image,Point(640 - x_start/3, 720 - z_start/3), Point(640 - x_end/3, 720 - z_end/3), Scalar(0,255,0),1,LINE_AA);
		  history_l2.push_back(line_best_ifo);

		  rvizmarker.markerline(marker_pub, (-1) * x_start, z_start, (-1) * x_end, z_end);

		}

		cout <<"z_start: " << z_start << " x_start: " << x_start << " z_end: " <<  z_end << " x_end: " <<  x_end<<endl;
		cout <<"K: " << K <<" theta_real: " << result[0] << " l_real: " << result[1] << " sin(90 * PI /180): " <<endl;
		cout << endl;
		***/
    }

    void LineProcess::get_line_ifo_one(Mat imgThresholded, Vec4f plines, double theta, double P_to_L, vector< vector<double> > &line_ifo, 
                          			    double &con_max, int &best_line,double delta_x, double delta_y, int &is_line){
      is_line++;
      vector<double> line_ifo_one;
      line_ifo_one.push_back(plines[0]);
      line_ifo_one.push_back(plines[1]);
      line_ifo_one.push_back(plines[2]);
      line_ifo_one.push_back(plines[3]);
      line_ifo_one.push_back(theta);
      line_ifo_one.push_back(P_to_L);
		    
      //获得每一条直线上的所有点
      int minstep;
      if(abs(delta_x)>abs(delta_y)){
          minstep = abs(delta_y);
      }
      else{
	      minstep = abs(delta_x);	
      }

      double xUnitstep = abs(delta_x) /minstep;
      double yUnitstep = abs(delta_y) /minstep;

      cout<< "xUnitstep: " << xUnitstep<<endl;
      cout<< "yUnitstep: " << yUnitstep<<endl;

      int x = plines[0];
      int y = plines[1];
      double left_point = 1;
      double right_point = 1;
      double mid_point = 1;
    
      for(int j=0; j<minstep; ++j){
          x = x + cvRound(j * xUnitstep);
          y = y + cvRound(j * yUnitstep);

          if(imgThresholded.at<uchar>(y,x) == 255){
              mid_point++;
	          if(imgThresholded.at<uchar>(y,x-1)==0 && imgThresholded.at<uchar>(y,x-2) == 0 ){
	              left_point++; 
              } 
	          if(imgThresholded.at<uchar>(y,x+1) == 255 &&  imgThresholded.at<uchar>(y,x+2) == 255 ){
	              right_point++;
	          }
	      }
          x = x - cvRound(j * xUnitstep);
          y = y - cvRound(j * yUnitstep);
      }

      double line_len = pow((pow(delta_x,2) + pow(delta_y,2)),0.5);
      double con_mid = mid_point / minstep;
      double con_left = left_point / mid_point;
      double con_right = right_point / mid_point;
      //double con = 5 * con_mid + 2 * con_left + con_right;
      double con = mid_point;
      //double con = mid_point + left_point + right_point;
      //double con = abs(theta);
      if((con_max) < con){
          (con_max) = con;
	      (best_line) = is_line - 1;
      }
      line_ifo_one.push_back(mid_point);
      line_ifo_one.push_back(con_max);
      line_ifo.push_back(line_ifo_one);
      cout<< "mid_point: " << mid_point << endl;
      cout<< "left_point: " << left_point << endl;
      cout<< "right_point: " << right_point << endl;
      cout<< "con_mid: " << con_mid << endl;
      cout<< "con_left: " << con_left << endl;
      cout<< "con_right: " << con_right << endl;
      cout<< "best_line: " << best_line << endl;
      cout<< "line_ifo_one0: " << line_ifo_one[0] << endl;
      cout<< "line_ifo_one1: " << line_ifo_one[1] << endl;
      cout<< "line_ifo_one2: " << line_ifo_one[2] << endl;
      cout<< "line_ifo_one3: " << line_ifo_one[3] << endl;
      cout<< "line_ifo_one4: " << line_ifo_one[4] << endl;
      cout<< "line_ifo_one5: " << line_ifo_one[5] << endl;
      cout<< "line_ifo_one6: " << line_ifo_one[6] << endl;
      cout<< "line_ifo_one7: " << line_ifo_one[7] << endl;
      cout<< "line_ifo.size(): " << line_ifo.size() << endl;
    }

    void LineProcess::get_line_meanvalue(list<vector<double> > history_l, double &mean_x_start, double &mean_y_start, double &mean_x_end, 
                            double &mean_y_end, double num_pic){

		for(list<vector<double> >::iterator his_line_ifo = history_l.begin(); his_line_ifo !=history_l.end(); ++his_line_ifo){
		  mean_x_start += (*his_line_ifo)[0];
		  mean_y_start += (*his_line_ifo)[1];
		  mean_x_end += (*his_line_ifo)[2];
		  mean_y_end += (*his_line_ifo)[3];
		}
		  mean_x_start /= num_pic;
		  mean_y_start /= num_pic;
		  mean_x_end /= num_pic;
		  mean_y_end /= num_pic;

		  cout<< "mean_x_start: " << mean_x_start <<endl;
		  cout<< "mean_y_start: " << mean_y_start <<endl;
		  cout<< "mean_x_end: " << mean_x_end <<endl;
		  cout<< "mean_y_end: " << mean_y_end <<endl;
		  cout<< "mean_x_end - mean_x_start " << mean_x_end - mean_x_start << endl;
		  cout<< "mean_y_end - mean_y_start " << mean_y_end - mean_y_start <<endl;
    }


    void LineProcess::get_theta_mean(list<double> history_theta, double &mean_theta, double num){
		for(list<double>::iterator his_theta_ifo = history_theta.begin(); his_theta_ifo !=history_theta.end(); ++his_theta_ifo){
		    mean_theta += *his_theta_ifo;
		}
		mean_theta /= num;
		cout<< "mean_theta: " << mean_theta <<endl;
    }


    double LineProcess::get_K(double mean_x_start, double mean_y_start, double mean_x_end, double mean_y_end){
		cout<< "K_right1 " << (mean_y_end - mean_y_start)/(mean_x_end - mean_x_start) <<endl;
        return (double)(mean_y_end - mean_y_start)/(double)(mean_x_end - mean_x_start);
    }


    void LineProcess::get_cross_point_left(double &cross_up_x_left, double &cross_up_y_left, double &cross_down_x_left, double &cross_down_y_left,
                                                           double mean_x_start, double mean_y_start, double K_right,
														   int width_birdimage, int height_birdimage){
	    int cross_up_x =  mean_x_start - mean_y_start / K_right;
	    int cross_up_y = 0;
	    int cross_down_x = mean_x_start + (height_birdimage - mean_y_start)/K_right;
	    int cross_down_y = height_birdimage;

	    cross_up_x_left = cross_up_x - 90;
	    cross_up_y_left = 0;
	    cross_down_x_left = cross_down_x - 90;
	    cross_down_y_left = height_birdimage;

	    if(cross_down_x > width_birdimage){
		    cross_down_x = width_birdimage;
		    cross_down_y = K_right * (width_birdimage - mean_x_start) + mean_y_start;

		    if(cross_down_x_left > width_birdimage){
		        cross_down_x_left = width_birdimage;
		        cross_down_y_left = K_right * (width_birdimage + 90 - mean_x_start) + mean_y_start;
		    }
	    }
	    cout << "cross_up_x" << cross_up_x <<endl;
	    cout << "cross_up_x_left" << cross_up_x_left <<endl;
    }


    void LineProcess::get_cross_point_right(double &cross_up_x_right, double &cross_up_y_right, double &cross_down_x_right,
							double &cross_down_y_right, double mean_x_start, double mean_y_start, double K_right,
							int width_birdimage, int height_birdimage){

		int cross_up_x =  mean_x_start - mean_y_start / K_right;
		int cross_up_y = 0;
		int cross_down_x = mean_x_start + (height_birdimage - mean_y_start)/K_right;
		int cross_down_y = height_birdimage;


		cross_up_x_right = cross_up_x + 120;
		cross_up_y_right = 0;
		cross_down_x_right = cross_down_x + 120;
		cross_down_y_right = height_birdimage;


		if(cross_down_x < 0){
		  cross_down_x = 0;
		  cross_down_y = (-1) * K_right * mean_x_start + mean_y_start;

		  if(cross_down_x_right < 0){
			  cross_down_x_right = 0;
			  cross_down_y_right = K_right * (-120 - mean_x_start) + mean_y_start;
		  }
		}
		cout << "cross_up_x" << cross_up_x <<endl;
		cout << "cross_up_x_right" << cross_up_x_right <<endl;
    }


	void LineProcess::get_Unitstep(double &minstep, double &xUnitstep, double &yUnitstep, double cross_up_x, 
								double cross_up_y, double cross_down_x, double cross_down_y, int &x, int &y){

		x = cross_up_x, y = cross_up_y;
		if(abs(cross_down_x - cross_up_x) > abs(cross_down_y - cross_up_y)){
		    minstep = abs(cross_down_y - cross_up_y);
		}
		else{
		    minstep = abs(cross_down_x - cross_up_x);	
		}

	    xUnitstep = double(abs(cross_down_x - cross_up_x) /minstep);
		yUnitstep = double(abs(cross_down_y - cross_up_y) /minstep);

		cout << "xUnitstep" << xUnitstep <<endl;
		cout << "yUnitstep" << yUnitstep <<endl;
		cout << "minstep" << minstep <<endl;
    }


    void LineProcess::Process_half_imageline(list<double> history_theta, double num_pic, Mat imgThresholded, Vec4f plines, double theta,
             double P_to_L, vector<vector<double> > &line_ifo, double con_max, int &best_line, double delta_x, double delta_y, int &is_line){

		if(history_theta.size() > num_pic){
			history_theta.pop_front();
			double mean_theta;
			get_theta_mean(history_theta, mean_theta, num_pic);
			cout<< "abs(mean_theta - theta): " << abs(mean_theta - theta) <<endl;
			if(abs(mean_theta - theta) < 20){
				cout<< "11111111111111111"<<endl;
				history_theta.push_back(theta);
				get_line_ifo_one(imgThresholded, plines, theta, P_to_L, line_ifo, con_max, best_line,
				                             delta_x, delta_y, is_line);
			}

		}
		else{
			history_theta.push_back(theta);
			get_line_ifo_one(imgThresholded, plines, theta, P_to_L, line_ifo, con_max, best_line,
			                             delta_x, delta_y, is_line);
			
		}

	}

	void LineProcess::pub_theta_l(int is_line1, int is_line2, vector<vector<double> > line_ifo_1, vector<vector<double> > line_ifo_2, 
                                  double PI, int best_line1, int best_line2, list<vector<double> > &history_l1,
								  list<vector<double> > &history_l2, Mat &dst, ros::Publisher pub, ros::Publisher marker_pub,
								  int width_birdimage, int height_birdimage){
		double result_1[2];
		double result_2[2];
		laneloc::encode message;

		if(is_line1>0 || is_line2>0){
			cout<< "ininininnininini"<<endl;
			//计算右边绿线的最优直线与机器人的距离和夹角
			if(is_line1>0){
				cout<< "ininininnininini:is_line1is_line1is_line1"<<endl;
				cout<< "line_ifo.size: " << line_ifo_1.size()<<endl;
				//get_theta_l(dst, line_ifo_1, best_line1, result_1, PI, history_l1, history_l2, marker_pub);
				get_thetaAndL_fisheye(dst, line_ifo_1, best_line1, result_1, PI, history_l1, history_l2, marker_pub, 
										width_birdimage, height_birdimage);

				message.theta = result_1[0];
				message.l = result_1[1];

                
			}
			//计算左边绿线的最优直线与机器人的距离和夹角
			if(is_line2>0){
				get_thetaAndL_fisheye(dst, line_ifo_2, best_line2, result_2, PI, history_l1, history_l2, marker_pub,
										width_birdimage, height_birdimage);
				message.theta = result_2[0];
				message.l = result_2[1];


			}
		}   
		pub.publish(message);
    }



	void LineProcess::Process_all_imageline(vector<Vec4f> lines, Mat imgThresholded, Mat dst, list<double> &history_theta1, 
                 				list<double> &history_theta2, double num_pic, ros::Publisher pub, ros::Publisher marker_pub, double PI, 
                				list< vector<double> > &history_l1, list<vector<double> > &history_l2,
								int width_birdimage, int height_birdimage){
		Scalar color = Scalar(0,0,255);
		double con_max1=180, con_max2=180;
		int best_line1=0, best_line2=0;
		int is_line1=0, is_line2=0;
		vector<vector<double> > line_ifo_1;
		vector<vector<double> > line_ifo_2;

		for (size_t i = 0; i < lines.size(); i++)
		{
			Vec4f plines=lines[i];

			double delta_x = plines[0] - plines[2], delta_y = plines[1] - plines[3];
			double K1 = delta_y / delta_x;
			double theta = atan(K1) * 180/PI;

			cout<< "theta: "  << theta << endl;
			cout<< "width_birdimage: " << width_birdimage << endl;
			cout<< "plines[0]: "  << plines[0] << endl;
			cout<< "plines[1]: "  << plines[1] << endl;
			cout<< "plines[2]: "  << plines[2] << endl;
			cout<< "plines[3]: "  << plines[3] << endl;


			double P_to_L = get_P_to_L(imgThresholded.cols/2,imgThresholded.rows, plines[0], plines[1], K1);
			//double line_len = pow((pow(delta_x,2) + pow(delta_y,2)),0.5);
			line(dst,Point(plines[0],plines[1]),Point(plines[2],plines[3]),color,1,LINE_AA);
			
			//右边绿线的检测、置信度计算、提取最优
			if(abs(theta) >= 80 && plines[2] > width_birdimage/2) {
				Process_half_imageline(history_theta1, num_pic, imgThresholded, plines, theta, P_to_L,
				                                  line_ifo_1, con_max1, best_line1, delta_x, delta_y, is_line1);
			}
			//左边绿线的检测、置信度计算、提取最优
			if(abs(theta) >=80 && plines[2] < width_birdimage/2) {
				Process_half_imageline(history_theta2, num_pic, imgThresholded, plines, theta, P_to_L,
				                                  line_ifo_2, con_max2, best_line2, delta_x, delta_y, is_line2);
			}
			/*** 
			if(theta >= 30 && theta <= 70) {
				Process_half_imageline(history_theta1, num_pic, imgThresholded, plines, theta, P_to_L,
				                                  line_ifo_1, con_max1, best_line1, delta_x, delta_y, is_line1);
			}
			//左边绿线的检测、置信度计算、提取最优
			if(theta >= -70 && theta <= -30) {
				Process_half_imageline(history_theta2, num_pic, imgThresholded, plines, theta, P_to_L,
				                                  line_ifo_2, con_max2, best_line2, delta_x, delta_y, is_line2);
			}
			***/
			cout<< "is_line1: " << is_line1 <<endl;
			cout<< "is_line2: " << is_line2 <<endl;
			cout<<endl;
		}
		pub_theta_l(is_line1, is_line2, line_ifo_1, line_ifo_2, PI, best_line1, best_line2, history_l1, 
				                history_l2, dst, pub, marker_pub, width_birdimage, height_birdimage);
	}

} //namespace


