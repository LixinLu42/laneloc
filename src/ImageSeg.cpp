// this should really be in the implementation (.cpp file)
#include <iostream>	
#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp> 
#include <cv_bridge/cv_bridge.h>  
#include <opencv2/opencv.hpp>
#include <math.h>
#include <limits.h>

#include <vector>
#include <algorithm>

#include "laneloc/encode.h"
#include "laneloc/ImageSeg.h"
#include "laneloc/laneloc_nodelet.h"



#include <laneloc/FindCircle.h>




using namespace std;
using namespace cv;

namespace laneloc
{

    LineProcess lineprocess;	


	Mat ImageSeg::get_edge(Mat imgThresholded, Mat pic, bool if_left){
		int left;
		int right; 
	   
		if(if_left) {left=0; right=255;}
		else {left=255; right=0;}

		for(int j=0; j < imgThresholded.rows; ++j){
		    for(int i=(imgThresholded.cols / 2 ); i >= 0 ; --i){
		        if(imgThresholded.at<uchar>(j,i-1) == right && imgThresholded.at<uchar>(j,i+1) == left ){
		             //--i;
		             continue;               
		        }
		        else{
		            pic.at<uchar>(j,i) = 0;
		        }
		       
		    }

		    for(int i=(imgThresholded.cols / 2 ); i < (imgThresholded.cols); ++i){
		        if(imgThresholded.at<uchar>(j,i-1) == left && imgThresholded.at<uchar>(j,i+1) == right){
		             //i++;
		             continue;               
		        }
		        else{
		            pic.at<uchar>(j,i) = 0;
		        }
		    }
		}
		return pic;
	}

    void ImageSeg::Process_left_ROI(int cross_down_y, double xUnitstep, Mat &imgTh, Mat &dst, vector<Vec4f> &lines, 
							int x, int width_birdimage, int height_birdimage, Mat birdImage, Mat &imgThresholded,
							double K_right){

		for(int j=0; j<min(height_birdimage,cross_down_y); ++j){
			if(K_right >= 1 || K_right <= -1){
				if(j % int(K_right) == 0){
					x = x + xUnitstep;
				}
			}
			int cross_x_right = min(x+80, width_birdimage);
			if(x <= width_birdimage){

				for(int i = x; i <= cross_x_right; ++i){
					if(birdImage.at<uchar>(j,i) > 0){
				
						imgTh.at<uchar>(j,i) = 255;
					} 
				}
			}
			
		}

		imshow("imgth",imgTh);

		//imwrite("/home/llx/geekplus/cut_video_handle/bird/imgTh.jpg",imgTh);
		
		imgThresholded = imgTh.clone();

		//cvtColor(imgTh,dst,CV_GRAY2BGR);
		//medianBlur(imgTh,imgTh,3);
		imgTh = get_edge(imgThresholded, imgTh, 1);

		imshow("edge", imgTh);
		//imwrite("/home/llx/geekplus/cut_video_handle/bird/edge.jpg",imgTh);


		//'1'生成极坐标时候的像素扫描步长，'CV_PI/180'生成极坐标时候的角度步长，'30'直线上最少点的数量','50'最小直线长度，
		//'100'最大间隔（能构成一条直线） 
		HoughLinesP(imgTh,lines,1,CV_PI/360, 30, 50, 100); 

		cout <<"lines.size(): " << lines.size() << endl;
    }


    void ImageSeg::Process_right_ROI(int cross_down_y, double xUnitstep, Mat &imgTh,Mat &dst, vector<Vec4f> &lines,
					int x, int width_birdimage, int height_birdimage, Mat birdImage, Mat &imgThresholded,
					double K_right){
		cout << "cross_down_y: " << cross_down_y <<endl;
		for(int j=0; j < min(height_birdimage, cross_down_y); ++j){

    	  //cout << "xUnitstep: " << xUnitstep <<endl;
			if(K_right >= 1 || K_right <= -1){
				if(j % int(K_right) == 0){
					x = x - xUnitstep;
				}
			}
			int cross_x_right = max(x-80, 0);
			if(x >= 0){
				for(int i = x; i >= cross_x_right; --i){
					if(birdImage.at<uchar>(j,i) > 0){
						imgTh.at<uchar>(j,i) = 255;
					} 
				}
			}
		}


		imshow("imgth",imgTh);

		//imwrite("/home/llx/geekplus/cut_video_handle/bird/imgTh.jpg",imgTh);



		imgThresholded = imgTh.clone();
		//cvtColor(imgTh,dst,CV_GRAY2BGR);

		//medianBlur(imgTh,imgTh,3);
		imgTh = get_edge(imgThresholded, imgTh, 1); 
		imshow("edge", imgTh);
		//'1'生成极坐标时候的像素扫描步长，'CV_PI/180'生成极坐标时候的角度步长，'20'直线上最少点的数量','10'最小直线长度，'0'最大间隔（能构成一条直线） 
		HoughLinesP(imgTh,lines,1,CV_PI/360, 30, 50, 100); 
		cout <<"lines.size(): " << lines.size() << endl;
    }

	void ImageSeg::initialize(Mat birdImage, Mat &imgThresholded, Mat &dst, Mat &imgTh, vector<Vec4f> &lines){
		/***
		int iLowH_G= 50;
		int iHighH_G = 95;  
		int iLowS_G = 60;   
		int iHighS_G = 255;  
		int iLowV_G = 50;  
		int iHighV_G = 255;  
		

		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
		imshow("imgThresholded", imgThresholded); 
		cout<< "size: "<< imgThresholded.size() << endl;
		cout<< "channels: "<< imgThresholded.channels() << endl;

		***/

		//cvtColor(birdImage,dst,CV_GRAY2BGR);

		imgTh = birdImage.clone();
		imgThresholded = imgTh.clone();

		//medianBlur(imgTh, imgTh,3);
		imgTh = get_edge(imgThresholded, imgTh, 1); 

		imshow("edge", imgTh);
		//'1'生成极坐标时候的像素扫描步长，'CV_PI/180'生成极坐标时候的角度步长，'20'直线上最少点的数量','10'最小直线长度，'0'最大间隔（能构成一条直线） 
		HoughLinesP(imgTh,lines,1,CV_PI/360, 30, 50, 100);
		cout <<"lines.size(): " << lines.size() << endl;
    }
                   
    void ImageSeg::Process_right_image(list<vector<double> > &history_l1, double &mean_x_start1, double &mean_y_start1,
                    double &mean_x_end1, double &mean_y_end1, double num_pic, Mat &imgTh, Mat &dst, 
					vector<Vec4f> &lines, int width_birdimage, int height_birdimage, Mat birdImage, Mat &imgThresholded){
		history_l1.pop_front();
		lineprocess.get_line_meanvalue(history_l1, mean_x_start1, mean_y_start1, mean_x_end1, mean_y_end1, num_pic);

		double K_right =  lineprocess.get_K(mean_x_start1, mean_y_start1, mean_x_end1, mean_y_end1);
		cout<< "K_right " << K_right <<endl;

		double cross_up_x_left = 0, cross_up_y_left = 0;
		double cross_down_x_left = 0, cross_down_y_left = 0;

		double minstep; 
		int x, y;
		double xUnitstep, yUnitstep;

		if(K_right  > -100 && K_right <= 100){
			cout << "K_right  > -100000 && K_right <= 100000" <<endl;
			lineprocess.get_cross_point_left(cross_up_x_left, cross_up_y_left, cross_down_x_left, cross_down_y_left,
					                           mean_x_start1, mean_y_start1, K_right, width_birdimage, height_birdimage);			
			lineprocess.get_Unitstep(minstep, xUnitstep, yUnitstep, cross_up_x_left, cross_up_y_left,
											cross_down_x_left, cross_down_y_left, x, y);
			Process_left_ROI(cross_down_y_left, xUnitstep, imgTh,
								dst, lines, x,width_birdimage, height_birdimage, birdImage, imgThresholded, K_right);
		}
		else{
			cout << "K_right == -inf" <<endl;

			cross_down_y_left = height_birdimage;
			xUnitstep = 0;
			x = mean_x_start1;
			Process_left_ROI(cross_down_y_left, xUnitstep, imgTh,
								dst, lines, x,width_birdimage, height_birdimage, birdImage, imgThresholded, K_right);

		}
    }


    void ImageSeg::Process_left_image(list<vector<double> > &history_l2, double &mean_x_start2, double &mean_y_start2,
                    double &mean_x_end2, double &mean_y_end2, double num_pic, Mat &imgTh, Mat &dst, 
					vector<Vec4f> &lines, int width_birdimage, int height_birdimage, Mat birdImage, Mat &imgThresholded){
		history_l2.pop_front(); 
		lineprocess.get_line_meanvalue(history_l2, mean_x_start2, mean_y_start2, mean_x_end2, mean_y_end2, num_pic);

		double K_right =  lineprocess.get_K(mean_x_start2, mean_y_start2, mean_x_end2, mean_y_end2);
		cout<< "K_right " << K_right <<endl;


		double cross_up_x_right = 0, cross_up_y_right = 0;
		double cross_down_x_right = 0, cross_down_y_right = 0;

		double minstep;
		int x, y;
		double xUnitstep, yUnitstep;

        if(K_right  > -100 && K_right <= 100){
			cout << "K_right  > -100000 && K_right <= 100000" <<endl;
			lineprocess.get_cross_point_right(cross_up_x_right, cross_up_y_right, cross_down_x_right, 
											cross_down_y_right, mean_x_start2, mean_y_start2, K_right,
											width_birdimage, height_birdimage);

			lineprocess.get_Unitstep(minstep, xUnitstep, yUnitstep, cross_up_x_right, cross_up_y_right,
										cross_down_x_right, cross_down_y_right, x, y);

			Process_right_ROI(cross_down_y_right, xUnitstep, imgTh, 
									dst, lines, x, width_birdimage, height_birdimage, birdImage, imgThresholded, K_right);

		}
		else{
			cout << "K_right == -inf" <<endl;
			cross_down_y_right = height_birdimage;
			xUnitstep = 0;
			x = mean_x_start2;
			Process_right_ROI(cross_down_y_right, xUnitstep, imgTh,
									dst, lines, x, width_birdimage, height_birdimage, birdImage, imgThresholded, K_right);
		}


    }



	/***
	//调用迎春的程序
	void ImageSeg::Detecte_Circle(Mat birdImage, ros::Publisher pub, double PI, Mat &birdimage1){

		IplImage* imgAlgo = cvCreateImage(cvSize(1280, 460), IPL_DEPTH_8U, 1);
        cout << "no problem!" <<endl;
		IplImage temp = (IplImage)birdImage;
		        cout << "no problem!1" <<endl;

		imgAlgo = &temp;
		        cout << "no problem!2" <<endl;

		cvShowImage("imgALGO",imgAlgo);


		int radius_min = 25 , radius_max = 29;
		float sensitivity = 0.85;


		struct Circles circles1;
		struct Circles* circles = &circles1;
		cout << "circles1->n: " << circles->n <<endl; 


		clock_t begin = clock();
		stImage stImageRaw;
		stImageRaw.buffer = (unsigned char*)imgAlgo->imageData;
		stImageRaw.cols = imgAlgo->width;
		stImageRaw.rows = imgAlgo->height;

        cout << "stImageRaw.cols: " << stImageRaw.cols <<endl;
        cout << "stImageRaw.rows: " << stImageRaw.rows <<endl;
        



		int xm = add(1,2);
		cout << "xm: " <<xm <<endl;
        cout << "no problem!3" <<endl;

		circles = imfindcircles_m(&stImageRaw, radius_min, radius_max, sensitivity);


		cvtColor(birdImage, birdImage, CV_GRAY2BGR);

		//publish the pose(x,y)
		laneloc::encode message1;
		if(circles->n == 1){

			//input and draw the circles
			double x_src = circles->circles[0].centre_x ;
			double y_src = circles->circles[0].centre_y ;

			double x_circle = x_src - 640 ;
			double y_circle = 230 - y_src;
			double theta_circle = atan(x_circle / y_circle) * 180 / PI;
			double radius_circle = circles->circles[0].radius;

			//input and draw the circles
			cout << "circles->n: " << circles->n <<endl; 
			cout<< "x_circle: " <<  x_src <<endl;
			cout<< "y_circle: " <<  y_src <<endl;
			cout<< "theta_circle: " <<  theta_circle <<endl;
			cout<< "radius_circle: " <<  radius_circle <<endl;

    		circle(birdImage, cv::Point(x_src, y_src),
							 radius_circle, cv::Scalar(0,0 , 255));


			message1.x_circle = x_circle ;
			message1.y_circle = y_circle;
			message1.theta_circle = theta_circle;

		}
		else if(circles->n > 1){

			float max = 0;
			int num;
			for(int i = 0; i < circles->n; ++i){

				circle(birdImage, cv::Point(circles->circles[i].centre_x, circles->circles[i].centre_y),
						circles->circles[i].radius, cv::Scalar(0, 0, 255));

				if(circles->circles[i].confidence > max){
					max = circles->circles[i].confidence;
					num = i;
				}
			}

			double x_src = circles->circles[num].centre_x ;
			double y_src = circles->circles[num].centre_y ;

			double x_circle = x_src - 640 ;
			double y_circle = 230 - y_src;
			double theta_circle = atan(x_circle / y_circle) * 180 / PI;
			double radius_circle = circles->circles[num].radius;

			//input and draw the circles
			cout << "circles->n: " << circles->n <<endl; 
			cout<< "x_circle: " <<  x_src <<endl;
			cout<< "y_circle: " <<  x_src <<endl;
			cout<< "theta_circle: " <<  theta_circle <<endl;
			cout<< "radius_circle: " <<  radius_circle <<endl;




			message1.x_circle = x_circle ;
			message1.y_circle = y_circle;
			message1.theta_circle = theta_circle;

		}
		imshow("out_put_circle",birdImage);
		imwrite("/home/llx/geekplus/cut_video_handle/bird/birdImage_gray.jpg",birdImage);

		pub.publish(message1);

	}
	***/



}//namespace



