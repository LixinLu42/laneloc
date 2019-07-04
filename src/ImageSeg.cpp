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
		        if(imgThresholded.at<uchar>(j,i-2) == right && imgThresholded.at<uchar>(j,i+1) == left ){
		             //--i;
		             continue;               
		        }
		        else{
		            pic.at<uchar>(j,i) = 0;
		        }
		       
		    }

		    for(int i=(imgThresholded.cols / 2 ); i < (imgThresholded.cols); ++i){
		        if(imgThresholded.at<uchar>(j,i-3) == left && imgThresholded.at<uchar>(j,i-2) == left && 
                   imgThresholded.at<uchar>(j,i-1) == left &&imgThresholded.at<uchar>(j,i+1) == right){
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

    void ImageSeg::Process_left_ROI(int cross_down_y, int xUnitstep, Mat img_S, Mat img_V,  Mat img_H, Mat &imgTh,
		Mat &imgThresholded, Mat &dst, vector<Vec4f> lines, int x, int width_birdimage, int height_birdimage){
		for(int j=0; j<min(height_birdimage,cross_down_y); ++j){
			x = x + cvRound(j * xUnitstep);
			//cout << "xUnitstep: " << xUnitstep <<endl;
			int cross_x_right = min(x+210, width_birdimage);
			if(x <= width_birdimage){

				for(int i = x; i <= cross_x_right; ++i){
					if(img_S.at<uchar>(j,i) >= 60 && img_V.at<uchar>(j,i) >= 50 && 
						img_H.at<uchar>(j,i) >= 50 && img_H.at<uchar>(j,i) <= 95){
				
						imgTh.at<uchar>(j,i) = 255;
					} 
					img_S.at<uchar>(j,i) = 0;       
       
				}
			}
			x = x - cvRound(j * xUnitstep);
		}

		imshow("0", img_H);
		imshow("1", img_S);
		imshow("2", img_V);

		imshow("imgth",imgTh);

		imgThresholded = imgTh.clone();
		cvtColor(imgTh,dst,CV_GRAY2BGR);

		medianBlur(imgTh,imgTh,3);
		imgTh = get_edge(imgThresholded, imgTh, 1);

		imshow("edge", imgTh);

		//'1'生成极坐标时候的像素扫描步长，'CV_PI/180'生成极坐标时候的角度步长，'30'直线上最少点的数量','50'最小直线长度，
		//'100'最大间隔（能构成一条直线） 
		HoughLinesP(imgTh,lines,1,CV_PI/360, 10, 10, 100); 

		cout <<"lines.size(): " << lines.size() << endl;
    }


    void ImageSeg::Process_right_ROI(int cross_down_y, int xUnitstep, Mat img_S, Mat img_V,  Mat img_H, Mat &imgTh, Mat &imgThresholded,
                          Mat &dst, vector<Vec4f> lines, int x, int width_birdimage, int height_birdimage){

		for(int j=0; j < min(height_birdimage, cross_down_y); ++j){
		  x = x - cvRound(j * xUnitstep);
    	  //cout << "xUnitstep: " << xUnitstep <<endl;

		  int cross_x_right = max(x-210, 0);
		  if(x >= 0){
			  for(int i = x; i >= cross_x_right; --i){

				  if(img_S.at<uchar>(j,i) >= iLowS && img_V.at<uchar>(j,i) >= iLowV && 
				  		img_H.at<uchar>(j,i) >= iLowH && img_H.at<uchar>(j,i) <= iHighH){
				
				      imgTh.at<uchar>(j,i) = 255;

				  } 
				  img_S.at<uchar>(j,i) = 0;              
			  }
		   }
		   x = x + cvRound(j * xUnitstep);
		}

		imshow("0", img_H);
		imshow("1", img_S);
		imshow("2", img_V);
		imshow("imgth",imgTh);

		imgThresholded = imgTh.clone();
		cvtColor(imgTh,dst,CV_GRAY2BGR);

		medianBlur(imgTh,imgTh,3);
		imgTh = get_edge(imgThresholded, imgTh, 1); 
		imshow("edge", imgTh);
		//'1'生成极坐标时候的像素扫描步长，'CV_PI/180'生成极坐标时候的角度步长，'20'直线上最少点的数量','10'最小直线长度，'0'最大间隔（能构成一条直线） 
		HoughLinesP(imgTh,lines,1,CV_PI/360, 10, 10, 100); 
		cout <<"lines.size(): " << lines.size() << endl;
    }

	void ImageSeg::initialize(Mat imgHSV, Mat &imgThresholded, Mat &dst, Mat &imgTh, vector<Vec4f> &lines){
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

		cvtColor(imgThresholded,dst,CV_GRAY2BGR);

		imgTh = imgThresholded.clone();
		//medianBlur(imgTh, imgTh,3);
		imgTh = get_edge(imgThresholded, imgTh, 1); 

		imshow("edge", imgTh);
		//'1'生成极坐标时候的像素扫描步长，'CV_PI/180'生成极坐标时候的角度步长，'20'直线上最少点的数量','10'最小直线长度，'0'最大间隔（能构成一条直线） 
		HoughLinesP(imgTh,lines,1,CV_PI/360, 20, 10, 100);
		cout <<"lines.size(): " << lines.size() << endl;
    }
                   
    void ImageSeg::Process_right_image(list<vector<double> > &history_l1, double &mean_x_start1, double &mean_y_start1,
                    double &mean_x_end1, double &mean_y_end1, double num_pic, Mat &img_S, Mat &img_V, Mat &img_H, Mat &imgTh, 
                    Mat &imgThresholded, Mat &dst, vector<Vec4f> lines, int width_birdimage, int height_birdimage){
		history_l1.pop_front();
		lineprocess.get_line_meanvalue(history_l1, mean_x_start1, mean_y_start1, mean_x_end1, mean_y_end1, num_pic);

		double K_right =  lineprocess.get_K(mean_x_start1, mean_y_start1, mean_x_end1, mean_y_end1);
		cout<< "K_right " << K_right <<endl;

		double cross_up_x_left = 0, cross_up_y_left = 0;
		double cross_down_x_left = 0, cross_down_y_left = 0;

		double minstep; 
		int x, y;
		double xUnitstep, yUnitstep;

		if(K_right  > -100000 && K_right <= 100000){
			cout << "K_right  > -100000 && K_right <= 100000" <<endl;
			lineprocess.get_cross_point_left(cross_up_x_left, cross_up_y_left, cross_down_x_left, cross_down_y_left,
					                           mean_x_start1, mean_y_start1, K_right, width_birdimage, height_birdimage);			
			lineprocess.get_Unitstep(minstep, xUnitstep, yUnitstep, cross_up_x_left, cross_up_y_left,
											cross_down_x_left, cross_down_y_left, x, y);
			Process_left_ROI(cross_down_y_left, xUnitstep, img_S, img_V, img_H, imgTh, imgThresholded, 
								dst, lines, x,width_birdimage, height_birdimage);
		}
		else{
			cout << "K_right == -inf" <<endl;

			cross_down_y_left = height_birdimage;
			xUnitstep = 0.01;
			x = mean_x_start1;
			Process_left_ROI(cross_down_y_left, xUnitstep, img_S, img_V, img_H, imgTh, imgThresholded, 
								dst, lines, x,width_birdimage, height_birdimage);

		}
    }


    void ImageSeg::Process_left_image(list<vector<double> > &history_l2, double &mean_x_start2, double &mean_y_start2,
                    double &mean_x_end2, double &mean_y_end2, double num_pic, Mat &img_S, Mat &img_V, Mat &img_H, Mat &imgTh, 
                    Mat &imgThresholded, Mat &dst, vector<Vec4f> lines, int width_birdimage, int height_birdimage){
		history_l2.pop_front(); 
		lineprocess.get_line_meanvalue(history_l2, mean_x_start2, mean_y_start2, mean_x_end2, mean_y_end2, num_pic);

		double K_right =  lineprocess.get_K(mean_x_start2, mean_y_start2, mean_x_end2, mean_y_end2);
		cout<< "K_right " << K_right <<endl;


		double cross_up_x_right = 0, cross_up_y_right = 0;
		double cross_down_x_right = 0, cross_down_y_right = 0;

		double minstep;
		int x, y;
		double xUnitstep, yUnitstep;

        if(K_right  > -100000 && K_right <= 100000){
			cout << "K_right  > -100000 && K_right <= 100000" <<endl;
			lineprocess.get_cross_point_right(cross_up_x_right, cross_up_y_right, cross_down_x_right, 
											cross_down_y_right, mean_x_start2, mean_y_start2, K_right,
											width_birdimage, height_birdimage);

			lineprocess.get_Unitstep(minstep, xUnitstep, yUnitstep, cross_up_x_right, cross_up_y_right,
										cross_down_x_right, cross_down_y_right, x, y);

			Process_right_ROI(cross_down_y_right, xUnitstep, img_S, img_V, img_H, imgTh, imgThresholded, 
									dst, lines, x, width_birdimage, height_birdimage);

		}
		else{
			cout << "K_right == -inf" <<endl;
			cross_down_y_right = height_birdimage;
			xUnitstep = 0.01;
			x = mean_x_start2;
			Process_right_ROI(cross_down_y_right, xUnitstep, img_S, img_V, img_H, imgTh, imgThresholded, 
									dst, lines, x, width_birdimage, height_birdimage);
		}


    }



}//namespace




