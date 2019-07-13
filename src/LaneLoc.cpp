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
#include "laneloc/LaneLoc.h"


using namespace std;
using namespace cv;

namespace laneloc
{		
	/***	
    void LaneLoc::laneloc(Mat src,ros::Publisher pub, ros::Publisher marker_pub, list<vector<double> > &history_l1, 
			list<vector<double> > &history_l2, list<double> &history_theta1, list<double> &history_theta2){


			double PI = 3.1415926535;
			Mat gray_src, dst,imgHSV, pic, imgThresholded;

            if(src.empty()){
                cout<< "src is empty!"<<endl;
            }
            else{

				double secs =ros::Time::now().toSec();
				int secs1 = (int)secs;

				stringstream ss;
				ss<<secs1;
				string s1 = ss.str();

				//畸变矫正
				imshow("fisheye",src);

				Size image_size = src.size();
				Mat mapx = Mat(image_size, CV_32FC1);
				Mat mapy = Mat(image_size, CV_32FC1);
				Mat R = Mat::eye(3, 3, CV_32F);

				cv::Mat intrinsic_matrix(3, 3, cv::DataType<float>::type); // Intrisic matrix
				intrinsic_matrix.at<float>(0, 0) = 518.5882356848658;
				intrinsic_matrix.at<float>(1, 0) = 0;
				intrinsic_matrix.at<float>(2, 0) = 0;
				
				intrinsic_matrix.at<float>(0, 1) = 0;
				intrinsic_matrix.at<float>(1, 1) = 520.5134568784141;
				intrinsic_matrix.at<float>(2, 1) = 0;
				
				intrinsic_matrix.at<float>(0, 2) = 647.4260896919551;
				intrinsic_matrix.at<float>(1, 2) = 319.4302823572648;
				intrinsic_matrix.at<float>(2, 2) = 1;
				
				cv::Mat distortion_coeffs(4, 1, cv::DataType<float>::type);   // Distortion vector
				distortion_coeffs.at<float>(0) = -0.0311047;
				distortion_coeffs.at<float>(1) = -0.00322158;
				distortion_coeffs.at<float>(2) = 0.00233885;
				distortion_coeffs.at<float>(3) = -0.00269523;


				Mat intrinsic_mat(intrinsic_matrix), new_intrinsic_mat;

				intrinsic_mat.copyTo(new_intrinsic_mat);
				//调节视场大小,乘的系数越小视场越大
				new_intrinsic_mat.at<float>(0, 0) *= 0.3;
				new_intrinsic_mat.at<float>(1, 1) *= 0.3;
				//调节校正图中心，建议置于校正图中心
				new_intrinsic_mat.at<float>(0, 2) = 0.5 * src.cols;
				new_intrinsic_mat.at<float>(1, 2) = 0.5 * src.rows;

				Mat src1;
        		fisheye::undistortImage(src, src1, intrinsic_matrix, distortion_coeffs, new_intrinsic_mat,image_size);

				//fisheye::initUndistortRectifyMap(intrinsic_matrix,distortion_coeffs,R,intrinsic_matrix,image_size,CV_32FC1,mapx,mapy);
				//fisheye::initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R,
					//getOptimalNewCameraMatrix(intrinsic_matrix, distortion_coeffs, image_size, 1, image_size, 0), image_size, CV_32FC1, mapx, mapy);
				//Mat src1 = src.clone();
				//cv::remap(src, src1, mapx, mapy, INTER_LINEAR);

				cout << "矫正完成" << endl;
				cv::imshow("remap", src1);
				cvtColor(src1, imgHSV, COLOR_BGR2HSV);//转为HSV
		 
				int width_birdimage = imgHSV.cols;
				int height_birdimage = imgHSV.rows;


                cout<< "channels"<<endl;
				vector<Mat> channels;
				cv::split(imgHSV, channels);

				Mat img_H = channels.at(0);
				Mat img_S = channels.at(1);
				Mat img_V = channels.at(2);

		        ImageSeg imageseg;
		        LineProcess lineprocess;

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
                                                    imgTh, imgThresholded, dst, lines, width_birdimage, height_birdimage);
				}

				else if(history_l2.size() > num_pic){
                    imageseg.Process_left_image(history_l2,  mean_x_start2, mean_y_start2, mean_x_end2, mean_y_end2, 
								num_pic, imgTh, imgThresholded, dst, lines, width_birdimage, height_birdimage);
				}

				else{
                    imageseg.initialize(imgHSV, imgThresholded, dst, imgTh, lines);
				}


				if(lines.size() > 0){

                    lineprocess.Process_all_imageline(lines, imgThresholded, dst, history_theta1, history_theta2,
						num_pic, pub, marker_pub, PI, history_l1, history_l2, width_birdimage, height_birdimage);
				} 
				imshow("output_line_pic", dst);
		    }

  	}
	***/

	void LaneLoc::laneloc_fisheye(Mat src, ros::Publisher pub, ros::Publisher marker_pub, list<vector<double> > &history_l1, 
			list<vector<double> > &history_l2, list<double> &history_theta1, list<double> &history_theta2){

			double PI = 3.1415926535;
			Mat gray_src, dst,imgHSV, pic, imgThresholded, fisheye_thr;

            if(src.empty()){
                cout<< "src is empty!"<<endl;
            }
            else{
				imshow("fisheye",src);
				//imwrite("/home/llx/geekplus/cut_video_handle/bird/circle.jpg",src);



				cvtColor(src, imgHSV, COLOR_BGR2HSV);//转为HSV

				/***
				cvtColor(src, pic, COLOR_BGR2GRAY);//转为GRAY

				threshold(pic, pic, 255, 255, CV_THRESH_BINARY);

				//cout << "pic.size(): " << pic.size() <<endl;
				imshow("thr",pic);
				//imwrite("/home/llx/geekplus/cut_video_handle/bird/thr.jpg",pic);

				Mat pic_h =  pic.clone();
				Mat pic_s =  pic.clone();
				Mat pic_v =  pic.clone();


				vector<Mat> channels;
				cv::split(imgHSV, channels);


				Mat img_H = channels.at(0);
				Mat img_S = channels.at(1);
				Mat img_V = channels.at(2);

				//imshow("img_H",img_H);
				//imshow("img_S",img_S);
				//imshow("img_V",img_V);


				for(int i =0 ; i< imgHSV.rows; ++i){
					for(int j =0 ; j< imgHSV.cols; ++j){
						//cout<< "1"<< " " << i  << " "<< j <<endl; 
						//cout<< "imgHSV.cols: "<< imgHSV.cols << "imgHSV.rows"  << imgHSV.rows <<endl; 

						if(img_H.at<uchar>(i,j) >= iLowH && img_H.at<uchar>(i,j) <= iHighH){
							//cout<< "continue" <<endl; 
							pic_h.at<uchar>(i,j) = 255;			

						} 
					}
				}
				imshow("H",pic_h);
				//imwrite("/home/llx/geekplus/cut_video_handle/bird/H.jpg",pic);



				for(int i =0 ; i<imgHSV.rows; ++i){
					for(int j =0 ; j<imgHSV.cols; ++j){

						if(img_V.at<uchar>(i,j) >= iLowV && img_V.at<uchar>(i,j) <= iHighV){
							pic_v.at<uchar>(i,j) = 255;
						} 
					}
				}

				imshow("V",pic_v);

				//imwrite("/home/llx/geekplus/cut_video_handle/bird/V.jpg",pic);


				for(int i =0 ; i<imgHSV.rows; ++i){
					for(int j =0 ; j<imgHSV.cols; ++j){
						if(img_S.at<uchar>(i,j) >= iLowS && img_S.at<uchar>(i,j) <= iHighS){
							pic_s.at<uchar>(i,j) = 255;						
						} 

					}
				}
				imshow("S",pic_s);
				//imwrite("/home/llx/geekplus/cut_video_handle/bird/S.jpg",pic);

				***/



				inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV),
									 fisheye_thr); //Threshold the image

				imshow("fisheye_thr", fisheye_thr); 
				//imwrite("/home/llx/geekplus/cut_video_handle/bird/src.jpg",fisheye_thr);
		
								

				cout<< "size: "<< fisheye_thr.size() << endl;
				cout<< "channels: "<< fisheye_thr.channels() << endl;
				
		
				double secs =ros::Time::now().toSec();
				int secs1 = (int)secs;

				stringstream ss;
				ss<<secs1;
				string s1 = ss.str();
				cout << "s1: " << s1 << endl;
				cout <<endl;
				cout <<endl;
				cout <<endl;

				//畸变矫正

				float mul = 1.0;

				//Size image_size = src.size();
				Size image_size = Size(int(1280*mul),int(720*mul));
				Mat mapx = Mat(image_size, CV_32FC1);
				Mat mapy = Mat(image_size, CV_32FC1);
				Mat R = Mat::eye(3, 3, CV_32F);


				cv::Mat intrinsic_matrix(3, 3, cv::DataType<float>::type); // Intrisic matrix
				intrinsic_matrix.at<float>(0, 0) = 526.0460621686325;
				intrinsic_matrix.at<float>(1, 0) = 0;
				intrinsic_matrix.at<float>(2, 0) = 0;
				
				intrinsic_matrix.at<float>(0, 1) = 0;
				intrinsic_matrix.at<float>(1, 1) = 527.7225009042863;
				intrinsic_matrix.at<float>(2, 1) = 0;
				
				intrinsic_matrix.at<float>(0, 2) = 654.1771804245265;
				intrinsic_matrix.at<float>(1, 2) = 320.8740042532168;
				intrinsic_matrix.at<float>(2, 2) = 1;
				
				cv::Mat distortion_coeffs(4, 1, cv::DataType<float>::type);   // Distortion vector
				distortion_coeffs.at<float>(0) = -0.0467926;
				distortion_coeffs.at<float>(1) = 0.00453962;
				distortion_coeffs.at<float>(2) = 0.000105393;
				distortion_coeffs.at<float>(3) = -0.00195177;


				Mat intrinsic_mat(intrinsic_matrix), new_intrinsic_mat;

				intrinsic_mat.copyTo(new_intrinsic_mat);
				//调节视场大小,乘的系数越小视场越大
				new_intrinsic_mat.at<float>(0, 0) *= 0.35 * mul;
				new_intrinsic_mat.at<float>(1, 1) *= 0.35 * mul;
				//调节校正图中心，建议置于校正图中心
				new_intrinsic_mat.at<float>(0, 2) = 0.5  * mul * src.cols;
				new_intrinsic_mat.at<float>(1, 2) = 0.5  * mul * src.rows;

				Mat src1, src2;
        		//fisheye::undistortImage(src, src1, intrinsic_matrix, distortion_coeffs, new_intrinsic_mat,image_size);

				fisheye::initUndistortRectifyMap(intrinsic_matrix,distortion_coeffs,R,new_intrinsic_mat,image_size,CV_32FC1,mapx,mapy);
				//fisheye::initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs, R,
					//getOptimalNewCameraMatrix(intrinsic_matrix, distortion_coeffs, image_size, 1, image_size, 0), image_size, CV_32FC1, mapx, mapy);
				//Mat src1 = src.clone();
				cv::remap(fisheye_thr, src1, mapx, mapy, INTER_LINEAR);
				cv::remap(src, src2, mapx, mapy, INTER_LINEAR);

				//line(src1,Point(0,260 * mul),Point(1280  * mul, 260 * mul),Scalar(0,255,0),1,LINE_AA);

				cout<< "remap size: " << src1.size() <<endl;
				cv::imshow("remap", src1);

				//imwrite("/home/llx/geekplus/cut_video_handle/bird/thr.jpg",src1);

				cout << "矫正完成" << endl;
				cout << endl;

				cout << "--------------转为IPM鸟瞰图-----------------" << endl;
				cv::Mat H(3, 3, cv::DataType<float>::type); // Intrisic matrix
				H.at<float>(0, 0) = 10.88987867674842;
				H.at<float>(1, 0) = -0.2676121544751742;
				H.at<float>(2, 0) = -0.0002788852115358155;
				
				H.at<float>(0, 1) = -32.13891009351453;
				H.at<float>(1, 1) = 1.307347092852366;
				H.at<float>(2, 1) = -0.05037470355339676;
				
				H.at<float>(0, 2) = 598.03955078125;
				H.at<float>(1, 2) = 140.5583038330078;
				H.at<float>(2, 2) = 12 ;

				Mat birdImage,birdimage1;
				Rect rect(0,260 * mul ,1280  * mul, 460  * mul);
				Mat temp(src1, rect);
				Mat temp1(src2,rect);
				imshow("roi", temp);
				cout << "temp.size(): " << temp.size() <<endl;
    			warpPerspective(temp, birdImage, H, temp.size() , WARP_INVERSE_MAP+INTER_LINEAR);
				warpPerspective(temp1, birdimage1, H, temp.size() , WARP_INVERSE_MAP+INTER_LINEAR);

				cv::imshow("birdImage", birdImage);
				cv::imshow("src2", birdimage1);
				//resize(birdimage1, birdimage1, cv::Size(2560, 1440), (0, 0), (0, 0), cv::INTER_LINEAR);

				//imwrite("/home/llx/geekplus/cut_video_handle/bird/bird.jpg",birdImage);
				

				//Rect birdImage_rect(550, 0, 180, 230);
				//Mat birdImage_ROI(birdImage, birdImage_rect);
				//imshow("birdeImage_ROI",birdImage_ROI);
				//cvtColor(birdImage, imgHSV, COLOR_BGR2HSV);//转为HSV


				int width_birdimage = birdImage.cols;
				int height_birdimage = birdImage.rows;

				cout<< "width_birdimage: "<< width_birdimage << endl;
                cout<< "height_birdimage: "<< height_birdimage << endl;
	
		        ImageSeg imageseg;
		        LineProcess lineprocess;

                cout<< "lines"<<endl;
				vector<Vec4f> lines;

				double mean_x_start1=0, mean_y_start1=0, mean_x_end1=0, mean_y_end1=0;
				double mean_x_start2=0, mean_y_start2=0, mean_x_end2=0, mean_y_end2=0;
				double num_pic = 5;


				cout<< "history_l1.size(): " << history_l1.size() <<endl;
				cout<< "history_l2.size(): " << history_l2.size() <<endl;

				//Mat imgTh(720,1280,CV_8UC1,Scalar(0));
				Mat imgTh(height_birdimage, width_birdimage,CV_8UC1,Scalar(0));



				if(history_l1.size()>num_pic || history_l2.size() > num_pic){
					if(history_l1.size()>num_pic){
						imageseg.Process_right_image(history_l1,  mean_x_start1, mean_y_start1, mean_x_end1, mean_y_end1, 
											num_pic, imgTh, birdimage1, lines,width_birdimage, 
											height_birdimage, birdImage, imgThresholded);
					}

					if(history_l2.size() > num_pic){
						imageseg.Process_left_image(history_l2,  mean_x_start2, mean_y_start2, mean_x_end2, mean_y_end2, num_pic, 
														imgTh, birdimage1, lines, width_birdimage, 
														height_birdimage, birdImage, imgThresholded);
					}


				}

				else{
                    imageseg.initialize( birdImage, imgThresholded, birdimage1, imgTh, lines);
				}
			
				if(lines.size() > 0){

					lineprocess.Process_all_imageline(lines, imgThresholded, birdimage1, history_theta1, history_theta2, 
						num_pic, pub, marker_pub, PI, history_l1, history_l2, width_birdimage, height_birdimage);
				} 





				Mat intrinsic_mat1(intrinsic_matrix), new_intrinsic_mat1;

				intrinsic_mat1.copyTo(new_intrinsic_mat1);
				//调节视场大小,乘的系数越小视场越大
				new_intrinsic_mat1.at<float>(0, 0) *= 0.8 * mul;
				new_intrinsic_mat1.at<float>(1, 1) *= 0.8 * mul;
				//调节校正图中心，建议置于校正图中心
				new_intrinsic_mat1.at<float>(0, 2) = 0.5  * mul * src.cols;
				new_intrinsic_mat1.at<float>(1, 2) = 0.5  * mul * src.rows;

				Mat src3, src4, src5;
				cvtColor(src, src4, CV_BGR2GRAY);

        
				fisheye::initUndistortRectifyMap(intrinsic_matrix,distortion_coeffs,R,new_intrinsic_mat1,image_size,CV_32FC1,mapx,mapy);
			
				cv::remap(src4, src3, mapx, mapy, INTER_LINEAR);
				cv::remap(fisheye_thr, src5, mapx, mapy, INTER_LINEAR);

				Rect rect2(0, 200, 1280, 460);
				Mat temp2(src3, rect2);
				Mat temp3(src5, rect2);

				cv::imshow("temp2", temp2);
				cv::imshow("temp3", temp3);


				cv::Mat H_1(3, 3, cv::DataType<float>::type); // Intrisic matrix
				H_1.at<float>(0, 0) = 24.48823083026838;
				H_1.at<float>(1, 0) = -0.5412420205443425;
				H_1.at<float>(2, 0) = -0.0001223212566417543;
				
				H_1.at<float>(0, 1) = -34.95068964872168;
				H_1.at<float>(1, 1) = 4.96647184676344;
				H_1.at<float>(2, 1) = -0.05441575608838902;
				
				H_1.at<float>(0, 2) = 548.7914428710938;
				H_1.at<float>(1, 2) = 252.5103759765625;
				H_1.at<float>(2, 2) = 25.5 ;

				Mat birdImage2, birdImage_hsv, birdImage_thr;
				cout << "temp2.size(): " << temp2.size() <<endl;
				cv::imshow("temp2", temp2);

    			warpPerspective(temp2, birdImage2, H_1, temp2.size() , WARP_INVERSE_MAP+INTER_LINEAR);
				warpPerspective(temp3, birdImage_thr, H_1, temp2.size() , WARP_INVERSE_MAP+INTER_LINEAR);

				//threshold( birdImage2 , birdImage3, 1, 255, CV_THRESH_BINARY );

				cv::imshow("birdImage2", birdImage2);
    			medianBlur(birdImage2, birdImage2,3);
				medianBlur(birdImage_thr, birdImage_thr,3);
				cv::imshow("birdImage2_medianBlur", birdImage2);
				//threshold( birdImage , birdImage, 1, 255, CV_THRESH_BINARY );


				cv::imshow("birdImage2_THR", birdImage_thr);


				vector<Vec3f> circles1, circles2; 
				//HoughCircles(birdImage2,circles,CV_HOUGH_GRADIENT,1,15,50,30,24,30); 
				HoughCircles(birdImage_thr,circles1,CV_HOUGH_GRADIENT,1, 12, 255, 10, 17, 20); 
				HoughCircles(birdImage_thr,circles2,CV_HOUGH_GRADIENT,1, 12, 255, 10, 25, 29); 



				cvtColor(birdImage2 , birdImage2, CV_GRAY2BGR);
				cout<< "circles.size()： " << circles1.size() << endl;
				cout<< "circles.size()： " << circles2.size() << endl;

				for(size_t i = 0; i < circles1.size(); i++) { 
					Vec3f c = circles1[i];
					circle(birdImage2, Point(c[0], c[1]), c[2], Scalar(0,255,255), 1, CV_AA);
				}
				for(size_t i = 0; i < circles2.size(); i++) { 
					Vec3f c = circles2[i];
					circle(birdImage2, Point(c[0], c[1]), c[2], Scalar(0,255,255), 1, CV_AA);
				}



				/***
				Mat circle_resize, circle_pyrup1, circle_pyrup2;
				resize(temp2, circle_resize, Size(temp2.cols * 4, temp2.rows * 4), (0,0), (0,0));
				imshow("circle_size",circle_resize);
				pyrUp(temp2, circle_pyrup1,Size(temp2.cols * 2, temp2.rows * 2));
				pyrUp(circle_pyrup1, circle_pyrup2,Size(temp2.cols * 4, temp2.rows * 4));

				imshow("circle_pyrup",circle_pyrup2);
				***/

				//imageseg.Detecte_Circle(birdImage2, pub, PI, birdimage1);
				

                cout << "imshow>>>>>>>>>>>>>>>>>" << endl;
				imshow("output_line_pic", birdImage_thr);
				imshow("output_line_pic1", birdImage2);


		    }


	}



} //namespace


