#ifndef IMAGESEG_H
#define IMAGESEG_H
#include <laneloc/laneloc_nodelet.h>
#include <vector>
#include "laneloc/LineProcess.h"

using namespace std;
using namespace cv;


namespace laneloc
{

    class ImageSeg
    {
        public:
			Mat get_edge(Mat imgThresholded, Mat pic, bool if_left);

            void Process_left_ROI(int cross_down_y, int xUnitstep, Mat img_S, Mat img_V,  Mat img_H, Mat &imgTh, Mat &imgThresholded, 
                             Mat &dst, vector<Vec4f> lines, int x);

     		void Process_right_ROI(int cross_down_y, int xUnitstep, Mat img_S, Mat img_V,  Mat img_H, Mat &imgTh, Mat &imgThresholded,
                             Mat &dst, vector<Vec4f> lines, int x);

			void initialize(Mat imgHSV, Mat &imgThresholded, Mat &dst, Mat &imgTh, vector<Vec4f> &lines);

    		void Process_right_image(list<vector<double> > &history_l1, double &mean_x_start1, double &mean_y_start1,
                    	double &mean_x_end1, double &mean_y_end1, double num_pic, Mat &img_S, Mat &img_V, Mat &img_H, Mat &imgTh, 
                    	Mat &imgThresholded, Mat &dst, vector<Vec4f> lines);

    		void Process_left_image(list<vector<double> > &history_l2, double &mean_x_start2, double &mean_y_start2,
                    	double &mean_x_end2, double &mean_y_end2, double num_pic, Mat &img_S, Mat &img_V, Mat &img_H, Mat &imgTh, 
                    	Mat &imgThresholded, Mat &dst, vector<Vec4f> lines);
    };
}

#endif 
