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
#include "laneloc/CircleProcess.h"

using namespace std;
using namespace cv;

namespace laneloc
{
    void Circle::Cirle_Process(list<Vec3f> &history_c1, Mat birdImage, Mat &birdimage1)
    {
        vector<Vec3f> circles1, circles2;
        float num_pic = 3;
        imshow("inininininini", birdImage);


        //HoughCircles(birdImage,circles2,CV_HOUGH_GRADIENT,1, 40, 255, 10, 25, 29); 
        HoughCircles(birdImage,circles1,CV_HOUGH_GRADIENT,1, 30, 255, 10, 17, 20); 


        if(history_c1.size() <= num_pic){
            cout<< "history_c1.size() < num_pic: "  <<  history_c1.size() << endl;

            if(circles1.size() == 1 && circles1[0][0] > 450 && circles1[0][0] < 830 && 
                circles1[0][1] > 300 && circles1[0][1] < 460){
                cout<< "circles1.size(): "  <<  circles1.size() << endl;
                cout<< "circles1[0][0]: "  <<  circles1[0][0] << endl;
                cout<< "circles1[0][1]: "  <<  circles1[0][1] << endl;
                cout<< "circles1[0][2]: "  <<  circles1[0][2] << endl;


                history_c1.push_back(circles1[0]);
            }
        } 
        else{
            cout<< "history_c1.size() > num_pic: "<< history_c1.size() <<endl;

            history_c1.pop_front();

            
            //cvtColor(birdImage2 , birdImage2, CV_GRAY2BGR);
            cout<< "circles1.size()： " << circles1.size() << endl;
            cout<< "circles.size()： " << circles2.size() << endl;
            cout<< "circles1[0][0]: "  <<  circles1[0][0] << endl;
            cout<< "circles1[0][1]: "  <<  circles1[0][1] << endl;
            cout<< "circles1[0][2]: "  <<  circles1[0][2] << endl;

            vector<float> meancircle;
            float min = 1280.0;
            float distance1;
            int best_circle1;
            meancircle = get_CircleCenterMean(history_c1, num_pic);

            for(size_t i = 0; i < circles1.size(); i++){ 
                Vec3f c = circles1[i];
                //circle(birdimage1, Point(c[0], c[1]), c[2], Scalar(0,255,255), 1, CV_AA);
                float mean_cir_x = meancircle[0];
                float mean_cir_y = meancircle[1];
                float cir_x = circles1[i][0];
                float cir_y = circles1[i][1];
                distance1 =  lineprocess.get_P_to_P(cir_x, cir_y, mean_cir_x, mean_cir_y);
            
                if( distance1 < min ){
                    min = distance1;
                    best_circle1 = i;
                }

            }
            for(size_t i = 0; i < circles2.size(); i++) { 
                Vec3f c = circles2[i];
                //circle(birdimage1, Point(c[0], c[1]), c[2], Scalar(0,255,255), 1, CV_AA);
            }


            if(min < 20 ){
                history_c1.push_back(circles1[best_circle1]);

                cout << "draw" <<endl;
                circle(birdimage1, Point(circles1[best_circle1][0], circles1[best_circle1][1]), 
                                    circles1[best_circle1][2], Scalar(0,255,255), 1, CV_AA);


            }
        }

    }

    //计算圆心均值
    vector<float> Circle::get_CircleCenterMean(list<Vec3f> history_c1, float num_pic){
        float MeanHisCir_x, MeanHisCir_y, MeanHisCir_radius;
        vector<float> meancircle;
        for(list<Vec3f>::iterator his_c1_ifo = history_c1.begin(); his_c1_ifo != history_c1.end(); ++his_c1_ifo){
            MeanHisCir_x += (*his_c1_ifo)[0];
            MeanHisCir_y += (*his_c1_ifo)[1];
            MeanHisCir_radius += (*his_c1_ifo)[2];
        }
        MeanHisCir_x /= num_pic;
        MeanHisCir_y /= num_pic;
        MeanHisCir_radius /= num_pic;

        meancircle.push_back(MeanHisCir_x);
        meancircle.push_back(MeanHisCir_y);
        meancircle.push_back(MeanHisCir_radius);

        int u = MeanHisCir_y;

        cout<< "MeanHisCir_x: " << MeanHisCir_x <<endl;
        cout<< "MeanHisCir_y: " << MeanHisCir_y <<endl;
        cout << "int: " << u;
        cout<< "MeanHisCir_radius: " << MeanHisCir_radius <<endl;

        return meancircle;
    }
}
