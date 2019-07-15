#ifndef CIRCLE_H
#define CIRCLE_H
#include <laneloc/laneloc_nodelet.h>
#include <laneloc/LineProcess.h>

using namespace std;
using namespace cv;

namespace laneloc
{
    class Circle{
        public:
            LineProcess lineprocess;
            
            void Cirle_Process(list<Vec3f> &history_c1, Mat birdImage, Mat &birdimage1);
            vector<float> get_CircleCenterMean(list<Vec3f> history_c1, float num_pic);
    };

}
#endif