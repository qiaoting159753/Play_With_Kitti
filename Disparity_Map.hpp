//
//  Disparity_Map.hpp
//  BasicApp
//
//  Created by 乔挺 on 3/13/17.
//
//

#ifndef Disparity_Map_hpp
#define Disparity_Map_hpp
#include <stdio.h>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "quasidensestereo2.hpp"

using namespace std;
using namespace cv;

class Disparity_Map{
public:
    Disparity_Map(vector<Mat>pair);
    Mat get_disparity();
    pair<vector<cv::Point>,vector<cv::Point>> features;
    Mat lrectified;
    Mat rrectified;
private:
    Mat left;
    Mat right;
    Mat disp;
    Mat gamma(Mat input);
};

#endif /* Disparity_Map_hpp */
