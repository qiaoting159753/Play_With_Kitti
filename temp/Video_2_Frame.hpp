//
//  Video_2_Frame.hpp
//  BasicApp
//
//  Created by 乔挺 on 3/10/17.
//
//

#ifndef Video_2_Frame_hpp
#define Video_2_Frame_hpp
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <stdio.h>
#include <vector>
#include <iostream>

using namespace cv;
using namespace std;

class Video_2_Frame{
public:
    /*Constructor*/
    Video_2_Frame(const char * t_left,const char * t_right);
    ~Video_2_Frame();
    vector<Mat> get_frame_pair(int index);
    Mat get_left();
private:
    VideoCapture vCapLeft;
    VideoCapture vCapRight;
    vector<Mat> * leftFrames;
    vector<Mat> * rightFrames;
};
#endif /* Video_2_Frame_hpp */
