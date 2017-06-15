//
//  Video_2_Frame.cpp
//  BasicApp
//
//  Created by 乔挺 on 3/10/17.
//
//

#include "Video_2_Frame.hpp"

Video_2_Frame::Video_2_Frame(const char * t_left,const char * t_right){
    /*Load the Video File*/
    //this->left = cv::String(t_left);
    //this->right = cv::String(t_right);
    
    vCapLeft.open(t_left);
    vCapRight.open(t_right);
    
    if (!(vCapLeft.isOpened() && vCapRight.isOpened()))
    {
        printf("Video Not Opened !\n");
    }
    
    /*Convert the frames to Images with time*/
    
    //size_t numOfFramesL = vCapLeft.get(CV_CAP_PROP_FRAME_COUNT);
    //size_t numOfFramesR = vCapRight.get(CV_CAP_PROP_FRAME_COUNT);
    size_t numOfFramesL = 10;
    size_t numOfFramesR = 10;
    
    leftFrames = new vector<Mat>;
    rightFrames = new vector<Mat>;
    
    for (int index = 0;index<numOfFramesL;index++)
    {
        cv::Mat frame = * new Mat;
        vCapLeft.read(frame);
        leftFrames->push_back(frame);

    }
    for (int index = 0;index<numOfFramesR;index++)
    {
        cv::Mat frame = * new Mat;
        vCapRight.read(frame);
        rightFrames->push_back(frame);
    }
};

vector<Mat> Video_2_Frame::get_frame_pair(int index)
{
    vector<Mat> temp = * new vector<Mat>;
    temp.push_back(leftFrames->at(index));
    temp.push_back(rightFrames->at(index));
    return temp;
}

Video_2_Frame::~Video_2_Frame(){
    /*Free the resource*/
    vCapRight.release();
    vCapLeft.release();
    leftFrames->clear();
    rightFrames->clear();
};

