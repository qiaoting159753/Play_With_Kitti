//
//  Disparity_Map.cpp
//  BasicApp
//
//  Created by 乔挺 on 3/13/17.
//
//
#include "Disparity_Map.hpp"
//#include "opencv2/contrib/contrib.hpp"



Disparity_Map::Disparity_Map(vector<Mat>pair)
{
    this->left = pair.at(0);
    this->right = pair.at(1);
}

Mat Disparity_Map::get_disparity()
{
    //RGB 2 Gray
    cv::Mat left_grey,right_grey;
    cv::cvtColor(left, left_grey, CV_RGB2GRAY);
    cv::cvtColor(right,right_grey, CV_RGB2GRAY);
    /*---------------------------------Quasi Dense Matching-----------------------------------*/
    QuasiDenseStereo2 tempQuasi = * new QuasiDenseStereo2;
    tempQuasi.Param.Tt = 255;
    tempQuasi.Param.BorderX = 3;				// borders around the image
    tempQuasi.Param.BorderY = 3;
    tempQuasi.Param.N = 7;						// neighbours
    tempQuasi.Param.Ct = 0.2;					// corre threshold for seeds
    tempQuasi.Param.Dg = 2;                   // disparity gradient
    tempQuasi.Param.WinSizeX = 5;				// ZNCC corr window size
    tempQuasi.Param.WinSizeY = 5;
    tempQuasi.process(left_grey,right_grey);
    
    disp = left.clone();
    features = tempQuasi.getDisparityImage(&disp);
    namedWindow("Hello");

    Mat channels[3];
    split(disp, channels);
    imshow("Hello",channels[0]);
    cvWaitKey(100000);
    return disp;
}
