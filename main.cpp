#include <list>
#include "Disparity_Map.hpp"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

int main(){

    /*Loading Image*/
    string left_path = "/Users/qiaoting/GitHub/Play_With_Kitti/assets/image_02/data/";
    string right_path = "/Users/qiaoting/GitHub/Play_With_Kitti/assets/image_03/data/";

    /*Loop all the frames*/
    size_t frame_start = 1;
    size_t frame_end = 2;
    size_t step = 3;
    for(int i = frame_start;i < frame_end;i+=step)
    {
        /*Load the current image pair*/
        string prefix;
        string temp = std::to_string(i);
        if (i < 10)
        {
            prefix = "000000000";
        }else
        {
            if(i < 100)
            {
                prefix = "00000000";
            }
        }

        string suffix = ".png";
        string file = prefix + temp +suffix;

        Mat left_image,right_image;
        left_image = imread((left_path + file).c_str(), CV_LOAD_IMAGE_COLOR);
        //cout << (left_path + file).c_str() << endl;
        right_image = imread(right_path + file,CV_LOAD_IMAGE_COLOR);

        if(! ((left_image.data)&&(right_image.data)) )                              // Check for invalid input
        {
            cout <<  "Could not open or find the image" << std::endl ;
            return -1;
        }
        vector<Mat> first_pair;
        first_pair.push_back(left_image);
        first_pair.push_back(right_image);

        /*Calculate First Disparity*/
        //Disparity_Map first_disp_map = * new Disparity_Map(first_pair);
        //cv::Mat first_result = first_disp_map.get_disparity();
        //std::pair<vector<cv::Point>,vector<cv::Point>> first_features = first_disp_map.features;

    }

//    //-- Localize the object
//    std::vector<Point2f> obj;
//    std::vector<Point2f> scene;
//
//    for( int i = 0; i < good_matches.size(); i++ )
//    {
//        //-- Get the keypoints from the good matches
//        obj.push_back( prev_keypoints[ good_matches[i].queryIdx ].pt );
//        scene.push_back( curr_keypoints[ good_matches[i].trainIdx ].pt );
//    }
//
//    Mat F = findFundamentalMat(obj, scene,temp);
//    //Refine Matching
//    correctMatches(F,obj,scene,obj,scene);
//    //-- Get the corners from the image_1 ( the object to be "detected" )
//    std::vector<Point2f> obj_corners(4);
//
//    obj_corners[0] = cvPoint(0,0);
//    obj_corners[1] = cvPoint( first_disp_map.lrectified.cols, 0 );
//    obj_corners[2] = cvPoint( first_disp_map.lrectified.cols, first_disp_map.lrectified.rows );
//    obj_corners[3] = cvPoint( 0, first_disp_map.lrectified.rows );
//
//    std::vector<Point2f> scene_corners(4);
//
//    perspectiveTransform( obj_corners,scene_corners,F);
//
//    //-- Draw lines between the features (the mapped object in the scene - image_2 )
//    line( img_matches, scene_corners[0] + Point2f( first_disp_map.lrectified.cols, 0), scene_corners[1] + Point2f( first_disp_map.lrectified.cols, 0), Scalar(0, 255, 0), 4 );
//    line( img_matches, scene_corners[1] + Point2f( first_disp_map.lrectified.cols, 0), scene_corners[2] + Point2f( first_disp_map.lrectified.cols, 0), Scalar( 0, 255, 0), 4 );
//    line( img_matches, scene_corners[2] + Point2f( first_disp_map.lrectified.cols, 0), scene_corners[3] + Point2f( first_disp_map.lrectified.cols, 0), Scalar( 0, 255, 0), 4 );
//    line( img_matches, scene_corners[3] + Point2f( first_disp_map.lrectified.cols, 0), scene_corners[0] + Point2f( first_disp_map.lrectified.cols, 0), Scalar( 0, 255, 0), 4 );


//    //Derive Essentail Matrix E = A2T * F * A1
//    Mat l_intrinsic = (Mat_<double>(3,3) << 391.656525,0.0       ,165.964371,
//            0.0       ,426.835144,154.498138,
//            0.0       ,0.0       ,1.0);
//
//    Mat r_intrinsic = (Mat_<double>(3,3) << 390.376862,0.0       ,190.896454,
//            0.0       ,426.228882,145.071411,
//            0.0       ,0.0       ,1.0);
//
//    Mat A2T;
//    transpose(r_intrinsic,A2T);
//
//    Mat tmpMat,E;
//    gemm (A2T.t(),F          ,1,NULL,0,tmpMat,GEMM_3_T);
//    gemm (tmpMat, l_intrinsic,1,NULL,0,E     ,GEMM_3_T);
//
//    //Decomposite Essential Matrix, There are 4 possible solutions
//    Mat L,U,VT;
//    SVDecomp(E,L,U,VT,cv::SVD::FULL_UV);
//    Mat tau1,rot1,rot2;

//    Mat W = (Mat_<double>(3,3) << 0.0,-1.0,0.0,
//            1.0, 0.0,0.0,
//            0.0, 0.0,1.0);



}

