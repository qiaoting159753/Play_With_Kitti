#include <list>
#include "Video_2_Frame.hpp"
#include "Disparity_Map.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <vector>
//#include <gtsam/geometry/Pose2.h>
//#include <Eigen/Dense>

using namespace std;
//using namespace cv;

int main(){

//    /*Loading Videos*/
//    const char * left = "/Users/qiaoting/Documents/Dissertation/cinder_0.9.0_mac/samples/BasicApp/assets/left.mov";
//    const char * right = "/Users/qiaoting/Documents/Dissertation/cinder_0.9.0_mac/samples/BasicApp/assets/right.mov";
//    /*Convert Video to frame pairs*/
//    Video_2_Frame v2f = * new Video_2_Frame(left,right);
//    vector<Mat> first_pair = v2f.get_frame_pair(1);
//
//    /*Calculate First Disparity*/
//    Disparity_Map first_disp_map = * new Disparity_Map(first_pair);
//    cv::Mat first_result = first_disp_map.get_disparity();
//    std::pair<vector<cv::Point>,vector<cv::Point>> first_features = first_disp_map.features;
//
//        namedWindow("Good Matches");
//        imshow( "Good Matches", first_result );
//        cvWaitKey(100000);
//
//
//
//    /*Calculate Second Disparity*/
//    vector<Mat> second_pair = v2f.get_frame_pair(5);
//    Disparity_Map second_disp_map = * new Disparity_Map(second_pair);
//    cv::Mat second_result = second_disp_map.get_disparity();
//    std::pair<vector<cv::Point>,vector<cv::Point>> second_features = second_disp_map.features;
//
//    /*Feature Matching*/
//    //Build keypoint
//    SiftFeatureDetector detector;
//    vector<cv::KeyPoint>prev_keypoints,curr_keypoints;
//    detector.detect(first_disp_map.lrectified,prev_keypoints);
//    detector.detect(second_disp_map.lrectified,curr_keypoints);
//
//    Mat temp;
//
//    //Build Descriptor
//    Mat descriptors_1, descriptors_2;
//    SiftDescriptorExtractor extractor;
//    extractor.compute(first_disp_map.lrectified, prev_keypoints, descriptors_1);
//    extractor.compute(second_disp_map.lrectified, curr_keypoints, descriptors_2);
//
//    //Matcher
//    FlannBasedMatcher matcher;
//    std::vector< DMatch > matches;
//    matcher.match( descriptors_1, descriptors_2, matches);
//
//    double max_dist = 0; double min_dist = 100;
//
//    //-- Quick calculation of max and min distances between keypoints
//    for( int i = 0; i < descriptors_1.rows; i++ )
//    {
//        double dist = matches[i].distance;
//        if( dist < min_dist ) min_dist = dist;
//        if( dist > max_dist ) max_dist = dist;
//    }
//
//    printf("-- Max dist : %f \n", max_dist );
//    printf("-- Min dist : %f \n", min_dist );
//
//    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
//    std::vector< DMatch > good_matches;
//
//    for( int i = 0; i < descriptors_2.rows; i++ )
//    {
//        if( matches[i].distance < 5*min_dist )
//        {
//            good_matches.push_back( matches[i]);
//        }
//    }
//
//
//    Mat img_matches;
//    drawMatches(first_disp_map.lrectified, prev_keypoints, second_disp_map.lrectified, curr_keypoints,
//                good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
//                vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
//    printf("Good Match:%lu\b\n",good_matches.size());
//
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
//    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
//    line( img_matches, scene_corners[0] + Point2f( first_disp_map.lrectified.cols, 0), scene_corners[1] + Point2f( first_disp_map.lrectified.cols, 0), Scalar(0, 255, 0), 4 );
//    line( img_matches, scene_corners[1] + Point2f( first_disp_map.lrectified.cols, 0), scene_corners[2] + Point2f( first_disp_map.lrectified.cols, 0), Scalar( 0, 255, 0), 4 );
//    line( img_matches, scene_corners[2] + Point2f( first_disp_map.lrectified.cols, 0), scene_corners[3] + Point2f( first_disp_map.lrectified.cols, 0), Scalar( 0, 255, 0), 4 );
//    line( img_matches, scene_corners[3] + Point2f( first_disp_map.lrectified.cols, 0), scene_corners[0] + Point2f( first_disp_map.lrectified.cols, 0), Scalar( 0, 255, 0), 4 );
//
//
//
//
//    //Derive Essentail Matrix E = A2T * F * A1
//    Mat l_intrinsic = (Mat_<double>(3,3) << 391.656525,0.0       ,165.964371,
//                       0.0       ,426.835144,154.498138,
//                       0.0       ,0.0       ,1.0);
//
//    Mat r_intrinsic = (Mat_<double>(3,3) << 390.376862,0.0       ,190.896454,
//                       0.0       ,426.228882,145.071411,
//                       0.0       ,0.0       ,1.0);
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
//
//    Mat W = (Mat_<double>(3,3) << 0.0,-1.0,0.0,
//                                  1.0, 0.0,0.0,
//                                  0.0, 0.0,1.0);
    
//    printf("%d %d\n",L.cols,L.rows);
//    
//    gemm(U,W.inv(),1,NULL,0,temp);
//    gemm(temp,VT,1,NULL,0,rot1);
//    
//    gemm(U,L,1, NULL, 0, temp);
//    gemm(temp,W,1,NULL,0,temp);
//    gemm(temp,U.t(),1,NULL,0,tau1);
//    
//    W = W.inv();
//    gemm(U,W.inv(),1,NULL,0,temp);
//    gemm(temp,VT,1,NULL,0,rot2);
//    printf("%d %d\n",tau1.cols,tau1.rows);
//    //rcout << tau1 << endl;
//    //Drawing
//    
//    //-- Show detected matches
//    namedWindow("Good Matches");
//    imshow( "Good Matches", img_matches );
//    cvWaitKey(100000);
//    imshow( "Good Matches & Object detection", img_matches );
    

}

