//
// Created by 乔挺 on 6/17/17.
//

#include "Feature_extraction.h"

void sift_feature_extraction(Mat first,Mat second)
{
    //Keypoints
    SiftFeatureDetector detector;
    vector<cv::KeyPoint> prev_keypoints,curr_keypoints;
    detector.detect(first,prev_keypoints);
    detector.detect(second,curr_keypoints);

    //Descriptor
    Mat descriptors_1, descriptors_2;
    SiftDescriptorExtractor extractor;
    extractor.compute(first, prev_keypoints, descriptors_1);
    extractor.compute(second, curr_keypoints, descriptors_2);

    //Matcher
    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
    matcher.match( descriptors_1, descriptors_2, matches);

    //Fast Filtering
    double max_dist = 0; double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
    std::vector< DMatch > good_matches;

    for( int i = 0; i < descriptors_2.rows; i++ )
    {
        if( matches[i].distance < 5*min_dist )
        {
            good_matches.push_back( matches[i]);
        }
    }

    Mat img_matches;
    drawMatches(first, prev_keypoints, second, curr_keypoints,
                good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    printf("Good Match:%lu\b\n",good_matches.size());
}
