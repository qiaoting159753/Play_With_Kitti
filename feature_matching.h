//
// Created by 乔挺 on 6/18/17.
//

#ifndef PLAY_WITH_KITTI_FEATURE_MATCHING_H
#define PLAY_WITH_KITTI_FEATURE_MATCHING_H

#include "exif.h"

class feature_matching {
    int window_size = 10;

    bool has_gps_data(char * image);
    void match_candidates_from_metadata()
    void matching_pair();
};


#endif //PLAY_WITH_KITTI_FEATURE_MATCHING_H
