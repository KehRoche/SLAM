#ifndef MAP_H
#define MAP_H

#include"ownslam/common_include.h"
#include"ownslam/frame.h"
#include"ownslam/mappoint.h"

namespace ownslam
{
    class Map
    {
    public:
        typedef shared_ptr<Map> Ptr;
        //hash table
        unordered_map<unsigned long, MapPoint::Ptr> map_points_;
        unordered_map<unsigned long, Frame::Ptr>    keyframes_;

        Map(){}

        void insertKeyFrame(Frame::Ptr frame);
        void insertMapPoint(MapPoint::Ptr map_point);
    };

}

#endif
