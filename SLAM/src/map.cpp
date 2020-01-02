#include"ownslam/map.h"

namespace ownslam
{

    void Map::insertKeyFrame(Frame::Ptr frame)
    {
        cout<<"key frame size = "<<keyframes_.size()<<endl;
        //judge if the frame you want to insert already in the map
        if(keyframes_.find(frame->id_)==keyframes_.end())
        {   //not exist then insert
            //make pair combine keyindex and value as a struct
            keyframes_.insert(make_pair(frame->id_,frame));
        }
        else
        {
            //already in, there refresh it
            keyframes_[ frame->id_] = frame;
        }
    }
    void Map::insertMapPoint ( MapPoint::Ptr map_point )
    {
        if ( map_points_.find(map_point->id_) == map_points_.end() )
        {
            map_points_.insert( make_pair(map_point->id_, map_point) );
        }
        else
        {
            map_points_[map_point->id_] = map_point;
        }
    }
}
