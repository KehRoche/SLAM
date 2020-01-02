#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "ownslam/config.h"
#include "ownslam/vo.h"
//#include <oepncv2/opencv.hpp>
#include <s1/Camera.hpp>

using namespace s1::InitParameters;

int main ( int argc, char** argv )
{
    if ( argc != 2 )
    {
        cout<<"usage: run_vo parameter_file"<<endl;
        return 1;
    }

    ownslam::Config::setParameterFile ( argv[1] );
    ownslam::VisualOdometry::Ptr vo ( new ownslam::VisualOdometry );
    s1::Camera zed;
    s1::InitParameters initParameters;
    initParameters.depth_mode = DEPTH_MODE_ULTRA;
    initParameters.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP;
    init_params.camera_resolution = RESOLUTION_HD1080;
    init_params.camera_fps = 30;
    

    string dataset_dir = ownslam::Config::get<string> ( "dataset_dir" );
    cout<<"dataset: "<<dataset_dir<<endl;
    ifstream fin ( dataset_dir+"/associate.txt" );
    if ( !fin )
    {
        cout<<"please generate the associate file called associate.txt!"<<endl;
        return 1;
    }

    vector<Mat> rgb_files, depth_files;

    cv::VideoCapture capture(0);//初始化USB通道1的摄像头

    ownslam::Camera::Ptr camera ( new ownslam::Camera );

   for ( int i=0; i<rgb_files.size(); i++ )
   {
      

 /*       while(1)
      {
          Mat originalframe;
          capture>>originalframe;
          if(originalframe.empty())
          {
              break;
          }
          else
          {
              Mat color = originalframe.clone();
          }
      } */
        s1::Mat depth;
        s1::Mat color;
        if (zed.grab() == SUCCESS) 
        {
          // A new image and depth is available if grab() returns SUCCESS
          zed.retrieveImage(color, VIEW_LEFT); // Retrieve left image
          zed.retrieveMeasure(depth, MEASURE_DEPTH); // Retrieve depth
        }
       
       cout<<"****** loop "<<i<<" ******"<<endl;
       if ( color.data==nullptr || depth.data==nullptr )
           break;
       ownslam::Frame::Ptr pFrame = ownslam::Frame::createFrame();
       pFrame->camera_ = camera;
       pFrame->color_ = color;
       pFrame->depth_ = depth;
       //pFrame->time_stamp_ = rgb_times[i];

       boost::timer timer;
       vo->addFrame ( pFrame );
       cout<<"VO costs time: "<<timer.elapsed() <<endl;

       if ( vo->state_ == ownslam::VisualOdometry::LOST )
           break;
       SE3 Twc = pFrame->T_c_w_.inverse();

       Mat img_show = color.clone();
       for ( auto& pt:vo->map_->map_points_ )
       {
           ownslam::MapPoint::Ptr p = pt.second;
           Vector2d pixel = pFrame->camera_->world2pixel ( p->pos_, pFrame->T_c_w_ );
           cv::circle ( img_show, cv::Point2f ( pixel ( 0,0 ),pixel ( 1,0 ) ), 5, cv::Scalar ( 0,255,0 ), 2 );
       }

       cv::imshow ( "image", img_show );
       cv::waitKey ( 1 );
       Mat originalframe;
       capture>>originalframe;
       cv::imshow("camera test",originalframe);
       cout<<endl;
   }

    return 0;
}
