#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "ownslam/config.h"
#include "ownslam/vo.h"
//#include <oepncv2/opencv.hpp>
#include "sl/Camera.hpp"

//using namespace sl::InitParameters;

cv::Mat slMat2cvMat(Mat& input);

int main ( int argc, char** argv )
{
    if ( argc != 2 )
    {
        cout<<"usage: run_vo parameter_file"<<endl;
        return 1;
    }

    ownslam::Config::setParameterFile ( argv[1] );
    ownslam::VisualOdometry::Ptr vo ( new ownslam::VisualOdometry );
    sl::Camera zed;
    sl::InitParameters initParameters;
    initParameters.depth_mode = sl::DEPTH_MODE_ULTRA;
    initParameters.coordinate_system = sl::COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP;
    initParameters.camera_resolution =sl:: RESOLUTION_HD1080;
    initParameters.camera_fps = 30;
    

    string dataset_dir = ownslam::Config::get<string> ( "dataset_dir" );
    cout<<"dataset: "<<dataset_dir<<endl;
    ifstream fin ( dataset_dir+"/associate.txt" );
    if ( !fin )
    {
        cout<<"please generate the associate file called associate.txt!"<<endl;
        return 1;
    }

    vector<Mat> rgb_files, depth_files;

    sl::ERROR_CODE err = zed.open(initParameters);
    if (err != sl::SUCCESS)
        exit(-1);

    sl::RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = sl::SENSING_MODE_STANDARD; // Use STANDARD sensing mode

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
        sl::Mat Depth;
        sl::Mat Color;
        if (zed.grab() == sl::SUCCESS) 
        {
          // A new image and depth is available if grab() returns SUCCESS
          zed.retrieveImage(Color, sl::VIEW_LEFT); // Retrieve left image
          zed.retrieveMeasure(Depth, sl::MEASURE_DEPTH); // Retrieve depth
        }
       
       cout<<"****** loop "<<i<<" ******"<<endl;
       
       cv::Mat color =slMat2cvMat(Depth);
       cv::Mat depth =slMat2cvMat(Color);
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
   }

    return 0;
}
cv::Mat slMat2cvMat(const sl::Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;


    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}
