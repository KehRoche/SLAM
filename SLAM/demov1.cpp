#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "ownslam/config.h"
#include "ownslam/vo.h"
//#include <oepncv2/opencv.hpp>
#include "sl/Camera.hpp"


#include <octomap/octomap.h>    // for octomap 

//using namespace sl::InitParameters;

cv::Mat slMat2cvMat(const sl::Mat& input);

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
    

  sl::ERROR_CODE err = zed.open(initParameters);
    while (err != sl::SUCCESS)
{        
	cout<<"camera open failed"<<endl;	
	sleep(2);
        err = zed.open(initParameters);
}

   	    sl::CalibrationParameters calibration_params = zed.getCameraInformation().calibration_parameters;
    float focal_left_x = calibration_params.left_cam.fx;
    float focal_left_y = calibration_params.left_cam.fy;
    float points_left_x = calibration_params.left_cam.cx;
    float points_left_y = calibration_params.left_cam.cy;
    cout<<focal_left_x<<endl<<focal_left_y<<endl<<points_left_x<<endl<<points_left_y<<endl;    


    sl::RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = sl::SENSING_MODE_STANDARD; // Use STANDARD sensing mode

    ownslam::Camera::Ptr camera ( new ownslam::Camera );

    // visualization
    cv::viz::Viz3d vis ( "Visual Odometry" );
    cv::viz::WCoordinateSystem world_coor ( 1.0 ), camera_coor ( 0.5 );
    cv::Point3d cam_pos ( 0, -1.0, -1.0 ), cam_focal_point ( 0,0,0 ), cam_y_dir ( 0,1,0 );
    cv::Affine3d cam_pose = cv::viz::makeCameraPose ( cam_pos, cam_focal_point, cam_y_dir );
    vis.setViewerPose ( cam_pose );

    world_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 2.0 );
    camera_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 1.0 );
    vis.showWidget ( "World", world_coor );
    vis.showWidget ( "Camera", camera_coor );
    
    
    sl::Resolution image_size = zed.getResolution();
    int new_width = image_size.width / 2;
    int new_height = image_size.height / 2;

    // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
    // Only the headers and pointer to the sl::Mat are copied, not the data itself
    sl::Mat Depth(new_width, new_height, sl::MAT_TYPE_8U_C4);
    sl::Mat Color(new_width, new_height,sl::MAT_TYPE_8U_C4);
  cout<<"camera open successed"<<endl;
  
  octomap::OcTree tree( 0.05 ); 
   for ( int i=0; i<200; i++ )
   {
      

        if (zed.grab() == sl::SUCCESS) 
        {
          // A new image and depth is available if grab() returns SUCCESS
          zed.retrieveImage(Color, sl::VIEW_LEFT); // Retrieve left image
          zed.retrieveMeasure(Depth, sl::MEASURE_DEPTH); // Retrieve depth
        }
       
       cout<<"****** loop "<<i<<" ******"<<endl;
       
       cv::Mat color =slMat2cvMat(Color);
       cv::Mat depth =slMat2cvMat(Depth);
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
        
        octomap::Pointcloud cloud;  // the point cloud in octomap         for ( int v=0; v<color.rows; v++ )
        for ( int v=0; v<color.rows; v++ )  
        {
            for ( int u=0; u<color.cols; u++ )
            {
                unsigned int d = depth.ptr<unsigned short> ( v )[u]; // 深度值
                if ( d==0 ) continue; // 为0表示没有测量到
                if ( d >= 7000 ) continue; // 深度太大时不稳定，去掉
                Eigen::Vector3d point; 
                point[2] = double(d)/camera.depth_scale; 
                point[0] = (u-points_left_x)*point[2]/focal_left_x;
                point[1] = (v-points_left_y)*point[2]/focal_left_y; 
                Eigen::Vector3d pointWorld = Twc*point;
                // 将世界坐标系的点放入点云
                cloud.push_back( pointWorld[0], pointWorld[1], pointWorld[2] ); 
            }
        }    
            tree.insertPointCloud( cloud, octomap::point3d( Twc(0,3), Twc(1,3), T_c_w_(2,3) ) );     

        cv::Affine3d M (
            cv::Affine3d::Mat3 (
                Twc.rotation_matrix() ( 0,0 ), Twc.rotation_matrix() ( 0,1 ), Twc.rotation_matrix() ( 0,2 ),
                Twc.rotation_matrix() ( 1,0 ), Twc.rotation_matrix() ( 1,1 ), Twc.rotation_matrix() ( 1,2 ),
                Twc.rotation_matrix() ( 2,0 ), Twc.rotation_matrix() ( 2,1 ), Twc.rotation_matrix() ( 2,2 )
            ),
            cv::Affine3d::Vec3 (
                Twc.translation() ( 0,0 ), Twc.translation() ( 1,0 ), Twc.translation() ( 2,0 )
            )
        );

       Mat img_show = color.clone();
       for ( auto& pt:vo->map_->map_points_ )
       {
           ownslam::MapPoint::Ptr p = pt.second;
           Vector2d pixel = pFrame->camera_->world2pixel ( p->pos_, pFrame->T_c_w_ );
           cv::circle ( img_show, cv::Point2f ( pixel ( 0,0 ),pixel ( 1,0 ) ), 5, cv::Scalar ( 0,255,0 ), 2 );
       }

       //cv::imshow ( "image", img_show );
       //cv::waitKey ( 1 );
	vis.setWidgetPose ( "Camera", M );
        vis.spinOnce ( 1, false );
        cout<<endl;
   }

    tree.updateInnerOccupancy();
    cout<<"saving octomap ... "<<endl;
    tree.writeBinary( "octomap.bt" );
    return 0;
}
cv::Mat slMat2cvMat(const sl::Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = CV_8UC4;


    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}
