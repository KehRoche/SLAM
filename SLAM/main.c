// -------------- test the visual odometry -------------
#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "ownslam/config.h"
#include "ownslam/visual_odometry.h"
int main (int argc, char** argv)
{
	ownslam::Config::setParameterFile( argv[1]);
	ownslam::VisualOdometry::Ptr vo (new ownslam::VisualOdometry);

	//dealing with the dateset 
	string	dataset_dir = ownslam::Config::get<string> ("dateset_dir");
	cout<<"dateset:"<<dateset_dir<<endl;
	ifstream fin (dateset_dir + "/associate.txt");

	vector<string> rgd_files, depth_files;
	vector<string> rgb_times. rgb_times;

	while (!fin.eof())
	{
		string rgb_time, rgb_file, depth_time, depth_file;
		fin>>rgb. 
	}
	
}

