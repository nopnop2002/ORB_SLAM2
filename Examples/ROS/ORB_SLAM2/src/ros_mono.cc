/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
	printf("main start\n");
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

	// bUseViewer = false
    //ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,false);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);

	printf("spin start\n");
    ros::spin();
	printf("spin end\n");

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

tf2::Transform TransformFromMat (cv::Mat position_mat) {
  cv::Mat rotation(3,3,CV_32F);
  cv::Mat translation(3,1,CV_32F);

  rotation = position_mat.rowRange(0,3).colRange(0,3);
  translation = position_mat.rowRange(0,3).col(3);


  tf2::Matrix3x3 tf_camera_rotation (rotation.at<float> (0,0), rotation.at<float> (0,1), rotation.at<float> (0,2),
                                    rotation.at<float> (1,0), rotation.at<float> (1,1), rotation.at<float> (1,2),
                                    rotation.at<float> (2,0), rotation.at<float> (2,1), rotation.at<float> (2,2)
                                   );

  tf2::Vector3 tf_camera_translation (translation.at<float> (0), translation.at<float> (1), translation.at<float> (2));

  //Coordinate transformation matrix from orb coordinate system to ros coordinate system
  const tf2::Matrix3x3 tf_orb_to_ros (0, 0, 1,
                                    -1, 0, 0,
                                     0,-1, 0);

  //Transform from orb coordinate system to ros coordinate system on camera coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  //Inverse matrix
  tf_camera_rotation = tf_camera_rotation.transpose();
  tf_camera_translation = -(tf_camera_rotation*tf_camera_translation);

  //Transform from orb coordinate system to ros coordinate system on map coordinates
  tf_camera_rotation = tf_orb_to_ros*tf_camera_rotation;
  tf_camera_translation = tf_orb_to_ros*tf_camera_translation;

  return tf2::Transform (tf_camera_rotation, tf_camera_translation);
}

geometry_msgs::Transform ConvertPositionToTransform (std_msgs::Header header, cv::Mat position) {
	tf2::Transform tf_position = TransformFromMat(position);
	//geometry_msgs::PoseStamped pose_msg;
	geometry_msgs::Transform trans_msg;
	//trans_msg.header = header;
	trans_msg = tf2::toMsg(tf_position); 
	//std::cout << "trans_msg=" << trans_msg << std::endl << std::endl;
	return trans_msg;
}

geometry_msgs::TransformStamped ConvertPositionToTransformStamped (std_msgs::Header header, cv::Mat position, string child_frame_id) {
	tf2::Transform tf_position = TransformFromMat(position);
	//geometry_msgs::PoseStamped pose_msg;
	geometry_msgs::TransformStamped trans_msg;
	trans_msg.header = header;
	trans_msg.transform = tf2::toMsg(tf_position); 
	trans_msg.child_frame_id = child_frame_id;
	//std::cout << "trans_msg=" << trans_msg << std::endl << std::endl;
	return trans_msg;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Proccess the given monocular frame
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
	// cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);
	printf("TrackMonocular timestamp=%f\n", cv_ptr->header.stamp.toSec());
    //mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    cv::Mat m = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
	// camera pose として 4x4 の配列が戻る
	std::cout << "m=" << m << std::endl << std::endl;
	if (!m.empty()) {
		// Convert to geometry_msgs::Transform
		geometry_msgs::Transform transform;
		transform = ConvertPositionToTransform(cv_ptr->header, m);
		std::cout << "transform=" << transform << std::endl << std::endl;

		// Convert to geometry_msgs::TransformStamped
		geometry_msgs::TransformStamped transformStamped;
		transformStamped = ConvertPositionToTransformStamped(cv_ptr->header, m, "/child");
		std::cout << "transformStamped=" << transformStamped << std::endl << std::endl;

		// Build to geometry_msgs::Pose
		geometry_msgs::Pose Pose;
		Pose.position.x = transform.translation.x;
		Pose.position.y = transform.translation.y;
		Pose.position.z = transform.translation.z;
		Pose.orientation = transform.rotation;

		// Build to geometry_msgs::PoseStamped
		geometry_msgs::PoseStamped PoseStamped;
		PoseStamped.header = transformStamped.header;
		PoseStamped.pose = Pose;
	}	
}


