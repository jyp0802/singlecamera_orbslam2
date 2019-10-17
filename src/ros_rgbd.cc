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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "geometry_msgs/TransformStamped.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>

#include<opencv2/core/core.hpp>

#include <darknet_ros_msgs/BoundingBoxes.h>

#include"System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD,const darknet_ros_msgs::BoundingBoxes::ConstPtr& msgObjects);

    ORB_SLAM2::System* mpSLAM;

    std::string from_tf;
    std::string to_tf;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rgbd_slam");
    ros::start();

    if(argc != 5)
    {
        cerr << endl << "Usage: rosrun singlecamera_orbslam2 rgbd_slam path_to_vocabulary path_to_settings from_tf to_tf" << endl;        
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    igb.from_tf = argv[3];
    igb.to_tf = argv[4];

    ros::NodeHandle nh;

    // message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/darknet_ros/original_image", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_rect_raw", 1);
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> object_sub(nh, "/darknet_ros/bounding_boxes", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub,object_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2,_3));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

tf::Quaternion hamiltonProduct(tf::Quaternion a, tf::Quaternion b) {

	tf::Quaternion c;

		c[0] = (a[0]*b[0]) - (a[1]*b[1]) - (a[2]*b[2]) - (a[3]*b[3]);
		c[1] = (a[0]*b[1]) + (a[1]*b[0]) + (a[2]*b[3]) - (a[3]*b[2]);
		c[2] = (a[0]*b[2]) - (a[1]*b[3]) + (a[2]*b[0]) + (a[3]*b[1]);
		c[3] = (a[0]*b[3]) + (a[1]*b[2]) - (a[2]*b[1]) + (a[3]*b[0]);

	return c;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD,const darknet_ros_msgs::BoundingBoxes::ConstPtr& msgObjects)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    int n = msgObjects->bounding_boxes.size();
    int x, y, w, h;
    std::vector<cv::Rect> objects;
    objects.reserve(n);
    for (int i = 0; i < n; i++)
    {
        x = msgObjects->bounding_boxes[i].xmin;
        y = msgObjects->bounding_boxes[i].ymin;
        w = msgObjects->bounding_boxes[i].xmax - x;
        h = msgObjects->bounding_boxes[i].ymax - y;
        objects.push_back(cv::Rect(x, y, w, h));
    }

    cv::Mat pose = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec(),objects);

    if (pose.empty())
        return;

    tf::Matrix3x3 tf3d;
	tf3d.setValue(pose.at<float>(0,0), pose.at<float>(0,1), pose.at<float>(0,2),
			pose.at<float>(1,0), pose.at<float>(1,1), pose.at<float>(1,2),
			pose.at<float>(2,0), pose.at<float>(2,1), pose.at<float>(2,2));

    tf::Quaternion tfqt;
	tf3d.getRotation(tfqt);
	double aux = tfqt[0];
		tfqt[0]=-tfqt[2];
		tfqt[2]=tfqt[1];
		tfqt[1]=aux;

    //Translation for camera
	tf::Vector3 origin;
	origin.setValue(pose.at<float>(0,3),pose.at<float>(1,3),pose.at<float>(2,3));
	//rotate 270deg about x and 270deg about x to get ENU: x forward, y left, z up
	const tf::Matrix3x3 rotation270degXZ(   0, 1, 0,
											0, 0, 1,
											-1, 0, 0);

    tf::Vector3 translationForCamera = origin * rotation270degXZ;

	//Hamilton (Translation for world)
	tf::Quaternion quaternionForHamilton(tfqt[3], tfqt[0], tfqt[1], tfqt[2]);
	tf::Quaternion secondQuaternionForHamilton(tfqt[3], -tfqt[0], -tfqt[1], -tfqt[2]);
	tf::Quaternion translationHamilton(0, translationForCamera[0], translationForCamera[1], translationForCamera[2]);

	tf::Quaternion translationStepQuat;
	translationStepQuat = hamiltonProduct(hamiltonProduct(quaternionForHamilton, translationHamilton), secondQuaternionForHamilton);

	tf::Vector3 translation(translationStepQuat[1], translationStepQuat[2], translationStepQuat[3]);

    //Creates transform and populates it with translation and quaternion
	tf::Transform transformCurrent;
	transformCurrent.setOrigin(translation);
	transformCurrent.setRotation(tfqt);
	//Publishes transform
	static tf::TransformBroadcaster br;
    br.sendTransform(tf::StampedTransform(transformCurrent, ros::Time::now(), from_tf, to_tf));
}


