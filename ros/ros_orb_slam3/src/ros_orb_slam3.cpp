/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <chrono>
#include <vector>
#include <queue>
#include <csignal>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/core/core.hpp>

#include <System.h>
#include <ImuTypes.h>

#include "publish.h"
#include "imu_grabber.h"
#include "image_grabber.h"


volatile bool is_run = true;

void handler(int s)
{
	std::cout << std::endl << "stopping..." << std::endl;
	is_run = false;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "orb_slam3_ros");
	ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
	if ((!ros::master::check()))
	{
		std::cout << "ROS master not started" << std::endl;
		return 1;
	}
	//
	if (argc != 2)
	{
		cerr << endl << "Usage: ros_orb_slam3 path_to_settings" << endl;
		ros::shutdown();
		return 1;
	}
	// Settings.
	cv::FileStorage fsSettings(argv[1], cv::FileStorage::READ);
	if (!fsSettings.isOpened())
	{
		cerr << "ERROR: Wrong path to settings" << endl;
		return -1;
	}

	std::string vocabulary;
	bool do_rectify = false;
	bool do_equalize = false;
	bool useIMU = false;
	bool logTime = false;
	fsSettings["vocabulary"] >> vocabulary;
	fsSettings["do_rectify"] >> do_rectify;
	fsSettings["do_equalize"] >> do_equalize;
	fsSettings["useIMU"] >> useIMU;
	fsSettings["logTime"] >> logTime;

	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM3::System::eSensor slam_type;
	if (useIMU)
		slam_type = ORB_SLAM3::System::eSensor::IMU_STEREO;
	else
		slam_type = ORB_SLAM3::System::eSensor::STEREO;
	ORB_SLAM3::System slam(vocabulary, argv[1], slam_type);

	ImuGrabber imugb;
	ImageGrabber igb(do_rectify, do_equalize);
	
	if (igb.do_rectify)
	{
		cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
		fsSettings["LEFT.K"] >> K_l;
		fsSettings["RIGHT.K"] >> K_r;

		fsSettings["LEFT.P"] >> P_l;
		fsSettings["RIGHT.P"] >> P_r;

		fsSettings["LEFT.R"] >> R_l;
		fsSettings["RIGHT.R"] >> R_r;

		fsSettings["LEFT.D"] >> D_l;
		fsSettings["RIGHT.D"] >> D_r;

		int rows_l = fsSettings["LEFT.height"];
		int cols_l = fsSettings["LEFT.width"];
		int rows_r = fsSettings["RIGHT.height"];
		int cols_r = fsSettings["RIGHT.width"];

		if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
			rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
		{
			cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
			return -1;
		}

		cv::initUndistortRectifyMap(K_l, D_l ,R_l, P_l.rowRange(0,3).colRange(0,3), cv::Size(cols_l, rows_l), CV_32F, igb.M1l, igb.M2l);
		cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0,3).colRange(0,3), cv::Size(cols_r, rows_r), CV_32F, igb.M1r, igb.M2r);
	}

	// ROS
	ros::NodeHandle n;

	std::string topic_imu, topic_left, topic_right;
	fsSettings["ROS.topic_imu"] >> topic_imu;
	fsSettings["ROS.topic_left"] >> topic_left;
	fsSettings["ROS.topic_right"] >> topic_right;

	ros::Subscriber sub_imu = n.subscribe(topic_imu, 1000, &ImuGrabber::GrabImu, &imugb); 
	ros::Subscriber sub_img_left = n.subscribe(topic_left, 10, &ImageGrabber::GrabImageLeft, &igb);
	ros::Subscriber sub_img_right = n.subscribe(topic_right, 10, &ImageGrabber::GrabImageRight, &igb);

	publish::pose_pub = n.advertise<geometry_msgs::PoseStamped>("/orb_slam3_ros/camera", 1);
	publish::map_points_pub = n.advertise<sensor_msgs::PointCloud2>("/orb_slam3_ros/map_points", 1);

	std::signal(SIGINT, handler);
	std::cout << std::endl << "orb_slam3_ros started." << std::endl;

	// Main loop.
	cv::Mat imLeft, imRight;
	double tImLeft = 0, tImRight = 0;
	std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
	while (is_run && ros::ok())
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		ros::spinOnce();

		std::queue<sensor_msgs::ImageConstPtr>& imgLeftBuf = igb.imgLeftBuf;
		std::queue<sensor_msgs::ImageConstPtr>& imgRightBuf = igb.imgRightBuf;
		std::queue<sensor_msgs::ImuConstPtr>& imuBuf = imugb.imuBuf;

		// Sync stereo pair.
		if (imgLeftBuf.empty() || imgRightBuf.empty())
			continue;
		tImLeft = imgLeftBuf.front()->header.stamp.toSec();
		tImRight = imgRightBuf.front()->header.stamp.toSec();
		while (std::abs(tImLeft - tImRight) > igb.maxTimeDiff && imgRightBuf.size() > 1)
		{
			imgRightBuf.pop();
			tImRight = imgRightBuf.front()->header.stamp.toSec();
		}
		while (std::abs(tImRight - tImLeft) > igb.maxTimeDiff && imgLeftBuf.size() > 1)
		{
			imgLeftBuf.pop();
			tImLeft = imgLeftBuf.front()->header.stamp.toSec();
		}
		if (std::abs(tImLeft - tImRight) > igb.maxTimeDiff)
			continue;

		// Sync imu.
		if (useIMU)
		{
			if (imuBuf.empty())
				continue;
			if (tImLeft > imuBuf.back()->header.stamp.toSec())
				continue;
			// Load imu measurements from buffer
			vImuMeas.clear();
			while (!imuBuf.empty() && imuBuf.front()->header.stamp.toSec() <= tImLeft)
			{
				double t = imuBuf.front()->header.stamp.toSec();
				cv::Point3f acc(imuBuf.front()->linear_acceleration.x, imuBuf.front()->linear_acceleration.y, imuBuf.front()->linear_acceleration.z);
				cv::Point3f gyr(imuBuf.front()->angular_velocity.x, imuBuf.front()->angular_velocity.y, imuBuf.front()->angular_velocity.z);
				vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
				imuBuf.pop();
			}
		}

		const auto beg = std::chrono::steady_clock::now();

		ros::Time current_frame_time = imgLeftBuf.front()->header.stamp;

		imLeft = igb.GetImage(imgLeftBuf.front());
		imgLeftBuf.pop();

		imRight = igb.GetImage(imgRightBuf.front());
		imgRightBuf.pop();

		if (igb.do_equalize)
		{
			igb.mClahe->apply(imLeft, imLeft);
			igb.mClahe->apply(imRight, imRight);
		}

		if (igb.do_rectify)
		{
			cv::remap(imLeft, imLeft, igb.M1l, igb.M2l, cv::INTER_LINEAR);
			cv::remap(imRight, imRight, igb.M1r, igb.M2r, cv::INTER_LINEAR);
		}

		cv::Mat Tcw;
		if (useIMU)
			Tcw = slam.TrackStereo(imLeft, imRight, tImLeft, vImuMeas);
		else
			Tcw = slam.TrackStereo(imLeft, imRight, tImLeft);

		publish::publish_ros_pose_tf(Tcw, current_frame_time, slam_type);
		// publish::publish_ros_tracking_mappoints(slam.GetTrackedMapPoints(), current_frame_time);

		const auto end = std::chrono::steady_clock::now();
		double dt = std::chrono::duration_cast<std::chrono::nanoseconds>(end - beg).count() * 1e-9;
		
		if (logTime)
			std::cout << "Process time: " << dt << std::endl;

		ros::spinOnce();
	}

	return 0;
}