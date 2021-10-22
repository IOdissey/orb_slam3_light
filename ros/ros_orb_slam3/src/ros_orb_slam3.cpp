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
#include <memory>
#include <csignal>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/core/core.hpp>

#include <System.h>
#include <ImuTypes.h>

#include "image_grabber.h"
#include "imu_grabber.h"
#include "publish.h"


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
	// 
	if ((!ros::master::check()))
	{
		std::cout << "ROS master not started" << std::endl;
		return 1;
	}
	// Check args.
	if (argc != 2)
	{
		cerr << endl << "Usage: ros_orb_slam3 path_to_settings" << endl;
		ros::shutdown();
		return 1;
	}
	// Settings.
	cv::FileStorage settings(argv[1], cv::FileStorage::READ);
	if (!settings.isOpened())
	{
		cerr << "ERROR: Wrong path to settings" << endl;
		ros::shutdown();
		return -1;
	}
	//
	ros::NodeHandle node;
	std::unique_ptr<ImageGrabber> image_grabber = std::make_unique<ImageGrabber>(settings, node);
	std::unique_ptr<ImuGrabber> imu_grabber = std::make_unique<ImuGrabber>(settings, node);
	//
	std::string vocabulary;
	bool do_rectify = false;
	bool log_time = false;
	settings["vocabulary"] >> vocabulary;
	settings["logTime"] >> log_time;
	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM3::System::eSensor slam_type;
	if (imu_grabber->use_imu())
		slam_type = ORB_SLAM3::System::eSensor::IMU_STEREO;
	else
		slam_type = ORB_SLAM3::System::eSensor::STEREO;
	ORB_SLAM3::System slam(vocabulary, argv[1], slam_type);
	//
	publish::pose_pub = node.advertise<geometry_msgs::PoseStamped>("/orb_slam3_ros/camera", 1);
	publish::map_points_pub = node.advertise<sensor_msgs::PointCloud2>("/orb_slam3_ros/map_points", 1);
	publish::setup_tf_orb_to_ros(slam_type);
	//
	std::signal(SIGINT, handler);
	std::cout << std::endl << "orb_slam3_ros started." << std::endl;
	// Main loop.
	cv::Mat img_left, img_right;
	std::vector<ORB_SLAM3::IMU::Point> imu_meas;
	ros::Time beg_time;
	while (is_run && ros::ok())
	{
		const auto beg = std::chrono::steady_clock::now();
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		ros::spinOnce();
		//
		if (!image_grabber->is_grab())
			continue;
		const ros::Time current_stamp = image_grabber->get_stamp();
		if (beg_time.isZero())
			beg_time = current_stamp;
		// Sync imu.
		if (imu_grabber->use_imu())
		{
			std::queue<sensor_msgs::ImuConstPtr>& imu_buf = imu_grabber->get_imu();
			if (imu_buf.empty() || current_stamp > imu_buf.back()->header.stamp)
			{
				// Do not receive new messages.
				image_grabber->set_skip(true);
				continue;
			}
			// Load imu measurements from buffer.
			imu_meas.clear();
			while (!imu_buf.empty() && imu_buf.front()->header.stamp <= current_stamp)
			{
				double t = (imu_buf.front()->header.stamp - beg_time).toSec();
				cv::Point3f acc(imu_buf.front()->linear_acceleration.x, imu_buf.front()->linear_acceleration.y, imu_buf.front()->linear_acceleration.z);
				cv::Point3f gyr(imu_buf.front()->angular_velocity.x, imu_buf.front()->angular_velocity.y, imu_buf.front()->angular_velocity.z);
				imu_meas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
				imu_buf.pop();
			}
			// Do receive new messages.
			image_grabber->set_skip(false);
		}
		if (!image_grabber->get_stereo(img_left, img_right))
			continue;
		//
		const double current_time = (current_stamp - beg_time).toSec();
		cv::Mat pos = slam.TrackStereo(img_left, img_right, current_time, imu_meas);

		publish::publish_ros_pose_tf(pos, current_stamp, slam_type);
		publish::publish_ros_tracking_mappoints(slam.GetTrackedMapPoints(), current_stamp);

		ros::spinOnce();

		const auto end = std::chrono::steady_clock::now();
		double dt = std::chrono::duration_cast<std::chrono::nanoseconds>(end - beg).count() * 1e-9;
		if (log_time)
			std::cout << "Process time: " << dt << std::endl;
	}

	ros::shutdown();
	return 0;
}