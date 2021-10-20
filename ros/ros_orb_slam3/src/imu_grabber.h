#pragma once

#include <queue>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>


struct ImuGrabber
{
	std::queue<sensor_msgs::ImuConstPtr> imuBuf;

	void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
	{
		imuBuf.push(imu_msg);
	}
};