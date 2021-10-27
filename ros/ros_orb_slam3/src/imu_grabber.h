#pragma once

#include <queue>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/core/core.hpp>


class ImuGrabber
{
private:
	std::queue<sensor_msgs::ImuConstPtr> _imu_buf;
	ros::Subscriber _sub_imu;
	bool _use_imu = false;

	void _grab_imu(const sensor_msgs::ImuConstPtr &imu_msg)
	{
		_imu_buf.push(imu_msg);
	}

public:
	ImuGrabber(cv::FileStorage& settings, ros::NodeHandle& node)
	{
		settings["useIMU"] >> _use_imu;
		if (_use_imu)
		{
			std::string topic_imu;
			settings["topic_imu"] >> topic_imu;
			_sub_imu = node.subscribe(topic_imu, 1000, &ImuGrabber::_grab_imu, this);
		}
	}

	bool use_imu() const
	{
		return _use_imu;
	}

	std::queue<sensor_msgs::ImuConstPtr>& get_imu()
	{
		return _imu_buf;
	}
};