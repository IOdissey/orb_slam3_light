#pragma once

#include <memory>

#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>


class ImageGrabber
{
private:
	typedef message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync_t;

	bool _do_rectify = false;
	bool _do_equalize = false;
	cv::Mat _m1l, _m2l, _m1r, _m2r;
	cv::Ptr<cv::CLAHE> _clahe;
	message_filters::Subscriber<sensor_msgs::Image> _sub_left;
	message_filters::Subscriber<sensor_msgs::Image> _sub_right;
	std::unique_ptr<sync_t> _sync;
	sensor_msgs::ImageConstPtr _msg_left;
	sensor_msgs::ImageConstPtr _msg_right;
	bool _is_grub = false;
	bool _skip = false;
	ros::Time _stamp;

	bool _msg_mat(const sensor_msgs::ImageConstPtr& msg, cv::Mat& mat, bool copy = true)
	{
		size_t size;
		int type;
		if (msg->encoding == "mono8")
		{
			type = CV_8UC1;
			size = msg->width * msg->height;
		}
		else if (msg->encoding == "bgr8" || msg->encoding == "rgb8")
		{
			type = CV_8UC3;
			size = 3 * msg->width * msg->height;
		}
		else
		{
			std::cout << "sensor_msgs::Image format '" << msg->encoding << "' not supported." << std::endl;
			return false;
		}
		// Check size.
		if (msg->width < 1 || msg->height < 1 || size != msg->data.size())
		{
			std::cout << "sensor_msgs::Image is incorrect." << std::endl;
			return false;
		}
		if (copy)
		{
			if (mat.cols != msg->width || mat.rows != msg->height || mat.type() != type)
				mat = cv::Mat(msg->height, msg->width, type);
			std::memcpy(mat.data, msg->data.data(), size);
		}
		else
			mat = cv::Mat(msg->height, msg->width, type, (void*)msg->data.data());
		return true;
	}

	void _grab_stereo(const sensor_msgs::ImageConstPtr& msg_left, const sensor_msgs::ImageConstPtr& msg_right)
	{
		if (_skip)
			return;
		_msg_left = msg_left;
		_msg_right = msg_right;
		_is_grub = true;
		_stamp = _msg_left->header.stamp;
	}

public:
	ImageGrabber(cv::FileStorage& settings, ros::NodeHandle& node)
	{
		settings["do_rectify"] >> _do_rectify;
		settings["do_equalize"] >> _do_equalize;
		if (_do_rectify)
		{
			cv::Mat K_l, K_r;
			settings["LEFT.K"] >> K_l;
			settings["RIGHT.K"] >> K_r;
			cv::Mat P_l, P_r;
			settings["LEFT.P"] >> P_l;
			settings["RIGHT.P"] >> P_r;
			cv::Mat R_l, R_r;
			settings["LEFT.R"] >> R_l;
			settings["RIGHT.R"] >> R_r;
			cv::Mat D_l, D_r;
			settings["LEFT.D"] >> D_l;
			settings["RIGHT.D"] >> D_r;
			//
			int rows_l = settings["LEFT.height"];
			int cols_l = settings["LEFT.width"];
			int rows_r = settings["RIGHT.height"];
			int cols_r = settings["RIGHT.width"];
			// 
			if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
					rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
			{
				_do_rectify = false;
				std::cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << std::endl;
			}
			else
			{
				cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0,3).colRange(0,3), cv::Size(cols_l, rows_l), CV_32F, _m1l, _m2l);
				cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0,3).colRange(0,3), cv::Size(cols_r, rows_r), CV_32F, _m1r, _m2r);
			}
		}
		if (_do_equalize)
			_clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
		// Topics.
		std::string topic_left, topic_right;
		settings["ROS.topic_left"] >> topic_left;
		settings["ROS.topic_right"] >> topic_right;
		_sub_left.subscribe(node, topic_left, 1);
		_sub_right.subscribe(node, topic_right, 1);
		_sync = std::make_unique<sync_t>(_sub_left, _sub_right, 10);
		_sync->registerCallback(boost::bind(&ImageGrabber::_grab_stereo, this, _1, _2));
	}

	const bool& is_grab() const
	{
		return _is_grub;
	}

	void set_skip(bool skip)
	{
		_skip = skip;
	}

	const ros::Time& get_stamp() const
	{
		return _stamp;
	}

	bool get_stereo(cv::Mat& img_left, cv::Mat& img_right)
	{
		if (!_is_grub)
			return false;
		_is_grub = false;
		const bool ok = _msg_mat(_msg_left, img_left, false) && _msg_mat(_msg_right, img_right, false);
		if (ok)
		{
			if (_do_equalize)
			{
				_clahe->apply(img_left, img_left);
				_clahe->apply(img_right, img_right);
			}
			if (_do_rectify)
			{
				cv::remap(img_left, img_left, _m1l, _m2l, cv::INTER_LINEAR);
				cv::remap(img_right, img_right, _m1r, _m2r, cv::INTER_LINEAR);
			}
		}
		return ok;
	}
};