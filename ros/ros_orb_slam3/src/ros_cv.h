#pragma once

#include <cstring>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/Image.h>


// Very simple conversion methods between cv::Mat and sensor_msgs::Image
namespace ros_cv
{
	namespace _
	{
		std::string mat_type_str(int type)
		{
			std::string r;
			uint8_t depth = type & CV_MAT_DEPTH_MASK;
			switch (depth)
			{
				case CV_8U:  r = "8U";   break;
				case CV_8S:  r = "8S";   break;
				case CV_16U: r = "16U";  break;
				case CV_16S: r = "16S";  break;
				case CV_32S: r = "32S";  break;
				case CV_32F: r = "32F";  break;
				case CV_64F: r = "64F";  break;
				default:     r = "User"; break;
			}
			r += "C";
			uint8_t chans = 1 + (type >> CV_CN_SHIFT);
			r += (chans + '0');
			return r;
		}
	}

	bool msg_mat(const sensor_msgs::ImageConstPtr& msg, cv::Mat& mat, bool copy = true)
	{
		size_t size;
		int type;
		if (msg->encoding == "mono8" || msg->encoding == "8UC1")
		{
			type = CV_8UC1;
			size = msg->width * msg->height;
		}
		else if (msg->encoding == "bgr8" || msg->encoding == "rgb8" || msg->encoding == "8UC3")
		{
			type = CV_8UC3;
			size = 3 * msg->width * msg->height;
		}
		else if (msg->encoding == "mono16" || msg->encoding == "16UC1")
		{
			type = CV_16UC1;
			size = 2 * msg->width * msg->height;
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

	bool mat_msg(const cv::Mat& mat, sensor_msgs::Image& msg)
	{
		if (!mat.isContinuous())
		{
			std::cout << "cv::Mat is not continuous." << std::endl;
			return false;
		}
		switch (mat.type())
		{
			case CV_8UC1:
			{
				msg.encoding = "mono8";
				break;
			}
			case CV_16UC1:
			{
				msg.encoding = "mono16";
				break;
			}
			case CV_8UC3:
			{
				msg.encoding = "bgr8";
				break;
			}
			default:
			{
				std::cout << "cv::Mat type '" << _::mat_type_str(mat.type()) << "' is not supported." << std::endl;
				return false;
			}
		}
		msg.width = mat.cols;
		msg.height = mat.rows;
		msg.step = mat.step;
		msg.is_bigendian = false;
		const size_t size = mat.step * msg.height;
		if (msg.data.size() != size)
			msg.data.resize(size);
		std::memcpy(msg.data.data(), mat.data, size);
		return true;
	}
}