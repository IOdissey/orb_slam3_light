#pragma once

#include <queue>

#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>


struct ImageGrabber
{
	const double maxTimeDiff = 0.01;
	const size_t queue_size = 5;

	std::queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;

	const bool do_rectify;
	cv::Mat M1l ,M2l, M1r, M2r;

	const bool do_equalize;
	cv::Ptr<cv::CLAHE> mClahe;

	ImageGrabber(const bool do_rectify, const bool do_equalize):
		do_rectify(do_rectify), do_equalize(do_equalize)
	{
		if (do_equalize)
			mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
	}

	void GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg)
	{
		if (imgLeftBuf.size() > queue_size)
			imgLeftBuf.pop();
		imgLeftBuf.push(img_msg);
	}

	void GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg)
	{
		if (imgRightBuf.size() > queue_size)
			imgRightBuf.pop();
		imgRightBuf.push(img_msg);
	}

	cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg)
	{
		// Copy the ros image message to cv::Mat.
		cv_bridge::CvImageConstPtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
		}
		
		if(cv_ptr->image.type()==0)
		{
			return cv_ptr->image.clone();
		}
		else
		{
			std::cout << "Error type" << std::endl;
			return cv_ptr->image.clone();
		}
	}
};