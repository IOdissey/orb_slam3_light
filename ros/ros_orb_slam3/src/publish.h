#pragma once

#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>

#include <System.h>


class Publish
{
private:
	std::string _map_frame_id;
	std::string _pose_frame_id;
	bool _publish_map_points;
	// Coordinate transformation matrix from orb coordinate system to ros coordinate systemm
	tf::Matrix3x3 _tf_orb_to_ros;

	tf::TransformBroadcaster _tf_broadcaster;
	ros::Publisher _pose_pub;
	ros::Publisher _map_points_pub;
	// image_transport::Publisher _rendered_image_pub;

	tf::Transform _from_orb_to_ros_tf_transform(const cv::Mat& transformation_mat)
	{
		cv::Mat orb_rotation(3, 3, CV_32F);
		cv::Mat orb_translation(3, 1, CV_32F);

		orb_rotation    = transformation_mat.rowRange(0, 3).colRange(0, 3);
		orb_translation = transformation_mat.rowRange(0, 3).col(3);

		tf::Matrix3x3 tf_camera_rotation(
			orb_rotation.at<float> (0, 0), orb_rotation.at<float> (0, 1), orb_rotation.at<float> (0, 2),
			orb_rotation.at<float> (1, 0), orb_rotation.at<float> (1, 1), orb_rotation.at<float> (1, 2),
			orb_rotation.at<float> (2, 0), orb_rotation.at<float> (2, 1), orb_rotation.at<float> (2, 2)
		);

		tf::Vector3 tf_camera_translation(orb_translation.at<float> (0), orb_translation.at<float> (1), orb_translation.at<float> (2));

		// cout << setprecision(9) << "Rotation: " << endl << orb_rotation << endl;
		// cout << setprecision(9) << "Translation xyz: " << orb_translation.at<float> (0) << " " << orb_translation.at<float> (1) << " " << orb_translation.at<float> (2) << endl;

		// Transform from orb coordinate system to ros coordinate system on camera coordinates
		tf_camera_rotation    = _tf_orb_to_ros * tf_camera_rotation;
		tf_camera_translation = _tf_orb_to_ros * tf_camera_translation;

		// Inverse matrix
		tf_camera_rotation    = tf_camera_rotation.transpose();
		tf_camera_translation = -(tf_camera_rotation * tf_camera_translation);

		// Transform from orb coordinate system to ros coordinate system on map coordinates
		tf_camera_rotation    = _tf_orb_to_ros * tf_camera_rotation;
		tf_camera_translation = _tf_orb_to_ros * tf_camera_translation;

		return tf::Transform(tf_camera_rotation, tf_camera_translation);
	}

	void _publish_tf_transform(const tf::Transform& tf_transform, const ros::Time& current_frame_time)
	{
		_tf_broadcaster.sendTransform(tf::StampedTransform(tf_transform, current_frame_time, _map_frame_id, _pose_frame_id));
	}

	void _publish_pose_stamped(const tf::Transform& tf_transform, const ros::Time& current_frame_time)
	{
		tf::Stamped<tf::Pose> grasp_tf_pose(tf_transform, current_frame_time, _map_frame_id);
		geometry_msgs::PoseStamped pose_msg;
		tf::poseStampedTFToMsg(grasp_tf_pose, pose_msg);
		_pose_pub.publish(pose_msg);
	}

	void _publish_tracked_mappoints(const std::vector<ORB_SLAM3::MapPoint*> map_points, const ros::Time& current_frame_time)
	{
		const int num_channels = 3; // x y z

		if (map_points.size() == 0)
			std::cout << "Map point vector is empty!" << std::endl;

		sensor_msgs::PointCloud2 cloud;

		cloud.header.stamp = current_frame_time;
		cloud.header.frame_id = _map_frame_id;
		cloud.height = 1;
		cloud.width = map_points.size();
		cloud.is_bigendian = false;
		cloud.is_dense = true;
		cloud.point_step = num_channels * sizeof(float);
		cloud.row_step = cloud.point_step * cloud.width;
		cloud.fields.resize(num_channels);

		std::string channel_id[] = { "x", "y", "z"};

		for (int i = 0; i < num_channels; i++)
		{
			cloud.fields[i].name = channel_id[i];
			cloud.fields[i].offset = i * sizeof(float);
			cloud.fields[i].count = 1;
			cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
		}

		cloud.data.resize(cloud.row_step * cloud.height);

		unsigned char *cloud_data_ptr = &(cloud.data[0]);

		for (unsigned int i = 0; i < cloud.width; i++)
		{
			if (map_points[i])
			{
				tf::Vector3 point_translation(map_points[i]->GetWorldPos().at<float> (0), map_points[i]->GetWorldPos().at<float> (1), map_points[i]->GetWorldPos().at<float> (2));
				point_translation = _tf_orb_to_ros * point_translation;
				float data_array[num_channels] = {(float)point_translation.x(), (float)point_translation.y(), (float)point_translation.z()};
				memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));
			}
		}

		_map_points_pub.publish(cloud);
	}

	// void _publish_ros_tracking_img(cv::Mat image, ros::Time current_frame_time)
	// {
	// 	std_msgs::Header header;
	// 	header.stamp = current_frame_time;
	// 	header.frame_id = _map_frame_id;
	// 	const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
	// 	_rendered_image_pub.publish(rendered_image_msg);
	// }

public:

	Publish(cv::FileStorage& settings, ros::NodeHandle& node, ORB_SLAM3::System::eSensor sensor_type)
	{
		settings["map_frame_id"] >> _map_frame_id;
		settings["pose_frame_id"] >> _pose_frame_id;
		settings["publish_map_points"] >> _publish_map_points;
		_pose_pub = node.advertise<geometry_msgs::PoseStamped>("/orb_slam3_ros/camera", 1);
		if (_publish_map_points)
			_map_points_pub = node.advertise<sensor_msgs::PointCloud2>("/orb_slam3_ros/map_points", 1);
		// The conversion depends on whether IMU is involved:
		//  z is aligned with camera's z axis = without IMU
		//  z is aligned with gravity = with IMU
		if (sensor_type == ORB_SLAM3::System::MONOCULAR || sensor_type == ORB_SLAM3::System::STEREO || sensor_type == ORB_SLAM3::System::RGBD)
		{
			_tf_orb_to_ros.setValue(
				 0,  0,  1,
				-1,  0,  0,
				 0, -1,  0);
		}
		else if (sensor_type == ORB_SLAM3::System::IMU_MONOCULAR || sensor_type == ORB_SLAM3::System::IMU_STEREO)
		{
			_tf_orb_to_ros.setValue(
				 0,  1,  0,
				-1,  0,  0,
				 0,  0,  1);
		}
		else
			_tf_orb_to_ros.setIdentity();
	}

	void publish(const cv::Mat& pos, const ros::Time& current_frame_time, ORB_SLAM3::System& slam)
	{
		if (!pos.empty())
		{
			tf::Transform tf_transform = _from_orb_to_ros_tf_transform(pos);
			_publish_tf_transform(tf_transform, current_frame_time);
			_publish_pose_stamped(tf_transform, current_frame_time);
		}
		if (_publish_map_points)
			_publish_tracked_mappoints(slam.GetTrackedMapPoints(), current_frame_time);
	}
};