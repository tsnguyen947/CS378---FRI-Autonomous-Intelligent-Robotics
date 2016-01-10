#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Twist.h"
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <cstdio>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

// camera_link for fixed frame in global option in rviz

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool new_cloud_available_flag = false;

// General point cloud to store the whole image
PointCloudT::Ptr cloud (new PointCloudT);

//Point Cloud to store out neon cap
PointCloudT::Ptr neon_cloud (new PointCloudT);

// Message required to publish the cloud - Convert from pcl to msg
sensor_msgs::PointCloud2 cloud_ros;

void cloud_sub(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	//convert the msg to PCL format
	pcl::fromROSMsg (*msg, *cloud);

	//state that a new cloud is available
	new_cloud_available_flag = true;

	/**PointCloudT::iterator myIterator;
		for(myIterator = cloud->begin();  
			myIterator != cloud->end();
			myIterator++)
		{
			std::cout<<*myIterator<<" ";
		}**/
}

PointCloudT::Ptr computeNeonVoxels(PointCloudT::Ptr in) {
	int total_neon = 0;

	//Point Cloud to store out neon cap
	PointCloudT::Ptr temp_neon_cloud (new PointCloudT);

	for (int i = 0; i < in->points.size(); i++) {
		unsigned int r, g, b;
		r = in->points[i].r;
		g = in->points[i].g;
		b = in->points[i].b;
		// Look for mostly neon value points
		if (g > 175 && (r + b) < 150) {
			temp_neon_cloud->push_back(in->points[i]);
		}
	}

	return temp_neon_cloud;
}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "kinect_fun");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("/nav_kinect/depth_registered/points", 1000, cloud_sub);

	//debugging publisher --> can create your own topic and then subscribe to it through rviz
	ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("detect_cap/cloud", 10);

	ros::Publisher move_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
	//refresh rate
	double ros_rate = 10.0;
	ros::Rate r(ros_rate);

	tf::TransformListener tfListen;
	tf::StampedTransform transform;

	while (ros::ok())
	{
		ros::spinOnce();
		r.sleep();

		if (new_cloud_available_flag){
			new_cloud_available_flag = false;

			ROS_INFO("Before voxel grid filter: %i points",(int)cloud->points.size());

			// Voxel Grid reduces the computation time. Its a good idea to do it if you will be doing
			//sequential processing or frame-by-frame
			// Create the filtering object: downsample the dataset using a leaf size of 1cm
			pcl::VoxelGrid<PointT> vg;
			pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
			vg.setInputCloud (cloud);
			vg.setLeafSize (0.005f, 0.005f, 0.005f);
			vg.filter (*cloud_filtered);

			ROS_INFO("After voxel grid filter: %i points",(int)cloud_filtered->points.size());

			int max_num_neon = 0;

			//Send the filtered point cloud to be processed in order to get the neon blob
			neon_cloud = computeNeonVoxels(cloud_filtered);

			//Publish the cloud with the neon cap
			pcl::toROSMsg(*neon_cloud,cloud_ros);

			//Set the frame ID to the first cloud we took in coz we want to replace that one
			cloud_ros.header.frame_id = cloud->header.frame_id;
			cloud_pub.publish(cloud_ros);

			// Find the centroid of the neon cap
			Eigen::Vector4f centroid;
			pcl::compute3DCentroid(*neon_cloud, centroid);

			ROS_INFO("The centroid of the neon cap is: (%f, %f, %f)", centroid(0), centroid(1), centroid(2));

			if(neon_cloud->points.size() > 50){
				try{
					tfListen.waitForTransform("/base_link","/nav_kinect_depth_optical_frame",ros::Time(0), ros::Duration(10.0));
					tfListen.lookupTransform("/base_link","/nav_kinect_depth_optical_frame",ros::Time(0), transform);
				}
				catch(tf::TransformException ex){
					ROS_ERROR("%s",ex.what());
					ros::Duration(1.0).sleep();
				}
				tf::Vector3 coordinates(centroid(0), centroid(1), centroid(2));

				tf::Vector3 distance = transform * coordinates;

				geometry_msgs::Twist movement;

				double dist = sqrt((distance[0] * distance[0]) + (distance[1] * distance[1]));

				ROS_INFO("The distance is %.5f, x is %.5f, y is %.5f, z is %.5f", dist, distance[0], distance[1], distance[2]);

				if(dist > 1.25 && dist < 5.0){
					movement.linear.x = 0.3;
					movement.angular.z = 0.5 * atan2(distance[1], distance[0]);
					move_pub.publish(movement);
				}
				else if(dist < 1.0){
					movement.linear.x = -0.15;
					movement.angular.z = 0.5 * atan2(distance[1], distance[0]);
					move_pub.publish(movement);
				}
			}
		}
	}
	return 0;
}
