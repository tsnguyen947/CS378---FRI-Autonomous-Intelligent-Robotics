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
PointCloudT::Ptr neon_green (new PointCloudT);
PointCloudT::Ptr neon_orange (new PointCloudT);

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

void computeNeonVoxels(PointCloudT::Ptr in, PointCloudT::Ptr green, PointCloudT::Ptr orange) {

	green->clear();
	orange->clear();
	//Point Cloud to store out neon cap
	//PointCloudT::Ptr temp_neon_cloud (new PointCloudT);

	for (int i = 0; i < in->points.size(); i++) {
		unsigned int r, g, b;
		r = in->points[i].r;
		g = in->points[i].g;
		b = in->points[i].b;
		// Look for mostly neon value points
		if (g > 175 && (r + b) < 150) {
			green->push_back(in->points[i]);
		}
		else if(r > 200 && (g + b) < 150){
			orange->push_back(in->points[i]);
		}
	}
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

			//Send the filtered point cloud to be processed to get the neon green points and neon orange points
			computeNeonVoxels(cloud_filtered, neon_green, neon_orange);

			//Publish the cloud with the neon caps
			pcl::toROSMsg(*neon_green + *neon_orange,cloud_ros);

			//Set the frame ID to the first cloud we took in coz we want to replace that one
			cloud_ros.header.frame_id = cloud->header.frame_id;
			cloud_pub.publish(cloud_ros);

			tf::TransformListener tfListen;
			tf::StampedTransform transform;

			try{
				tfListen.waitForTransform("/base_link","/nav_kinect_depth_optical_frame",ros::Time(0), ros::Duration(10.0));
				tfListen.lookupTransform("/base_link","/nav_kinect_depth_optical_frame",ros::Time(0), transform);
			}
			catch(tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
			}

			tf::Vector3 coordinates;
			Eigen::Vector4f centroid;
			tf::Vector3 distance;
			geometry_msgs::Twist movement;
			double dist;

			if(neon_green->points.size() > 50 && neon_orange->points.size() > 50){
				// Find the centroid of the neon cap
				Eigen::Vector4f other_centroid;

				pcl::compute3DCentroid(*neon_green, centroid);
				pcl::compute3DCentroid(*neon_orange, other_centroid);

				coordinates.setX(centroid(0));
				coordinates.setY(centroid(1));
				coordinates.setZ(centroid(2));

				tf::Vector3 other_coordinates(other_centroid(0),other_centroid(1), other_centroid(2));

				distance = transform * coordinates;
				tf::Vector3 other_distance = transform * other_coordinates;

				double first_dist = sqrt((distance[0] * distance[0]) + (distance[1] * distance[1]));
				double second_dist = sqrt((other_distance[0] * other_distance[0]) + (other_distance[1] * other_distance[1]));

				dist = first_dist < second_dist ? first_dist : second_dist;

				if(dist > 1.25 && dist < 5.0){
					movement.linear.x = 0.3;
					if(dist == first_dist)
						movement.angular.z = 0.5 * atan2(distance[1], distance[0]);
					else
						movement.angular.z = 0.5 * atan2(other_distance[1], other_distance[0]);
					move_pub.publish(movement);
				}
				else if(dist < 1.0){
					movement.linear.x = -0.15;
					if(dist == first_dist)
						movement.angular.z = 0.5 * atan2(distance[1], distance[0]);
					else
						movement.angular.z = 0.5 * atan2(other_distance[1], other_distance[0]);
					move_pub.publish(movement);
				}
			}

			else if(neon_green->points.size() > 50 || neon_orange->points.size() > 50){
				if(neon_green->points.size() > 50)
					pcl::compute3DCentroid(*neon_green, centroid);
				else
					pcl::compute3DCentroid(*neon_orange, centroid);

				coordinates.setX(centroid(0));
				coordinates.setY(centroid(1));
				coordinates.setZ(centroid(2));

				distance = transform * coordinates;

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
