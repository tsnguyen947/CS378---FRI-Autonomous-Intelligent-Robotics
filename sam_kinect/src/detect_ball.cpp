#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Twist.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
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

// camera_link for fixed frame in global option in rviz

/* define what kind of point clouds we're using */
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool new_cloud_available_flag = false;

// General point cloud to store the whole image
PointCloudT::Ptr cloud (new PointCloudT);

PointCloudT::Ptr cloud_sphere (new PointCloudT);

PointCloudT::Ptr closest_cloud (new PointCloudT);

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

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "kinect_fun");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("/nav_kinect/depth_registered/points", 1000, cloud_sub);

	//debugging publisher --> can create your own topic and then subscribe to it through rviz
	ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("detect_cap/cloud", 10);

	ros::Publisher move_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",10);

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
			PointCloudT::Ptr cloud_filtered (new PointCloudT);
			vg.setInputCloud (cloud);
			vg.setLeafSize (0.005f, 0.005f, 0.005f);
			vg.filter (*cloud_filtered);

			ROS_INFO("After voxel grid filter: %i points",(int)cloud_filtered->points.size());

			pcl::SACSegmentation<PointT> seg;
			pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
			pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
			seg.setOptimizeCoefficients (true);
			seg.setModelType (pcl::SACMODEL_SPHERE);
			seg.setMethodType (pcl::SAC_RANSAC);
			seg.setMaxIterations (100);
			seg.setDistanceThreshold (0.5);

			// Segment the largest spherical component from the remaining cloud
			seg.setInputCloud (cloud_filtered);
			seg.segment (*inliers, *coefficients);
			if (inliers->indices.size () == 0){}
			else{
				// Extract the spherical inliers from the input cloud
				pcl::ExtractIndices<PointT> extract;
				extract.setInputCloud (cloud_filtered);
				extract.setIndices (inliers);
				extract.setNegative (false);

				// Get the points associated with the planar surface
				extract.filter (*cloud_sphere);
			}

			/* Creating the KdTree from input point cloud*/
			pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
			tree->setInputCloud (cloud_sphere);

			/* Here we are creating a vector of PointIndices, which contains the actual index
			 * information in a vector<int>. The indices of each detected cluster are saved here.
			 * Cluster_indices is a vector containing one instance of PointIndices for each detected
			 * cluster. Cluster_indices[0] contain all indices of the first cluster in input point cloud.
			 */
			std::vector<pcl::PointIndices> cluster_indices;
			pcl::EuclideanClusterExtraction<PointT> ec;
			ec.setClusterTolerance (0.06);
			ec.setMinClusterSize (50);
			ec.setMaxClusterSize (1250);
			ec.setSearchMethod (tree);
			ec.setInputCloud (cloud_sphere);
			ec.extract (cluster_indices);

			/* To separate each cluster out of the vector<PointIndices> we have to
			 * iterate through cluster_indices, create a new PointCloud for each
			 * entry and write all points of the current cluster in the PointCloud.
			 */

			std::vector<pcl::PointIndices>::const_iterator it;
			std::vector<int>::const_iterator pit;
			std::vector<PointCloudT::Ptr> clouds;

			for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
				PointCloudT::Ptr cloud_cluster (new PointCloudT);
				for(pit = it->indices.begin(); pit != it->indices.end(); pit++) {
					//push_back: add a point to the end of the existing vector
					cloud_cluster->points.push_back(cloud_sphere->points[*pit]);
				}
				clouds.push_back(cloud_cluster);
			}

			try{
				tfListen.waitForTransform("/base_link","/nav_kinect_depth_optical_frame",ros::Time(0), ros::Duration(10.0));
				tfListen.lookupTransform("/base_link","/nav_kinect_depth_optical_frame",ros::Time(0), transform);
			}
			catch(tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
			}

			Eigen::Vector4f centroid;
			double closest = 1000;
			int index = 0;
			for(int i = 0; i < clouds.size(); i++){
				PointCloudT::Ptr current_cloud = clouds[i];

				Eigen::Vector4f centroid;
				pcl::compute3DCentroid(*current_cloud, centroid);

				tf::Vector3 coordinates(centroid(0), centroid(1), centroid(2));
				tf::Vector3 distance = transform * coordinates;

				double dist = sqrt((distance[0] * distance[0]) + (distance[1] * distance[1]));

				if(dist < closest){
					closest = dist;
					index = i;
				}
			}

			closest_cloud = clouds[index];
			//Publish the cloud with the neon cap
			pcl::toROSMsg(*closest_cloud,cloud_ros);

			//Set the frame ID to the first cloud we took in coz we want to replace that one
			cloud_ros.header.frame_id = cloud->header.frame_id;
			cloud_pub.publish(cloud_ros);

			pcl::compute3DCentroid(*closest_cloud, centroid);
			tf::Vector3 coordinates(centroid(0), centroid(1), centroid(2));

			tf::Vector3 distance = transform * coordinates;

			geometry_msgs::Twist movement;

			double dist = sqrt((distance[0] * distance[0]) + (distance[1] * distance[1]));

			ROS_INFO("The distance is %.5f, x is %.5f, y is %.5f, z is %.5f", dist, distance[0], distance[1], distance[2]);

			if(dist > 1.5 && dist < 4.0){
				movement.linear.x = 0.3;
				movement.angular.z = 0.4 * atan2(distance[1], distance[0]);
				move_pub.publish(movement);
			}
		}
	}
	return 0;
}
