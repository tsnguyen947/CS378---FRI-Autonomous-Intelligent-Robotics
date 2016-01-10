#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Bool.h"
#include <vector>

using namespace std;

boost::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> > navigator;
bool personFound;

void visualCallback(const std_msgs::Bool::ConstPtr& msg){
	personFound = msg->data;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "navigate_client");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/facial_recog/visual", 1000, visualCallback);

	navigator.reset(new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true));

	vector<geometry_msgs::PoseStamped> points (4);
	float coordinates[] = {-47.4090456954, -11.3658098428, 0.776182503291, 0.630508304136, 
				-47.5671985814, -4.37876182561, 0.065318072597, 0.997864494504, 
				-8.25520593555, -5.30878216775, -0.454712536739, 0.890638259302, 
				-7.75911151744, -11.6939711716, -0.997820745301, 0.0659830299912};
	int index = 0;

	for(int i = 0; i < points.size(); i++){
		points[i].pose.position.x = coordinates[index];
		points[i].pose.position.y = coordinates[index + 1];
		points[i].pose.orientation.z = coordinates[index + 2];
		points[i].pose.orientation.w = coordinates[index + 3];
		points[i].header.frame_id = "map";
		index += 4;
	}

	double ros_rate = 20.0;
	ros::Rate r(ros_rate);

	navigator->waitForServer();

	while(ros::ok()){
		ros::spinOnce();
		r.sleep();

		for(int i = 0; i < points.size(); i++){
			move_base_msgs::MoveBaseGoal goal;
			goal.target_pose = points[i];
			navigator->sendGoal(goal);
			bool reachedGoal = false;
			while(!reachedGoal){
				ros::spinOnce();
				if(personFound){
					navigator->cancelGoal();
					reachedGoal = true;
					while(personFound)
						ros::spinOnce();
				}
				else
					reachedGoal = navigator->waitForResult(ros::Duration(0.5));
			}
			actionlib::SimpleClientGoalState state = navigator->getState();
    			if(state != actionlib::SimpleClientGoalState::SUCCEEDED)
				i--;
		}
	}
}
