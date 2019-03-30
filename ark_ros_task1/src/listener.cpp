#include<ros/ros.h>
#include<ark_ros_task1/board_pose.h>
#include<ark_ros_task1/danger_region.h>
#include<sstream>

void chatterCallback(const ark_ros_task1::board_pose::ConstPtr& msg)
{
	ROS_INFO("x coordinate : %d\n", msg->x);
	ROS_INFO("y coordinate : %d\n", msg->y);
}

bool danger(ark_ros_task1::danger_region::Request &req,ark_ros_task1::danger_region::Response &res)
{
	if(req.status)
	{
		ROS_INFO("Entered Danger region\n");
	}
	res.out="Danger";
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc,argv, "listener");
	ros::NodeHandle n;
	ros::Subscriber sub=n.subscribe("check_pose",1,chatterCallback);
	ros::ServiceServer service = n.advertiseService("check_pose",danger);
	ros::spin();
	return 0;
}