#include <iostream>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/StatusText.h>
#include <std_srvs/Trigger.h>

#include "fast_drone/takeoff.h"
#include "fast_drone/publishPose.h"


ros::ServiceClient arming, set_mode;

ros::Publisher position_pub, position_raw_pub;

mavros_msgs::PositionTarget position_raw_msg;
geometry_msgs::PoseStamped position_msg;

ros::Publisher move_base_pub;
geometry_msgs::PoseStamped move_base_msg;


// bool arm = false, already = true;



void offboardAndArm() {
	ros::Rate r(10);
	static mavros_msgs::SetMode sm;
	sm.request.custom_mode = "OFFBOARD";
	set_mode.call(sm);
	ros::spinOnce();
	mavros_msgs::CommandBool srv;
	srv.request.value = true;
	arming.call(srv);
	ros::spinOnce();
}

void navi(double x, double y, double z, std::string farme_id) {
    ros::Time time_now = ros::Time::now();
    position_msg.header.stamp = time_now;
    position_msg.header.frame_id = farme_id;

    position_msg.pose.position.x = x;
    position_msg.pose.position.y = y;
    position_msg.pose.position.z = z;

    position_msg.pose.orientation.x = 0;
    position_msg.pose.orientation.y = 0;
    position_msg.pose.orientation.z = 0;
    position_msg.pose.orientation.w = 1;
    position_pub.publish(position_msg);

	offboardAndArm();
}

bool land (std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
	static mavros_msgs::SetMode sm;
	sm.request.custom_mode = "AUTO.LAND";
	set_mode.call(sm);
	ros::spinOnce();
    res.success=true;
	return true;
}

bool takeOff (fast_drone::takeoff::Request& req, fast_drone::takeoff::Response& res) {
	ros::Rate rate(20.0);
	for(int i = 150; ros::ok() && i > 0; --i){
        navi(0, 0, req.z, "body");
        ros::spinOnce();
        rate.sleep();
    }
    for(int i = 150; ros::ok() && i > 0; --i){
        navi(0, 0, req.z, "map");
        ros::spinOnce();
        rate.sleep();
    }
	res.s=true;
	return true;
}

bool publishPose (fast_drone::publishPose::Request& req, fast_drone::publishPose::Response& res) {
    ros::Time time_now = ros::Time::now();
    move_base_msg.header.stamp = time_now;
    move_base_msg.header.frame_id = "map";
	move_base_msg.pose.position.x = req.x;
    move_base_msg.pose.position.y = req.y;
    move_base_msg.pose.position.z = req.z;
    move_base_msg.pose.orientation.x = 0;
    move_base_msg.pose.orientation.y = 0;
    move_base_msg.pose.orientation.z = 0;
    move_base_msg.pose.orientation.w = 1;
    move_base_pub.publish(move_base_msg);
	res.s=true;
	return true;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "fly_fast_drone");
	ros::NodeHandle nh, nh_priv("~");
    
    arming = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	set_mode = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    position_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
	position_raw_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);
    
    move_base_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);

	
	auto ld_serv = nh.advertiseService("land", &land);

	auto takeOff_serv = nh.advertiseService("takeOff", &takeOff);

	auto publishPose_serv = nh.advertiseService("publish_pose", &publishPose);

	ROS_INFO("ready");
	ros::spin();

    return 0;
}
