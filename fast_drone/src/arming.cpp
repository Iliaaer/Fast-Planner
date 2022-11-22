#include <iostream>
#include <string>

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/StatusText.h>
#include <std_srvs/Trigger.h>

#include "Fast_drone/takeoff.h"
#include "Fast_drone/publishPose.h"

ros::ServiceClient arming, set_mode;
mavros_msgs::State state;
/*
tf2_ros::Buffer tf_buffer;
std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster;
std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster;
std::map<std::string, std::string> reference_frames;
geometry_msgs::TransformStamped body;
geometry_msgs::PoseStamped local_position;
*/


ros::Duration offboard_timeout;
ros::Duration arming_timeout;
mavros_msgs::StatusText statustext;

ros::Publisher position_pub, position_raw_pub;

mavros_msgs::PositionTarget position_raw_msg;
geometry_msgs::PoseStamped position_msg;

ros::Publisher move_base_pub;
geometry_msgs::PoseStamped move_base_msg;

ros::Publisher telemetry_pub;
geometry_msgs::PoseStamped pose_msg;

bool arm = true, already = true, offbord = true;
bool telebetry_dont_call = false;

double x_tf = 0, y_tf = 0, z_tf = 0;


void offboardAndArm() {
   	ros::Rate r(10);
	if (state.mode != "OFFBOARD") {
		auto start = ros::Time::now();
		ROS_INFO("switch to OFFBOARD");
		static mavros_msgs::SetMode sm;
		sm.request.custom_mode = "OFFBOARD";

		if (!set_mode.call(sm))
			throw std::runtime_error("Error calling set_mode service");

		// wait for OFFBOARD mode
		while (ros::ok()) {
			ros::spinOnce();
			if (state.mode == "OFFBOARD") {
				break;
			} else if (ros::Time::now() - start > offboard_timeout) {
				std::string report = "OFFBOARD timed out";
				if (statustext.header.stamp > start)
					report += ": " + statustext.text;
				throw std::runtime_error(report);
			}
			ros::spinOnce();
			r.sleep();
		}
	}

	if (!state.armed) {
		ros::Time start = ros::Time::now();
		ROS_INFO("arming");
		mavros_msgs::CommandBool srv;
		srv.request.value = true;
		if (!arming.call(srv)) {
			throw std::runtime_error("Error calling arming service");
		}

		// wait until armed
		while (ros::ok()) {
			ros::spinOnce();
			if (state.armed) {
				break;
			} else if (ros::Time::now() - start > arming_timeout) {
				std::string report = "Arming timed out";
				if (statustext.header.stamp > start)
					report += ": " + statustext.text;
				throw std::runtime_error(report);
			}
			ros::spinOnce();
			r.sleep();
		}
	}
}

void navi(double x, double y, double z, std::string farme_id) {
    ros::Time time_now = ros::Time::now();
    position_msg.header.stamp = time_now;
    position_msg.header.frame_id = farme_id;

    position_msg.pose.position.x = x;
    position_msg.pose.position.y = y;
    position_msg.pose.position.z = z;

    // tf2::Quaternion myQuaternion;
    // myQuaternion.setRPY(0, 0, yaw);
    // myQuaternion=myQuaternion.normalize();
    // cmd_mavros.pose.orientation.x = myQuaternion[0];
    // cmd_mavros.pose.orientation.y = myQuaternion[1];
    // cmd_mavros.pose.orientation.z = myQuaternion[2];
    // cmd_mavros.pose.orientation.w = myQuaternion[3];

    position_msg.pose.orientation.x = pose_msg.pose.orientation.x;
    position_msg.pose.orientation.y = pose_msg.pose.orientation.y;
    position_msg.pose.orientation.z = pose_msg.pose.orientation.z;
    position_msg.pose.orientation.w = pose_msg.pose.orientation.w;
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

bool send_stabilazed (std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    static mavros_msgs::SetMode sm;
    sm.request.custom_mode = "STABILIZED";
    set_mode.call(sm);
    ros::spinOnce();
    res.success=true;
    return true;
}

bool killSwitch (std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    ros::Rate r(10);
    for(int i = 20; ros::ok() && i > 0; --i){ 
        mavros_msgs::CommandBool srv;
        srv.request.value = false;
        arming.call(srv);
        ros::spinOnce();
        r.sleep();
    }
    return true;
}

bool takeOff (Fast_drone::takeoff::Request& req, Fast_drone::takeoff::Response& res) {
    ros::Rate rate(20.0);
    for(int i = 100; ros::ok() && i > 0; --i){
        navi(0, 0, 1, "body");
        ros::spinOnce();
        rate.sleep();
        std::cout << "body z=" << req.z << "; step = " << i << std::endl;
    }
    for(int i = 200; ros::ok() && i > 0; --i){
        navi(0, 0, 1, "map");
        ros::spinOnce();
        rate.sleep();
        telebetry_dont_call = true;
        std::cout << "map z=" << req.z << "; step = " << i << std::endl;
    }
    res.s=true;
    return true;
}

bool takeOff_Land (Fast_drone::takeoff::Request& req, Fast_drone::takeoff::Response& res) {
    ros::Rate rate(20.0);
    for(int i = 50; ros::ok() && i > 0; --i){
        navi(0, 0, req.z, "body");
        ros::spinOnce();
        rate.sleep();
        std::cout << "body z=" << req.z << "; step = " << i << std::endl;
    }
    for(int i = 200; ros::ok() && i > 0; --i){
        navi(x_tf, y_tf, z_tf+0.5, "map");
        ros::spinOnce();
        rate.sleep();
        telebetry_dont_call = true;
        std::cout << "map z=" << req.z << "; step = " << i << std::endl;
    }
    std::cout << "LAND" << std::endl;
    static mavros_msgs::SetMode sm;
    sm.request.custom_mode = "AUTO.LAND";
    set_mode.call(sm);
    ros::spinOnce();
    res.s=true;
    return true;
}

bool publishPose (Fast_drone::publishPose::Request& req, Fast_drone::publishPose::Response& res) {
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

bool point (Fast_drone::publishPose::Request& req, Fast_drone::publishPose::Response& res) {
    ros::Rate rate(20.0);
    for(int i = 300; ros::ok() && i > 0; --i){
        navi(req.x, req.y, req.z, "map");
        ros::spinOnce();
        rate.sleep();
    }
    res.s=true;
    return true;
}

void handleState (const mavros_msgs::State& s) {
	state = s;
    // std::cout << (state.armed == true) << std::endl;
}

bool calibr (std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
    
    return true;
}
/*
inline void publishBodyFrame()
{
	if (body.child_frame_id.empty()) return;
	if (!body.header.stamp.isZero() && body.header.stamp == local_position.header.stamp) {
		return; // avoid TF_REPEATED_DATA warnings
	}

	tf::Quaternion q;
	q.setRPY(0, 0, tf::getYaw(local_position.pose.orientation));
	tf::quaternionTFToMsg(q, body.transform.rotation);

	body.transform.translation.x = local_position.pose.position.x;
	body.transform.translation.y = local_position.pose.position.y;
	body.transform.translation.z = local_position.pose.position.z;
	body.header.frame_id = local_position.header.frame_id;
	body.header.stamp = local_position.header.stamp;
	transform_broadcaster->sendTransform(body);
}

void handleLocalPosition(const geometry_msgs::PoseStamped& pose)
{
    ros:
	local_position = pose;
	publishBodyFrame();
	// TODO: terrain?, home?
}*/
int main(int argc, char **argv) {

    ros::init(argc, argv, "fly_fast_drone");
    ros::NodeHandle nh, nh_priv("~");

    offboard_timeout = ros::Duration(3.0);
	arming_timeout = ros::Duration(4.0);

    arming = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    position_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1);
    position_raw_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);

    telemetry_pub = nh.advertise<geometry_msgs::PoseStamped>("/fast/telemetry", 30);

    // ros::Subscriber move_base = nh.subscribe("move_base_simple/goal", 10, callback);
    move_base_pub = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);

    auto ld_serv = nh.advertiseService("fast/land", &land);

    auto takeOff_serv = nh.advertiseService("fast/takeOff", &takeOff);

    auto publishPose_serv = nh.advertiseService("fast/publish_pose", &publishPose);

    auto takeOff_and_land_serv = nh.advertiseService("fast/takeOff_land", &takeOff_Land);
    auto send_stabilazed_serv = nh.advertiseService("fast/stabilazed", &send_stabilazed);
    auto point_serv = nh.advertiseService("fast/point", &point);
    auto killSwitch_serv = nh.advertiseService("fast/killSwitch", &killSwitch);
    auto calibr_serv = nh.advertiseService("fast/calibr", &calibr);

    ros::Subscriber state_sub = nh.subscribe("/mavros/state", 30, handleState);

    // auto local_position_sub = nh.subscribe("mavros/local_position/pose", 1, &handleLocalPosition);

    // tf2_ros::TransformListener tf_listener(tf_buffer);
	// transform_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>();
	// static_transform_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>();

    // std::map<std::string, std::string> default_reference_frames;
	// default_reference_frames["body"] = "map";
	// default_reference_frames["base_link"] = "map";
	// reference_frames.insert(default_reference_frames.begin(), default_reference_frames.end());

    ROS_INFO("ready");
    // ros::spin();

    tf::TransformListener listener;
    ros::Rate rate_telem(30);
    while (nh.ok()) {
    //     tf_buffer.canTransform("map", "base_link", ros::Time(0));
        tf::StampedTransform transform;
        try {
            listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
        }
        pose_msg.header.frame_id = "map";
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.pose.position.x = transform.getOrigin().x();
        pose_msg.pose.position.y = transform.getOrigin().y();
        pose_msg.pose.position.z = transform.getOrigin().z();
        pose_msg.pose.orientation.x = transform.getRotation().getX();
        pose_msg.pose.orientation.y = transform.getRotation().getY();
        pose_msg.pose.orientation.z = transform.getRotation().getZ();
        pose_msg.pose.orientation.w = transform.getRotation().getW();
        telemetry_pub.publish(pose_msg);
        rate_telem.sleep();
        ros::spinOnce();
    }

    return 0;
}
