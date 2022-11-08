#include "bspline/non_uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "plan_manage/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include <std_msgs/Bool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "mavros_msgs/PositionTarget.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>
#include <cmath>

ros::Publisher cmd_vis_pub, pos_cmd_pub, traj_pub;


nav_msgs::Odometry odom;
quadrotor_msgs::PositionCommand cmd;


quadrotor_msgs::PositionCommand cmd_last;
ros::Publisher clover_cmd_pub;
bool mavros_position_sub = true;
// geometry_msgs::PoseStamped cmd_clover;
mavros_msgs::PositionTarget cmd_clover, cmd_clover_last;
geometry_msgs::PoseStamped cmd_move_base;
bool flag_move_base = false;


// double pos_gain[3] = {5.7, 5.7, 6.2};
// double vel_gain[3] = {3.4, 3.4, 4.0};
double pos_gain[3] = { 5.7, 5.7, 6.2 };
double vel_gain[3] = { 3.4, 3.4, 4.0 };

using fast_planner::NonUniformBspline;

bool receive_traj_ = false;
vector<NonUniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;

// yaw control
double last_yaw_;
double time_forward_;

vector<Eigen::Vector3d> traj_cmd_, traj_real_;

void displayTrajWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                          int id) {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.type = visualization_msgs::Marker::SPHERE_LIST;
    mk.action = visualization_msgs::Marker::DELETE;
    mk.id = id;

    traj_pub.publish(mk);

    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);

    mk.scale.x = resolution;
    mk.scale.y = resolution;
    mk.scale.z = resolution;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(path.size()); i++) {
        pt.x = path[i](0);
        pt.y = path[i](1);
        pt.z = path[i](2);
        mk.points.push_back(pt);
    }
    traj_pub.publish(mk);
    ros::Duration(0.001).sleep();
}

void drawCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
             const Eigen::Vector4d& color) {
    visualization_msgs::Marker mk_state;
    mk_state.header.frame_id = "world";
    mk_state.header.stamp = ros::Time::now();
    mk_state.id = id;
    mk_state.type = visualization_msgs::Marker::ARROW;
    mk_state.action = visualization_msgs::Marker::ADD;

    mk_state.pose.orientation.w = 1.0;
    mk_state.scale.x = 0.1;
    mk_state.scale.y = 0.2;
    mk_state.scale.z = 0.3;

    geometry_msgs::Point pt;
    pt.x = pos(0);
    pt.y = pos(1);
    pt.z = pos(2);
    mk_state.points.push_back(pt);

    pt.x = pos(0) + vec(0);
    pt.y = pos(1) + vec(1);
    pt.z = pos(2) + vec(2);
    mk_state.points.push_back(pt);

    mk_state.color.r = color(0);
    mk_state.color.g = color(1);
    mk_state.color.b = color(2);
    mk_state.color.a = color(3);

    cmd_vis_pub.publish(mk_state);
}

void bsplineCallback(plan_manage::BsplineConstPtr msg) {
    // parse pos traj

    Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);

    Eigen::VectorXd knots(msg->knots.size());
    for (int i = 0; i < msg->knots.size(); ++i) {
        knots(i) = msg->knots[i];
    }

    for (int i = 0; i < msg->pos_pts.size(); ++i) {
        pos_pts(i, 0) = msg->pos_pts[i].x;
        pos_pts(i, 1) = msg->pos_pts[i].y;
        pos_pts(i, 2) = msg->pos_pts[i].z;
    }

    NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
    pos_traj.setKnot(knots);

    // parse yaw traj

    Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
    for (int i = 0; i < msg->yaw_pts.size(); ++i) {
        yaw_pts(i, 0) = msg->yaw_pts[i];
    }

    NonUniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

    start_time_ = msg->start_time;
    traj_id_ = msg->traj_id;

    traj_.clear();
    traj_.push_back(pos_traj);
    traj_.push_back(traj_[0].getDerivative());
    traj_.push_back(traj_[1].getDerivative());
    traj_.push_back(yaw_traj);
    traj_.push_back(yaw_traj.getDerivative());

    traj_duration_ = traj_[0].getTimeSum();

    receive_traj_ = true;
}

void replanCallback(std_msgs::Empty msg) {
    /* reset duration */
    const double time_out = 0.01;
    ros::Time time_now = ros::Time::now();
    double t_stop = (time_now - start_time_).toSec() + time_out;
    traj_duration_ = min(t_stop, traj_duration_);
}

void newCallback(std_msgs::Empty msg) {
    traj_cmd_.clear();
    traj_real_.clear();
}

void odomCallbck(const nav_msgs::Odometry& msg) {
    if (msg.child_frame_id == "X" || msg.child_frame_id == "O") return;

    odom = msg;

    traj_real_.push_back(
            Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

    if (traj_real_.size() > 10000) traj_real_.erase(traj_real_.begin(), traj_real_.begin() + 1000);
}

void visCallback(const ros::TimerEvent& e) {
    // displayTrajWithColor(traj_real_, 0.03, Eigen::Vector4d(0.925, 0.054, 0.964,
    // 1),
    //                      1);

    displayTrajWithColor(traj_cmd_, 0.05, Eigen::Vector4d(0, 1, 0, 1), 2);
}

void move_base_simpleCallbck(const geometry_msgs::PoseStamped& msg) {
    cmd_move_base = msg;
    flag_move_base = true;
    // double delta_x = cmd_move_base.pose.position.x - cmd_clover.position.x;
    // double delta_y = cmd_move_base.pose.position.y - cmd_clover.position.y;
    // if (abs(delta_y) > 0.001) {
    //     // cout << endl << endl << "!!!!!!!!!!!!!!!!!!!!!![INFO] CMD = " << atan(delta_x / delta_y) << endl << endl;
    //     cmd_clover_last.yaw -= atan(delta_y / delta_x);
    //     clover_cmd_pub.publish(cmd_clover_last);

    //     cmd_last.yaw = cmd_clover_last.yaw;
    //     pos_cmd_pub.publish(cmd_last);
    // }


    if (!receive_traj_) return;

    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - start_time_).toSec();
    double yaw;

    if (t_cur < traj_duration_ && t_cur >= 0.0) {
        yaw = traj_[3].evaluateDeBoorT(t_cur)[0];
    } else if (t_cur >= traj_duration_) {
        yaw = traj_[3].evaluateDeBoorT(traj_duration_)[0];
    } else {
        cout << "[Traj server]: invalid time." << endl;
    }
    cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! " << yaw << " !!!!!!!!!!!!!!!!!!!! "<< cmd_clover_last.yaw << endl;
    cmd_clover_last.yaw = yaw;
    clover_cmd_pub.publish(cmd_clover_last);

    cmd_last.yaw = cmd_clover_last.yaw;
    pos_cmd_pub.publish(cmd_last);
}

void flagMavrosPositionCallbck(const std_msgs::Bool& Reached){
    mavros_position_sub = Reached.data; // true or fals
}

void cmdCallback(const ros::TimerEvent& e) {
    if (flag_move_base) {
        ros::Duration(0.5).sleep();
        flag_move_base = false;
    }
    else{
        if (!receive_traj_) return;

        ros::Time time_now = ros::Time::now();
        double t_cur = (time_now - start_time_).toSec();

        Eigen::Vector3d pos, vel, acc, pos_f;
        double yaw, yawdot;

        if (t_cur < traj_duration_ && t_cur >= 0.0) {
            pos = traj_[0].evaluateDeBoorT(t_cur);
            vel = traj_[1].evaluateDeBoorT(t_cur);
            acc = traj_[2].evaluateDeBoorT(t_cur);
            yaw = traj_[3].evaluateDeBoorT(t_cur)[0];
            yawdot = traj_[4].evaluateDeBoorT(t_cur)[0];

            double tf = min(traj_duration_, t_cur + 2.0);
            pos_f = traj_[0].evaluateDeBoorT(tf);

        } else if (t_cur >= traj_duration_) {
            /* hover when finish traj_ */
            pos = traj_[0].evaluateDeBoorT(traj_duration_);
            vel.setZero();
            acc.setZero();
            yaw = traj_[3].evaluateDeBoorT(traj_duration_)[0];
            yawdot = traj_[4].evaluateDeBoorT(traj_duration_)[0];

            pos_f = pos;

        } else {
            cout << "[Traj server]: invalid time." << endl;
        }

        cmd.header.stamp = time_now;
        cmd.header.frame_id = "world";
        cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
        cmd.trajectory_id = traj_id_;

        cmd.position.x = pos(0);
        cmd.position.y = pos(1);
        cmd.position.z = pos(2);

        cmd.velocity.x = vel(0);
        cmd.velocity.y = vel(1);
        cmd.velocity.z = vel(2);

        cmd.acceleration.x = acc(0);
        cmd.acceleration.y = acc(1);
        cmd.acceleration.z = acc(2);

        cmd.yaw = yaw;
        cmd.yaw_dot = yawdot;

        auto pos_err = pos_f - pos;
        // if (pos_err.norm() > 1e-3) {
        //   cmd.yaw = atan2(pos_err(1), pos_err(0));
        // } else {
        //   cmd.yaw = last_yaw_;
        // }
        // cmd.yaw_dot = 1.0;

        last_yaw_ = cmd.yaw;
        // draw cmd

        // drawCmd(pos, vel, 0, Eigen::Vector4d(0, 1, 0, 1));
        // drawCmd(pos, acc, 1, Eigen::Vector4d(0, 0, 1, 1));

        Eigen::Vector3d dir(cos(yaw), sin(yaw), 0.0);
        drawCmd(pos, 2 * dir, 2, Eigen::Vector4d(1, 1, 0, 0.7));
        // drawCmd(pos, pos_err, 3, Eigen::Vector4d(1, 1, 0, 0.7));

        // cmd_clover.header.stamp = cmd.header.stamp;
        // cmd_clover.header.frame_id = "map";

        // cmd_clover.pose.position.x = cmd.position.x;
        // cmd_clover.pose.position.y = cmd.position.y;
        // cmd_clover.pose.position.z = cmd.position.z;

        // tf2::Quaternion myQuaternion;
        // myQuaternion.setRPY(0, 0, yaw);
        // myQuaternion=myQuaternion.normalize();
        // cmd_clover.pose.orientation.x = myQuaternion[0];
        // cmd_clover.pose.orientation.y = myQuaternion[1];
        // cmd_clover.pose.orientation.z = myQuaternion[2];
        // cmd_clover.pose.orientation.w = myQuaternion[3];

        cmd_clover.header.stamp = cmd.header.stamp;
        cmd_clover.header.frame_id = "map";
        cmd_clover.coordinate_frame = 1;
        cmd_clover.position.x = cmd.position.x;
        cmd_clover.position.y = cmd.position.y;
        cmd_clover.position.z = cmd.position.z;
        cmd_clover.velocity = cmd.velocity;
        cmd_clover.acceleration_or_force = cmd.acceleration;
        cmd_clover.yaw = cmd.yaw;
        cmd_clover.yaw_rate = cmd.yaw_dot;

//     if (flag_move_base) {
// //        cmd_clover.yaw -= cmd_move_base.position;
//         double delta_x = cmd_move_base.pose.position.x - cmd_clover.position.x;
//         double delta_y = cmd_move_base.pose.position.y - cmd_clover.position.y;
//         if (abs(delta_y) > 0.001) {
//             cout << endl << endl << "!!!!!!!!!!!!!!!!!!!!!![INFO] CMD = " << atan(delta_x / delta_y) << endl << endl;
// //            cmd_clover.yaw += atan(delta_x / delta_y);
//             cmd_clover_last.yaw += atan(delta_x / delta_y);
//             clover_cmd_pub.publish(cmd_clover_last);

//             cmd_last.yaw = cmd_clover_last.yaw;
//             pos_cmd_pub.publish(cmd_last);

//             clover_cmd_pub.publish(cmd_clover);
//             traj_cmd_.push_back(pos);
//             ros::Duration(1.0).sleep();
//         }
//         flag_move_base = false;
//     }
//     else{
//         if (mavros_position_sub){
//             pos_cmd_pub.publish(cmd);
//             clover_cmd_pub.publish(cmd_clover);
//             traj_cmd_.push_back(pos);
//         }
//     }

        if (mavros_position_sub){
            pos_cmd_pub.publish(cmd);
            clover_cmd_pub.publish(cmd_clover);
            traj_cmd_.push_back(pos);
        }
        cmd_clover_last = cmd_clover;
        cmd_last = cmd;

        if (traj_cmd_.size() > 10000) traj_cmd_.erase(traj_cmd_.begin(), traj_cmd_.begin() + 1000);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "traj_server");
    ros::NodeHandle node;
    ros::NodeHandle nh("~");

    ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);
    ros::Subscriber replan_sub = node.subscribe("planning/replan", 10, replanCallback);
    ros::Subscriber new_sub = node.subscribe("planning/new", 10, newCallback);
    ros::Subscriber odom_sub = node.subscribe("/odom_world", 50, odomCallbck);

    cmd_vis_pub = node.advertise<visualization_msgs::Marker>("planning/position_cmd_vis", 10);
    pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
    traj_pub = node.advertise<visualization_msgs::Marker>("planning/travel_traj", 10);

    ros::Subscriber move_base_simple_sub = node.subscribe("/move_base_simple/goal", 10, move_base_simpleCallbck);
    ros::Subscriber flag_mavros_sub = node.subscribe("/flag/mavros/position", 10, flagMavrosPositionCallbck);
    // clover_cmd_pub = node.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 50);
    clover_cmd_pub = node.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 50);

    //
    ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);
    ros::Timer vis_timer = node.createTimer(ros::Duration(0.25), visCallback);

    /* control parameter */
    cmd.kx[0] = pos_gain[0];
    cmd.kx[1] = pos_gain[1];
    cmd.kx[2] = pos_gain[2];

    cmd.kv[0] = vel_gain[0];
    cmd.kv[1] = vel_gain[1];
    cmd.kv[2] = vel_gain[2];

    nh.param("traj_server/time_forward", time_forward_, -1.0);
    last_yaw_ = 0.0;

    ros::Duration(1.0).sleep();

    ROS_WARN("[Traj server]: ready.");

    ros::spin();

    return 0;
}
