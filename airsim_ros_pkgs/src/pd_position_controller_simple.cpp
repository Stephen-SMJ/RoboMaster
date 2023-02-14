#include "pd_position_controller_simple.h"

bool PIDParams::load_from_rosparams(const ros::NodeHandle& nh)
{
    bool found = true;

    // found = found && nh.getParam("kp_x", kp_x);
    // found = found && nh.getParam("kp_y", kp_y);
    // found = found && nh.getParam("kp_z", kp_z);
    // found = found && nh.getParam("kp_yaw", kp_yaw);

    // found = found && nh.getParam("kd_x", kd_x);
    // found = found && nh.getParam("kd_y", kd_y);
    // found = found && nh.getParam("kd_z", kd_z);
    // found = found && nh.getParam("kd_yaw", kd_yaw);

    // found = found && nh.getParam("reached_thresh_xyz", reached_thresh_xyz);
    // found = found && nh.getParam("reached_yaw_degrees", reached_yaw_degrees);
    // ROS_INFO("found: %i", found);
    kp_x = 0.5;
    kp_y = 0.5;
    kp_z = 0.5;
    kd_yaw = 0.5;

    kd_x = 0.1;
    kd_y = 0.1;
    kd_z = 0.1;
    kd_yaw = 0.1;

    reached_thresh_xyz = 1;
    reached_yaw_degrees = 0.1;
    return found;
}

bool DynamicConstraints::load_from_rosparams(const ros::NodeHandle& nh)
{
    bool found = true;

    // found = found && nh.getParam("max_vel_horz_abs", max_vel_horz_abs);
    // found = found && nh.getParam("max_vel_vert_abs", max_vel_vert_abs);
    // found = found && nh.getParam("max_yaw_rate_degree", max_yaw_rate_degree);
    max_vel_horz_abs = 5;
    max_vel_vert_abs = 5;
    max_yaw_rate_degree = 90;

    return found;
}

PIDPositionController::PIDPositionController(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), has_home_geo_(false), reached_goal_(false), has_goal_(false), has_odom_(false), got_goal_once_(false)
{
    params_.load_from_rosparams(nh_private_);
    constraints_.load_from_rosparams(nh_);
    initialize_ros();
    reset_errors();
}

void PIDPositionController::reset_errors()
{
    prev_error_.x = 0.0;
    prev_error_.y = 0.0;
    prev_error_.z = 0.0;
    prev_error_.yaw = 0.0;
}

void PIDPositionController::initialize_ros()
{
    vel_cmd_ = airsim_ros_pkgs::VelCmd();
    // ROS params
    double update_control_every_n_sec;
    update_control_every_n_sec = 0.03;

    std::string vehicle_name;
    while (vehicle_name == "") {
        nh_private_.getParam("/vehicle_name", vehicle_name);
        ROS_INFO_STREAM("Waiting vehicle name");
    }
    // ROS publishers
    // airsim_vel_cmd_world_frame_pub_ = nh_.advertise<airsim_ros_pkgs::VelCmd>("/airsim_node/drone_1/vel_cmd_world_frame", 1);
    airsim_vel_cmd_body_frame_pub_ = nh_.advertise<airsim_ros_pkgs::VelCmd>("/airsim_node/drone_1/vel_cmd_body_frame", 1);
    // ROS subscribers
    airsim_odom_sub_ = nh_.subscribe("/airsim_node/drone_1/odom_local_ned", 50, &PIDPositionController::airsim_odom_cb, this);
    home_geopoint_sub_ = nh_.subscribe("/airsim_node/home_geo_point", 50, &PIDPositionController::home_geopoint_cb, this);
    // todo publish this under global nodehandle / "airsim node" and hide it from user
    local_position_goal_srvr_ = nh_.advertiseService("/airsim_node/local_position_goal", &PIDPositionController::local_position_goal_srv_cb, this);
    local_position_goal_override_srvr_ = nh_.advertiseService("/airsim_node/local_position_goal/override", &PIDPositionController::local_position_goal_srv_override_cb, this);
    // gps_goal_srvr_ = nh_.advertiseService("/airsim_node/gps_goal", &PIDPositionController::gps_goal_srv_cb, this);
    // gps_goal_override_srvr_ = nh_.advertiseService("/airsim_node/gps_goal/override", &PIDPositionController::gps_goal_srv_override_cb, this);

    // ROS timers
    update_control_cmd_timer_ = nh_private_.createTimer(ros::Duration(update_control_every_n_sec), &PIDPositionController::update_control_cmd_timer_cb, this);
}

void PIDPositionController::home_geopoint_cb(const airsim_ros_pkgs::GPSYaw& gps_msg)
{
    ROS_INFO("get home geo.");
}


void PIDPositionController::airsim_odom_cb(const nav_msgs::Odometry& odom_msg)
{
    has_odom_ = true;
    curr_odom_ = odom_msg;
    curr_position_.x = odom_msg.pose.pose.position.x;
    curr_position_.y = odom_msg.pose.pose.position.y;
    curr_position_.z = odom_msg.pose.pose.position.z;
    curr_position_.yaw = utils::get_yaw_from_quat_msg(odom_msg.pose.pose.orientation);
}

// todo maintain internal representation as eigen vec?
// todo check if low velocity if within thresh?
// todo maintain separate errors for XY and Z
void PIDPositionController::check_reached_goal()
{
    double diff_xyz = sqrt((target_position_.x - curr_position_.x) * (target_position_.x - curr_position_.x) + (target_position_.y - curr_position_.y) * (target_position_.y - curr_position_.y) + (target_position_.z - curr_position_.z) * (target_position_.z - curr_position_.z));

    double diff_yaw = math_common::angular_dist(target_position_.yaw, curr_position_.yaw);

    // todo save this in degrees somewhere to avoid repeated conversion
    if (diff_xyz < params_.reached_thresh_xyz && diff_yaw < math_common::deg2rad(params_.reached_yaw_degrees))
        reached_goal_ = true;
}

bool PIDPositionController::local_position_goal_srv_cb(airsim_ros_pkgs::SetLocalPosition::Request& request, airsim_ros_pkgs::SetLocalPosition::Response& response)
{
    response.success = false;

    // this tells the update timer callback to not do active hovering
    if (!got_goal_once_)
        got_goal_once_ = true;

    if (has_goal_ && !reached_goal_) {
        // todo maintain array of position goals
        ROS_ERROR_STREAM("[PIDPositionController] denying position goal request. I am still following the previous goal");
        return response.success;
    }

    if (!has_goal_) {
        target_position_.x = request.x;
        target_position_.y = request.y;
        target_position_.z = request.z;
        target_position_.yaw = request.yaw;
        ROS_INFO_STREAM("[PIDPositionController] got goal: x=" << target_position_.x << " y=" << target_position_.y << " z=" << target_position_.z << " yaw=" << target_position_.yaw);

        // todo error checks
        // todo fill response
        has_goal_ = true;
        reached_goal_ = false;
        reset_errors(); // todo
        response.success = true;
        return response.success;
    }

    // Already have goal, and have reached it
    ROS_INFO_STREAM("[PIDPositionController] Already have goal and have reached it");
    return response.success;
}

bool PIDPositionController::local_position_goal_srv_override_cb(airsim_ros_pkgs::SetLocalPosition::Request& request, airsim_ros_pkgs::SetLocalPosition::Response& response)
{
    // this tells the update timer callback to not do active hovering
    if (!got_goal_once_)
        got_goal_once_ = true;

    target_position_.x = request.x;
    target_position_.y = request.y;
    target_position_.z = request.z;
    target_position_.yaw = request.yaw;
    ROS_INFO_STREAM("[PIDPositionController] got goal: x=" << target_position_.x << " y=" << target_position_.y << " z=" << target_position_.z << " yaw=" << target_position_.yaw);

    // todo error checks
    // todo fill response
    has_goal_ = true;
    reached_goal_ = false;
    reset_errors(); // todo
    response.success = true;
    return response.success;
}


void PIDPositionController::update_control_cmd_timer_cb(const ros::TimerEvent& event)
{
    // todo check if odometry is too old!!
    // if no odom, don't do anything.
    if (!has_odom_) {
        ROS_ERROR_STREAM("[PIDPositionController] Waiting for odometry!");
        return;
    }

    if (has_goal_) {
        check_reached_goal();
        if (reached_goal_) {
            ROS_INFO_STREAM("[PIDPositionController] Reached goal! Hovering at position.");
            has_goal_ = false;
            // dear future self, this function doesn't return coz we need to keep on actively hovering at last goal pose. don't act smart
        }
        else {
            ROS_INFO_STREAM("[PIDPositionController] Moving to goal.");
        }
    }

    // only compute and send control commands for hovering / moving to pose, if we received a goal at least once in the past
    if (got_goal_once_) {
        compute_control_cmd();
        enforce_dynamic_constraints();
        publish_control_cmd();
    }
}

void PIDPositionController::compute_control_cmd()
{
    ROS_INFO("cur position:%f %f %f %f", curr_position_.x, curr_position_.y, curr_position_.z, curr_position_.yaw);
    ROS_INFO("tar position:%f %f %f %f", target_position_.x, target_position_.y, target_position_.z, target_position_.yaw);
    curr_error_.x = target_position_.x - curr_position_.x;
    curr_error_.y = target_position_.y - curr_position_.y;
    curr_error_.z = target_position_.z - curr_position_.z;
    curr_error_.yaw = math_common::angular_dist(curr_position_.yaw, target_position_.yaw);

    double p_term_x = params_.kp_x * curr_error_.x;
    double p_term_y = params_.kp_y * curr_error_.y;
    double p_term_z = params_.kp_z * curr_error_.z;
    double p_term_yaw = params_.kp_yaw * curr_error_.yaw;

    double d_term_x = params_.kd_x * prev_error_.x;
    double d_term_y = params_.kd_y * prev_error_.y;
    double d_term_z = params_.kd_z * prev_error_.z;
    double d_term_yaw = params_.kp_yaw * prev_error_.yaw;

    prev_error_ = curr_error_;

    vel_cmd_.twist.linear.x = p_term_x + d_term_x;
    vel_cmd_.twist.linear.y = p_term_y + d_term_y;
    vel_cmd_.twist.linear.z = p_term_z + d_term_z;
    vel_cmd_.twist.angular.z = p_term_yaw + d_term_yaw; // todo
}

void PIDPositionController::enforce_dynamic_constraints()
{
    double vel_norm_horz = sqrt((vel_cmd_.twist.linear.x * vel_cmd_.twist.linear.x) + (vel_cmd_.twist.linear.y * vel_cmd_.twist.linear.y));

    if (vel_norm_horz > constraints_.max_vel_horz_abs) {
        vel_cmd_.twist.linear.x = (vel_cmd_.twist.linear.x / vel_norm_horz) * constraints_.max_vel_horz_abs;
        vel_cmd_.twist.linear.y = (vel_cmd_.twist.linear.y / vel_norm_horz) * constraints_.max_vel_horz_abs;
    }

    if (std::fabs(vel_cmd_.twist.linear.z) > constraints_.max_vel_vert_abs) {
        // todo just add a sgn funciton in common utils? return double to be safe.
        // template <typename T> double sgn(T val) { return (T(0) < val) - (val < T(0)); }
        vel_cmd_.twist.linear.z = (vel_cmd_.twist.linear.z / std::fabs(vel_cmd_.twist.linear.z)) * constraints_.max_vel_vert_abs;
    }
    // todo yaw limits
    if (std::fabs(vel_cmd_.twist.linear.z) > constraints_.max_yaw_rate_degree) {
        // todo just add a sgn funciton in common utils? return double to be safe.
        // template <typename T> double sgn(T val) { return (T(0) < val) - (val < T(0)); }
        vel_cmd_.twist.linear.z = (vel_cmd_.twist.linear.z / std::fabs(vel_cmd_.twist.linear.z)) * constraints_.max_yaw_rate_degree;
    }
}

void PIDPositionController::publish_control_cmd()
{
    airsim_vel_cmd_body_frame_pub_.publish(vel_cmd_);
    ROS_INFO("velcmd: %f %f %f %f", vel_cmd_.twist.linear.x, vel_cmd_.twist.linear.y, vel_cmd_.twist.linear.z, vel_cmd_.twist.angular.z);
}
