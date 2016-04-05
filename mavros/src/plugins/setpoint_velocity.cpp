/**
 * @brief SetpointVelocity plugin
 * @file setpoint_velocity.cpp
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 Nuno Marques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/TwistStamped.h>

namespace mavplugin {
/**
 * @brief Setpoint velocity plugin
 *
 * Send setpoint velocities to FCU controller.
 */
class SetpointVelocityPlugin : public MavRosPlugin,
	private SetPositionTargetLocalNEDMixin<SetpointVelocityPlugin> {
public:
	SetpointVelocityPlugin() :
		sp_nh("~setpoint_velocity"),
		uas(nullptr)
	{ };

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		//cmd_vel usually is the topic used for velocity control in many controllers / planners
		vel_sub = sp_nh.subscribe("cmd_vel", 10, &SetpointVelocityPlugin::vel_cb, this);
		vel_body_sub = sp_nh.subscribe("cmd_vel_body", 10, &SetpointVelocityPlugin::vel_body_cb, this);

	}

	const message_map get_rx_handlers() {
		return { /* Rx disabled */ };
	}

private:
	friend class SetPositionTargetLocalNEDMixin;
	ros::NodeHandle sp_nh;
	UAS *uas;

	ros::Subscriber vel_sub;
	ros::Subscriber vel_body_sub;

	/* -*- mid-level helpers -*- */

	/**
	 * @brief Send velocity to FCU velocity controller
	 *
	 * @warning Send only VX VY VZ. ENU frame.
	 */
	void send_setpoint_velocity(const ros::Time &stamp, Eigen::Vector3d &vel_enu, double yaw_rate) {
		/**
		 * Documentation start from bit 1 instead 0;
		 * Ignore position and accel vectors, yaw.
		 */
		uint16_t ignore_all_except_v_xyz_yr = (1 << 10) | (7 << 6) | (7 << 0);

		auto vel = UAS::transform_frame_enu_ned(vel_enu);
		auto yr = UAS::transform_frame_baselink_aircraft(Eigen::Vector3d(0.0, 0.0, yaw_rate));

		set_position_target_local_ned(stamp.toNSec() / 1000000,
				MAV_FRAME_LOCAL_NED,
				ignore_all_except_v_xyz_yr,
				0.0, 0.0, 0.0,
				vel.x(), vel.y(), vel.z(),
				0.0, 0.0, 0.0,
				0.0, yr.z());
	}

	/* -*- callbacks -*- */

	void vel_cb(const geometry_msgs::TwistStamped::ConstPtr &req) {
		Eigen::Vector3d vel_enu;

		tf::vectorMsgToEigen(req->twist.linear, vel_enu);
		send_setpoint_velocity(req->header.stamp, vel_enu,
				req->twist.angular.z);
	}

/** CHANGES from Kuehn for FLAIR Project **/
void send_body_setpoint_velocity(const ros::Time &stamp, Eigen::Vector3d &vel_ros, double yaw_rate) {
    /**
     * Documentation start from bit 1 instead 0;
     * Ignore position and accel vectors, yaw.
     */
    uint16_t ignore_all_except_v_xyz_yr = (1 << 10) | (7 << 6) | (7 << 0);

    //Own conversion from ROS body frame (X: forward Y: left Z: u; REP103) to NED (Ardupilot Body Representation: X: Forward Y: Right Z: Down)
    Eigen::Vector3d vel_ardupilot;
    vel_ardupilot(0) = vel_ros(0);
    vel_ardupilot(1) = vel_ros(1) * -1.0;
    vel_ardupilot(2) = vel_ros(2) * -1.0;

	auto yr = UAS::transform_frame_baselink_aircraft(Eigen::Vector3d(0.0, 0.0, yaw_rate));

    set_position_target_local_ned(stamp.toNSec() / 1000000,
        MAV_FRAME_BODY_OFFSET_NED, //BIG DIFFERENCE
        ignore_all_except_v_xyz_yr,
        0.0, 0.0, 0.0,
        vel_ardupilot.x(), vel_ardupilot.y(), vel_ardupilot.z(),
        0.0, 0.0, 0.0,
        0.0, yr.z());
  }


  void vel_body_cb(const geometry_msgs::TwistStamped::ConstPtr &req) {
    Eigen::Vector3d vel_enu;

    tf::vectorMsgToEigen(req->twist.linear, vel_enu);
    send_body_setpoint_velocity(req->header.stamp, vel_enu,
        req->twist.angular.z);
  }

};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::SetpointVelocityPlugin, mavplugin::MavRosPlugin)
