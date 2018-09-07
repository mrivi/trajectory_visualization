#ifndef _VISUALIZE_TRAJ_
#define _VISUALIZE_TRAJ_

#include <ros/ros.h>
#include <boost/bind.hpp>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/PoseStamped.h>

#include <mavros_msgs/Trajectory.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>


class VisualizeTraj {
 public:
  VisualizeTraj();
  ~VisualizeTraj() = default;

private:
	ros::NodeHandle _nh;
	ros::Subscriber _position_sub;
	ros::Subscriber _fcu_input_sub;

	tf::TransformListener _tf_listener;

	void positionCallback(const geometry_msgs::PoseStamped &msg);
	void fcuInputGoalCallback(const mavros_msgs::Trajectory &msg);
	void publishNavigatorWaypoints(const geometry_msgs::Point &target);
	void publishSetpoints(const geometry_msgs::Point &pos_sp, const geometry_msgs::Point &vel_sp);
	void publishAcceptanceRadius(const geometry_msgs::Point &target);
	void publishPosition(const geometry_msgs::PoseStamped &pos);

	ros::Publisher _path_pub;
	ros::Publisher _marker_target_pub;
	ros::Publisher _marker_acc_rad_pub;
	ros::Publisher _marker_position_pub;
	ros::Publisher _vel_sp_length_pub;

	geometry_msgs::Point _position;
	geometry_msgs::Point _velocity;
	geometry_msgs::Point _position_sp;
	geometry_msgs::Point _velocity_sp;
	geometry_msgs::Point _target;
	geometry_msgs::Point _next;

	int _counter_nav_wp = 0;
	int _counter_pos = 0;

	float _acceptance_radius = 5.0f;
};

#endif // _VISUALIZE_TRAJ_