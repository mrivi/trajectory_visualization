#include "visualize_traj.hpp"

VisualizeTraj::VisualizeTraj() {
  _nh = ros::NodeHandle("~");

  _position_sub = _nh.subscribe(
      "/mavros/local_position/pose", 1, &VisualizeTraj::positionCallback,
      this);

  _fcu_input_sub = _nh.subscribe("/mavros/trajectory/desired", 1,
                                 &VisualizeTraj::fcuInputGoalCallback, this);

  _path_pub = _nh.advertise<nav_msgs::Path>("/path_actual", 1);
  _marker_target_pub =
      _nh.advertise<visualization_msgs::MarkerArray>("/target", 1);
   _marker_acc_rad_pub =
      _nh.advertise<visualization_msgs::MarkerArray>("/acceptance_radius", 1);

  _marker_position_pub = _nh.advertise<visualization_msgs::Marker>("/position", 1);

  _vel_sp_length_pub = _nh.advertise<std_msgs::Float32>("/vel_sp_length", 1);
}

void VisualizeTraj::positionCallback(const geometry_msgs::PoseStamped &msg) {
  // auto rot_msg = msg;

 
  // // _tf_listener.transformPose("world", ros::Time(0), msg, "local_origin",
  // //                            rot_msg);
  // _position = rot_msg;
  // //curr_yaw_ = tf::getYaw(rot_msg.pose.orientation);
  // publishPosition(_position);
}


void VisualizeTraj::publishPosition(const geometry_msgs::PoseStamped &msg) {
  nav_msgs::Path path_actual;
  path_actual.header.stamp = msg.header.stamp;
  path_actual.header.frame_id ="local_origin";
  path_actual.poses.push_back(msg);
  _path_pub.publish(path_actual);
}

void VisualizeTraj::fcuInputGoalCallback(
    const mavros_msgs::Trajectory &msg) {

	_target.x = msg.point_2.position.x;
	_target.y = msg.point_2.position.y;
	_target.z = msg.point_2.position.z;
  publishNavigatorWaypoints(_target);
  publishAcceptanceRadius(_target);

  _next.x = msg.point_3.position.x;
  _next.y = msg.point_3.position.y;
  _next.z = msg.point_3.position.z;
  publishNavigatorWaypoints(_next);
  publishAcceptanceRadius(_next);

	// _velocity.x = msg.point_1.velocity.x;
	// _velocity.y = msg.point_1.velocity.y;
	// _velocity.z = msg.point_1.velocity.z;

 //  _position.x = msg.point_1.position.x;
 //  _position.y = msg.point_1.position.y;
 //  _position.z = msg.point_1.position.z;
  // publishPosition(_position);

  _velocity_sp.x = msg.point_1.velocity.x;
  _velocity_sp.y = msg.point_1.velocity.y;
  _velocity_sp.z = msg.point_1.velocity.z;

  _position_sp.x = msg.point_1.position.x;
  _position_sp.y = msg.point_1.position.y;
  _position_sp.z = msg.point_1.position.z;
  publishSetpoints(_position_sp, _velocity_sp);

  std_msgs::Float32 vel_sp_length;
  vel_sp_length.data = sqrt(_velocity_sp.x * _velocity_sp.x +
    _velocity_sp.y * _velocity_sp.y +
    _velocity_sp.z * _velocity_sp.z);

  _vel_sp_length_pub.publish(vel_sp_length);
  
  
}

void VisualizeTraj::publishSetpoints(const geometry_msgs::Point &pos_sp, const geometry_msgs::Point &vel_sp) {
  visualization_msgs::Marker m = {};

  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.type = visualization_msgs::Marker::ARROW;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 0.05;
  m.scale.y = 0.05;
  m.scale.z = 0.05;
  m.color.a = 1.0;
  m.color.r = 0.0;
  m.color.g = 1.0;
  m.color.b = 0.0;
  m.lifetime = ros::Duration();
  m.id = _counter_pos;
  m.points.push_back(pos_sp);
  geometry_msgs::Point end_point;
  end_point.x = vel_sp.x + pos_sp.x;
   end_point.y = vel_sp.y + pos_sp.y;
    end_point.z = vel_sp.z + pos_sp.z;
  m.points.push_back(end_point);
  
  _marker_position_pub.publish(m);
  _counter_pos++;
}

void VisualizeTraj::publishNavigatorWaypoints(const geometry_msgs::Point &target) {
  visualization_msgs::MarkerArray marker_target;
  visualization_msgs::Marker m;

  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.type = visualization_msgs::Marker::SPHERE;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = 0.8;
  m.scale.y = 0.8;
  m.scale.z = 0.8;
  m.color.a = 1.0;
  m.color.r = 1.0;
  m.color.g = 1.0;
  m.color.b = 0.0;
  m.lifetime = ros::Duration();
  m.id = _counter_nav_wp;
  m.pose.position = target;
  marker_target.markers.push_back(m);
  _marker_target_pub.publish(marker_target);
  _counter_nav_wp++;
}

void VisualizeTraj::publishAcceptanceRadius(const geometry_msgs::Point &target) {

  visualization_msgs::MarkerArray marker;
  visualization_msgs::Marker m;

  m.header.frame_id = "map";
  m.header.stamp = ros::Time::now();
  m.type = visualization_msgs::Marker::SPHERE;
  m.action = visualization_msgs::Marker::ADD;
  m.scale.x = _acceptance_radius;
  m.scale.y = _acceptance_radius;
  m.scale.z = _acceptance_radius;
  m.color.a = 0.01;
  m.color.r = 0.0;
  m.color.g = 0.0;
  m.color.b = 0.8;
  m.lifetime = ros::Duration();
  m.id = _counter_nav_wp;
  m.pose.position = target;
  marker.markers.push_back(m);
  _marker_acc_rad_pub.publish(marker);

}


int main(int argc, char** argv) {

  ros::init(argc, argv, "vizualize_traj");
  VisualizeTraj visualize;
  ros::spin();
  return 0;
}