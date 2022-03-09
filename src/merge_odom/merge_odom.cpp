#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ecl/threads.hpp>
#include <tf/transform_broadcaster.h>

const char base_link[] = "base_footprint";
const char odom[] = "odom";
bool publish_tf = true;

geometry_msgs::PoseWithCovarianceStamped shared_elbrus_pose;
nav_msgs::Odometry shared_wheel_odom;

ecl::Mutex mutex_elbrus;
ecl::Mutex mutex_wheel;

void elbrusPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose_cov) {
  mutex_elbrus.lock();
  shared_elbrus_pose = pose_cov;
  mutex_elbrus.unlock();

  // ROS_INFO("Elbrus msg");
}

void wheelOdomCallback(const nav_msgs::Odometry& odom) {
  mutex_elbrus.lock();
  shared_wheel_odom = odom;
  mutex_elbrus.unlock();

  // ROS_INFO("Wheel msg");
}

void mergeOdoms(nav_msgs::Odometry& output_odom) {
  mutex_elbrus.lock();
  mutex_wheel.lock();

  output_odom.header.stamp          = ros::Time::now();
  output_odom.header.frame_id       = odom;
  output_odom.child_frame_id        = base_link;

  output_odom.pose.pose.position.x  = 0.5*shared_elbrus_pose.pose.pose.position.x + 0.5*shared_wheel_odom.pose.pose.position.x;  // _cur_x;
  output_odom.pose.pose.position.y  = 0.5*shared_elbrus_pose.pose.pose.position.y + 0.5*shared_wheel_odom.pose.pose.position.y;  // _cur_y;
  output_odom.pose.pose.position.z  = 0.5*shared_elbrus_pose.pose.pose.position.z + 0.5*shared_wheel_odom.pose.pose.position.z;
  output_odom.pose.pose.orientation.x = 0.5*shared_elbrus_pose.pose.pose.orientation.x + 0.5*shared_wheel_odom.pose.pose.orientation.x;
  output_odom.pose.pose.orientation.y = 0.5*shared_elbrus_pose.pose.pose.orientation.y + 0.5*shared_wheel_odom.pose.pose.orientation.y;
  output_odom.pose.pose.orientation.z = 0.5*shared_elbrus_pose.pose.pose.orientation.z + 0.5*shared_wheel_odom.pose.pose.orientation.z;
  output_odom.pose.pose.orientation.w = 0.5*shared_elbrus_pose.pose.pose.orientation.w + 0.5*shared_wheel_odom.pose.pose.orientation.w; // _cur_theta;

  output_odom.twist.twist.linear.x  = shared_wheel_odom.twist.twist.linear.x;  // vx;
  output_odom.twist.twist.linear.y  = shared_wheel_odom.twist.twist.linear.y;
  output_odom.twist.twist.angular.z = shared_wheel_odom.twist.twist.angular.z;  // vth;

  mutex_elbrus.unlock();
  mutex_wheel.unlock();

  if (publish_tf) {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(output_odom.pose.pose.position.x,
                                    output_odom.pose.pose.position.y,
                                    output_odom.pose.pose.position.z));
    tf::Quaternion q;
    q.setRPY(output_odom.pose.pose.orientation.x,
             output_odom.pose.pose.orientation.y,
             output_odom.pose.pose.orientation.z);

    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), odom, base_link));
    ROS_INFO("Merged!");
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "merge_odom");
  ros::NodeHandle n;

  ros::Publisher merge_odom_pub = n.advertise<nav_msgs::Odometry>("/merge_odom", 10);

  ros::Subscriber elbrus_sub = n.subscribe("/elbrus_pose_cov", 10, elbrusPoseCallback);
  ros::Subscriber wheel_sub = n.subscribe("/wheel_odometry", 10, wheelOdomCallback);

  ros::Rate loop_rate(30);

  while (ros::ok())
  {
    nav_msgs::Odometry output_odom;
    mergeOdoms(output_odom);
    merge_odom_pub.publish(output_odom);

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
