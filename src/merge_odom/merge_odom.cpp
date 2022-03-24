#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <ecl/threads.hpp>
#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

const char base_link[] = "base_footprint";
const char odom[] = "odom";
bool publish_tf = true;

geometry_msgs::PoseWithCovarianceStamped shared_elbrus_pose;
geometry_msgs::PoseStamped shared_wheel_pose;

nav_msgs::Odometry output_odom;

ecl::Mutex mutex_elbrus;
ecl::Mutex mutex_wheel;

void elbrusPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose_cov) {
  mutex_elbrus.lock();
  shared_elbrus_pose = pose_cov;
  mutex_elbrus.unlock();

  output_odom.header.stamp = ros::Time::now();

  output_odom.pose.pose.position.x  = shared_elbrus_pose.pose.pose.position.x;  // _cur_x;
  output_odom.pose.pose.position.y  = shared_elbrus_pose.pose.pose.position.y;  // _cur_y;
  output_odom.pose.pose.position.z  = shared_elbrus_pose.pose.pose.position.z;

  output_odom.pose.pose.orientation.x = shared_elbrus_pose.pose.pose.orientation.x;
  output_odom.pose.pose.orientation.y = shared_elbrus_pose.pose.pose.orientation.y;
  output_odom.pose.pose.orientation.z = shared_elbrus_pose.pose.pose.orientation.z;
  output_odom.pose.pose.orientation.w = shared_elbrus_pose.pose.pose.orientation.w;

  ROS_INFO("Elbrus msg");
}

void wheelOdomCallback(const geometry_msgs::PoseStamped& pose) {
  mutex_wheel.lock();
  shared_wheel_pose = pose;
  mutex_wheel.unlock();

  output_odom.header.stamp = ros::Time::now();

  tf2::Quaternion quat_tf_output, quat_tf_input, quat_tf_new;

  tf2::convert(output_odom.pose.pose.orientation, quat_tf_output);
  tf2::convert(shared_wheel_pose.pose.orientation, quat_tf_input);

  quat_tf_new = quat_tf_output * quat_tf_input;
  quat_tf_new.normalize();

  tf2::convert(quat_tf_new, output_odom.pose.pose.orientation);

  tf2::Vector3 translateVector(shared_wheel_pose.pose.position.x,
                               shared_wheel_pose.pose.position.y,
                               shared_wheel_pose.pose.position.z);

  translateVector = tf2::quatRotate(quat_tf_output, translateVector);

  output_odom.pose.pose.position.x  += translateVector.x();  // _cur_x;
  output_odom.pose.pose.position.y  += translateVector.y();  // _cur_y;
  output_odom.pose.pose.position.z  += translateVector.z();

  ROS_INFO("Wheel msg");
}

void mergeOdoms(nav_msgs::Odometry& output_odom) {
  mutex_elbrus.lock();
  mutex_wheel.lock();

  output_odom.header.stamp          = ros::Time::now();
  output_odom.header.frame_id       = odom;
  output_odom.child_frame_id        = base_link;

  // output_odom.pose.pose.position.x  = 0.5*shared_elbrus_pose.pose.pose.position.x + 0.5*shared_wheel_pose.pose.pose.position.x;  // _cur_x;
  // output_odom.pose.pose.position.y  = 0.5*shared_elbrus_pose.pose.pose.position.y + 0.5*shared_wheel_pose.pose.pose.position.y;  // _cur_y;
  // output_odom.pose.pose.position.z  = 0.5*shared_elbrus_pose.pose.pose.position.z + 0.5*shared_wheel_pose.pose.pose.position.z;
  // output_odom.pose.pose.orientation.x = 0.5*shared_elbrus_pose.pose.pose.orientation.x + 0.5*shared_wheel_pose.pose.pose.orientation.x;
  // output_odom.pose.pose.orientation.y = 0.5*shared_elbrus_pose.pose.pose.orientation.y + 0.5*shared_wheel_pose.pose.pose.orientation.y;
  // output_odom.pose.pose.orientation.z = 0.5*shared_elbrus_pose.pose.pose.orientation.z + 0.5*shared_wheel_pose.pose.pose.orientation.z;
  // output_odom.pose.pose.orientation.w = 0.5*shared_elbrus_pose.pose.pose.orientation.w + 0.5*shared_wheel_pose.pose.pose.orientation.w; // _cur_theta;
  //
  // output_odom.twist.twist.linear.x  = shared_wheel_pose.twist.twist.linear.x;  // vx;
  // output_odom.twist.twist.linear.y  = shared_wheel_pose.twist.twist.linear.y;
  // output_odom.twist.twist.angular.z = shared_wheel_pose.twist.twist.angular.z;  // vth;

  output_odom.pose.pose.position.x  = shared_wheel_pose.pose.position.x;  // _cur_x;
  output_odom.pose.pose.position.y  = shared_wheel_pose.pose.position.y;  // _cur_y;
  output_odom.pose.pose.position.z  = shared_wheel_pose.pose.position.z;
  output_odom.pose.pose.orientation.x = shared_wheel_pose.pose.orientation.x;
  output_odom.pose.pose.orientation.y = shared_wheel_pose.pose.orientation.y;
  output_odom.pose.pose.orientation.z = shared_wheel_pose.pose.orientation.z;
  output_odom.pose.pose.orientation.w = shared_wheel_pose.pose.orientation.w; // _cur_theta;

  mutex_elbrus.unlock();
  mutex_wheel.unlock();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "merge_odom");
  ros::NodeHandle n;

  ros::Publisher merge_odom_pub = n.advertise<nav_msgs::Odometry>("/merge_odom", 10);
  ros::Publisher update_w_odom_pub = n.advertise<std_msgs::Empty>("/updated_pose", 10);

  ros::Subscriber elbrus_sub = n.subscribe("/elbrus_pose_cov", 10, elbrusPoseCallback);
  ros::Subscriber wheel_sub = n.subscribe("/wheel_pose", 10, wheelOdomCallback);

  output_odom.header.frame_id = odom;
  output_odom.child_frame_id  = base_link;
  output_odom.pose.pose.orientation.w = 1;

  ros::Rate loop_rate(30);

  while (ros::ok())
  {
    // nav_msgs::Odometry output_odom;
    // mergeOdoms(output_odom);
    merge_odom_pub.publish(output_odom);

    if (publish_tf) {
      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(output_odom.pose.pose.position.x,
                                      output_odom.pose.pose.position.y,
                                      output_odom.pose.pose.position.z));
      tf::Quaternion q(output_odom.pose.pose.orientation.x,
                     output_odom.pose.pose.orientation.y,
                     output_odom.pose.pose.orientation.z,
                     output_odom.pose.pose.orientation.w);

      transform.setRotation(q);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), odom, base_link));
    }

    std_msgs::Empty updateWOdomMsg;;
    update_w_odom_pub.publish(updateWOdomMsg);

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
