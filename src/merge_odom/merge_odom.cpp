#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <ecl/threads.hpp>
#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>

const char base_link[] = "base_footprint";
const char odom[] = "odom";

const char elbrus_frame[] = "camera_link";
bool publish_tf = true;

geometry_msgs::PoseStamped shared_elbrus_pose;
geometry_msgs::PoseStamped shared_wheel_pose;

nav_msgs::Odometry output_odom;

ecl::Mutex mutex_elbrus;
ecl::Mutex mutex_wheel;

tf2_ros::Buffer* tfBuffer;
tf2_ros::TransformListener* tfListener;

ros::Time lastWheelRequest(0);
double wheelSpan = 0;

void elbrusPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& pose_cov) {
  if (output_odom.twist.twist.angular.z > 0.4) {
    return;
  }

  // Сохранение сообщения geometry_msgs::PoseStamped во временную переменную
  mutex_elbrus.lock();
  // shared_elbrus_pose.pose.position.x  = pose_cov.pose.pose.position.x;  // _cur_x;
  // shared_elbrus_pose.pose.position.y  = pose_cov.pose.pose.position.y;  // _cur_y;
  // shared_elbrus_pose.pose.position.z  = pose_cov.pose.pose.position.z;
  //
  // shared_elbrus_pose.pose.orientation.x = pose_cov.pose.pose.orientation.x;
  // shared_elbrus_pose.pose.orientation.y = pose_cov.pose.pose.orientation.y;
  // shared_elbrus_pose.pose.orientation.z = pose_cov.pose.pose.orientation.z;
  // shared_elbrus_pose.pose.orientation.w = pose_cov.pose.pose.orientation.w;
  shared_elbrus_pose.pose = pose_cov.pose.pose;
  mutex_elbrus.unlock();

  // Преобразование одометрии Elbrus в систему координат связанную с центром робота
  shared_elbrus_pose.header.frame_id = elbrus_frame;
  geometry_msgs::PoseStamped transformed_elbrus_pose = tfBuffer->transform(shared_elbrus_pose, "base_footprint");

  // Формирование выходного сообщения nav_msgs::Odometry в буфер
  output_odom.header.stamp = ros::Time::now();

  // output_odom.pose.pose.position.x  = transformed_elbrus_pose.pose.position.x;  // _cur_x;
  // output_odom.pose.pose.position.y  = transformed_elbrus_pose.pose.position.y;  // _cur_y;
  // output_odom.pose.pose.position.z  = transformed_elbrus_pose.pose.position.z;
  //
  // output_odom.pose.pose.orientation.x = transformed_elbrus_pose.pose.orientation.x;
  // output_odom.pose.pose.orientation.y = transformed_elbrus_pose.pose.orientation.y;
  // output_odom.pose.pose.orientation.z = transformed_elbrus_pose.pose.orientation.z;
  // output_odom.pose.pose.orientation.w = transformed_elbrus_pose.pose.orientation.w;
  output_odom.pose.pose = transformed_elbrus_pose.pose;

  ROS_INFO("Elbrus msg");
}

void wheelOdomCallback(const geometry_msgs::PoseStamped& pose) {
  // Сохранение сообщения geometry_msgs::PoseStamped во временную переменную
  mutex_wheel.lock();
  shared_wheel_pose = pose;
  mutex_wheel.unlock();

  output_odom.header.stamp = ros::Time::now();

  // Преобразование относительного смещения робота согласно колесной одометрии в смещение в абсолютной системе
  tf2::Quaternion quat_tf_output, quat_tf_input, quat_tf_new;

  tf2::convert(output_odom.pose.pose.orientation, quat_tf_output);
  tf2::convert(shared_wheel_pose.pose.orientation, quat_tf_input);

  // Подсчет нового поворота согласно смещению колесной одометрии
  quat_tf_new = quat_tf_output * quat_tf_input;
  quat_tf_new.normalize();

  tf2::convert(quat_tf_new, output_odom.pose.pose.orientation);

  tf2::Vector3 translateVector(shared_wheel_pose.pose.position.x,
                               shared_wheel_pose.pose.position.y,
                               shared_wheel_pose.pose.position.z);

  // Поворот вектора смещения и инкрементирование координат робота
  translateVector = tf2::quatRotate(quat_tf_output, translateVector);

  output_odom.pose.pose.position.x  += translateVector.x();  // _cur_x;
  output_odom.pose.pose.position.y  += translateVector.y();  // _cur_y;
  output_odom.pose.pose.position.z  += translateVector.z();

  // Подсчет скорости согласно смещению
  output_odom.twist.twist.linear.x = std::sqrt(std::pow(shared_wheel_pose.pose.position.x, 2) + std::pow(shared_wheel_pose.pose.position.y, 2)) / wheelSpan;

  double yaw_angle = tf::getYaw(pose.pose.orientation);
  output_odom.twist.twist.angular.z = yaw_angle / wheelSpan;

  ROS_INFO("Wheel msg");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "merge_odom");
  ros::NodeHandle n;

  tfBuffer = new tf2_ros::Buffer();
  tfListener = new tf2_ros::TransformListener(*tfBuffer);

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
    merge_odom_pub.publish(output_odom);

    // Публикация преобразования odom->base_link в дереве tf
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

    // Формирование сообщения std_msgs::Empty запроса колесной одометрии
    // и подсчет времени между запросами
    std_msgs::Empty updateWOdomMsg;
    wheelSpan = ros::Time::now().toSec() - lastWheelRequest.toSec();
    lastWheelRequest = ros::Time::now();
    update_w_odom_pub.publish(updateWOdomMsg);

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
