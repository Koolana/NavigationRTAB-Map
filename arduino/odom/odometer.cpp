#include "odometer.h"

const char base_link[] = "base_footprint";
const char odom[] = "odom";

Odometer::Odometer(const double baseWidth, const double dT) {
  this->odomMsg = new nav_msgs::Odometry();
  this->odomPub = new ros::Publisher(odom, this->odomMsg);

  this->baseWidth = baseWidth;
  this->dT =  dT;
}

void Odometer::setupPublisher(ros::NodeHandle &nh) {
    nh.advertise(*odomPub);
}

void Odometer::update(const float velocityLeft, const float velocityRight) {
  // фактическая линейная скорость центра робота
  this->V = (velocityRight + velocityLeft) / 2;  //m/s
  // фактическая угловая скорость поворота робота
  this->omega = (velocityRight - velocityLeft) / this->baseWidth;

  this->yaw += (this->omega * this->dT);  // направление в рад
  this->x += V*cos(this->yaw) * this->dT;  // в метрах
  this->y += V*sin(this->yaw) * this->dT;
}

void Odometer::publish(ros::Time current_time) {
  this->odomMsg->header.stamp          = current_time;
  this->odomMsg->header.frame_id       = odom;
  this->odomMsg->child_frame_id        = base_link;

  this->odomMsg->pose.pose.position.x  = this->x;  // _cur_x;
  this->odomMsg->pose.pose.position.y  = this->y;  // _cur_y;
  this->odomMsg->pose.pose.position.z  = 0.0;
  this->odomMsg->pose.pose.orientation = tf::createQuaternionFromYaw(this->yaw); // _cur_theta;

  this->odomMsg->twist.twist.linear.x  = this->V;  // vx;
  this->odomMsg->twist.twist.linear.y  = 0.0;
  this->odomMsg->twist.twist.angular.z = this->omega;  // vth;

  this->odomPub->publish(odomMsg);
}
