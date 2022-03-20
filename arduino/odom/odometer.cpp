#include "odometer.h"

const char base_link[] = "base_footprint";
const char frame_name[] = "pose_wheel";
const char topic_name[] = "pose_wheel";
// const char frame_name[] = "odom";

Odometer::Odometer(const double baseWidth /*, const double dT */) {
  this->odomMsg = new geometry_msgs::PoseStamped();
  this->odomPub = new ros::Publisher(topic_name, this->odomMsg);

  this->baseWidth = baseWidth;
  // this->dT = dT;

  this->prevTime = micros();
}

void Odometer::setupPublisher(ros::NodeHandle &nh) {
    nh.advertise(*odomPub);
}

void Odometer::update(const float velocityLeft, const float velocityRight) {
  unsigned long currTime = micros();
  double dT = (currTime - this->prevTime) / double(1000000);
  this->prevTime = currTime;

  // фактическая линейная скорость центра робота
  this->V = (velocityRight + velocityLeft) / 2;  //m/s
  // фактическая угловая скорость поворота робота
  this->omega = (velocityRight - velocityLeft) / this->baseWidth;

  this->yaw += (this->omega * dT);  // направление в рад
  this->x += V*cos(this->yaw) * dT;  // в метрах
  this->y += V*sin(this->yaw) * dT;
}

void Odometer::publish(ros::Time current_time) {
  this->odomMsg->header.stamp     = current_time;
  this->odomMsg->header.frame_id  = frame_name;

  this->odomMsg->pose.position.x  = this->x;  // _cur_x;
  this->odomMsg->pose.position.y  = this->y;  // _cur_y;
  this->odomMsg->pose.position.z  = 0.0;
  this->odomMsg->pose.orientation = tf::createQuaternionFromYaw(this->yaw); // _cur_theta;

  this->odomPub->publish(odomMsg);
}
