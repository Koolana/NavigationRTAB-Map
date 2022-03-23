#include "odometer.h"

const char base_link[] = "base_footprint";
const char frame_name[] = "wheel_pose";
const char topic_name[] = "/wheel_pose";
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

void Odometer::updateByWDistance(const double distLeft, const double distRight) {
  // double radiusRotate = ((distRight - distLeft) < 0.001 ? 0.0 : distLeft * this->baseWidth / (distRight - distLeft));
  //
  // this->yaw += (radiusRotate < 0.001 ? 0.0 : distLeft / (2 * 3.14 * radiusRotate));
  // if (this->yaw > PI) {yaw = yaw - 2 * PI;}
  // if (this->yaw < -PI) {yaw = yaw + 2 * PI;}
  // this->x += (radiusRotate + this->baseWidth / 2) * sin(this->yaw);
  // this->y += (radiusRotate + this->baseWidth / 2) * (1 - cos(this->yaw));

  // направление робота и новые координаты
  // this->yaw += ((0.068 + 0.068)/ (2 * this->baseWidth))*(distRight/0.068 - distLeft/0.068);
  // if (this->yaw > PI) {yaw = yaw - 2 * PI;}
  // if (this->yaw < -PI) {yaw = yaw + 2 * PI;}
  // this->x += ((0.068 + 0.068)/4)*(distRight/0.068 + distLeft/0.068) * cos(this->yaw);
  // this->y += ((0.068 + 0.068)/4)*(distRight/0.068 + distLeft/0.068) * sin(this->yaw);

  this->yaw = (1 / this->baseWidth) * (distRight - distLeft);
  if (this->yaw > PI) {yaw = yaw - 2 * PI;}
  if (this->yaw < -PI) {yaw = yaw + 2 * PI;}
  this->x = 0.5 * (distRight + distLeft) * cos(this->yaw);
  this->y = 0.5 * (distRight + distLeft) * sin(this->yaw);
}

// Rename type this->odomMsg geometry_msgs::PoseStamped to geometry_msgs/Twist
void Odometer::publish(ros::Time current_time) {
  this->odomMsg->header.stamp     = current_time;
  this->odomMsg->header.frame_id  = frame_name;

  this->odomMsg->pose.position.x  = this->x;  // _cur_x;
  this->odomMsg->pose.position.y  = this->y;  // _cur_y;
  this->odomMsg->pose.position.z  = 0.0;
  this->odomMsg->pose.orientation = tf::createQuaternionFromYaw(this->yaw); // _cur_theta;

  this->odomPub->publish(odomMsg);
}
