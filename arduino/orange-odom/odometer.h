#ifndef ODOMETER_H
#define ODOMETER_H

#include <ros.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

class Odometer {
  public:
    Odometer(const double baseWidth, const double dT);

    void setupPublisher(ros::NodeHandle &nh);
    void update(const float velocityLeft, const float velocityRight);
    void publish(ros::Time current_time);

  private:
    nav_msgs::Odometry* odomMsg;
    ros::Publisher* odomPub;

    double baseWidth = 0;
    double dT = 0;

    double V = 0;
    double omega = 0;

    double yaw = 0;
    double x = 0;
    double y = 0;
  };

#endif  /* ODOMETER_H */
