#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include <ras_arduino_msgs/PWM.h>
#include <ras_arduino_msgs/Encoders.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <iostream>
#include <cmath>
#include <phidget21.h>
#include <sensor_msgs/Imu.h>
#include <robo_globals.h>

class imu_reader
{
private:
	double angle;
	double dt;
	double accA;
	double weight;

public:

	ros::NodeHandle n_;
	ros::Subscriber imu_subscriber_;
	ros::Publisher angle_publisher_;
	double gyro; // gyro z angular velocity
	double acc[2]; //accelerometer x and y linear accelerations

	imu_reader() : angle(0), dt(1/CTRL_FREQ), accA(0), weight(0.98)
	{
		n_ = ros::NodeHandle("~");
		imu_subscriber_ = n_.subscribe("/imu/data", 1, &imu_reader::imuCallback, this);
		angle_publisher_ = n_.advertise<std_msgs::Float64>("/imu_angle", 1000);
	}

	void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
	{
		gyro = msg->angular_velocity.z;
		acc[0] = msg->linear_acceleration.x;
		acc[1] = msg->linear_acceleration.y;
	}

	// Calculate angle
	void anglecal()
	{
		accA = atan2(acc[0],acc[1]);
		angle = weight * (angle + gyro * dt) + (1-weight) * accA;
		angle = (angle * 360) / TWOPI;
		ROS_INFO("Result Angle is: [%fl]", angle);
	}

	void publish()
	{
		anglecal();
		std_msgs::Float64 msg;
		msg.data = angle;
		angle_publisher_.publish(msg);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imu_reader");
	imu_reader imu_reader_node;
	ros::Rate loop_rate(CTRL_FREQ);

  while (ros::ok())
	{
		ros::spinOnce();
		imu_reader_node.publish();
		loop_rate.sleep();
  }

  return 0;
}
