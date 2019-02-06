#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <string>

//Change these constants for vehicle kinematics
#define DEFAULT_LINEAR 0.1	     //LINEAR SPEED for no obstacle detected
#define DEFAULT_ANGULAR 0	      //ANGULAR SPEED (Turn)
#define DISTANCE 3		       //Maximum distance to consider point an obstacle
#define NEW_LINEARX 0.3		       //
#define TURN_ANGULAR_SPEED 0.5	   //Turn speed
#define MINIMUM_DISTANCE_THRESHOLD 0.1 //how sensitive LiDAR is to small distance values (DEFAULT: 0.1)
#define BACK_ANGLE_PROPORTION_THRESHOLD 0.24 //0.24 = 60 deg / 270 , 0.33 = 90 deg / 270


ros::Publisher pub;

//HELPER FUNCTION
//PURPOSE: Compute the minimum element contained in a vector to determine
// the minimum distance to the closest obstacle in the given partition of sensor.
float min_element(std::vector<float> first)
{
	float min = 50.0;

	for (std::vector<float>::iterator it = first.begin(); it < first.end(); it++)
	{
		if (*it<min && * it> MINIMUM_DISTANCE_THRESHOLD)
		{
			min = *it;
		}
	}
	return min;
}

//HELPER FUNCTION
//PURPOSE: Compute the direction based on where an obstacle is detected.
//Function publishes a message with new direction, linear, and angular velocity.
//IDEA: New angle can be dependent on how close the object is (inverse sigmoid)
void computeDirection(float backLeft, float front, float backRight)
{

	geometry_msgs::Twist message;
	float linearx = DEFAULT_LINEAR;
	float angularz = DEFAULT_ANGULAR;
	std::string case_description;

	bool objectFront = (front < DISTANCE);
	bool objectBackLeft = (backLeft < DISTANCE);
	bool objectBackRight = (backRight < DISTANCE);

	//If no obstacles is within DISTANCE, then proceed forward.
	//LEFT = -TurnAngular
	//Right = +TurnAngular
	if (!objectFront && !objectBackLeft && !objectBackRight)
	{
		case_description = "Case 1: No Obstacle Detected";
		linearx = NEW_LINEARX;
		angularz = 0;
	}
	else if (objectFront && !objectBackLeft && !objectBackRight)
	{
		case_description = "Case 2: Object in front.";
		linearx = 0;
		angularz = TURN_ANGULAR_SPEED;
	}
	else if (objectFront && objectBackLeft && !objectBackRight){
		case_description = "Case 3: Object in backLeft and front.";
		linearx = 0;
		angularz = -TURN_ANGULAR_SPEED;
	}
	else if (objectFront && !objectBackLeft && objectBackRight){
		case_description = "Case 4: Object in backRight and front";
		linearx = 0;
		angularz = TURN_ANGULAR_SPEED;
	}
	else if (!objectFront && objectBackLeft && objectBackRight)
	{
		case_description = "Case 3: Object in front right area.";
		linearx = NEW_LINEARX;
		angularz = 0;
	}	
	else
	{
		case_description = "Unknown case";
	}

	ROS_INFO("%s", case_description.c_str());
	message.linear.x = linearx;
	message.angular.z = angularz;
	pub.publish(message);
}

//Callback Function To Process Lidar Data and Partition Into 5 Arrays:
//Left, FrontLeft, Front, FrontRight, and Right
//720 total laser scans divided by 5 = 144 scans for each partition.
//Introduce clustering algorithm that passes angle ranges of detected obstacles...
void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{

	//ROS_INFO("MINRANGE: %f", msg->range_min);
	std::vector<float> backLeft, front, backRight;

	int size = msg->ranges.size();
	int leftLimit, rightLimit;
	leftLimit = BACK_ANGLE_PROPORTION_THRESHOLD * size;
	rightLimit = size - leftLimit;

	for (int i = 0; i <= leftLimit; i++)
	{
		backLeft.push_back(msg->ranges[i]);
	}

	float minBackLeft = min_element(left);

	for (int i = leftLimit; i < rightLimit; i++)
	{
		front.push_back(msg->ranges[i]);
	}
	float minFront = min_element(frontLeft);

	for (int i = rightLimit; i < size - 1; i++)
	{
		backRight.push_back(msg->ranges[i]);
	}
	float minBackRight = min_element(front);


	computeDirection(minBackLeft, minFront, minBackRight);

	ROS_INFO("min range on back-left: %f", minBackLeft);
	ROS_INFO("min range on front: %f", minFront);
	ROS_INFO("min range on back-right: %f", minBackRight);

}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "laser_avoidance");
	ros::NodeHandle nh;
	pub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 100);
	ros::Subscriber sub = nh.subscribe("/scan", 10, laserCallback);

	ros::spin();
	return 0;
}
