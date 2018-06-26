#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <string>

float linearx, angularz = 0.1;
float distance = 3;

ros::Publisher pub;

float min_element(std::vector<float> first){
	float min=50.0;

	for (std::vector<float>::iterator it = first.begin(); it<first.end();it++){
		if (*it<min && *it> 0.1){
			min = *it;
		}
	}
	return min;	
} 
void computeDirection(float left, float frontLeft, float front, float frontRight, float right){

	geometry_msgs::Twist message;
	linearx = 0.1;
	angularz = 0;
	std::string case_description;

	if(front>distance && frontLeft > distance && frontRight >distance){
		case_description = "Case 1: No Obstacle Detected";
		linearx = 0.3;
		angularz = 0;
	}

	else if(front <distance && frontLeft>distance && frontRight>distance){
		case_description = "Case 2: Object in front.";
		linearx = 0;
		angularz=3;
	}
        else if(front >distance && frontLeft>distance && frontRight<distance){
                case_description = "Case 3: Object in front right area.";
                linearx = 0;
                angularz=3;
        }
        else if(front >distance && frontLeft<distance && frontRight>distance){
                case_description = "Case 4: Object in front left area.";
                linearx = 0;
                angularz=-3;
        }
        else if(front <distance && frontLeft>distance && frontRight<distance){
                case_description = "Case 5: Object in front and front right areas.";
                linearx = 0;
                angularz=3;
        }
        else if(front <distance && frontLeft<distance && frontRight>distance){
                case_description = "Case 6: Object in front and front left areas.";
                linearx = 0;
                angularz=-3;
        }
        else if(front <distance && frontLeft<distance && frontRight<distance){
                case_description = "Case 7: Object in front, front left, and front right areas.";
                linearx = 0;
                angularz=3;
        }
	else if(front >distance && frontLeft<distance && frontRight<distance){
                case_description = "Case 8: Object in front left and front right.";
                linearx = 3;
                angularz=0;
        }

	else{
		case_description = "Unknown case";
	}


//		ROS_INFO("MINLEFT: %f", frontLeft);
//		ROS_INFO("MINFRONT: %f", front);
//		ROS_INFO("MINRIGHT: %f", frontRight);
		ROS_INFO("%s",case_description.c_str());
		message.linear.x = linearx;
		message.angular.z = angularz;
		pub.publish(message);

}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	//720 divided by 5 = 144
	ROS_INFO("MINRANGE: %f", msg->range_min);
	std::vector<float> left,frontLeft,front,frontRight,right;
	for(int i=0;i<64;i++){
		left.push_back(msg->ranges[i]);
	}
	float minLeft = min_element(left);

        for(int i=144;i<288;i++){
                frontLeft.push_back(msg->ranges[i]);
        }
        float minFrontLeft = min_element(frontLeft);

	for(int i=288;i<432;i++){
                front.push_back(msg->ranges[i]);
        }
        float minFront = min_element(front);

        for(int i=432;i<576;i++){
                frontRight.push_back(msg->ranges[i]);
        }
        float minFrontRight = min_element(frontRight);

        for(int i=576;i<right.size()-1;i++){
                right.push_back(msg->ranges[i]);
        }
        float minRight = min_element(right);

	computeDirection(minLeft, minFrontLeft, minFront, minFrontRight, minRight);

	ROS_INFO("min range on left: %f", minLeft);
	ROS_INFO("min range on frontleft: %f", minFrontLeft);
	ROS_INFO("min range on front: %f", minFront);
	ROS_INFO("min range on frontRight: %f", minFrontRight);
	ROS_INFO("min range on right: %f", minRight);

}


int main(int argc , char **argv) {

	ros::init(argc, argv, "laser_avoidance");
	ros::NodeHandle nh;
	pub = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel",100);
	ros::Subscriber sub = nh.subscribe("/scan", 10, laserCallback);



	ros::spin();
	return 0;
}


