#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <string>


ros::Publisher pub;

void computeDirection(float left, float frontLeft, float front, float frontRight, float right){

	geometry_msgs::Twist message;
	float linearx,angularz;
	linearx = 0.1;
	std::string case_description;

				if(front>1 && frontLeft > 1 && frontRight >1){
							case_description = "Case 1: No Obstacle Detected";
							linearx = 0.3;
							angularz = 0;
				}

				else if(front <1 && frontLeft>1 && frontRight>1){
							case_description = "Case 2: Object in front.";
							linearx = 0;
							angularz=3;
				}
        else if(front >1 && frontLeft>1 && frontRight<1){
                case_description = "Case 3: Object in front right area.";
                linearx = 0;
                angularz=3;
        }
        else if(front >1 && frontLeft<1 && frontRight>1){
                case_description = "Case 4: Object in front left area.";
                linearx = 0;
                angularz=-3;
        }
        else if(front <1 && frontLeft>1 && frontRight<1){
                case_description = "Case 5: Object in front and front right areas.";
                linearx = 0;
                angularz=3;
        }
        else if(front <1 && frontLeft<1 && frontRight>1){
                case_description = "Case 6: Object in front and front left areas.";
                linearx = 0;
                angularz=-3;
        }
        else if(front <1 && frontLeft<1 && frontRight<1){
                case_description = "Case 7: Object in front, front left, and front right areas.";
                linearx = 0;
                angularz=3;
        }
				else if(front >1 && frontLeft<1 && frontRight<1){
                case_description = "Case 8: Object in front left and front right.";
                linearx = 3;
                angularz=0;
        }

				else{
								case_description = "Unknown case";
				}

		ROS_INFO("%s",case_description.c_str());
		message.linear.x = linearx;
		message.angular.z = angularz;
		pub.publish(message);

}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	//720 divided by 5 = 144
	std::vector<float> left,frontLeft,front,frontRight,right;
	for(int i=0;i<64;i++){
		left.push_back(msg->ranges[i]);
	}
	float minLeft = *std::min_element(left.begin(), left.end());

        for(int i=144;i<288;i++){
                frontLeft.push_back(msg->ranges[i]);
        }
        float minFrontLeft = *std::min_element(frontLeft.begin(), frontLeft.end());

	for(int i=288;i<432;i++){
                front.push_back(msg->ranges[i]);
        }
        float minFront = *std::min_element(front.begin(), front.end());

        for(int i=432;i<576;i++){
                frontRight.push_back(msg->ranges[i]);
        }
        float minFrontRight = *std::min_element(frontRight.begin(), frontRight.end());

        for(int i=576;i<right.size()-1;i++){
                right.push_back(msg->ranges[i]);
        }
        float minRight = *std::min_element(right.begin(), right.end());

	computeDirection(minLeft, minFrontLeft, minFront, minFrontRight, minRight);

	//ROS_INFO("min range on left: %f", minLeft);
	//ROS_INFO("min range on frontleft: %f", minFrontLeft);
	//ROS_INFO("min range on front: %f", minFront);
	//ROS_INFO("min range on frontRight: %f", minFrontRight);
	//ROS_INFO("min range on right: %f", minRight);

}


int main(int argc , char **argv) {

	ros::init(argc, argv, "laser_avoidance");
	ros::NodeHandle nh;
	pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",100);
	ros::Subscriber sub = nh.subscribe("scan", 10, laserCallback);




	ros::spin();
	return 0;
}
