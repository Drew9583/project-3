#include <stdlib.h>  
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

	// constant global variable to set robot speed
const double SPEED = 0.2;

	// global variable to halt program
bool bumperHit = false;

	// global variables for scanCallback
double wallDistance = 0;
double leftDistance = 0;
double rightDistance = 0;

	// global variables for odometerCallback
ros::Time start;
double original_x = -1;
double original_y = -1;
double odometer = 0;
geometry_msgs::Point now_position;

	// global variable to determine traveled distance since last turn
double lastTurn = 0;

	// global variables for publishers and twist messages
ros::Publisher cmd_vel_pub;
geometry_msgs::Twist cmd_vel_msg;

	// function to halt robot and exit program if a wall is hit 
void haltRobot() {
	cmd_vel_msg.linear.x = 0.0;
	cmd_vel_msg.angular.z = 0.0;
	cmd_vel_pub.publish(cmd_vel_msg);
	//ROS_INFO("Robot has been halted!");
	exit(0);
}

	// function to get keyboard input from a user without waiting for an input to occur
	// function provided by ChatGPT https://chat.openai.com/share/c5e0891f-7488-41d0-9792-0faa99b4d32b
char nonBlockingGetch() {
	struct termios oldt, newt;
	char ch = 0;
	int oldf = fcntl(STDIN_FILENO, F_GETFL);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);

	int ret = read(STDIN_FILENO, &ch, 1);

	if (ret < 0) {
		ch = 0;
	}

	fcntl(STDIN_FILENO, F_SETFL, oldf);
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

	return ch;
}

	// function to detect if a wall is hit
void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg) {
	if (msg->state == kobuki_msgs::BumperEvent::PRESSED){
		bumperHit = true;
		haltRobot();
	}

}

	// function to calculate the closest object from the left side, right side, and center of the robot
	// The fake laser outputs an array where each index is the distance to an object
	// the scan is a swath in front of the robot where index 0-107 is the right side of the robot,
	// 107-537 is directly infront of the robot, where the object will be hit, and > 537 is the left side
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

	// find closest object in front of robot
	double min = msg->ranges[107];
	for (int i = 107; i < 537; i++) {
		if(!isnan(msg->ranges[i]) && isnan(min)){
			min = msg->ranges[i];
		}
		else if (msg->ranges[i] < min) {
			min = msg->ranges[i];
		}
	} 
	wallDistance = (min*3.28084)-(.5);   

	// find closest object to the right of robot
	min = msg->ranges[0];
	for (int i = 0; i < 107; i++) {
		if(!isnan(msg->ranges[i]) && isnan(min)){
			min = msg->ranges[i];
		}
		else if (msg->ranges[i] < min) {
			min = msg->ranges[i];
		}
	} 

	rightDistance = (min*3.28084)-(1);

	// find closest object to the left of robot
	min = msg->ranges[537];
	for (int i = 537; i < msg->ranges.size(); i++) {
		if(!isnan(msg->ranges[i]) && isnan(min)){
			min = msg->ranges[i];
		}
		else if (msg->ranges[i] < min) {
			min = msg->ranges[i];
		}
	} 

	leftDistance = (min*3.28084)-(1);

	/*
	for (int i = 0; i < msg->ranges.size(); i++) {
	    ROS_INFO("Range %d: %.2f feet", i, (msg->ranges[i]-.5)*3.28084);
	}

	ROS_INFO("I am %f feet from the wall", wallDistance);
	ROS_INFO("I am %f feet from the wall to the left", leftDistance);
	ROS_INFO("I am %f feet from the wall to the right", rightDistance);
	*/
}

	// function to determine the total traveled distance of the robot
void odometerCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	geometry_msgs::Pose pose = msg->pose.pose;
	now_position = pose.position;

		// sets the initial position and every .5 seconds, calculates distance traveled using the distance formula
	if(original_x == -1 && original_y == -1){
		original_x = now_position.x;
		original_y = now_position.y;
	}
	if (((ros::Time::now() - start).toSec() >= 0.5)) {
		double distance = sqrt(pow((abs(original_x) - abs(now_position.x)), 2) + (pow((abs(original_y) - abs(now_position.y)), 2)));

		odometer += distance*3.28084; // converts meters to feet
		//ROS_INFO("distance %f", odometer);

		start = ros::Time::now();
		original_x = now_position.x;
		original_y = now_position.y;
	}
}

int main(int argc, char **argv) {   
	ros::init(argc, argv, "robot_control_node");
	ros::NodeHandle nh;

		// input publisher
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 10);

		// subscibers for the bumper, laser, and odometer
	ros::Subscriber bumper_sub = nh.subscribe("/mobile_base/events/bumper", 1, bumperCallback);
	ros::Subscriber laser_sub = nh.subscribe("/scan", 1, scanCallback);
	ros::Subscriber odometer_sub = nh.subscribe("/odom", 1, odometerCallback);

	ros::Rate loop_rate(10); // controls the frequency that the loop executes (in Hertz)


	char option = ' ';
	while (ros::ok()){

		option = nonBlockingGetch();
		cmd_vel_msg.linear.x = 0.0;
		cmd_vel_msg.angular.z = 0.0;

		if(option == 'i'){      // drive forward
			cmd_vel_msg.linear.x = SPEED;
			cmd_vel_msg.angular.z = 0.0;
		}
		else if(option == 'k'){ // drive backwards
			cmd_vel_msg.linear.x = -SPEED;
			cmd_vel_msg.angular.z = 0.0;
		}
		else if(option == 'j'){ // turn left
			cmd_vel_msg.linear.x = 0.0;
			cmd_vel_msg.angular.z = SPEED;
		}
		else if(option == 'l'){ // turn right
			cmd_vel_msg.linear.x = 0.0;
			cmd_vel_msg.angular.z = -SPEED;
		}
		else if((wallDistance < 1.5) && (rightDistance > leftDistance) && (rightDistance > wallDistance)){ // Avoid asymmetric obstances by turning right
			while(wallDistance < 1.5){
				//ROS_INFO("I TURNED 5 DEGRESS RIGHT");
				cmd_vel_msg.angular.z = -0.5;

		    		double turn_duration = 0.0872665; // 5 degrees in radians

		    		ros::Time start_time = ros::Time::now();

		    		while (ros::ok() && (ros::Time::now() - start_time).toSec() < turn_duration){
					cmd_vel_pub.publish(cmd_vel_msg);
					loop_rate.sleep();
		    		}
				ros::spinOnce(); // processes pending ROS messages and/or callbacks
			}
		}
		else if((wallDistance < 1.5) && (leftDistance > rightDistance) && (leftDistance > wallDistance)){ // Avoid asymmetric obstances by turning left
			while(wallDistance < 1.5){
				//ROS_INFO("I TURNED 5 DEGRESS LEFT");
				cmd_vel_msg.angular.z = 0.5;

		    		double turn_duration = 0.0872665;  // 5 degrees in radians

		    		ros::Time start_time = ros::Time::now();

		    		while (ros::ok() && (ros::Time::now() - start_time).toSec() < turn_duration){
					cmd_vel_pub.publish(cmd_vel_msg);
					loop_rate.sleep();
		    		}
				ros::spinOnce(); // processes pending ROS messages and/or callbacks
			}
		}
		else if(wallDistance < 1.5){ // escape symmetric obstacles if avoid did not work
			cmd_vel_msg.angular.z = 0.5;

			double turn_duration = 6.28;

			ros::Time start_time = ros::Time::now();

			while (ros::ok() && (ros::Time::now() - start_time).toSec() < turn_duration){
				cmd_vel_pub.publish(cmd_vel_msg);
				loop_rate.sleep();
			}
		}
		else if ((odometer - lastTurn) > 1.0){ // turn randomly +-15 degress every 1ft of forward movement
			double turnRad = (rand() % 16) * (3.14 / 180); // random 1-15 degrees in radians
			double turnDir = (rand() % 2); // random direction

			ros::Time start_time = ros::Time::now();

			//ROS_INFO("%f", turnRad);
			//ROS_INFO("%f", turnDir);
			
			if(turnDir == 0){
				while (ros::ok() && (ros::Time::now() - start_time).toSec() < (turnRad)){
					cmd_vel_msg.angular.z = (0.5 * -1);
					cmd_vel_pub.publish(cmd_vel_msg);
					loop_rate.sleep();
				}
			}
			else{
				while (ros::ok() && (ros::Time::now() - start_time).toSec() < (turnRad)){
					cmd_vel_msg.angular.z = (0.5);
					cmd_vel_pub.publish(cmd_vel_msg);
					loop_rate.sleep();
				}
			}
			lastTurn = odometer;
		}
		else if (!bumperHit){ // If no other criteria are met, continue moving forward
		    	cmd_vel_msg.linear.x = SPEED;
			cmd_vel_msg.angular.z = 0.0;
			//ROS_INFO("I am Moving");
		}

		cmd_vel_pub.publish(cmd_vel_msg); // publish movement commands
		ros::spinOnce(); // processes pending ROS messages and/or callbacks
		loop_rate.sleep();
	}
	return 0;
}
