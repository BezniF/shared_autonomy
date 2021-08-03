#include "turtle_control.h"
#include <signal.h>

void mySigintHandler(int sig){

	// Used to stop the robots and the Omega controller as soon as i close the ROS node
	ros::NodeHandle nh;

	ros::Publisher tb1_pub, tb2_pub, tb3_pub, wrench_pub;
	tb1_pub = nh.advertise<geometry_msgs::Twist>("/turtle3/cmd_vel" ,1);
	tb2_pub = nh.advertise<geometry_msgs::Twist>("/turtle4/cmd_vel" ,1);
	tb3_pub = nh.advertise<geometry_msgs::Twist>("/turtle5/cmd_vel" ,1);
	
	wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("/fdo6/wrench_cmd", 1);

	geometry_msgs::Twist stop_msg;
	geometry_msgs::WrenchStamped stop_force;

	stop_msg.linear.x = 0.0;
	stop_msg.angular.z = 0.0;

	stop_force.wrench.force.x = 0.0;
	stop_force.wrench.force.y = 0.0;

	tb1_pub.publish(stop_msg);
	tb2_pub.publish(stop_msg);
	tb3_pub.publish(stop_msg);

	wrench_pub.publish(stop_force);

	ros::shutdown();
}

int main(int argc, char **argv){

	ros::init(argc, argv, "turtle_control",ros::init_options::NoSigintHandler);
	TurtleControl* tc = new TurtleControl();

	signal(SIGINT, mySigintHandler);

	ros::Rate r(500);

	while(ros::ok()) {
		ros::spinOnce();
		tc->spin();
		r.sleep();
	}
		
	delete tc;

}