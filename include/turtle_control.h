#ifndef TURTLE_CONTROL_H_
#define TURTLE_CONTROL_H_
#define _USE_MATH_DEFINES
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <fstream>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

// tf
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <tf2_eigen/tf2_eigen.h>

// Gazebo
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelStates.h>

// Eigen
#include <eigen3/Eigen/Core>

// CVXgen
extern "C" {
    #include "solver.h"
}

class TurtleControl {

		public:

			TurtleControl();
			~TurtleControl();
			void spin();

			// Callbacks
			void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
			void forceCallback(const geometry_msgs::WrenchStamped& msg);
			void omegaCallback(const geometry_msgs::PoseStamped& msg);
			void buttonCallback(const std_msgs::Bool& msg);
			void odomTb1Callback(const nav_msgs::Odometry& odom);
			void odomTb2Callback(const nav_msgs::Odometry& odom);
			void odomTb3Callback(const nav_msgs::Odometry& odom);
			void optiTb1Callback(const geometry_msgs::PoseStamped& msg);
			void optiTb2Callback(const geometry_msgs::PoseStamped& msg);
			void optiTb3Callback(const geometry_msgs::PoseStamped& msg);
			void gazeboObsCallback(const gazebo_msgs::ModelStates& msg);


			// Functions
			void computeForceInteraction();
			void computeTotalForce();
			void computeVelocities();
			void computePowers();
			void computeObsForce();
			Eigen::Vector4d optimizationProblem(Eigen::Vector4d v, Eigen::Vector4d F, double T, double Pin, double Pout);
			Eigen::VectorXd singleTankOptProblem(Eigen::VectorXd v, Eigen::VectorXd F, double T);
			void computeTankEnergy();
			void computeIOSLF();
			void writeToFiles();
			void saturateSpeed();
			double quaternionToRPY(geometry_msgs::Quaternion q);
			Eigen::Vector2d calculateForcesObs(double x_tb, double y_tb, double x_obs, double y_obs);

		private:

			// NodeHandle
        	ros::NodeHandle nh_;

			// Subscribers
			ros::Subscriber joy_sub_;
			ros::Subscriber omega_sub_;
			ros::Subscriber button_sub_;
			ros::Subscriber odom_tb1_sub_;
			ros::Subscriber odom_tb2_sub_;
			ros::Subscriber odom_tb3_sub_;
			ros::Subscriber opti_tb1_sub_;
			ros::Subscriber opti_tb2_sub_;
			ros::Subscriber opti_tb3_sub_;
			ros::Subscriber gazebo_obs_sub_;
			
			// Publisher
			ros::Publisher vel_tb1_pub_;
			ros::Publisher vel_tb2_pub_;
			ros::Publisher vel_tb3_pub_;
			ros::Publisher force_feed_pub_;
			
			// Initial variables
			nav_msgs::Odometry initial_base_odom_;

			// Mode selection
			bool SINGLE_TANK_;
			bool DELAYED_INPUTS_;
			bool NO_TANK_;

			// Force message
			geometry_msgs::Wrench f_cont;
			Eigen::Vector2d F_int_12;
			Eigen::Vector2d F_int_13;
			Eigen::Vector2d F_int_21;
			Eigen::Vector2d F_int_23;
			Eigen::Vector2d F_int_31;
			Eigen::Vector2d F_int_32;
			Eigen::Vector2d F_pass_12;
			Eigen::Vector2d F_pass_13;
			Eigen::Vector2d F_pass_21;
			Eigen::Vector2d F_pass_23;
			Eigen::Vector2d F_pass_31;
			Eigen::Vector2d F_pass_32;
			geometry_msgs::Wrench f_int_1;
			geometry_msgs::Wrench f_int_2;
			geometry_msgs::Wrench f_int_3;
			Eigen::Vector2d F_tot_tb1;
			Eigen::Vector2d F_tot_tb2;
			Eigen::Vector2d F_tot_tb3;
			Eigen::Vector2d F_int_tb1;
			Eigen::Vector2d F_int_tb2;
			Eigen::Vector2d F_int_tb3;
			Eigen::Vector4d Fc_tb1;
			Eigen::Vector4d Fc_tb2;
			Eigen::Vector4d Fc_tb3;
			double des_dist_12[3] = {0.75, -0.75, 0.0};
			double des_dist_13[3] = {0.75, 0.75, 0.0};
			double des_dist_23[3] = {0.0, 1.5, 0.0};
			double K;
			double K_P;
			double K_D; // was 5.0
			double FORCE_GAIN;
			double MASS_BOT; // was 1.0
			double D;
			double SCALING_FACTOR;
			double MAX_FORCE;

			// Obstacles repulsive forces
			Eigen::Vector2d F_cyl1_1;
			Eigen::Vector2d F_cyl1_2;
			Eigen::Vector2d F_cyl1_3;
			Eigen::Vector2d F_cyl2_1;
			Eigen::Vector2d F_cyl2_2;
			Eigen::Vector2d F_cyl2_3;

			// Position messages
			geometry_msgs::Pose pose_tb1;
			geometry_msgs::Pose pose_tb2;
			geometry_msgs::Pose pose_tb3;
			geometry_msgs::Pose pose_tb1_real;
			geometry_msgs::Pose pose_tb2_real;
			geometry_msgs::Pose pose_tb3_real;
			std::vector<geometry_msgs::Pose> pose_tb1_del_;
			std::vector<geometry_msgs::Pose> pose_tb2_del_;
			std::vector<geometry_msgs::Pose> pose_tb3_del_;
			double yaw_tb1;
			double yaw_tb2;
			double yaw_tb3;
			std::vector<double> yaw_tb1_del_;
			std::vector<double> yaw_tb2_del_;
			std::vector<double> yaw_tb3_del_;
			geometry_msgs::Pose cyl1_pose_;
			geometry_msgs::Pose cyl2_pose_;
			double D_MAX_;
			double K_OBS_;

			// Velocity messages
			geometry_msgs::Twist twist_tb1;
			geometry_msgs::Twist twist_tb2;
			geometry_msgs::Twist twist_tb3;
			geometry_msgs::Twist twist_tb1_real;
			geometry_msgs::Twist twist_tb2_real;
			geometry_msgs::Twist twist_tb3_real;
			std::vector<geometry_msgs::Twist> twist_tb1_del_;
			std::vector<geometry_msgs::Twist> twist_tb2_del_;
			std::vector<geometry_msgs::Twist> twist_tb3_del_;
			Eigen::Vector2d vel_tb1;
    		Eigen::Vector2d vel_tb2;
    		Eigen::Vector2d vel_tb3;
			double v_tb1;
			double v_tb2;
			double v_tb3;
			double omega_tb1;
			double omega_tb2;
			double omega_tb3;
			Eigen::Vector2d v12;
			Eigen::Vector2d v13;
			Eigen::Vector2d v23;

			// Delay variables
			int DELAY_WINDOW;
			std::vector<double> Pin_vec_tb1;
			std::vector<double> Pin_vec_tb2;
			std::vector<double> Pin_vec_tb3;
			double Pout_tb1;
			double Pout_tb2;
			double Pout_tb3;

			// Cycle control variables
			bool first_cycle_;
			double start_;
			double cycle_time;
			bool interaction_;
			bool pressed_;

			// TF variables
			tf2_ros::Buffer* tfBuffer;
			tf2_ros::TransformListener* tf2_listener;
			
			// Tank parameters
    		float tank_state_[3];
    		float tank_energy_[3];
			float single_tank_state;
			float single_tank_energy;
    		double xt_dot_[3];
			float TANK_INITIAL_VALUE = 30; //! HIGH = 40 LOW = 25
    		float TANK_MAX_VALUE = 100;
    		float TANK_MIN_VALUE = 5;
    		double sum_y2_;
			double channel_energy_;
			double stored_energy_;

			// Output to file
			std::ofstream tank_file_;
			std::ofstream force_file_;
			std::ofstream pos_file_;
			std::ofstream vel_file_;
			std::ofstream btn_file_;
			std::ofstream master_file_;
            double start_time_;
			geometry_msgs::Pose master_pose_;

};

#endif /* TURTLE_CONTROL_H */