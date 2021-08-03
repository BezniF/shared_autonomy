#include "turtle_control.h"
#include "solver.h"
#include "solver.c"
#include "ldl.c"
#include "matrix_support.c"
#include "util.c"

TurtleControl::TurtleControl(){

    // Subscribers
    joy_sub_ = nh_.subscribe("/joy", 1, &TurtleControl::joyCallback, this);
    omega_sub_ = nh_.subscribe("/fdo6/pose", 1, &TurtleControl::omegaCallback, this);
    button_sub_ = nh_.subscribe("/fdo6/button", 1, &TurtleControl::buttonCallback, this);
    odom_tb1_sub_ = nh_.subscribe("/tb3_0/odom", 1, &TurtleControl::odomTb1Callback, this); 
    odom_tb2_sub_ = nh_.subscribe("/tb3_1/odom", 1, &TurtleControl::odomTb2Callback, this);
    odom_tb3_sub_ = nh_.subscribe("/tb3_2/odom", 1, &TurtleControl::odomTb3Callback, this);
    opti_tb1_sub_ = nh_.subscribe("vrpn_client_node/turtle2/pose", 1, &TurtleControl::optiTb1Callback, this); //! CHECK THE ORDER!
    opti_tb2_sub_ = nh_.subscribe("vrpn_client_node/turtle0/pose", 1, &TurtleControl::optiTb2Callback, this);
    opti_tb3_sub_ = nh_.subscribe("vrpn_client_node/turtle1/pose", 1, &TurtleControl::optiTb3Callback, this);

    // Publishers
    vel_tb1_pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle5/cmd_vel" ,1); //! CHECK THE ORDER!
    vel_tb2_pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle3/cmd_vel" ,1);
    vel_tb3_pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle4/cmd_vel" ,1);
    force_feed_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("/fdo6/wrench_cmd", 1);

    // Tank init
    for(int i = 0; i < 3; i++){
        tank_energy_[i] = TANK_INITIAL_VALUE;
        tank_state_[i] = sqrt(2 * tank_energy_[i]);
    }

    // Force init
    F_tot_tb1.setZero();
    F_tot_tb2.setZero();
    F_tot_tb3.setZero();

    // Cycle time
    cycle_time = 0.002;
    double time_delay_sec;

    // Params loading
    nh_.getParam("K", K);
    nh_.getParam("K_P", K_P);
    nh_.getParam("K_D", K_D);
    nh_.getParam("FORCE_GAIN", FORCE_GAIN);
    nh_.getParam("MASS_BOT", MASS_BOT);
    nh_.getParam("D", D);
    nh_.getParam("DELAY_TIME", time_delay_sec);

    //!DEBUG
    K_P = 90.0; // was 30.0
    K_D = 1.0; // was 1.0
    D = 2.0; // was 2.0
    MASS_BOT = 1.0;

    // Delay window
    time_delay_sec = 0.0; //200ms delay in the communication channel
    DELAY_WINDOW = int(time_delay_sec / cycle_time);

    if(DELAY_WINDOW < 1)
        DELAY_WINDOW = 1;

    ROS_INFO_STREAM("DELAY WINDOW: " << DELAY_WINDOW);
    channel_energy_ = 0.0;

    // Output to file
    std::stringstream file_path;
    file_path << "/home/federico/Documents/Experiments/TRO-20/Simulations/Exp1/tank.txt";
    tank_file_.open(file_path.str());

    std::stringstream file_path2;
    file_path2 << "/home/federico/Documents/Experiments/TRO-20/Simulations/Exp1/force.txt";
    force_file_.open(file_path2.str());

    std::stringstream file_path3;
    file_path3 << "/home/federico/Documents/Experiments/TRO-20/Simulations/Exp1/pos.txt";
    pos_file_.open(file_path3.str());

    std::stringstream file_path4;
    file_path4 << "/home/federico/Documents/Experiments/TRO-20/Simulations/Exp1/vel.txt";
    vel_file_.open(file_path4.str());

    std::stringstream file_path5;
    file_path5 << "/home/federico/Documents/Experiments/TRO-20/Simulations/Exp1/btn.txt";
    btn_file_.open(file_path5.str());

    start_time_ = ros::Time::now().toSec();

    pressed_ = false;

    // Vector init
    Pin_vec_tb1.resize(DELAY_WINDOW, 0.0);
    Pin_vec_tb2.resize(DELAY_WINDOW, 0.0);
    Pin_vec_tb3.resize(DELAY_WINDOW, 0.0);

    // Time init
    first_cycle_ = true;

    // Mode selection
    SINGLE_TANK_ = true;
    DELAYED_INPUTS_ = false;
    NO_TANK_ = true;

    if(SINGLE_TANK_){
        single_tank_energy = TANK_INITIAL_VALUE;
        single_tank_state = sqrt(2 * single_tank_energy);
    }

}

TurtleControl::~TurtleControl(){

    tank_file_.close();
    force_file_.close();
    pos_file_.close();
    vel_file_.close();
    btn_file_.close();

}

void TurtleControl::joyCallback(const sensor_msgs::JoyConstPtr& msg){
    
    //! Commented since I'm using Omega now
    /*sensor_msgs::Joy joy = *msg;

    f_cont.force.x = FORCE_GAIN * joy.axes[1];
    f_cont.force.y = FORCE_GAIN * joy.axes[0];
    f_cont.force.z = 0.0;
    f_cont.torque.x = 0.0;
    f_cont.torque.y = 0.0;
    f_cont.torque.z = 0.0;

    if (bool(joy.buttons[4]))
        K = 0.5; // was 2.0
    else
        K = 1.0;*/


}

void TurtleControl::forceCallback(const geometry_msgs::WrenchStamped& msg){

    //TODO
    // ? DIRECT FORCE MSG FROM OMEGA?
}

void TurtleControl::omegaCallback(const geometry_msgs::PoseStamped& msg){

    //! DEBUG
    FORCE_GAIN = 40.0; // Was 50.0

    double delta_x = msg.pose.position.x;
    double delta_y = msg.pose.position.y;

    f_cont.force.x = - FORCE_GAIN * (delta_x / 0.05);
    // f_cont.force.y = - (FORCE_GAIN / 5)* (delta_y / 0.1);
    // f_cont.force.x = 0.0;
    f_cont.force.y = 0.0;

    // ROS_INFO_STREAM("F master: " << f_cont);

}

void TurtleControl::buttonCallback(const std_msgs::Bool& msg){
    if(msg.data == true){
        if(K == 1.0)
            K = 0.33;
        else if (K == 0.33)
            K = 1.0;
        
        pressed_ = true;
    }

    // ROS_INFO_STREAM("K: " << K);

}

void TurtleControl::odomTb1Callback(const nav_msgs::Odometry& odom){

    pose_tb1 = odom.pose.pose;
    twist_tb1 = odom.twist.twist;

    yaw_tb1 = quaternionToRPY(pose_tb1.orientation);

}

void TurtleControl::odomTb2Callback(const nav_msgs::Odometry& odom){

    pose_tb2 = odom.pose.pose;
    twist_tb2 = odom.twist.twist;

    yaw_tb2 = quaternionToRPY(pose_tb2.orientation);
}

void TurtleControl::odomTb3Callback(const nav_msgs::Odometry& odom){

    pose_tb3 = odom.pose.pose;
    twist_tb3 = odom.twist.twist;

    yaw_tb3 = quaternionToRPY(pose_tb3.orientation);
}

void TurtleControl::optiTb1Callback(const geometry_msgs::PoseStamped& msg){

    pose_tb1 = msg.pose;

    yaw_tb1 = quaternionToRPY(pose_tb1.orientation);

    if((first_cycle_) && (DELAYED_INPUTS_)){
        pose_tb1_del_.resize(DELAY_WINDOW, pose_tb1);
    }

    if(DELAYED_INPUTS_){
        pose_tb1_del_.push_back(pose_tb1);
        pose_tb1_del_.erase(pose_tb1_del_.begin());

        pose_tb1 = pose_tb1_del_[0];
    }

    pose_tb1_real = pose_tb1;

}

void TurtleControl::optiTb2Callback(const geometry_msgs::PoseStamped& msg){

    pose_tb2 = msg.pose;

    yaw_tb2 = quaternionToRPY(pose_tb2.orientation);

    if((first_cycle_) && (DELAYED_INPUTS_)){
        pose_tb2_del_.resize(DELAY_WINDOW, pose_tb2);
    }

    if(DELAYED_INPUTS_){
        pose_tb2_del_.push_back(pose_tb2);
        pose_tb2_del_.erase(pose_tb2_del_.begin());

        pose_tb2 = pose_tb2_del_[0];
    }

    pose_tb2_real = pose_tb2;

}

void TurtleControl::optiTb3Callback(const geometry_msgs::PoseStamped& msg){

    pose_tb3 = msg.pose;

    yaw_tb3 = quaternionToRPY(pose_tb3.orientation);

    if(first_cycle_){
        pose_tb3_del_.resize(DELAY_WINDOW, pose_tb3);
    }

    if(DELAYED_INPUTS_){
        pose_tb3_del_.push_back(pose_tb3);
        pose_tb3_del_.erase(pose_tb3_del_.begin());

        pose_tb3 = pose_tb3_del_[0];
    }

    pose_tb3_real = pose_tb3;

}


double TurtleControl::quaternionToRPY(geometry_msgs::Quaternion q){

    tf::Quaternion quat(q.x, q.y, q.z, q.w);
    tf::Matrix3x3 m(quat);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    return yaw;
}

void TurtleControl::computeForceInteraction(){

    // Computing the single interaction forces
    /*F_int_12[0] = K_P * (pose_tb2.position.x - pose_tb1.position.x + K * des_dist_12[0]);
    F_int_12[1] = K_P * (pose_tb2.position.y - pose_tb1.position.y + K * des_dist_12[1]);
    F_int_13[0] = K_P * (pose_tb3.position.x - pose_tb1.position.x + K * des_dist_13[0]);
    F_int_13[1] = K_P * (pose_tb3.position.y - pose_tb1.position.y + K * des_dist_13[1]);
    F_int_23[0] = K_P * (pose_tb3.position.x - pose_tb2.position.x + K * des_dist_23[0]);
    F_int_23[1] = K_P * (pose_tb3.position.y - pose_tb2.position.y + K * des_dist_23[1]);*/

    // DELAY FORMULATION
    F_int_12[0] = K_P * (pose_tb2.position.x - pose_tb1_real.position.x + K * des_dist_12[0]);
    F_int_12[1] = K_P * (pose_tb2.position.y - pose_tb1_real.position.y + K * des_dist_12[1]);
    F_int_13[0] = K_P * (pose_tb3.position.x - pose_tb1_real.position.x + K * des_dist_13[0]);
    F_int_13[1] = K_P * (pose_tb3.position.y - pose_tb1_real.position.y + K * des_dist_13[1]);
    F_int_21[0] = K_P * (pose_tb1.position.x - pose_tb2_real.position.x - K * des_dist_12[0]);
    F_int_21[1] = K_P * (pose_tb1.position.y - pose_tb2_real.position.y - K * des_dist_12[1]);
    F_int_23[0] = K_P * (pose_tb3.position.x - pose_tb2_real.position.x + K * des_dist_23[0]);
    F_int_23[1] = K_P * (pose_tb3.position.y - pose_tb2_real.position.y + K * des_dist_23[1]);
    F_int_31[0] = K_P * (pose_tb1.position.x - pose_tb3_real.position.x - K * des_dist_13[0]);
    F_int_31[1] = K_P * (pose_tb1.position.y - pose_tb3_real.position.y - K * des_dist_13[1]);
    F_int_32[0] = K_P * (pose_tb2.position.x - pose_tb3_real.position.x - K * des_dist_23[0]);
    F_int_32[1] = K_P * (pose_tb2.position.y - pose_tb3_real.position.y - K * des_dist_23[1]);

    // ULTIMATE DEBUGGING
    /*ROS_INFO_STREAM("*****************************");
    ROS_INFO_STREAM("Pose TB1 :" << pose_tb1);
    ROS_INFO_STREAM("Pose TB2 :" << pose_tb2);
    ROS_INFO_STREAM("Pose TB3 :" << pose_tb3);
    ROS_INFO_STREAM("Pose TB1 REAL:" << pose_tb1_real);
    ROS_INFO_STREAM("Pose TB2 REAL:" << pose_tb2_real);
    ROS_INFO_STREAM("Pose TB3 REAL:" << pose_tb3_real);
    ROS_INFO_STREAM("*****************************");
    ROS_INFO_STREAM("F_spring 12x: " << F_int_12[0]);
    ROS_INFO_STREAM("F_spring 12y: " << F_int_12[1]);
    ROS_INFO_STREAM("F_spring 13x: " << F_int_13[0]);
    ROS_INFO_STREAM("F_spring 13y: " << F_int_13[1]);
    ROS_INFO_STREAM("F_spring 21x: " << F_int_21[0]);
    ROS_INFO_STREAM("F_spring 21y: " << F_int_21[1]);
    ROS_INFO_STREAM("F_spring 23x: " << F_int_23[0]);
    ROS_INFO_STREAM("F_spring 23y: " << F_int_23[1]);
    ROS_INFO_STREAM("F_spring 31x: " << F_int_31[0]);
    ROS_INFO_STREAM("F_spring 31y: " << F_int_31[1]);
    ROS_INFO_STREAM("F_spring 32x: " << F_int_32[0]);
    ROS_INFO_STREAM("F_spring 32y: " << F_int_32[1]);
    ROS_INFO_STREAM("*****************************");*/

    // Add the damping term
    F_int_12[0] += K_D * (twist_tb2.linear.x*cos(yaw_tb2) - twist_tb1_real.linear.x*cos(yaw_tb1));
    F_int_12[1] += K_D * (twist_tb2.linear.x*sin(yaw_tb2) - twist_tb1_real.linear.x*sin(yaw_tb1));
    F_int_13[0] += K_D * (twist_tb3.linear.x*cos(yaw_tb3) - twist_tb1_real.linear.x*cos(yaw_tb1));
    F_int_13[1] += K_D * (twist_tb3.linear.x*sin(yaw_tb3) - twist_tb1_real.linear.x*sin(yaw_tb1));
    F_int_21[0] += K_D * (twist_tb1.linear.x*cos(yaw_tb1) - twist_tb2_real.linear.x*cos(yaw_tb2));
    F_int_21[1] += K_D * (twist_tb1.linear.x*sin(yaw_tb1) - twist_tb2_real.linear.x*sin(yaw_tb2));
    F_int_23[0] += K_D * (twist_tb3.linear.x*cos(yaw_tb3) - twist_tb2_real.linear.x*cos(yaw_tb2));
    F_int_23[1] += K_D * (twist_tb3.linear.x*sin(yaw_tb3) - twist_tb2_real.linear.x*sin(yaw_tb2));
    F_int_31[0] += K_D * (twist_tb1.linear.x*cos(yaw_tb1) - twist_tb3_real.linear.x*cos(yaw_tb3));
    F_int_31[1] += K_D * (twist_tb1.linear.x*sin(yaw_tb1) - twist_tb3_real.linear.x*sin(yaw_tb3));
    F_int_32[0] += K_D * (twist_tb2.linear.x*cos(yaw_tb2) - twist_tb3_real.linear.x*cos(yaw_tb3));
    F_int_32[1] += K_D * (twist_tb2.linear.x*sin(yaw_tb2) - twist_tb3_real.linear.x*sin(yaw_tb3));


    // Spring force for preserving formation
    f_int_1.force.x = K_P * (pose_tb2.position.x - pose_tb1.position.x + K * des_dist_12[0]) + K_P * (pose_tb3.position.x - pose_tb1.position.x + K * des_dist_13[0]);
    f_int_1.force.y = K_P * (pose_tb2.position.y - pose_tb1.position.y + K * des_dist_12[1]) + K_P * (pose_tb3.position.y - pose_tb1.position.y + K * des_dist_13[1]);

    f_int_2.force.x = K_P * (pose_tb1.position.x - pose_tb2.position.x - K * des_dist_12[0]) + K_P * (pose_tb3.position.x - pose_tb2.position.x + K * des_dist_23[0]);
    f_int_2.force.y = K_P * (pose_tb1.position.y - pose_tb2.position.y - K * des_dist_12[1]) + K_P * (pose_tb3.position.y - pose_tb2.position.y + K * des_dist_23[1]);

    f_int_3.force.x = K_P * (pose_tb1.position.x - pose_tb3.position.x - K * des_dist_13[0]) + K_P * (pose_tb2.position.x - pose_tb3.position.x - K * des_dist_23[0]);
    f_int_3.force.y = K_P * (pose_tb1.position.y - pose_tb3.position.y - K * des_dist_13[1]) + K_P * (pose_tb2.position.y - pose_tb3.position.y - K * des_dist_23[1]);

    // Additional damping term
    f_int_1.force.x += K_D * (twist_tb2.linear.x*cos(yaw_tb2) - twist_tb1.linear.x*cos(yaw_tb1)) + K_D * (twist_tb3.linear.x*cos(yaw_tb3) - twist_tb1.linear.x*cos(yaw_tb1));
    f_int_1.force.y += K_D * (twist_tb2.linear.x*sin(yaw_tb2) - twist_tb1.linear.x*sin(yaw_tb1)) + K_D * (twist_tb3.linear.x*sin(yaw_tb3) - twist_tb1.linear.x*sin(yaw_tb1));

    f_int_2.force.x += K_D * (twist_tb1.linear.x*cos(yaw_tb1) - twist_tb2.linear.x*cos(yaw_tb2)) + K_D * (twist_tb3.linear.x*cos(yaw_tb3) - twist_tb2.linear.x*cos(yaw_tb2));
    f_int_2.force.y += K_D * (twist_tb1.linear.x*sin(yaw_tb1) - twist_tb2.linear.x*sin(yaw_tb2)) + K_D * (twist_tb3.linear.x*sin(yaw_tb3) - twist_tb2.linear.x*sin(yaw_tb2));

    f_int_3.force.x += K_D * (twist_tb2.linear.x*cos(yaw_tb2) - twist_tb3.linear.x*cos(yaw_tb3)) + K_D * (twist_tb1.linear.x*cos(yaw_tb1) - twist_tb3.linear.x*cos(yaw_tb3));
    f_int_3.force.y += K_D * (twist_tb2.linear.x*sin(yaw_tb2) - twist_tb3.linear.x*sin(yaw_tb3)) + K_D * (twist_tb1.linear.x*sin(yaw_tb1) - twist_tb3.linear.x*sin(yaw_tb3));

}


void TurtleControl::computeVelocities(){

    //* NB: Fc is only F_int in this case, there is no F_cont //
    F_int_tb1[0] = f_int_1.force.x;
    F_int_tb1[1] = f_int_1.force.y;
    F_int_tb2[0] = f_int_2.force.x;
    F_int_tb2[1] = f_int_2.force.y;
    F_int_tb3[0] = f_int_3.force.x;
    F_int_tb3[1] = f_int_3.force.y;

    vel_tb1[0] = twist_tb1.linear.x * cos(yaw_tb1);
    vel_tb1[1] = twist_tb1.linear.x * sin(yaw_tb1);

    vel_tb2[0] = twist_tb2.linear.x * cos(yaw_tb2);
    vel_tb2[1] = twist_tb2.linear.x * sin(yaw_tb2);

    vel_tb3[0] = twist_tb3.linear.x * cos(yaw_tb3);
    vel_tb3[1] = twist_tb3.linear.x * sin(yaw_tb3);

    computePowers();

    // Store variables in a suitable way for the optimization problem
    Eigen::Vector4d F1 = { F_int_12[0], F_int_12[1], F_int_13[0], F_int_13[1]};
    Eigen::Vector4d F2 = { F_int_21[0], F_int_21[1], F_int_23[0], F_int_23[1]};
    Eigen::Vector4d F3 = { F_int_31[0], F_int_31[1], F_int_32[0], F_int_32[1]};

    v12 = vel_tb1 - vel_tb2;
    v13 = vel_tb1 - vel_tb3;
    v23 = vel_tb2 - vel_tb3;

    // ROS_INFO_STREAM("F1 BEFORE opt: " << F1);

    Eigen::Vector4d dotx_1 = { v12[0], v12[1], v13[0], v13[1]};    
    Eigen::Vector4d dotx_2 = {-v12[0],-v12[1], v23[0], v23[1]};
    Eigen::Vector4d dotx_3 = {-v13[0],-v13[1],-v23[0],-v23[1]};

    // Compute the optimal input for preserving passivity according to the currently selected mode
    if(!NO_TANK_){
        if(!SINGLE_TANK_){
            Fc_tb1 = optimizationProblem(dotx_1, F1, tank_energy_[0], Pin_vec_tb1[0], Pout_tb1);
            Fc_tb2 = optimizationProblem(dotx_2, F2, tank_energy_[1], Pin_vec_tb2[0], Pout_tb2);
            Fc_tb3 = optimizationProblem(dotx_3, F3, tank_energy_[2], Pin_vec_tb3[0], Pout_tb3);
        }
        else{
            Eigen::VectorXd F_st(12);
            F_st << F1, F2, F3;

            Eigen::VectorXd v_st(12);
            v_st << dotx_1, dotx_2, dotx_3;

            Eigen::VectorXd Fc_tot = singleTankOptProblem(v_st, F_st, single_tank_energy);

            for(int i = 0; i < Fc_tb1.size(); i++)
                Fc_tb1[i] = Fc_tot[i];

            for(int i = 0; i < Fc_tb2.size(); i++)
                Fc_tb2[i] = Fc_tot[i + 4];

            for(int i = 0; i < Fc_tb3.size(); i++)
                Fc_tb3[i] = Fc_tot[i + 8];   
        }
    }
    else{
        Fc_tb1 = F1;
        Fc_tb2 = F2;
        Fc_tb3 = F3;
    }

    // ROS_INFO_STREAM("F1 AFTER opt: " << Fc_tb1);

    computeTotalForce();

    // Apply the computed acceleration
    Eigen::Vector2d acc_tb1 = F_tot_tb1 / MASS_BOT;
    Eigen::Vector2d acc_tb2 = F_tot_tb2 / MASS_BOT;
    Eigen::Vector2d acc_tb3 = F_tot_tb3 / MASS_BOT;

    vel_tb1[0] +=  acc_tb1[0] * cycle_time;
    vel_tb1[1] +=  acc_tb1[1] * cycle_time;

    vel_tb2[0] +=  acc_tb2[0] * cycle_time;
    vel_tb2[1] +=  acc_tb2[1] * cycle_time;

    vel_tb3[0] +=  acc_tb3[0] * cycle_time;
    vel_tb3[1] +=  acc_tb3[1] * cycle_time;
    
}

void TurtleControl::computePowers(){

    //? OLD DEFINITION
    /*double P_tb1 = F_int_tb1.transpose() * vel_tb1;
    double P_tb2 = F_int_tb2.transpose() * vel_tb2;
    double P_tb3 = F_int_tb3.transpose() * vel_tb3;

    double Pin_tb1 = 0.3334 * (P_tb2 + P_tb3);
    double Pin_tb2 = 0.3334 * (P_tb1 + P_tb3);
    double Pin_tb3 = 0.3334 * (P_tb1 + P_tb2);

    Pout_tb1 = 0.6667 * (P_tb1);
    Pout_tb2 = 0.6667 * (P_tb2);
    Pout_tb3 = 0.6667 * (P_tb3);*/
    
    //! NEW DEFINITION
    double P_EXCH = 10;

    double Pin_tb1;
    double Pin_tb2;
    double Pin_tb3;

    if(tank_energy_[0] >= 3 * TANK_MIN_VALUE)
        Pout_tb1 = P_EXCH;
    else
        Pout_tb1 = 0.0;
    
    if(tank_energy_[1] >= 3 * TANK_MIN_VALUE)
        Pout_tb2 = P_EXCH;
    else
        Pout_tb2 = 0.0;
    
    if(tank_energy_[2] >= 3 * TANK_MIN_VALUE)
        Pout_tb3 = P_EXCH;
    else
        Pout_tb3 = 0.0;
    
    Pin_tb1 = 0.5 * (Pout_tb2 + Pout_tb3);
    Pin_tb2 = 0.5 * (Pout_tb1 + Pout_tb3);
    Pin_tb3 = 0.5 * (Pout_tb1 + Pout_tb2);

    Pin_vec_tb1.push_back(Pin_tb1);
    Pin_vec_tb2.push_back(Pin_tb2);
    Pin_vec_tb3.push_back(Pin_tb3);

    Pin_vec_tb1.erase(Pin_vec_tb1.begin());
    Pin_vec_tb2.erase(Pin_vec_tb2.begin());
    Pin_vec_tb3.erase(Pin_vec_tb3.begin());

    // Compute the value of the energy stored in the communication channel at the current time
    channel_energy_ = 0.0;

    for(int i= 1; i < DELAY_WINDOW; i++){
        channel_energy_ += Pin_vec_tb1[i];
        channel_energy_ += Pin_vec_tb2[i];
        channel_energy_ += Pin_vec_tb3[i]; 
    }
}

Eigen::Vector4d TurtleControl::optimizationProblem(Eigen::Vector4d v, Eigen::Vector4d F, double T, double Pin, double Pout){

    // Optimization problem for finding the best passive approximation of the force input
    set_defaults();  // Set basic algorithm parameters.
    setup_indexing();

    params.dotx[0] = v[0];
    params.dotx[1] = v[1];
    params.dotx[2] = v[2];
    params.dotx[3] = v[3];
    params.Fd[0] = F[0];
    params.Fd[1] = F[1];
    params.Fd[2] = F[2];
    params.Fd[3] = F[3];
    params.tau[0] = cycle_time;
    params.T0[0] = T;
    params.varepsilon[0] = TANK_MIN_VALUE;
    // params.P_in[0] = Pin; //! EXPERIMENT WITH DELAY
    // params.P_out[0] = Pout;
    // params.P_in[0] = 0.0; //! EXPERIMENT WITHOUT DELAY
    // params.P_out[0] = 0.0;

    settings.verbose = 0;
    settings.max_iters = 50;
    long num_iters = solve();

    Eigen::Vector4d Fc = {vars.Fc[0], vars.Fc[2], vars.Fc[1], vars.Fc[3]};

    return Fc;

}

Eigen::VectorXd TurtleControl::singleTankOptProblem(Eigen::VectorXd v, Eigen::VectorXd F, double T){
    
    // Optimization problem for a single N ports tank
    set_defaults();  // Set basic algorithm parameters.
    setup_indexing();

    for(int i=0; i < F.size(); i++)
        params.Fd[i] = F[i];

    for(int i=0; i < v.size(); i++)
        params.dotx[i] = v[i];

    params.tau[0] = cycle_time;
    params.T0[0] = T;
    params.varepsilon[0] = TANK_MIN_VALUE;

    settings.verbose = 0;
    settings.max_iters = 50;
    long num_iters = solve();

    Eigen::VectorXd Fc(12);

    for(int i= 0; i < Fc.size(); i++)
        Fc[i] = vars.Fc[i];

    return Fc;
}

void TurtleControl::computeTotalForce(){

    // Store the values of the passive forces coming from the optimizer
    F_pass_12 = {Fc_tb1[0], Fc_tb1[2]};
    F_pass_13 = {Fc_tb1[1], Fc_tb1[3]};
    F_pass_21 = {Fc_tb2[0], Fc_tb2[2]};
    F_pass_23 = {Fc_tb2[1], Fc_tb2[3]};
    F_pass_31 = {Fc_tb3[0], Fc_tb3[2]};
    F_pass_32 = {Fc_tb3[1], Fc_tb3[3]};

    //Summing the total interaction force coming from the optimizer
    Eigen::Vector2d Fc_tot1, Fc_tot2, Fc_tot3; 
    Fc_tot1[0] = Fc_tb1[0] + Fc_tb1[2];
    Fc_tot1[1] = Fc_tb1[1] + Fc_tb1[3];

    Fc_tot2[0] = Fc_tb2[0] + Fc_tb2[2];
    Fc_tot2[1] = Fc_tb2[1] + Fc_tb2[3];

    Fc_tot3[0] = Fc_tb3[0] + Fc_tb3[2];
    Fc_tot3[1] = Fc_tb3[1] + Fc_tb3[3];

    // Compute the final overall force, including a damping term
    F_tot_tb1[0] = f_cont.force.x + Fc_tot1[0] - D * vel_tb1[0]; 
    F_tot_tb1[1] = f_cont.force.y + Fc_tot1[1] - D * vel_tb1[1];

    F_tot_tb2[0] = Fc_tot2[0] - D * vel_tb2[0];
    F_tot_tb2[1] = Fc_tot2[1] - D * vel_tb2[1];

    F_tot_tb3[0] = Fc_tot3[0] - D * vel_tb3[0];
    F_tot_tb3[1] = Fc_tot3[1] - D * vel_tb3[1];

}

void TurtleControl::computeTankEnergy(){

    // Update the tank energy
    double P1 = F_pass_12.transpose()* v12;
    P1 += F_pass_13.transpose() * v13;

    double P2 = F_pass_21.transpose() * (-v12);
    P2 += F_pass_23.transpose() * v23;

    double P3 = F_pass_31.transpose() * (-v13);
    P3 += F_pass_32.transpose() * (-v23);

    if(!SINGLE_TANK_){
        tank_energy_[0] += cycle_time * (-P1);
        tank_energy_[1] += cycle_time * (-P2);
        tank_energy_[2] += cycle_time * (-P3);

        // Power exchange along the channel
        tank_energy_[0] += cycle_time * (Pin_vec_tb1[0] - Pout_tb1); //! EXPERIMENT WITH / WITHOUT DELAY
        tank_energy_[1] += cycle_time * (Pin_vec_tb2[0] - Pout_tb2);
        tank_energy_[2] += cycle_time * (Pin_vec_tb3[0] - Pout_tb3);

        // Update tanks' state
        tank_state_[0] = sqrt(2 * tank_energy_[0]);
        tank_state_[1] = sqrt(2 * tank_energy_[1]);
        tank_state_[2] = sqrt(2 * tank_energy_[2]);

        for(int i=0; i < 3; i++){
            if(tank_state_[i] >= TANK_MAX_VALUE)
                tank_state_[i] = TANK_MAX_VALUE;
        }
    }

    else{
        single_tank_energy += cycle_time * (-P1 -P2 -P3);
        single_tank_state = sqrt(2 * single_tank_energy);
    }

    // Compute the stored energy int{0}{t}Fs*v*dt
    stored_energy_ += cycle_time * (f_cont.force.x * vel_tb1[0] + f_cont.force.y * vel_tb1[1]);
    // stored_energy_ += cycle_time * (f_cont.force.x * v_tb1);

}

void TurtleControl::computeIOSLF(){

    // Utilize IO-SFL for computing v and omega
    double b = 0.05;

    //ROS_WARN_STREAM("Vel 1: " << vel_tb1);

    v_tb1 = vel_tb1[0] * cos(yaw_tb1) + vel_tb1[1] * sin(yaw_tb1);
    v_tb2 = vel_tb2[0] * cos(yaw_tb2) + vel_tb2[1] * sin(yaw_tb2);
    v_tb3 = vel_tb3[0] * cos(yaw_tb3) + vel_tb3[1] * sin(yaw_tb3);

    // ROS_WARN_STREAM("V 1: " << v_tb1);

    omega_tb1 = -sin(yaw_tb1) * vel_tb1[0] / b + cos(yaw_tb1) * vel_tb1[1] / b;
    omega_tb2 = -sin(yaw_tb2) * vel_tb2[0] / b + cos(yaw_tb2) * vel_tb2[1] / b;
    omega_tb3 = -sin(yaw_tb3) * vel_tb3[0] / b + cos(yaw_tb3) * vel_tb3[1] / b;

}

void::TurtleControl::saturateSpeed(){

    //Saturating the speed of each turtlebot
    double v_max = 0.1;
    double omega_max = 0.5;

    if(v_tb1 >= v_max)
        v_tb1 = v_max;
    else if (v_tb1 <= -v_max)
        v_tb1 = -v_max;
    
    if(v_tb2 >= v_max)
        v_tb2 = v_max;
    else if (v_tb2 <= -v_max)
        v_tb2 = -v_max;
    
    if(v_tb3 >= v_max)
        v_tb3 = v_max;
    else if (v_tb3 <= -v_max)
        v_tb3 = -v_max;


    if(omega_tb1 >= omega_max)
        omega_tb1 = omega_max;
    else if (omega_tb1 <= -omega_max)
        omega_tb1 = -omega_max;
    
    if(omega_tb2 >= omega_max)
        omega_tb2 = omega_max;
    else if (omega_tb2 <= -omega_max)
        omega_tb2 = -omega_max;
    
    if(omega_tb3 >= omega_max)
        omega_tb3 = omega_max;
    else if (omega_tb3 <= -omega_max)
        omega_tb3 = -omega_max;

}

void::TurtleControl::writeToFiles(){

    tank_file_ << ros::Time::now().toSec() - start_time_;

    for(int i=0; i<3; i++)
        tank_file_ << " " << tank_state_[i] << " " << tank_energy_[i];

    tank_file_ << " " << channel_energy_;
    tank_file_ << " " << single_tank_energy;
    tank_file_ << " " << stored_energy_;
    tank_file_ << std::endl;

    force_file_ << ros::Time::now().toSec() - start_time_;

    force_file_ << " " << F_tot_tb1[0] << " " << F_tot_tb1[1];
    force_file_ << " " << F_tot_tb2[0] << " " << F_tot_tb2[1];
    force_file_ << " " << F_tot_tb3[0] << " " << F_tot_tb3[1];
    force_file_ << " " << F_int_tb1[0] << " " << F_int_tb1[1];
    force_file_ << " " << f_cont.force.x << " " << f_cont.force.y;

    force_file_ << std::endl;

    pos_file_ << ros::Time::now().toSec() - start_time_;

    pos_file_ << " " << pose_tb1.position.x << " " << pose_tb1.position.y;
    pos_file_ << " " << pose_tb2.position.x << " " << pose_tb2.position.y;
    pos_file_ << " " << pose_tb3.position.x << " " << pose_tb3.position.y;

    pos_file_ << std::endl;

    vel_file_ << ros::Time::now().toSec() - start_time_;

    vel_file_ << " " << vel_tb1[0] << " " << vel_tb1[1];
    vel_file_ << " " << vel_tb2[0] << " " << vel_tb2[1];
    vel_file_ << " " << vel_tb3[0] << " " << vel_tb3[1];

    vel_file_ << std::endl;

    btn_file_ << ros::Time::now().toSec() - start_time_;

    if(pressed_){

        btn_file_ << " " << 100;
        pressed_ = false;
    }
    else
    {
        btn_file_ << " " << 0;
    }
    
    btn_file_ << std::endl;

}

void TurtleControl::spin(){

    computeForceInteraction();
    computeVelocities();
    computeTankEnergy();
    computeIOSLF();
    saturateSpeed();
    writeToFiles();

    geometry_msgs::Twist cmd_vel_1;
    geometry_msgs::Twist cmd_vel_2;
    geometry_msgs::Twist cmd_vel_3;

    cmd_vel_1.linear.x = - v_tb1; //! ATM for some reason turtle5 motion along x is inverted WTF
    // cmd_vel_1.angular.z = omega_tb1; //!DEBUG
    cmd_vel_1.angular.z = 0.0;

    cmd_vel_2.linear.x = v_tb2;
    cmd_vel_2.angular.z = omega_tb2;

    cmd_vel_3.linear.x = v_tb3;
    cmd_vel_3.angular.z = omega_tb3;

    vel_tb1_pub_.publish(cmd_vel_1);
    vel_tb2_pub_.publish(cmd_vel_2);
    vel_tb3_pub_.publish(cmd_vel_3);

    // Saving the current velocity commands as variables for the next cycle
    twist_tb1.linear.x = v_tb1;
    twist_tb2.linear.x = v_tb2;
    twist_tb3.linear.x = v_tb3;

    twist_tb1.angular.z = omega_tb1;
    twist_tb2.angular.z = omega_tb2;
    twist_tb3.angular.z = omega_tb3;

    if((first_cycle_) && (DELAYED_INPUTS_)){
        twist_tb1_del_.resize(DELAY_WINDOW, twist_tb1);
        twist_tb2_del_.resize(DELAY_WINDOW, twist_tb2);
        twist_tb3_del_.resize(DELAY_WINDOW, twist_tb3);
    }
    if(DELAYED_INPUTS_){
        twist_tb1_del_.push_back(twist_tb1);
        twist_tb2_del_.push_back(twist_tb2);
        twist_tb3_del_.push_back(twist_tb3);

        twist_tb1_del_.erase(twist_tb1_del_.begin());
        twist_tb2_del_.erase(twist_tb2_del_.begin());
        twist_tb3_del_.erase(twist_tb3_del_.begin());

        twist_tb1 = twist_tb1_del_[0];
        twist_tb2 = twist_tb2_del_[0];
        twist_tb3 = twist_tb3_del_[0];
    }

    twist_tb1_real = twist_tb1;
    twist_tb2_real = twist_tb2;
    twist_tb3_real = twist_tb3;
    
    first_cycle_ = false;

    // Publish the force feedback to the Omega master
    geometry_msgs::WrenchStamped force_feed;
    double SCALING_FACTOR = 30;

    force_feed.wrench.force.x = (F_pass_12[0] + F_pass_13[0]) / SCALING_FACTOR;
    force_feed.wrench.force.y = (F_pass_12[1] + F_pass_13[1]) / SCALING_FACTOR;

    // force_feed_pub_.publish(force_feed);

}