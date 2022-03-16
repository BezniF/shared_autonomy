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
    opti_tb1_sub_ = nh_.subscribe("vrpn_client_node/turtle1/pose", 1, &TurtleControl::optiTb1Callback, this); //! CHECK THE ORDER!
    opti_tb2_sub_ = nh_.subscribe("vrpn_client_node/turtle2/pose", 1, &TurtleControl::optiTb2Callback, this);
    opti_tb3_sub_ = nh_.subscribe("vrpn_client_node/turtle3/pose", 1, &TurtleControl::optiTb3Callback, this);
    gazebo_obs_sub_ = nh_.subscribe("/gazebo/model_states", 1, &TurtleControl::gazeboObsCallback, this);
    ellipse_sub_ = nh_.subscribe("vrpn_client_node/Ellipse/pose", 1, &TurtleControl::ellipseCallback, this);
    object_sub_ = nh_.subscribe("vrpn_client_node/object1/pose", 1, &TurtleControl::objectCallback, this);

    // Dynamic Reconfigure
    dynamic_reconfigure::Server<shared_autonomy::SharedAutonomyConfig>::CallbackType f;
    f = boost::bind(&TurtleControl::setControllerParameters, this, _1, _2);
    srv.setCallback(f);

    // Publishers
    nh_.getParam("SIM", SIM_);

    // change publishers' topic according to sim or real robots
    if (SIM_)
    {
        vel_tb1_pub_ = nh_.advertise<geometry_msgs::Twist>("/tb3_0/cmd_vel", 1); //! CHECK THE ORDER!
        vel_tb2_pub_ = nh_.advertise<geometry_msgs::Twist>("/tb3_1/cmd_vel", 1);
        vel_tb3_pub_ = nh_.advertise<geometry_msgs::Twist>("/tb3_2/cmd_vel", 1);
    }
    else
    {
        vel_tb1_pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1); //! CHECK THE ORDER!
        vel_tb2_pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 1);
        vel_tb3_pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle3/cmd_vel", 1);
    }
    
    force_feed_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("/fdo6/wrench_cmd", 1);

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
    nh_.getParam("B_INJ", B_);
    nh_.getParam("DELAY_TIME", time_delay_sec);
    nh_.getParam("SINGLE_TANK_", SINGLE_TANK_);
    nh_.getParam("DAMPING_INJ", DAMPING_INJ_);
    nh_.getParam("DELAYED_INPUTS_", DELAYED_INPUTS_);
    nh_.getParam("NO_TANK_", NO_TANK_);
    nh_.getParam("TANK_INITIAL_VALUE", TANK_INITIAL_VALUE);
    nh_.getParam("TANK_MAX_VALUE", TANK_MAX_VALUE);
    nh_.getParam("TANK_MIN_VALUE", TANK_MIN_VALUE);
    nh_.getParam("D_MAX_", D_MAX_);
    nh_.getParam("K_OBS_", K_OBS_);
    nh_.getParam("SCALING_FACTOR", SCALING_FACTOR);
    nh_.getParam("MAX_FORCE", MAX_FORCE);
    nh_.getParam("SIM_X_GAIN", sim_x_gain_);
    nh_.getParam("SIM_Y_GAIN", sim_y_gain_);

    // Tank init
    for(int i = 0; i < 3; i++){
        tank_energy_[i] = TANK_INITIAL_VALUE;
        tank_state_[i] = sqrt(2 * tank_energy_[i]);
    }

    // Delay window
    DELAY_WINDOW = int(time_delay_sec * 120.0); //! USE THE HZ RATE FROM OPTITRACK! WATCH THE CAMERA FRAME RATE THERE!

    if(DELAY_WINDOW < 1)
        DELAY_WINDOW = 1;

    // ROS_INFO_STREAM("DELAY WINDOW: " << DELAY_WINDOW);
    channel_energy_ = 0.0;

    // Output to file
    std::string file_path;

    file_path = "/home/federico/MATLAB_ws/T-RO21/Simulations";

    // if (DAMPING_INJ_)
    // {
    //     file_path = file_path + "/INJ";
    // }
    
    
    tank_file_.open(file_path + "/tank.txt");

    force_file_.open(file_path + "/force.txt");

    pos_file_.open(file_path + "/pos.txt");

    vel_file_.open(file_path + "/vel.txt");

    btn_file_.open(file_path + "/btn.txt");

    master_file_.open(file_path + "/master.txt");

    if (DAMPING_INJ_)
    {
        comparison_file_.open(file_path + "/COMPARE_2/" + std::to_string(int(TANK_INITIAL_VALUE)) + "/compare_inj.txt");
    }
    else
    {
        comparison_file_.open(file_path + "/COMPARE_2/" + std::to_string(int(TANK_INITIAL_VALUE)) + "/compare.txt");
    }

    pressed_ = false;

    // Vector init
    Pin_vec_tb1.resize(DELAY_WINDOW, 0.0);
    Pin_vec_tb2.resize(DELAY_WINDOW, 0.0);
    Pin_vec_tb3.resize(DELAY_WINDOW, 0.0);

    // Time init
    first_cycle_ = true;

    // Mode selection
    if(SINGLE_TANK_){
        single_tank_energy = TANK_INITIAL_VALUE;
        single_tank_state = sqrt(2 * single_tank_energy);
    }

    // Init automatic open-close in simulation
    if (SIM_)
    {
        first_target_ = false;
        second_target_ = false;
    }

    ros::Duration(1.0).sleep();

    start_time_ = ros::Time::now().toSec();

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

}

void TurtleControl::gazeboObsCallback(const gazebo_msgs::ModelStates& msg){
    std::string cyl1_name = "unit_cylinder";
    std::string cyl2_name = "unit_cylinder_0";

    for (int i = 0; i < msg.name.size(); i++)
    {
        if (msg.name[i] == cyl1_name)
        {
            cyl1_pose_ = msg.pose[i];
        }

        if (msg.name[i] == cyl2_name)
        {
            cyl2_pose_ = msg.pose[i];
        }
        
    }
    
}

void TurtleControl::ellipseCallback(const geometry_msgs::PoseStamped& msg){
    geometry_msgs::Pose pose = msg.pose;

    cyl1_pose_ = pose;
}

void TurtleControl::objectCallback(const geometry_msgs::PoseStamped& msg){
    geometry_msgs::Pose pose = msg.pose;

    cyl2_pose_ = pose;
}

void TurtleControl::omegaCallback(const geometry_msgs::PoseStamped& msg){

    //! DEBUG
    FORCE_GAIN = 40.0; // Was 50.0

    master_pose_ = msg.pose;

    double delta_x = msg.pose.position.x;
    double delta_y = msg.pose.position.y;

    f_cont.force.x = - FORCE_GAIN * (delta_x / 0.05);
    f_cont.force.y = - (FORCE_GAIN / 2)* (delta_y / 0.1);
    // f_cont.force.x = 0.0;
    // f_cont.force.y = 0.0;

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
    twist_tb1 = odom.twist.twist; //!!!!!

    yaw_tb1 = quaternionToRPY(pose_tb1.orientation);

    pose_tb1_real = pose_tb1;
    twist_tb1_real = twist_tb1;

}

void TurtleControl::odomTb2Callback(const nav_msgs::Odometry& odom){

    pose_tb2 = odom.pose.pose;
    twist_tb2 = odom.twist.twist; //!!!!!

    yaw_tb2 = quaternionToRPY(pose_tb2.orientation);

    pose_tb2_real = pose_tb2;
    twist_tb2_real = twist_tb2;
}

void TurtleControl::odomTb3Callback(const nav_msgs::Odometry& odom){

    pose_tb3 = odom.pose.pose;
    twist_tb3 = odom.twist.twist; //!!!!!

    yaw_tb3 = quaternionToRPY(pose_tb3.orientation);

    pose_tb3_real = pose_tb3;
    twist_tb3_real = twist_tb3;

}

void TurtleControl::optiTb1Callback(const geometry_msgs::PoseStamped& msg){

    pose_tb1 = msg.pose;

    pose_tb1_real = pose_tb1;

    yaw_tb1 = quaternionToRPY(pose_tb1.orientation);

    if((first_cycle_) && (DELAYED_INPUTS_)){
        pose_tb1_del_.resize(DELAY_WINDOW, pose_tb1);
    }

    if(DELAYED_INPUTS_){
        pose_tb1_del_.push_back(pose_tb1);
        pose_tb1_del_.erase(pose_tb1_del_.begin());

        pose_tb1 = pose_tb1_del_[0];
    }

}

void TurtleControl::optiTb2Callback(const geometry_msgs::PoseStamped& msg){

    pose_tb2 = msg.pose;

    pose_tb2_real = pose_tb2;

    yaw_tb2 = quaternionToRPY(pose_tb2.orientation);

    if((first_cycle_) && (DELAYED_INPUTS_)){
        pose_tb2_del_.resize(DELAY_WINDOW, pose_tb2);
    }

    if(DELAYED_INPUTS_){
        pose_tb2_del_.push_back(pose_tb2);
        pose_tb2_del_.erase(pose_tb2_del_.begin());

        pose_tb2 = pose_tb2_del_[0];
    }

}

void TurtleControl::optiTb3Callback(const geometry_msgs::PoseStamped& msg){

    pose_tb3 = msg.pose;

    pose_tb3_real = pose_tb3;

    yaw_tb3 = quaternionToRPY(pose_tb3.orientation);

    if((first_cycle_) && (DELAYED_INPUTS_)){
        pose_tb3_del_.resize(DELAY_WINDOW, pose_tb3);
    }

    if(DELAYED_INPUTS_){
        pose_tb3_del_.push_back(pose_tb3);
        pose_tb3_del_.erase(pose_tb3_del_.begin());

        pose_tb3 = pose_tb3_del_[0];
    }

}


double TurtleControl::quaternionToRPY(geometry_msgs::Quaternion q){

    tf::Quaternion quat(q.x, q.y, q.z, q.w);
    tf::Matrix3x3 m(quat);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    return yaw;
}

void TurtleControl::setControllerParameters(shared_autonomy::SharedAutonomyConfig &config, uint32_t level){
    
    // SIM_ = config.SIM;
    // DAMPING_INJ_ = config.DAMPING_INJ_;
    // SINGLE_TANK_ = config.SINGLE_TANK_;
    // DELAYED_INPUTS_ = config.DELAYED_INPUTS_;
    // NO_TANK_ = config.NO_TANK_;
    
    // if(config.reset_tank)
    // {
    //     for (auto n : tank_energy_)
    //     {
    //         n = config.TANK_INITIAL_VALUE;
    //     }
        
    //     ROS_WARN("Tank has been reset!");
    //     config.reset_tank = false;
    // }

    // if (config.reset_impendace)
    // {
    //     K_P = config.K_P;
    //     K_D = config.K_D;
    //     K = config.K;
    //     sim_x_gain_ = config.SIM_X_GAIN;
    //     sim_y_gain_ = config.SIM_Y_GAIN;
    //     MASS_BOT = config.MASS_BOT;
    //     D = config.D;
    // }
    
}

void TurtleControl::simulationsCmd(){

    if (SIM_)
    {
        f_cont.force.x = sim_x_gain_;
        f_cont.force.y = -sim_y_gain_ * pose_tb1.position.y;

        if (pose_tb1.position.x > 2.0 && !first_target_)
        {
            K = 0.33;
            first_target_ = true;
        }

        if (pose_tb1.position.x > 4.0 && !second_target_)
        {
            K = 1.0;
            second_target_ = true;
        }
        
    }
    
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
    // ROS_INFO_STREAM("*****************************");
    // ROS_INFO_STREAM("Pose TB1 :" << pose_tb1);
    // ROS_INFO_STREAM("Pose TB2 :" << pose_tb2);
    // ROS_INFO_STREAM("Pose TB3 :" << pose_tb3);
    // ROS_INFO_STREAM("Pose TB1 REAL:" << pose_tb1_real);
    // ROS_INFO_STREAM("Pose TB2 REAL:" << pose_tb2_real);
    // ROS_INFO_STREAM("Pose TB3 REAL:" << pose_tb3_real);
    // ROS_INFO_STREAM("*****************************");
    // ROS_INFO_STREAM("K_P: " << K_P);
    // ROS_INFO_STREAM("K: " << K);
    // ROS_INFO_STREAM("D12 X : " << des_dist_12[0]);
    // ROS_INFO_STREAM("D12 Y : " << des_dist_12[1]);
    // ROS_INFO_STREAM("D13 X : " << des_dist_13[0]);
    // ROS_INFO_STREAM("D13 Y : " << des_dist_13[1]);
    // ROS_INFO_STREAM("*****************************");
    // ROS_INFO_STREAM("F_spring 12x: " << F_int_12[0]);
    // ROS_INFO_STREAM("F_spring 12y: " << F_int_12[1]);
    // ROS_INFO_STREAM("F_spring 13x: " << F_int_13[0]);
    // ROS_INFO_STREAM("F_spring 13y: " << F_int_13[1]);
    // ROS_INFO_STREAM("F_spring 21x: " << F_int_21[0]);
    // ROS_INFO_STREAM("F_spring 21y: " << F_int_21[1]);
    // ROS_INFO_STREAM("F_spring 23x: " << F_int_23[0]);
    // ROS_INFO_STREAM("F_spring 23y: " << F_int_23[1]);
    // ROS_INFO_STREAM("F_spring 31x: " << F_int_31[0]);
    // ROS_INFO_STREAM("F_spring 31y: " << F_int_31[1]);
    // ROS_INFO_STREAM("F_spring 32x: " << F_int_32[0]);
    // ROS_INFO_STREAM("F_spring 32y: " << F_int_32[1]);
    // ROS_INFO_STREAM("*****************************");

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

void TurtleControl::computeObsForce(){

    // ROS_INFO_STREAM("CYL 1 POSE: \n" <<  cyl1_pose_.position);
    // ROS_INFO_STREAM("CYL 2 POSE: \n" <<  cyl2_pose_.position);

    F_cyl1_1 = calculateForcesObs(pose_tb1_real.position.x, pose_tb1_real.position.y, cyl1_pose_.position.x, cyl1_pose_.position.y);
    F_cyl1_2 = calculateForcesObs(pose_tb2_real.position.x, pose_tb2_real.position.y, cyl1_pose_.position.x, cyl1_pose_.position.y);
    F_cyl1_3 = calculateForcesObs(pose_tb3_real.position.x, pose_tb3_real.position.y, cyl1_pose_.position.x, cyl1_pose_.position.y);
    F_cyl2_1 = calculateForcesObs(pose_tb1_real.position.x, pose_tb1_real.position.y, cyl2_pose_.position.x, cyl2_pose_.position.y);
    F_cyl2_2 = calculateForcesObs(pose_tb2_real.position.x, pose_tb2_real.position.y, cyl2_pose_.position.x, cyl2_pose_.position.y);
    F_cyl2_3 = calculateForcesObs(pose_tb3_real.position.x, pose_tb3_real.position.y, cyl2_pose_.position.x, cyl2_pose_.position.y);

}

Eigen::Vector2d TurtleControl::calculateForcesObs(double x_tb, double y_tb, double x_obs, double y_obs){
    Eigen::Vector2d F;
    double dist;

    dist = sqrt(pow(x_tb - x_obs,2) + pow(y_tb - y_obs,2));

    if (dist < D_MAX_)
    {
        F(0) =  - K_OBS_ * 1 * (dist - D_MAX_ ) * (x_tb - x_obs)/ dist;
        F(1) =  - K_OBS_ * 1 * (dist - D_MAX_ ) * (y_tb - y_obs)/ dist;
    }
    else
    {
        F.setZero();
    }

    return F;
    
}

void TurtleControl::computeVelocities(){

    //* NB: Fc is only F_int in this case, there is no F_cont //
    F_int_tb1[0] = f_int_1.force.x;
    F_int_tb1[1] = f_int_1.force.y;
    F_int_tb2[0] = f_int_2.force.x;
    F_int_tb2[1] = f_int_2.force.y;
    F_int_tb3[0] = f_int_3.force.x;
    F_int_tb3[1] = f_int_3.force.y;

    vel_tb1[0] = twist_tb1_real.linear.x * cos(yaw_tb1);
    vel_tb1[1] = twist_tb1_real.linear.x * sin(yaw_tb1);

    vel_tb2[0] = twist_tb2_real.linear.x * cos(yaw_tb2);
    vel_tb2[1] = twist_tb2_real.linear.x * sin(yaw_tb2);

    vel_tb3[0] = twist_tb3_real.linear.x * cos(yaw_tb3);
    vel_tb3[1] = twist_tb3_real.linear.x * sin(yaw_tb3);

    computePowers();

    // Store variables in a suitable way for the optimization problem
    Eigen::Vector4d F1 = { F_int_12[0], F_int_12[1], F_int_13[0], F_int_13[1]};
    Eigen::Vector4d F2 = { F_int_21[0], F_int_21[1], F_int_23[0], F_int_23[1]};
    Eigen::Vector4d F3 = { F_int_31[0], F_int_31[1], F_int_32[0], F_int_32[1]};

    // ROS_INFO_STREAM("F1: " << F1.transpose() << " F2: " << F2.transpose() << " F3: " << F3.transpose());

    v12 = vel_tb1 - vel_tb2;
    v13 = vel_tb1 - vel_tb3;
    v23 = vel_tb2 - vel_tb3;

    // ROS_INFO_STREAM("F1 BEFORE opt: " << F1);

    Eigen::Vector4d dotx_1 = { v12[0], v12[1], v13[0], v13[1]};    
    Eigen::Vector4d dotx_2 = {-v12[0],-v12[1], v23[0], v23[1]};
    Eigen::Vector4d dotx_3 = {-v13[0],-v13[1],-v23[0],-v23[1]};

    F1_pre_opt_ = F1;
    F2_pre_opt_ = F2;
    F3_pre_opt_ = F3;

    // Compute the optimal input for preserving passivity according to the currently selected mode
    if(!NO_TANK_){
        if(!SINGLE_TANK_){
            if(!DAMPING_INJ_)
            {
                Fc_tb1 = optimizationProblem(dotx_1, F1, tank_energy_[0], Pin_vec_tb1[0], Pout_tb1);
                Fc_tb2 = optimizationProblem(dotx_2, F2, tank_energy_[1], Pin_vec_tb2[0], Pout_tb2);
                Fc_tb3 = optimizationProblem(dotx_3, F3, tank_energy_[2], Pin_vec_tb3[0], Pout_tb3);
            }
            
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

    F1_post_opt_ = Fc_tb1;
    F2_post_opt_ = Fc_tb2;
    F3_post_opt_ = Fc_tb3;

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

void TurtleControl::computeTotalForce(){

    F_pass_12 = {Fc_tb1[0], Fc_tb1[1]};
    F_pass_13 = {Fc_tb1[2], Fc_tb1[3]};
    F_pass_21 = {Fc_tb2[0], Fc_tb2[1]};
    F_pass_23 = {Fc_tb2[2], Fc_tb2[3]};
    F_pass_31 = {Fc_tb3[0], Fc_tb3[1]};
    F_pass_32 = {Fc_tb3[2], Fc_tb3[3]};

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

    // Add the forces due to the obstacle repulsive potential
    // F_tot_tb1[0] += (F_cyl1_1[0] + F_cyl2_1[0]);
    // F_tot_tb1[1] += (F_cyl1_1[1] + F_cyl2_1[1]);

    // F_tot_tb2[0] += (F_cyl1_2[0] + F_cyl2_2[0]);
    // F_tot_tb2[1] += (F_cyl1_2[1] + F_cyl2_2[1]);

    // F_tot_tb3[0] += (F_cyl1_3[0] + F_cyl2_3[0]);
    // F_tot_tb3[1] += (F_cyl1_3[1] + F_cyl2_3[1]);

}

void TurtleControl::computePowers(){
    
    //! NEW DEFINITION
    double P_EXCH = 10;

    double Pin_tb1;
    double Pin_tb2;
    double Pin_tb3;

    if(!DAMPING_INJ_){
        if(tank_energy_[0] >= TANK_MIN_VALUE + 1)
            Pout_tb1 = P_EXCH;
        else
            Pout_tb1 = 0.0;
        
        if(tank_energy_[1] >= TANK_MIN_VALUE + 1)
            Pout_tb2 = P_EXCH;
        else
            Pout_tb2 = 0.0;
        
        if(tank_energy_[2] >= TANK_MIN_VALUE + 1)
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

    // Eigen::Vector4d Fc = {vars.Fc[0], vars.Fc[2], vars.Fc[1], vars.Fc[3]};
    // !!!!!!!!!!!!!!!!!!!!!
    Eigen::Vector4d Fc = {vars.Fc[0], vars.Fc[1], vars.Fc[2], vars.Fc[3]};

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
    
}

void TurtleControl::compareApproximations(){
    // Compute approximation values
    F1_compare_ = F1_pre_opt_ - F1_post_opt_;
    F2_compare_ = F2_pre_opt_ - F2_post_opt_;
    F3_compare_ = F3_pre_opt_ - F3_post_opt_;

    approx_norm_ = F1_compare_.norm() + F2_compare_.norm() + F3_compare_.norm();

    double err_12, err_13, err_23;

    err_12 = sqrt(pow((pose_tb1.position.x - pose_tb2.position.x) - K * des_dist_12[0], 2) 
             + pow((pose_tb1.position.y - pose_tb2.position.y) - K * des_dist_12[1], 2));

    err_13 = sqrt(pow((pose_tb1.position.x - pose_tb3.position.x) - K * des_dist_13[0], 2) 
             + pow((pose_tb1.position.y - pose_tb3.position.y) - K * des_dist_13[1], 2));

    err_23 = sqrt(pow((pose_tb2.position.x - pose_tb3.position.x) - K * des_dist_23[0], 2) 
             + pow((pose_tb2.position.y - pose_tb3.position.y) - K * des_dist_23[1], 2));

    formation_error_ = err_12 + err_13 + err_23;
}

// !!!!!!!!!!@@@@@@@@@@@@@@@@@@@@@@@@!!!!!!!!!!!!!!!!!!!!!
void TurtleControl::useDampingInjection(){
    F_int_tb1[0] = f_int_1.force.x;
    F_int_tb1[1] = f_int_1.force.y;
    F_int_tb2[0] = f_int_2.force.x;
    F_int_tb2[1] = f_int_2.force.y;
    F_int_tb3[0] = f_int_3.force.x;
    F_int_tb3[1] = f_int_3.force.y;

    vel_tb1[0] = twist_tb1_real.linear.x * cos(yaw_tb1);
    vel_tb1[1] = twist_tb1_real.linear.x * sin(yaw_tb1);

    vel_tb2[0] = twist_tb2_real.linear.x * cos(yaw_tb2);
    vel_tb2[1] = twist_tb2_real.linear.x * sin(yaw_tb2);

    vel_tb3[0] = twist_tb3_real.linear.x * cos(yaw_tb3);
    vel_tb3[1] = twist_tb3_real.linear.x * sin(yaw_tb3);

    // Store variables in a suitable way for the optimization problem
    Eigen::Vector4d F1 = { F_int_12[0], F_int_12[1], F_int_13[0], F_int_13[1]};
    Eigen::Vector4d F2 = { F_int_21[0], F_int_21[1], F_int_23[0], F_int_23[1]};
    Eigen::Vector4d F3 = { F_int_31[0], F_int_31[1], F_int_32[0], F_int_32[1]};

    v12 = vel_tb1 - vel_tb2;
    v13 = vel_tb1 - vel_tb3;
    v23 = vel_tb2 - vel_tb3;

    Eigen::Vector4d dotx_1 = { v12[0], v12[1], v13[0], v13[1]};    
    Eigen::Vector4d dotx_2 = {-v12[0],-v12[1], v23[0], v23[1]};
    Eigen::Vector4d dotx_3 = {-v13[0],-v13[1],-v23[0],-v23[1]};

    F1_pre_opt_ = F1;
    F2_pre_opt_ = F2;
    F3_pre_opt_ = F3;

    if(tank_energy_[0] <= TANK_MIN_VALUE){
        Fc_tb1 = -B_ * dotx_1;
        ROS_WARN("PORCODIO 1");
    }
    else
        Fc_tb1 = F1;

    if(tank_energy_[1] <= TANK_MIN_VALUE){
        Fc_tb2 = -B_ * dotx_2;
        ROS_WARN("PORCODIO 2");
    }
    else
        Fc_tb2 = F2;

    if(tank_energy_[2] <= TANK_MIN_VALUE){
        Fc_tb3 = -B_ * dotx_3;
        ROS_WARN("PORCODIO 3");
    }
    else
        Fc_tb3 = F3;

    computeTotalForce();

    F1_post_opt_ = Fc_tb1;
    F2_post_opt_ = Fc_tb2;
    F3_post_opt_ = Fc_tb3;

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
// !!!!!!!!!!@@@@@@@@@@@@@@@@@@@@@@@@!!!!!!!!!!!!!!!!!!!!!

void TurtleControl::computeIOSLF(){

    // Utilize IO-SFL for computing v and omega
    double b = 0.05;

    v_tb1 = vel_tb1[0] * cos(yaw_tb1) + vel_tb1[1] * sin(yaw_tb1);
    v_tb2 = vel_tb2[0] * cos(yaw_tb2) + vel_tb2[1] * sin(yaw_tb2);
    v_tb3 = vel_tb3[0] * cos(yaw_tb3) + vel_tb3[1] * sin(yaw_tb3);

    omega_tb1 = -sin(yaw_tb1) * vel_tb1[0] / b + cos(yaw_tb1) * vel_tb1[1] / b;
    omega_tb2 = -sin(yaw_tb2) * vel_tb2[0] / b + cos(yaw_tb2) * vel_tb2[1] / b;
    omega_tb3 = -sin(yaw_tb3) * vel_tb3[0] / b + cos(yaw_tb3) * vel_tb3[1] / b;

    // ROS_INFO_STREAM("v_tb1: " << v_tb1 << " omega_tb1: " << omega_tb1);

}

void::TurtleControl::saturateSpeed(){

    //Saturating the speed of each turtlebot
    double v_max = 0.25;
    double omega_max = 0.8;

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
    force_file_ << " " << F_cyl1_2[0] << " " << F_cyl1_2[1];
    force_file_ << " " << F_cyl2_3[0] << " " << F_cyl2_3[1];
    

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

    master_file_ << ros::Time::now().toSec() - start_time_;

    master_file_ << " " << master_pose_.position.x << " " << master_pose_.position.y << " " << master_pose_.position.z;

    master_file_ << " " << f_cont.force.x << " " << f_cont.force.y;

    double scaling = 1 / SCALING_FACTOR;

    master_file_ << " " <<  (F_pass_12[0] + F_pass_13[0]) * scaling << " " << (F_pass_12[1] + F_pass_13[1]) * scaling;

    master_file_ << std::endl;

    comparison_file_ << ros::Time::now().toSec() - start_time_;

    comparison_file_ << " " << approx_norm_ << " " << formation_error_; 

    comparison_file_ << std::endl;

}

void TurtleControl::spin(){

    simulationsCmd();
    computeForceInteraction();
    computeObsForce();

    if(!DAMPING_INJ_){
        computeVelocities();
    }
    else{
        useDampingInjection();
    }
    
    computeTankEnergy();
    compareApproximations();
    computeIOSLF();
    saturateSpeed();
    writeToFiles();

    geometry_msgs::Twist cmd_vel_1;
    geometry_msgs::Twist cmd_vel_2;
    geometry_msgs::Twist cmd_vel_3;

    cmd_vel_1.linear.x = v_tb1;
    cmd_vel_1.angular.z = omega_tb1;

    cmd_vel_2.linear.x = v_tb2;
    cmd_vel_2.angular.z = omega_tb2;

    cmd_vel_3.linear.x = v_tb3;
    cmd_vel_3.angular.z = omega_tb3;

    vel_tb1_pub_.publish(cmd_vel_1);
    vel_tb2_pub_.publish(cmd_vel_2);
    vel_tb3_pub_.publish(cmd_vel_3);
    // ROS_INFO_STREAM("Vel cmd:\n" << cmd_vel_1    );

    // Saving the current velocity commands as variables for the next cycle
    twist_tb1.linear.x = v_tb1;
    twist_tb2.linear.x = v_tb2;
    twist_tb3.linear.x = v_tb3;

    twist_tb1.angular.z = omega_tb1;
    twist_tb2.angular.z = omega_tb2;
    twist_tb3.angular.z = omega_tb3;

    geometry_msgs::Twist zero_speed;
    zero_speed.linear.x = 0.0;
    zero_speed.angular.z = 0.0;

    if((first_cycle_) && (DELAYED_INPUTS_)){
        twist_tb1_del_.resize(DELAY_WINDOW, zero_speed);
        twist_tb2_del_.resize(DELAY_WINDOW, zero_speed);
        twist_tb3_del_.resize(DELAY_WINDOW, zero_speed);
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

    // Compute the force feedback to the Omega master
    geometry_msgs::WrenchStamped force_feed;

    double scaling = 1 / SCALING_FACTOR;

    force_feed.wrench.force.x = (F_pass_12[0] + F_pass_13[0]) * scaling;
    force_feed.wrench.force.y = (F_pass_12[1] + F_pass_13[1]) * scaling;

    // !! Add repulsive potential contribution
    // force_feed.wrench.force.x += ( F_cyl1_1[0] + F_cyl2_1[0]) * scaling;
    // force_feed.wrench.force.y += ( F_cyl1_1[1] + F_cyl2_1[1]) * scaling;

    // Saturate force commands
    if(force_feed.wrench.force.x >= MAX_FORCE)
        force_feed.wrench.force.x = MAX_FORCE;
    else if (force_feed.wrench.force.x <= -MAX_FORCE)
        force_feed.wrench.force.x = -MAX_FORCE;

    if(force_feed.wrench.force.y >= MAX_FORCE)
        force_feed.wrench.force.y = MAX_FORCE;
    else if (force_feed.wrench.force.y <= -MAX_FORCE)
        force_feed.wrench.force.y = -MAX_FORCE;

    // ROS_INFO_STREAM("Force msg: \n" << force_feed);

    // Publish force commands
    // force_feed_pub_.publish(force_feed);

}