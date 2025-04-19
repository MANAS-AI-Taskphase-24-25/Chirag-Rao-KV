#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "PID.cpp"
#include "Stanley.cpp"
#include <tf2/utils.h>  

class Controller : public rclcpp::Node
{
public:
double k_lin[3] ={0.2, 0.3, 0.6}; 
//double k_twist[3] ={0.3,0.01,0.7};
PID linear_0 = PID(2,k_lin);
PID linear_1 = PID(2,k_lin);
PID linear_2 = PID(2,k_lin);  
//PID Yaw = PID(1,k_twist);
Stanley stanley = Stanley(1);


    Controller()
        : Node("Controller"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planned_path", 10,
            std::bind(&Controller::path_callback, this, std::placeholders::_1));
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/goal_pose", 10, std::bind(&Controller::goal_callback, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Waiting for path on topic: /planned_path...");
        

    }

private:
    double yaw,target_twist,yaw_out;
    double vel = 0;  
    std::vector<std::pair<double,double>> velocity_log;

    tf2_ros::Buffer tf_buffer_; 
    tf2_ros::TransformListener tf_listener_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_; 
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    std::pair<double,double> end_pos;
    std::vector<double> current;
    std::vector<double> target_0,target_1,target_2;
    std::vector<double> pre_target; // used for stanley, update it after control commands are executed
   


    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        geometry_msgs::msg::PoseStamped goal_pose = *msg;
    
        // Simply store the goal position in world coordinates without transforming
        end_pos.first = goal_pose.pose.position.x;
        end_pos.second = goal_pose.pose.position.y;
    
        RCLCPP_INFO(this->get_logger(), "Received goal position: (%.2f, %.2f)", 
                    end_pos.first, end_pos.second);
        pre_target.resize(2);
    
    }
    
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    //path is published at 15 Hz so the whole vel commands are inside path callback to keep the cmd_vel frequency at 15Hz.
    {  
        geometry_msgs::msg::Twist cmd_msg;
        bool plus3 = false;

        if (msg->poses.empty() || msg->poses.size() == 1) {
            double velo = 0;
            RCLCPP_WARN(this->get_logger(), "Received empty or single-point path - no valid path to goal");
            std::size_t i;
            // reached end of path print velocity avg
            for(i = 0;i<velocity_log.size();i++){
                velo+= sqrt(velocity_log[i].first*velocity_log[i].first + velocity_log[i].second*velocity_log[i].second);
                velocity_log.pop_back();
            }
            double avg_vel = velo/i;
            if(!std::isnan(avg_vel)){
                RCLCPP_INFO(this->get_logger(), "Average Velocity of the travel: %.2f m/s",avg_vel);
                avg_vel = 0;
            }
            cmd_msg.linear.x = 0;
            cmd_msg.linear.y = 0;
            cmd_msg.angular.z = 0; 
            cmd_pub_->publish(cmd_msg);     //publishes 0 vel commands for invalid end or small path.
            return;
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Received new path with %zu poses", msg->poses.size());
            geometry_msgs::msg::PoseStamped target_pose_0 = msg->poses[0];
            geometry_msgs::msg::PoseStamped target_pose_1;
            geometry_msgs::msg::PoseStamped target_pose_2;
            if(msg->poses.size()>3){
            target_pose_1= msg->poses[1];
            target_pose_2 = msg->poses[2];
            plus3 = true;
            }
            geometry_msgs::msg::TransformStamped transformStamped;
            try {
                transformStamped = tf_buffer_.lookupTransform(
                    "map",        // target frame
                    "base_link",  // source frame (robot)
                    tf2::TimePointZero);
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
                return;
            }
            tf2::Quaternion q(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w
            );
            //geometry_msgs::msg::Pose pose = target_pose.pose;

            //linear 
            current.resize(2);
            target_0.resize(2);
            target_1.resize(2);
            target_2.resize(2);

            current[0] = transformStamped.transform.translation.x;
            current[1]  = transformStamped.transform.translation.y;
            target_0[0]= target_pose_0.pose.position.x;
            target_0[1]= target_pose_0.pose.position.y;
            if(plus3){
                target_1[0]= target_pose_1.pose.position.x;
                target_1[1]= target_pose_1.pose.position.y;
                target_2[0]= target_pose_2.pose.position.x;
                target_2[1]= target_pose_2.pose.position.y;
            }
            else{
                target_1[0]= 0;
                target_1[1]= 0;
                target_2[0]= 0;
                target_2[1]= 0;

            }
            RCLCPP_INFO(this->get_logger(), "Received path with %zu poses", msg->poses.size());
            RCLCPP_INFO(this->get_logger(), "Current position: (%.2f, %.2f)", current[0], current[1]);
            RCLCPP_INFO(this->get_logger(), "start pid linear");


            //Lookahead control
            std::vector<double> out0 = linear_0.Command_output(target_0, current);
            std::vector<double> out1 = linear_1.Command_output(target_1, current);
            std::vector<double> out2 = linear_2.Command_output(target_2, current);
            std::vector<double> linear_out(2, 0.0); // assuming 2D
            
            for (int i = 0; i < 2; ++i) {
                if(plus3){
                linear_out[i] = 1* out0[i] + 0 * out1[i] + 0 * out2[i];
            }
            else{
                linear_out[i] = out0[i];
            }
            }     

        // capping the speed
            double max_vel = 0.15;
            if(linear_out[0]> max_vel){
                linear_out[0] = max_vel;
            }
            else if(linear_out[0]< -max_vel){
                linear_out[0] = -max_vel;
            }
            if(linear_out[1]> max_vel){
                linear_out[1] = max_vel;
            }
            else if(linear_out[1]< -max_vel){
                linear_out[1] = -max_vel;
            }
        
        
            // reached goal
            double dx = end_pos.first - current[0];
            double dy = end_pos.second - current[1];
            double distance = sqrt(dx * dx + dy * dy);
            yaw = tf2::getYaw(q);
            // the target and current pos is used to get target yae
            double delta_y = target_0[1] - current[1];
            double delta_x = target_0[0] - current[0];
            target_twist = atan2(delta_y, delta_x); 

            
            RCLCPP_INFO(this->get_logger(), "target yaw %.2f:",target_twist);
            vel = std::hypot(linear_out[0], linear_out[1]);
            yaw_out = stanley.yaw_command(current,target_0,pre_target,target_twist,yaw,vel);
            
            if (distance <=0.075){
               
                linear_out[0] = 0;
                linear_out[1] = 0;
                linear_0.clear_pid();
                linear_1.clear_pid();
                linear_2.clear_pid();
                
            }


            double delta = std::abs(yaw - target_twist);// if very agressive angle turn first then start moving
            if(delta>M_PI/2){
                linear_out[0] = 0;
                linear_out[1] = 0;
                
            }
            
            cmd_msg.linear.x = linear_out[0];
            cmd_msg.linear.y = linear_out[1];
            velocity_log.push_back({linear_out[0],linear_out[1]});
            cmd_msg.angular.z = yaw_out; //only yaw element as no controll for roll and pitch
            cmd_pub_->publish(cmd_msg);
            pre_target[0] = target_0[0];
            pre_target[1] = target_0[1];
        
            }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}
