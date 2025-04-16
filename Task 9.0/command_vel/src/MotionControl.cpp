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
PID linear = PID(2,k_lin);  
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
    double twist_max = 0;
    double twist_min = 0;
    double yaw,target_twist,yaw_out;

    


    tf2_ros::Buffer tf_buffer_; 
    tf2_ros::TransformListener tf_listener_;
    std::pair<double,double> end_pos;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_; 

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    std::vector<double> current;
    std::vector<double> target;
    std::vector<double> pre_target; // used for stanley, update it after control commands are executed
    double vel = 0;  
    double end_yaw;   


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
    {   
        geometry_msgs::msg::Twist cmd_msg;

    if (msg->poses.empty() || msg->poses.size() == 1) {
        RCLCPP_WARN(this->get_logger(), "Received empty or single-point path - no valid path to goal");
        cmd_msg.linear.x = 0;
        cmd_msg.linear.y = 0;
        cmd_msg.angular.z = 0; 
        cmd_pub_->publish(cmd_msg);
        return;
    }
        else
        {
        RCLCPP_INFO(this->get_logger(), "Received new path with %zu poses", msg->poses.size());


        geometry_msgs::msg::PoseStamped target_pose = msg->poses[0];

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
        geometry_msgs::msg::Pose pose = target_pose.pose;

        tf2::Quaternion q_end(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        );
        
        
        //linear 
        current.resize(2);
        target.resize(2);
        current[0] = transformStamped.transform.translation.x;
        current[1]  = transformStamped.transform.translation.y;
        target[0]= target_pose.pose.position.x;
        target[1]= target_pose.pose.position.y;
        RCLCPP_INFO(this->get_logger(), "Received path with %zu poses", msg->poses.size());
        RCLCPP_INFO(this->get_logger(), "Current position: (%.2f, %.2f)", current[0], current[1]);
        RCLCPP_INFO(this->get_logger(), "Target position: (%.2f, %.2f)", target[0], target[1]);
        RCLCPP_INFO(this->get_logger(), "start pid linear");
        

        std::vector<double> linear_out = linear.Command_output(target,current); // PID controls here

        //yaw
        

        //double yaw_out = Yaw.Command_output(target_twist,yaw); //PID controls here commented out to test stanley
      

        RCLCPP_INFO(this->get_logger(), "Current: (%.2f, %.2f, %.2f), Target: (%.2f, %.2f, %.2f)", current[0], current[1], yaw, target[0], target[1], target_twist);
        
      
        // capping the speed
        double max_vel = 0.1;
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
        
        yaw = tf2::getYaw(q);
        end_yaw = tf2::getYaw(q_end);
        // the target and current pos is used to get target yae
        double delta_y = target[1] - current[1];
        double delta_x = target[0] - current[0];
        target_twist = atan2(delta_y, delta_x); 
        // When the path is opposite to its orientation, the -ve x and -ve y cancells out so add the -ve sign.
        /*if(delta_x<0 && delta_y <0){
            target_twist = - atan2(delta_y, delta_x); 
        }*/
        
        RCLCPP_INFO(this->get_logger(), "target yaw %.2f:",target_twist);
        yaw_out = stanley.yaw_command(current,target,pre_target,target_twist,yaw,vel);
        vel = std::hypot(linear_out[0], linear_out[1]);
        
        double dx = end_pos.first - current[0];
        double dy = end_pos.second - current[1];
        double distance = sqrt(dx * dx + dy * dy);
        if (distance <=0.075) {
            RCLCPP_INFO(this->get_logger(), "set all to 0, reached goal");
               linear_out[0] = 0;
               linear_out[1] = 0;
               linear.clear_pid();
           }
        double delta = std::abs(yaw - target_twist);// if very agressive angle turn first then start moving
        if(delta>M_PI/2){
            linear_out[0] = 0;
            linear_out[1] = 0;
        }
        
        cmd_msg.linear.x = linear_out[0];
        cmd_msg.linear.y = linear_out[1];
        cmd_msg.angular.z = yaw_out; //only yaw element as no controll for roll and pitch
        cmd_pub_->publish(cmd_msg);
        pre_target[0] = target[0];
        pre_target[1] = target[1];
        RCLCPP_INFO(this->get_logger(), "Z goal is: %f",end_yaw);
    
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
