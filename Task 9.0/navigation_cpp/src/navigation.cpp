#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iostream>
#include <string>
#include <limits>
#include <queue>
#include <vector>
#include <optional>
#include <functional>
#include <stack>
#include <utility> 
#include <cmath> 
#include "Astar.cpp"
#include <sensor_msgs/msg/laser_scan.hpp>
class MapSubscriber : public rclcpp::Node {
public:
    MapSubscriber()
    : Node("map_subscriber"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_) {
        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&MapSubscriber::map_callback, this, std::placeholders::_1));

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&MapSubscriber::goal_callback, this, std::placeholders::_1));
        
           /* lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan", rclcpp::QoS(10),
                std::bind(&MapSubscriber::lidar_callback, this, std::placeholders::_1));*/
            
            
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(67),
            [this]() {
                this->explore();
            }
        );        
    }

private:
    std::mutex data_mutex_;
    Astar pathfinder;
    bool map_received = false;
    bool goal_received = false; 
    //rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_; 
    //std::vector<std::pair<float,float>> map_points;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::pair<int, int> bot_pos;
    std::pair<int, int> end_pos;


        void get_bot_pos(const geometry_msgs::msg::TransformStamped &transform_stamped) {
            double robot_x = transform_stamped.transform.translation.x;
            double robot_y = transform_stamped.transform.translation.y;
            RCLCPP_INFO(this->get_logger(), "bot real world position: (%.4f,%.4f )", robot_x, robot_y);
            float x = (robot_x - map_->info.origin.position.x - 0.025) / map_->info.resolution;
            float y = (robot_y - map_->info.origin.position.y - 0.025) / map_->info.resolution;
            bot_pos.first = static_cast<int>(std::round(x));
            bot_pos.second = static_cast<int>(std::round(y));
            RCLCPP_INFO(this->get_logger(), "float (%.2f,%.2f) Updated bot position: (%d, %d) and resolution is %.2f",x,y, bot_pos.first, bot_pos.second,map_->info.resolution);
        }
    
    

        void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            geometry_msgs::msg::PoseStamped stamped_msg = *msg;
            // Set timestamp to zero (which tells tf2 to use the latest available transform)
            stamped_msg.header.stamp = rclcpp::Time(0, 0);
            geometry_msgs::msg::PoseStamped goal_in_map;
            try {
                tf_buffer_.transform(stamped_msg, goal_in_map, "map");
                end_pos.first = static_cast<int>(std::round((goal_in_map.pose.position.x - map_->info.origin.position.x - 0.025) / map_->info.resolution));
                end_pos.second = static_cast<int>(std::round((goal_in_map.pose.position.y - map_->info.origin.position.y - 0.025) / map_->info.resolution));
                RCLCPP_INFO(this->get_logger(), "Transformed goal position: (%.2f, %.2f) in resolution %.2f", goal_in_map.pose.position.x, goal_in_map.pose.position.y,map_->info.resolution);
                //RCLCPP_INFO(this->get_logger(), "Goal grid coordinates: (%d, %d)", end_pos.first, end_pos.second);
                goal_received = true;
            } catch (tf2::TransformException &ex) {
                RCLCPP_ERROR(this->get_logger(), "Failed to transform goal from %s to map: %s", 
                            msg->header.frame_id.c_str(), ex.what());
            }
        }
        

   
        
    
    // this will update the map and path and keep doing until the goal is reached.
    void explore() {
        std::stack<std::vector<int>>path;
        if (!map_received || !goal_received) 
            return;
        std::vector<std::vector<int>> grid = Convert(map_);
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_.lookupTransform(
                "map", "base_link", tf2::TimePointZero);  
    
            get_bot_pos(transform_stamped);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform base_link to map: %s", ex.what());
            return;
        }
        int width = map_->info.width;
        int height = map_->info.height;
        RCLCPP_INFO(this->get_logger(), "Going from (%d, %d) to (%d, %d)", 
                    bot_pos.first, bot_pos.second, end_pos.first, end_pos.second);
        RCLCPP_INFO(this->get_logger(), "Astar initialised and finding path (%d,%d)",width,height);

        pathfinder.Get_map(grid,bot_pos, end_pos, width, height);
        RCLCPP_INFO(this->get_logger(), "Astar initialised and finding path; seg error in path var");
        path = pathfinder.Find_path();
        RCLCPP_INFO(this->get_logger(), "got path");
        //map points are added to force make borders based on lidar data put lidar call back in timer to call frequently
        
        if (!path.empty()) {
            nav_msgs::msg::Path ros_path = convertPathToROS(path, map_);
            RCLCPP_INFO(this->get_logger(), "Publishing path");
            path_pub_->publish(ros_path);
        } else {
            RCLCPP_WARN(this->get_logger(), "No valid path found.");    
        }
    }
    
   

        void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            map_ = msg;
            map_received = true;
            RCLCPP_INFO(this->get_logger(), "got the map");
        }

        nav_msgs::msg::Path convertPathToROS(const std::stack<std::vector<int>>& path, const nav_msgs::msg::OccupancyGrid::SharedPtr& msg) {
            nav_msgs::msg::Path ros_path;
            ros_path.header = msg->header;
            std::stack<std::vector<int>> path_copy = path;
            RCLCPP_INFO(this->get_logger(), "Converting path to ros");
            while (!path_copy.empty()) {
                auto coord = path_copy.top();
                path_copy.pop();
                geometry_msgs::msg::PoseStamped pose;
                pose.header = ros_path.header;
                pose.pose.position.x = coord[1] * msg->info.resolution + msg->info.origin.position.x + 0.025;  
                pose.pose.position.y = coord[0] * msg->info.resolution + msg->info.origin.position.y + 0.025;
                pose.pose.position.z = 0.0;
                pose.pose.orientation.w = 1.0;
                ros_path.poses.push_back(pose);
            }
            RCLCPP_INFO(this->get_logger(), "grid to path resolution used: %.2f",msg->info.resolution);
        
            return ros_path;    
        }
        
        

        std::vector<std::vector<int>> Convert(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            //converts 1d occupancy grid to 2d to feed to astar
            int width = msg->info.width;
            int height = msg->info.height;

            std::vector<std::vector<int>> map_data(height, std::vector<int>(width, 0));

            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    int index = y * width + x;
                    map_data[y][x] = msg->data[index];
                }
            }

            return map_data;
        }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapSubscriber>());
    rclcpp::shutdown();
    return 0;
}
