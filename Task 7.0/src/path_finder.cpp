#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
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


class Block
{
    public:
    int x,y,s;
    int parent[2];
    float huestric,actual_huesteric,total_huesteric;
   
    Block(){
        x = 0;
        y = 0;
        s = 0;
       huestric = 0;
       actual_huesteric = 0;
       parent[0] = 0; parent[1] = 0;
    }

    Block(int xval,int yval,int status){
        x = xval;
        y = yval;
        s = status;
        if(s == 1 || s== 0){
            huestric = std::numeric_limits<float>::infinity();
            actual_huesteric = std::numeric_limits<float>::infinity();
            total_huesteric = std::numeric_limits<float>::infinity();

        }
        else{
            huestric = 0;
            actual_huesteric = 0;
            total_huesteric = 0;
        }

    }
    bool CheckParent(int new_parent[2]){
        float h_new  = std::sqrt(std::pow(x - new_parent[0], 2) + std::pow(y - new_parent[1], 2));
        if(huestric>h_new){
            return true;
        }
        else return false;
        
    }
    
    void Update(int s_New,float H,float G,int new_parent[2]){
        if(CheckParent(new_parent)){
            huestric = H;
            actual_huesteric = G;
            total_huesteric = G+H;
            s = s_New;
            parent[0] = new_parent[0];
            parent[1] = new_parent[1];
        }

    }
};


class CompareBlocks {
    public:
        bool operator()(const Block* a, const Block* b) const {
            return a->total_huesteric > b->total_huesteric; 
        }
    };

class Astar {
private:
    int length, bredth;
    std::vector<std::vector<int>> encoded_data;
    std::vector<std::vector<Block>> block;
    std::priority_queue<Block*, std::vector<Block*>, CompareBlocks> open_blocks;
    std::pair<int,int> End;
    std::pair<int,int> Start;
    static float ManhattanDistance(Block &a,Block &b){  // not used in code, was used previously for 4 dir movement.
        return abs(a.x - b.x) + abs(a.y - b.y);}
    static float EuclideanDistance(Block &a,Block &b){
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }

    void UpdateNode(Block &Neighbour,Block &current,float cost){

        float h = EuclideanDistance(Neighbour,block[End.first][End.second]);
        float g = cost + current.actual_huesteric;
        int parent[2];
        parent[0] = current.x;
        parent[1] = current.y;
        if(g< Neighbour.actual_huesteric){    
            if(Neighbour.s == 4 || Neighbour.s == 1){
                Neighbour.Update(5,h,g,parent);
                open_blocks.push(&Neighbour);
            }
            if( Neighbour.s == 5)
                Neighbour.Update(5,h,g,parent);
                else return;
    
        }
            else return;
        }

    void Evaluate_neighbours(Block &current){
        int i,nx,ny;
        float cost[8] = {1,1,1,1,1.41,1.41,1.41,1.41};
        current.s = 4;
        std::vector<std::pair<int,int>> directions = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{-1,-1},{1,-1},{-1,1}};
        for(i=0;i<8;i++){
            nx = current.x + directions[i].first;
            ny = current.y +directions[i].second;
            if (nx >= 0 && nx < length && ny >= 0 && ny < bredth) { 
                UpdateNode(block[nx][ny], current,cost[i]);
            }
        }
    }

 
    
    std::stack<std::vector<int>> BackTrack(){
        std::stack<std::vector<int>> path;
        int parent_x = End.first;
        int parent_y = End.second;
        
        while(parent_x != Start.first || parent_y != Start.second){
            path.push({parent_y, parent_x});
            Block* node = &block[parent_x][parent_y];
    
            if(node->parent[0] == -1 && node->parent[1] == -1)
                break;
    
            parent_x = node->parent[0]; 
            parent_y = node->parent[1];
        }
        return path;
    }

public:
std::stack<std::vector<int>> path;

    Astar(const std::vector<std::vector<int>>& data, std::pair<int,int> sta, std::pair<int,int> en, int l, int b)
     {
        length = l;
        bredth = b;
        encoded_data.resize(l, std::vector<int>(b));
        block.resize(l, std::vector<Block>(b));
        Start = sta;
        End = en;
        int status;

        for (int i = 0; i < l; i++) {
            for (int j = 0; j < b; j++) {
                encoded_data[i][j] = data[i][j];
                if (i == Start.first && j == Start.second)
                    {status = 2;} // Start block
                else if (data[i][j] != 100)
                    {status = 1;}
                else if (data[i][j] == 100)
                    {status = 0;}
                
                block[i][j] = Block(i, j, status); 
            
        }
    }
        block[Start.first][Start.second].parent[0] = -1;
        block[Start.first][Start.second].parent[1] = -1;
        
         
    }
    //    # 0:not accessable  1:Untouched   2:Start 3:End   4:Closed  5:Open  6: path

    std::stack<std::vector<int>> Find_path() {
        std::stack<std::vector<int>> output;
        open_blocks.push(&block[Start.first][Start.second]); 
        Block* current = &block[Start.first][Start.second];
    
        while (current->x != End.first || current->y != End.second) { 
            current = open_blocks.top();
            open_blocks.pop(); 
            Evaluate_neighbours(*current);
            if (open_blocks.empty()) {
                //remember: upon failure the way points will be start, ensuring no movement
                std::cout<<"failed"<<std::endl;
                output.push({Start.first, Start.second});
                return output;
            }
        }
    
        output = BackTrack();
        return output;
    }


};

class MapSubscriber : public rclcpp::Node {
    public:
            MapSubscriber() : Node("map_subscriber") {
                subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                    "/map", 10, std::bind(&MapSubscriber::map_callback, this, std::placeholders::_1));
        
                path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
            }
        
    private:
            void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                int height = msg->info.height;
                int width = msg->info.width;
                RCLCPP_INFO(this->get_logger(), "Received map: %dx%d", width, height); //(60,30)
                
                auto grid = Convert(msg);  
             
        
                std::pair<int, int> start = {0, 0};  
                std::pair<int, int> goal = {height -1, width -1};  
                RCLCPP_INFO(this->get_logger(), "initializing Astar");
                RCLCPP_INFO(this->get_logger(), "goal set to at %d,%d",goal.first,goal.second);
                //Astar runs here
                Astar pathfinder(grid, start, goal,height,width);
                RCLCPP_INFO(this->get_logger(), "Running Astar");
                auto path = pathfinder.Find_path();
                RCLCPP_INFO(this->get_logger(),"path written");
        
                if (!path.empty()) {
                    nav_msgs::msg::Path ros_path = convertPathToROS(path, msg);
                    RCLCPP_INFO(this->get_logger(), "hooray ,Path found");
                    path_pub_->publish(ros_path);
                } 
                else {
                    RCLCPP_WARN(this->get_logger(), "No valid path found!");
                }
            }
            nav_msgs::msg::Path convertPathToROS(const std::stack<std::vector<int>>& path, 
                const nav_msgs::msg::OccupancyGrid::SharedPtr& msg) {
                nav_msgs::msg::Path ros_path;
                ros_path.header = msg->header;  
                std::stack<std::vector<int>> path_copy = path; 
                RCLCPP_INFO(this->get_logger(),"CONVERING");
                while (!path_copy.empty()) {
                    auto coord = path_copy.top();  
                    path_copy.pop();

                    geometry_msgs::msg::PoseStamped pose;
                    pose.header = ros_path.header;
                    pose.pose.position.x = coord[0] * msg->info.resolution + msg->info.origin.position.x;
                    pose.pose.position.y = coord[1] * msg->info.resolution + msg->info.origin.position.y;
                    pose.pose.position.z = 0.0; 
                    pose.pose.orientation.w = 1.0;  

                    ros_path.poses.push_back(pose);
                }

                return ros_path;
}
        
            std::vector<std::vector<int>> Convert(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                int width = msg->info.width;
                int height = msg->info.height;
                
                std::vector<std::vector<int>> map_data(height, std::vector<int>(width, 0)); 
                RCLCPP_INFO(this->get_logger(),"map data");

                for (int y = 0; y < height; y++) {
                    for (int x = 0; x < width; x++) {
                        int index = y*width + x;
                        
                        map_data[y][x] = msg->data[index];
                    }
                }
        
                return map_data;
            }
        
            rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_; 
        };
        
        int main(int argc, char **argv) {
            rclcpp::init(argc, argv);
            rclcpp::spin(std::make_shared<MapSubscriber>());
            rclcpp::shutdown();
            return 0;
        }