#ifndef ASTAR_HPP_
#define ASTAR_HPP_
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
#include <algorithm>  
#include "Collinear_smoothing.cpp"

class Block
{
    public:
    int x,y,s;
    int parent[2];
    float heuristic,actual_heuristic,total_heuristic;
   
    Block(){
        x = 0;
        y = 0;
        s = 0;
        heuristic = 0;
       actual_heuristic = 0;
       parent[0] = 0; parent[1] = 0;
    }

    Block(int col,int row,int status){
        x = col;
        y = row;
        s = status;
        if(s == 1 || s== 0){
            heuristic = std::numeric_limits<float>::infinity();
            actual_heuristic = std::numeric_limits<float>::infinity();
            total_heuristic = std::numeric_limits<float>::infinity();

        }
        else{
            heuristic = 0;
            actual_heuristic = 0;
            total_heuristic = 0;
        }

    }
    bool CheckParent(int new_parent[2]){
        float h_new  = std::sqrt(std::pow(x - new_parent[0], 2) + std::pow(y - new_parent[1], 2));
        if(heuristic>h_new){
            return true;
        }
        else return false;
        
    }
    
    void Update(int s_New,float H,float G,int new_parent[2]){
        if(CheckParent(new_parent)){
            heuristic = H;
            actual_heuristic = G;
            total_heuristic = G+H;
            s = s_New;
            parent[0] = new_parent[0];
            parent[1] = new_parent[1];
        }

    }
};


class CompareBlocks {
    public:
        bool operator()(const Block* a, const Block* b) const {
            return a->total_heuristic > b->total_heuristic; 
        }
    };

class Astar {
private:
    bool end_blocked;
    int length, bredth;
    double tol = 1e-2;
    Collinear_smoothing smooth = Collinear_smoothing(tol);
    std::vector<std::vector<int>> encoded_data;
    std::vector<std::vector<Block>> block;
    std::priority_queue<Block*, std::vector<Block*>, CompareBlocks> open_blocks;
    std::pair<int,int> End;
    std::pair<int,int> Start;
    std::vector<std::pair<int,int>>Obstacle_coordinates;
    int blocked_no = 0;
    static float ManhattanDistance(Block &a,Block &b){  // not used in code, was used previously for 4 dir movement.
        return abs(a.x - b.x) + abs(a.y - b.y);}
    static float EuclideanDistance(Block &a,Block &b){
        return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    }

    void UpdateNode(Block &Neighbour, Block &current, float cost) {
        float h = EuclideanDistance(Neighbour, block[End.first][End.second]);
        float g = cost + current.actual_heuristic;
        int parent[2] = { current.x, current.y };
    
        if (g < Neighbour.actual_heuristic) {
            if (Neighbour.s == 0)
                return;
    
            else if (Neighbour.s == 4 || Neighbour.s == 1) {
                Neighbour.Update(5, h, g, parent);
                open_blocks.push(&Neighbour);
            }
    
            else if (Neighbour.s == 5) {
                Neighbour.Update(5, h, g, parent);
            } else {
                return;
            }
        } else {
            return;
        }
    }
    

        void Evaluate_neighbours(Block &current){
            int i, nx, ny;
            float cost[8] = {1,1,1,1,1.41,1.41,1.41,1.41};
            current.s = 4;
            
            std::vector<std::pair<int,int>> directions = {
                {1,0},{-1,0},{0,1},{0,-1},{1,1},{-1,-1},{1,-1},{-1,1}};
        
            for(i = 0; i < 8; i++){
                nx = current.x + directions[i].first;
                ny = current.y + directions[i].second;
                if (nx < 0 || nx >= length || ny < 0 || ny >= bredth)
                    continue;
        
                if (abs(directions[i].first) + abs(directions[i].second) == 2) {
                    int mid1_x = current.x + directions[i].first;
                    int mid1_y = current.y;
                    int mid2_x = current.x;
                    int mid2_y = current.y + directions[i].second;
        
                    if (mid1_x < 0 || mid1_x >= length || mid1_y < 0 || mid1_y >= bredth ||
                        mid2_x < 0 || mid2_x >= length || mid2_y < 0 || mid2_y >= bredth ||
                         encoded_data[mid1_x][mid1_y] == 100 || encoded_data[mid2_x][mid2_y] == 100)
                        continue;
                }
        
                if(encoded_data[nx][ny] != 100) {
                    UpdateNode(block[nx][ny], current, cost[i]);
                }
            }
        }
        

 
    
    std::stack<std::vector<int>> BackTrack(){
        std::stack<std::vector<int>> path;
        int parent_x = End.first;
        int parent_y = End.second;
        while(parent_x != Start.first || parent_y != Start.second){
            path.push({parent_x, parent_y}); 
            Block* node = &block[parent_x][parent_y];
            if(node->parent[0] == -1 && node->parent[1] == -1){
                break;
               }            
            parent_x = node->parent[0];   
            parent_y = node->parent[1];
            if(encoded_data[parent_x][parent_y] == 100){
            }
        }
        return path;
    }

     void Clear_data(){

            while(!open_blocks.empty()) {
                open_blocks.pop();
            }
            encoded_data.clear();
             block.clear();
             Obstacle_coordinates.clear();
        }

public:
std::stack<std::vector<int>> path;    

    void Get_map(const std::vector<std::vector<int>> data, std::pair<int,int> sta, std::pair<int,int> en, int b, int l)
     {  
        while (!path.empty()) {
            path.pop();
        }
        length = l;
        bredth = b;
        blocked_no = 0;
        encoded_data.resize(data.size(), std::vector<int>(data[0].size()));
        block.resize(data.size(), std::vector<Block>(data[0].size()));
        Start.first = sta.second;
        Start.second = sta.first;
        End.second = en.first;
        End.first = en.second;
        
        if (Start.first < 0 || Start.second >= b || Start.second < 0 || Start.first>= l||
            End.first < 0 || End.second >= b || End.second< 0 || End.first>= l) {
            std::cerr << "Start or goal is out of bounds!" << std::endl;
            return;
        }
        
        for (int i = 0; i < l; i++) {
            for (int j = 0; j < b; j++) {
                int status = 0; 
                encoded_data[i][j] = data[i][j];
                if (i == Start.first && j == Start.second) {
                    status = 2;  // Start
                } else if (i == End.first && j == End.second) {
                    
                    status = 1;  // End
                    
                } else if (data[i][j] == 100) {
                    {status = 0;
                    Obstacle_coordinates.push_back({i,j});
                    blocked_no++; 
                }  
                }
                else if (data[i][j] == -1) {
                    status = 1;  // Unknown (treat as traversable)
                } else {
                    status = 1;  // Free
                }
                block[i][j] = Block(i, j, status); 
        }
    }   
        Cushioning();
        block[Start.first][Start.second].parent[0] = -1;
        block[Start.first][Start.second].parent[1] = -1;
        // boolean to check for end pos on obstale. Used to arrest path finding and return
        if(data[End.first][End.second] == 100){
            end_blocked = true;
        }
        else{
            end_blocked = false;
        }
    }


    void Cushioning(){
        int nx,ny;
        //by default uses 2 blocks for inflation.

        std::vector<std::pair<int,int>> directions = {
            {1,0},{-1,0},{0,1},{0,-1},{1,1},{-1,-1},{1,-1},{-1,1},
        {2,0},{-2,0},{0,2},{0,-2},{2,2},{-2,-2},{2,-2},{-2,2}};

        for(size_t   i = 0;i<Obstacle_coordinates.size();i++){
        for(int j = 0;j<16;j++){
            nx = Obstacle_coordinates[i].first + directions[j].first;
            ny = Obstacle_coordinates[i].second+ directions[j].second;
            if (nx >= 0 && nx < length && ny >= 0 && ny < bredth) {
                block[nx][ny].s = 0;
            }
        }

        }
    }
  
    //    # 0:not accessable  1:Untouched   2:Start 3:End   4:Closed  5:Open  6: path

    std::stack<std::vector<int>> Find_path() {
        std::stack<std::vector<int>> output;
        std::stack<std::vector<int>> output_smooth;


        if (end_blocked) {                          // this is where the end block is checked returns the start pos if blocked.
            output.push({Start.first, Start.second});
            return output;
        }
        
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
        output = BackTrack(); // backtracks from end using parents to get back to start.
        if(output.size()>3)
        {   // if there are considerable amount of points, then remove points based on colinearity otherwise pass the same waypoints;
            output_smooth = smooth.smooth_path(output);
           
        }
        else {
            output_smooth = output;
        }
        printf("\n size before and after smoothing %zu, %zu",output.size(),output_smooth.size());
        Clear_data();
        return output_smooth;
    }


};


#endif 