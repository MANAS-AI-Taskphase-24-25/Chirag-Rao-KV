#ifndef Collinear_smoothing_HPP_
#define Collinear_smoothing_HPP_
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
//Collinearity-based path simplification

class Collinear_smoothing{
    private:
    double tolerance;
        bool check_coliear(float x0,float y0,float x1,float y1,float x2,float y2){
            // calculate slope of 2 points wrt to first point, if the slope is same, drop the middle point.
            double dx_wrt_2 = x2 - x0;
            double dy_wrt_2 = y2 - y0;
            double dx_wrt_1 = x1 - x0;
            double dy_wrt_1 = y1 - y0;
            // calculate slope
            if (dx_wrt_2 == 0 || dx_wrt_1 == 0) {
                return false; // If  0 return false
            }
            double slope_wrt_2 = dy_wrt_2/dx_wrt_2;
            double slope_wrt_1 = dy_wrt_1/dx_wrt_1;
            return abs(slope_wrt_1 - slope_wrt_2) <= tolerance;
            
        }

    public:
        
        Collinear_smoothing(double t){ // specify tolerence
            tolerance = t;
        }
        Collinear_smoothing(){ // default tolerence
            tolerance =  1e-6;
        }

    std::stack<std::vector<int>>  smooth_path(std::stack<std::vector<int>> path){
        std::vector<std::vector<int>> path_temp;
        std::stack<std::vector<int>> result_path;
        while(!path.empty()){
            path_temp.push_back(path.top());
            path.pop();
        }
        for( size_t i = 0; i< path_temp.size()-2;i++){
            float x0 = path_temp[i][0];
            float y0 = path_temp[i][1];
            float x1 = path_temp[i+1][0];
            float y1 = path_temp[i+1][1];
            float x2 = path_temp[i+2][0];
            float y2 = path_temp[i+2][1];
            if(check_coliear(x0,y0,x1,y1,x2,y2)){
                int index_remove = i+1;
                path_temp.erase(path_temp.begin()+ index_remove);
                i--;
            }
        }
        //std::reverse(path_temp.begin(),path_temp.end());
        for(size_t i = 0;i<path_temp.size();i++){
            result_path.push(path_temp[i]);
        }
        return result_path;
    }
    std::stack<std::vector<float>>  smooth_path(std::stack<std::vector<float>> path){
        std::vector<std::vector<float>> path_temp;
        std::stack<std::vector<float>> result_path;
        while(!path.empty()){
            path_temp.push_back(path.top());
            path.pop();
        }
        for( size_t i = 0; i< path_temp.size()-2;i++){
            float x0 = path_temp[i][0];
            float y0 = path_temp[i][1];
            float x1 = path_temp[i+1][0];
            float y1 = path_temp[i+1][1];
            float x2 = path_temp[i+2][0];
            float y2 = path_temp[i+2][1];
            if(check_coliear(x0,y0,x1,y1,x2,y2)){
                int index_remove = i+1;
                path_temp.erase(path_temp.begin()+ index_remove);
                i--;
            }
        }
        //std::reverse(path_temp.begin(),path_temp.end());
        for(size_t i = 0;i<path_temp.size();i++){
            result_path.push(path_temp[i]);
        }
        return result_path;
    }
};
#endif 
