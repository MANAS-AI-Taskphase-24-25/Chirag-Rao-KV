#ifndef Bezier_HPP
#define Bezier_HPP
#include <iostream>
#include <vector>
#include <cmath>
#include <utility> 
#include <stack> 
#include <algorithm>



class Bezier {
private:
    int axis, no;
    double step;

    std::vector<float> Smooth(std::vector<float> p0, std::vector<float> p1, std::vector<float> p2, std::vector<float> p3, double t) {
        std::vector<float> output(2);

        for (int i = 0; i < 2; i++) {
            //previously tried catmull rom 
            output[i] = std::pow((1 - t), 3) * p0[i] 
          + 3 * std::pow((1 - t), 2) * t * p1[i] 
          + 3 * (1 - t) * std::pow(t, 2) * p2[i] 
          + std::pow(t, 3) * p3[i];


        }

        return output;
    }

public:
    Bezier(int a, int no_points) {
        axis = a;
        if(no_points<=1){
            no = 2;
        }else{
            no = no_points;
        }
        step = 1.0 / (no - 1);  
    }

    std::stack<std::vector<float>> Get_curve(std::stack<std::vector<float>> way_points) {
        std::stack<std::vector<float>> spline_pts;
        std::vector<std::vector<float>> points;
        std::vector<std::vector<float>> reverse;
        while(!way_points.empty()){
            points.push_back(way_points.top());
            way_points.pop();
        }
        std::reverse(points.begin(), points.end());
    
        for (size_t i = 0; i + 3 < points.size(); i++) {
            std::vector<float> p0 = points[i];
            std::vector<float> p1 = points[i + 1];
            std::vector<float> p2 = points[i + 2];
            std::vector<float> p3 = points[i + 3];

            for (int j = 0; j <= no; j++) {
                double t = j * step;  
                std::vector<float> interpolated = Smooth(p0, p1, p2, p3, t);
                spline_pts.push(interpolated);
            }
        }
        return spline_pts;
    }
};

#endif