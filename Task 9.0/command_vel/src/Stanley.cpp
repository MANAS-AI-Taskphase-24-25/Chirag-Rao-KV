#ifndef Stanley_HPP_
#define Stanley_HPP_
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm> 
#include "PID.cpp"


class Stanley{
public:
double damp_const;
//double k[3]= {1.5,0,0.5};
//PID lateral  = PID(1,k); used for pid stanley,
double current_yaw,expected_yaw,crosstrack_error,heading_error;
std::vector<double>V1;
std::vector<double>V2;
std::vector<double>local_pre_way;


    Stanley(double k){
        V1.resize(2,0.0);
        V2.resize(2,0.0);
        damp_const = k;
        current_yaw = 0;
        expected_yaw = 0;
        crosstrack_error = 0;
        local_pre_way.resize(2,0.0);
    }

    double yaw_command(std::vector<double>Current_pos,std::vector<double>expected_pos,std::vector<double>pre_waypoint,
        double e_yaw,double c_yaw,double vel){
      
        if (pre_waypoint.empty()) {
                local_pre_way[0] = Current_pos[0];
                local_pre_way[1] = Current_pos[1];
        }
        else {
            local_pre_way[0] = pre_waypoint[0];
            local_pre_way[1] = pre_waypoint[1];
        }

        V1 = {expected_pos[0] -Current_pos[0],expected_pos[1] - Current_pos[1]};
        V2 = {expected_pos[0] -local_pre_way[0],expected_pos[1] - local_pre_way[1]};
        crosstrack_error = cross_product(V1,V2);
        heading_error = e_yaw-c_yaw;
        double output = out_put(crosstrack_error,heading_error,vel);
        double max = M_PI / 4;
        
        
        if(std::abs(output)> M_PI / 2){ // when goal is at 90o+ give out heading error capped at 45o.
            output = std::clamp(heading_error,-max,max);
            //when the heading error is very large, it causes the bot to stop, if the bot is stopped it is allowed to take large turns.
            if(vel < 0.001){
                if(heading_error>max){
                    output = heading_error;
                }
            }
        }
        double capped_output = std::clamp(output, -max, max);

        return capped_output;
        //return lateral.Command_output(pid_feed,c_yaw);    used for pid stanley

    }

    private:
    double cross_product(std::vector<double>v1,std::vector<double>v2)
    {
        return (v1[0]*v2[1] - v1[1]*v2[0]);
    }

    double out_put(double ct,double head_error,double vel){
        vel = std::max(vel,0.001);  // velocity zero devide handeling, needs tuning
        return head_error + atan2(damp_const*ct,vel);
    }
    
};



#endif