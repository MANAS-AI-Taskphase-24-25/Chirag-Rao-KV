#ifndef PID_HPP_
#define PID_HPP_
#include <iostream>
#include <vector>

class PID {
private:
   double K[3];
   std::vector<double> error;
   std::vector<double> pre_error;
   std::vector<double> inte;
   std::vector<double> dere; 
   int axis;
public:
    PID(int n,double k[3]) {
        for(int i = 0;i<3;i++){
            K[i] = k[i];
        }
        axis = n;
        error.resize(axis, 0.0);
        pre_error.resize(axis, 0.0);
        inte.resize(axis, 0.0);
        dere.resize(axis, 0.0);
    }

    std::vector<double> Command_output(std::vector<double> expected,std::vector<double> current) {
        std::vector<double> output(axis, 0.0);

        for(int i = 0;i<axis;i++){
            error[i] = expected[i] - current[i];
            printf("eorror %f",error[i]);
            inte[i] += error[i];
            dere[i] = error[i] - pre_error[i];
            output[i] = K[0]*error[i] + K[1]*inte[i] + K[2]*dere[i];
            pre_error[i] = error[i];
            
        }
        return output;
    }

    double Command_output(double expected,double current) {
        double output  = 0;
                error[0] = expected- current;
                printf("error %f",error[0]);
                inte[0] += error[0];
                dere[0] = error[0] - pre_error[0];
              output = K[0]*error[0] + K[1]*inte[0] + K[2]*dere[0];
                pre_error[0] = error[0];
            
            return output;
    }
    void clear_pid(){
        error.resize(axis, 0.0);
        pre_error.resize(axis, 0.0);
        inte.resize(axis, 0.0);
        dere.resize(axis, 0.0);
        printf("\npid cleared\n");

    }
    
};


#endif