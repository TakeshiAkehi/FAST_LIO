#pragma once
#include <stdint.h>

class lioQuality{
    public:
        double q_min=0;
        double q_max=0;
        int calc(double sx, double sy, double sz);
};

int lioQuality::calc(double sx, double sy, double sz){
    double smax = std::max({sx,sy,sz});
    double s = smax - this->q_min;
    double s_th = this->q_max - this->q_min;
    double quality_f = s/s_th;
    int quality_i = (quality_f<0)? 100 : (quality_f>1)? 0 : int(100-quality_f*100);
    return quality_i;
}