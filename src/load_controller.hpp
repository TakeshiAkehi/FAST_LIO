#pragma once
#include <stdint.h>
#include "movmean.hpp"

struct controlStatus{
    float ms_per_kpts_ave=-1;
    float predicted_process_time_raw=-1;
    float predicted_process_time_ctrl=-1;
    int estimated_optim_pts=-1;
    float down_size_coef=-1;
    bool down_size_limited=false;
};

class loadController{
    private:
        Movmean mm_proc_time_ratio;
    public:
        loadController(uint16_t nmean):mm_proc_time_ratio(nmean){}
        void update_ratio(int point_size, float dt_nsec);
        bool predict(int poist_size);

        controlStatus status;
        float target_slack_ms=0;
        float minimum_downsample_coef=0;
        float interval_time_ms=0;
};

void loadController::update_ratio(int point_size, float dt_nsec){
    float ms_per_kpts = (dt_nsec/1000000 )/(float(point_size)/1000);
    this->mm_proc_time_ratio.update(ms_per_kpts);
}

bool loadController::predict(int point_size){
    controlStatus *s = &this->status;
    s->down_size_coef = 100;
    if(!mm_proc_time_ratio.available()){
        return false;
    }

    s->ms_per_kpts_ave = this->mm_proc_time_ratio.get();
    s->predicted_process_time_raw = s->ms_per_kpts_ave * point_size /1000;
    float available_time = this->interval_time_ms - this->target_slack_ms;
    int max_th_pts = int(available_time / s->ms_per_kpts_ave *1000);
    int excess_pts = point_size - max_th_pts;
    if(excess_pts <= 0){
        s->predicted_process_time_ctrl = s->predicted_process_time_raw;
        return false;
    }

    float opt_siz = point_size - excess_pts;
    float dsc = opt_siz / float(point_size) * 100;
    s->down_size_limited = dsc < this->minimum_downsample_coef;
    dsc = (s->down_size_limited)? this->minimum_downsample_coef : dsc;
    s->estimated_optim_pts = point_size*dsc/100 ;
    s->down_size_coef = dsc;
    s->predicted_process_time_ctrl = s->ms_per_kpts_ave * s->estimated_optim_pts / 1000;
    return true;
}