#pragma once
#include <stdint.h>
#include "smoothing.hpp"
#include <ros/ros.h>

struct loadStatus{
    float ms_per_kpts_ave=-1;
    bool predict_success=false;
    float predicted_process_time_raw=-1;
    float predicted_process_time_up=-1;
    float predicted_process_time_down=-1;

    bool downsize_required=false;
    float predicted_process_time_ctrl=-1;
    int estimated_optim_pts=-1;
    float down_size_coef=-1;
    bool down_size_limited=false;

    float in_time_interval_ratio;
    bool upscale_required=false;
    bool downscale_required=false;
};

class loadEstimator{
    private:
        Movmean mm_proc_time_ratio;
        Movmean mm_proc_time_raw;
        Movmean mm_interval;
    public:
        loadEstimator(uint16_t nmean=30)
        :mm_proc_time_ratio(nmean),
         mm_proc_time_raw(nmean),
         mm_interval(nmean){}
        void update_processing_rate(int point_size, float dt_nsec);
        void predict_processing_time(int point_size,float grid_size,float grid_step);
        void update_interval(float ms);

        loadStatus status;
        float target_slack_ms=0;
        float minimum_downsample_coef=0;
        float interval_time_ms=0;
        float grid_coef_a = 0;
        float grid_coef_b = 0;
        float lower_ms = 0;
        float upper_ms = 0;

        float interval_range = 0;
        float sanity_in_time_interval_ratio = 0;
};

void loadEstimator::update_processing_rate(int point_size, float dt_nsec){
    float ms_per_kpts = (dt_nsec/1000000)/(float(point_size)/1000);
    this->mm_proc_time_ratio.update(ms_per_kpts);
}

void loadEstimator::update_interval(float ms){
    bool ok = ((this->interval_time_ms - this->interval_range) < ms) && (ms < (this->interval_time_ms + this->interval_range));
    if(ok){
        this->mm_interval.update(0.0);
    }else{
        this->mm_interval.update(1.0);
    }
}


void loadEstimator::predict_processing_time(int point_size,float grid_size,float grid_step){
    loadStatus *s = &this->status;
    s->down_size_coef = 100;
    s->predict_success = false;
    s->upscale_required = false;
    s->downscale_required = false;
    s->downsize_required = false;

    if(!mm_proc_time_ratio.available()){
        return;
    }
    s->predict_success=true;

    s->ms_per_kpts_ave = this->mm_proc_time_ratio.get();
    float predicted_process_time_raw = s->ms_per_kpts_ave * point_size /1000;
    this->mm_proc_time_raw.update(predicted_process_time_raw);
    s->predicted_process_time_raw = (this->mm_proc_time_raw.available())? this->mm_proc_time_raw.get() : predicted_process_time_raw;
    float available_time = this->interval_time_ms - this->target_slack_ms;
    int max_th_pts = int(available_time / s->ms_per_kpts_ave *1000);
    int excess_pts = point_size - max_th_pts;

    s->predicted_process_time_up = s->predicted_process_time_raw * (this->grid_coef_a*(grid_size - grid_step/2)+this->grid_coef_b);
    s->upscale_required = (s->predicted_process_time_raw < this->lower_ms) && (s->predicted_process_time_up < this->upper_ms);
    s->predicted_process_time_down = s->predicted_process_time_raw / (this->grid_coef_a*(grid_size + grid_step/2)+this->grid_coef_b);
    float in_time_interval_ratio = this->mm_interval.get();
    s->downscale_required = (this->sanity_in_time_interval_ratio < in_time_interval_ratio)
                         || (this->upper_ms < s->predicted_process_time_raw) && (this->lower_ms < s->predicted_process_time_down);
    s->in_time_interval_ratio = in_time_interval_ratio;

    if(excess_pts <= 0){
        s->predicted_process_time_ctrl = s->predicted_process_time_raw;
        return;
    }
    s->downsize_required=true;

    float opt_siz = point_size - excess_pts;
    float dsc = opt_siz / float(point_size) * 100;
    s->down_size_limited = dsc < this->minimum_downsample_coef;
    dsc = (s->down_size_limited)? this->minimum_downsample_coef : dsc;
    s->estimated_optim_pts = point_size*dsc/100 ;
    s->down_size_coef = dsc;
    s->predicted_process_time_ctrl = s->ms_per_kpts_ave * s->estimated_optim_pts / 1000;
    return;
}

class gridController{
    private:
        std::function<void(float)> _grid_set_func;
        ros::Time last_update;
        float grid_size;
        bool _set_grid_size(float grid);
        bool _scale_grid_size(bool invert);
        bool enabled = false;

    public:
        gridController();
        void register_grid_func(std::function<void(float)> grid_set_func);
        bool reset_grid();
        float get_grid_size();
        bool upscale_grid_size();
        bool downscale_grid_size();

        float grid_max = 0;
        float grid_min = 0;
        float grid_step = 0;
        float cool_time = 0;
};

gridController::gridController(){
    this->last_update = ros::Time(0);
    this->enabled = false;
}

void gridController::register_grid_func(std::function<void(float)> grid_set_func){
    this->_grid_set_func = grid_set_func;
    this->enabled = true;
    this->reset_grid();
}

bool gridController::_set_grid_size(float grid){
    if(!this->enabled){
        // ROS_WARN("grid func not registered");
        return false;
    }
    this->_grid_set_func(grid);
    this->last_update = ros::Time::now();
    this->grid_size = grid;
    ROS_INFO("set grid size : %f",grid);
    return true;
}

bool gridController::_scale_grid_size(bool invert){
    float new_grid_size_f = (invert)? (this->grid_size + grid_step) : (this->grid_size - grid_step);
    float new_grid_size = roundf(new_grid_size_f*10.0)/10.0;
    if(grid_max < new_grid_size){
        // ROS_WARN("grid max %f < new grid %f (current %f)",grid_max,new_grid_size,grid_size);
        return false;
    }
    if(new_grid_size < grid_min){
        // ROS_WARN("new grid %f < grid min %f (current %f)",new_grid_size,grid_min,grid_size);
        return false;
    }
    float duration = (ros::Time::now() - this->last_update).toSec();
    if(duration < cool_time){
        // ROS_WARN("duration %f < cool time %f",duration,cool_time);
        return false;
    }
    return this->_set_grid_size(new_grid_size);
}

bool gridController::reset_grid(){
    ROS_WARN("reset grid to %f",this->grid_max);
    this->_set_grid_size(this->grid_max);
}

bool gridController::upscale_grid_size(){
    return this->_scale_grid_size(false);
}

bool gridController::downscale_grid_size(){
    return this->_scale_grid_size(true);
}

float gridController::get_grid_size(){
    return this->grid_size;
}