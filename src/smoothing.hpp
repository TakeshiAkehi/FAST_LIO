#pragma once
#include <stdint.h>

class Movmean{
    private:
        uint16_t siz;
        std::vector<float> arr;
        float sum;
        int i;
        bool avail;

    public:
        Movmean(uint16_t siz);
        float update(float val);
        float get();
        bool available();
};

Movmean::Movmean(uint16_t siz){
    this->arr = std::vector<float>(siz,0.0);
    this->siz = siz;
    this->sum = 0.0;
    this->i = 0;
    this->avail = false;
}

float Movmean::update(float val){
    if(this->i ==0){
        this->sum = std::accumulate(this->arr.begin(),this->arr.end(),0.0); //reset
        this->arr[this->i] = val;
    }else{
        this->sum -= this->arr[this->i];
        this->arr[this->i] = val;
        this->sum += val;
    }
    float ret = this->sum / float(this->siz);
    this->i++;
    if(this->i==this->siz){
        this->i = 0;
        this->avail = true;
    }
    return ret;
}

float Movmean::get(){
    float ret = this->sum / float(this->siz);
    return ret;
}

bool Movmean::available(){
    return this->avail;
}
