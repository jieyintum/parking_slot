/*
 * @Author: guxiaojie
 * @Date: 2022-05-02 20:45:12
 * @LastEditTime: 2022-05-30 15:12:47
 * @LastEditors: guxiaojie
 * @Description: base time deque
 * @FilePath: /loc_hdm/include/common/utils/time_deque.h
 */
#ifndef FUSION_TIME_FIXED_SIZE_DEQUE_H_
#define FUSION_TIME_FIXED_SIZE_DEQUE_H_

#include "fixed_size_deque.h"
#include "time_stamp.h"

namespace Fusion {
template <typename T>
class TimeFixedSizeDeque : public FixedSizeDeque<T> {
public:
    TimeFixedSizeDeque(const std::uint16_t fixedSize) : FixedSizeDeque<T>(fixedSize) {}
    ~TimeFixedSizeDeque() = default;

    bool GetDataBeforeTime(const TimeStamp& t, T& output)
    {
        std::unique_lock<std::mutex> lock(this->mtx);
        if (this->empty() || (t < this->front()->GetTime())) {
            return false;
        }

        for (auto iter = this->rbegin(); iter != this->rend(); ++iter) {
            if ((*iter)->GetTime() < t) {
                output = *iter;
                return true;
            }
        }
        return false;
    }

    T GetInterpolation(const TimeStamp& timestamp)
    {
        std::unique_lock<std::mutex> lock(this->mtx);
        T interPtr = nullptr;
        if (this->empty()) {
            //std::cout<<"expty"<<"\n";
            return interPtr;
        }
        if ((this->front()->GetTime() < timestamp) && (timestamp <= this->back()->GetTime())) {
            for (auto iter = this->rbegin(); iter != this->rend(); ++iter) {
                if ((*iter)->GetTime() <= timestamp) {
                    //std::cout<<"found equal timestamp"<<"\n";
                    interPtr = Interpolation(*iter, *(iter-1), timestamp);
                    break;
                }
            }
        }
        else {
            //std::cout<<"out of range"<<"\n";
        }

        // if(interPtr == nullptr) {
        //     std::cout<<std::fabs(timestamp.nanostamp - (*(this->rend()))->GetTime().nanostamp)/1000000000.0f<<"\n";
        //     if(std::fabs(timestamp.nanostamp - (*(this->rend()))->GetTime().nanostamp)/1000000000.0f < 0.01f)  {
        //         interPtr = *(this->rend());
        //     }
        //  }

        return interPtr;
    }

protected:
    virtual T Interpolation(const T& msg1, const T& msg2, const TimeStamp& timestamp) = 0;
};
}

#endif