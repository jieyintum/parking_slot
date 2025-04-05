
#ifndef FUSION_GLOC_BUFFER_H
#define FUSION_GLOC_BUFFER_H

#include "utils/time_fixed_size_deque.h"
#include "global_loc.h"

namespace Fusion {
class GlocBuffer : public TimeFixedSizeDeque<GlobalLoc::Ptr> {
public:
    static GlocBuffer& GetInstance() 
    {
        static GlocBuffer instance(250U);
        return instance;
    }

private:
    GlocBuffer(const std::uint16_t fixedSize) : TimeFixedSizeDeque<GlobalLoc::Ptr>(fixedSize) {}
    GlocBuffer(GlocBuffer&) = delete;
    GlocBuffer& operator= (const GlocBuffer&) = delete;

    GlobalLoc::Ptr Interpolation(const GlobalLoc::Ptr& ins_1, const GlobalLoc::Ptr& ins_2, const TimeStamp& timestamp) override
    {
        if (ins_1->GetTime() == timestamp) {
            return ins_1;
        }
        if (ins_2->GetTime() == timestamp) {
            return ins_2;
        }   
        GlobalLoc::Ptr res = std::make_shared<GlobalLoc>();
        double t1 = ins_1->GetTime().Sec();
        double t2 = ins_2->GetTime().Sec();
        double t = timestamp.Sec();
        double alpha = (t - t1) / (t2 - t1);
        res->timestamp = timestamp;
        res->llaDev = ins_1->llaDev;
        res->velDev = ins_1->velDev;
        res->attDev = ins_1->attDev;
        res->lla = ins_1->lla + alpha * (ins_2->lla - ins_1->lla);
        res->vel = ins_1->vel + alpha * (ins_2->vel - ins_1->vel);
        res->eular = ins_1->eular + alpha * (ins_2->eular - ins_1->eular);

        res->eular.z() = WarpAngle(ins_1->eular.z() + alpha * WarpAngle(ins_2->eular.z() - ins_1->eular.z()));
        double vel = ins_1->vel.head<2>().norm()+ alpha * (ins_2->vel.head<2>().norm() - ins_1->vel.head<2>().norm());
        res->vel << vel*cos(res->eular.z()), vel*sin(res->eular.z()),res->vel.z();
  
        return res;
    }
};

}

#endif