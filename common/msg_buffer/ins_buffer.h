
#ifndef FUSION_INS_BUFFER_H
#define FUSION_INS_BUFFER_H

#include "utils/time_fixed_size_deque.h"
#include "inner_msg/ins.h"

namespace Fusion {
class InsBuffer : public TimeFixedSizeDeque<Ins::Ptr> {
public:
    static InsBuffer& GetInstance() {
        static InsBuffer instance(250U);
        return instance;
    }

private:
    InsBuffer(const std::uint16_t fixedSize) : TimeFixedSizeDeque<Ins::Ptr>(fixedSize) {}
    InsBuffer(InsBuffer&) = delete;
    InsBuffer& operator= (const InsBuffer&) = delete;

private:
    Ins::Ptr Interpolation(const Ins::Ptr& ins_1, const Ins::Ptr& ins_2, const TimeStamp& timestamp) override
    {
        if (ins_1->GetTime() == timestamp) {
            return ins_1;
        }
        if (ins_2->GetTime() == timestamp) {
            return ins_2;
        }   
        Ins::Ptr res = std::make_shared<Ins>();
        double t1 = ins_1->GetTime().Sec();
        double t2 = ins_2->GetTime().Sec();
        double t = timestamp.Sec();
        double alpha = (t - t1) / (t2 - t1);
        res->timestamp = timestamp;
        res->transAccu = ins_1->transAccu;
        res->velAccu = ins_1->velAccu;
        res->attAccu = ins_1->attAccu;
        res->lla = ins_1->lla + alpha * (ins_2->lla - ins_1->lla);
        res->vel = ins_1->vel + alpha * (ins_2->vel - ins_1->vel);
        res->eular = ins_1->eular + alpha * (ins_2->eular - ins_1->eular);
        return res;
    }
};
}

#endif