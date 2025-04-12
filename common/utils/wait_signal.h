#ifndef FUSION_WAIT_SIGNAL_H_
#define FUSION_WAIT_SIGNAL_H_

#include <condition_variable>
#include <mutex>
namespace Fusion
{
    class WaitSignal
    {
    public:
        WaitSignal() = default;
        ~WaitSignal() = default;

        void Wait()
        {
            std::unique_lock<std::mutex> lk(cvMutex_);
            cv_.wait(lk);
        }

        void Notify()
        {
            cv_.notify_all();
        }

    private:
        std::condition_variable cv_;
        std::mutex cvMutex_;
    };
}

#endif