#ifndef FUSION_FIXED_SIZE_DEQUE_H_
#define FUSION_FIXED_SIZE_DEQUE_H_

#include <deque>
#include <mutex>

namespace Fusion {
template <typename T>
class FixedSizeDeque : public std::deque<T> {
public:
    FixedSizeDeque(const std::uint16_t fixedSize) : fixedSize_(fixedSize) {}
    ~FixedSizeDeque() = default;

    inline void Push(const T& input)
    {
        std::unique_lock<std::mutex> lock(mtx);
        this->push_back(input);
        if (this->size() > fixedSize_) {
            this->pop_front();
        }
    }

    inline bool PopFront(T& output)
    {
        std::unique_lock<std::mutex> lock(mtx);
        if (this->empty()) {
            return false;
        }
        std::swap(output, this->front());
        this->pop_front();
        return true;
    }

    inline bool PopBack(T& output)
    {
        std::unique_lock<std::mutex> lock(mtx);
        if (this->empty()) {
            return false;
        }
        std::swap(output, this->back());
        this->pop_back();
        return true;
    }

    inline T& Front()
    {
        std::unique_lock<std::mutex> lock(mtx);
        return this->front();
    }

    inline T& Back()
    {
        std::unique_lock<std::mutex> lock(mtx);
        return this->back();
    }

    inline bool GetNewest(T& output)
    {
        std::unique_lock<std::mutex> lock(mtx);
        if (this->empty()) {
            return false;
        }
        std::swap(output, this->back());
        this->clear();
        return true;
    }

    inline bool Empty()
    {
        std::unique_lock<std::mutex> lock(mtx);
        return this->empty();
    }

    inline size_t Size()
    {
        std::unique_lock<std::mutex> lock(mtx);
        return this->size();
    }

    inline bool Full()
    {
        std::unique_lock<std::mutex> lock(mtx);
        return this->size() == fixedSize_;
    }

    inline void Lock()
    {
        mtx.lock();
    }

    inline void Unlock()
    {
        mtx.unlock();
    }

    void operator = (FixedSizeDeque<T>& in)
    {
        std::unique_lock<std::mutex> lock(mtx);
        in.Lock();
        for (const auto& i : in) {
            this->emplace_back(i);
        }
        in.Unlock();
    }
public:
    std::mutex mtx;
protected:
    std::uint16_t fixedSize_ = 2U;
    
};

}


#endif