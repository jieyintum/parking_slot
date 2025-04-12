#ifndef _FUSION_MIDDLE_FILTER
#define _FUSION_MIDDLE_FILTER

#include <algorithm>
#include <deque>
#include <Eigen/Eigen>

namespace Fusion
{
    class MiddleFilter
    {
    public:
        MiddleFilter(const std::uint16_t size) : fixedSize_(size) {}
        ~MiddleFilter() = default;

        void Push(const double data)
        {
            buffer.push_back(data);
            while (buffer.size() > fixedSize_)
            {
                buffer.pop_front();
            }
        }

        bool IsFull() const
        {
            return (buffer.size() == fixedSize_);
        }

        double GetMiddle() const
        {
            auto bufferTmp = buffer;
            std::sort(bufferTmp.begin(), bufferTmp.end());

            const auto middleIdx = (bufferTmp.size() / 2U);
            return bufferTmp[middleIdx];
        }

        size_t Size() const
        {
            return buffer.size();
        }

        void Clear()
        {
            buffer.clear();
        }

    private:
        std::uint16_t fixedSize_;
        std::deque<double> buffer;
    };

    class MiddleFilter3D
    {
    public:
        MiddleFilter3D(std::uint16_t size) : x_(size), y_(size), z_(size) {}
        ~MiddleFilter3D() = default;
        void Push(const Eigen::Vector3d &vec)
        {
            x_.Push(vec.x());
            y_.Push(vec.y());
            z_.Push(vec.z());
        }

        bool IsFull() const
        {
            return x_.IsFull() && y_.IsFull() && z_.IsFull();
        }

        Eigen::Vector3d GetMiddle() const
        {
            const double xMid = x_.GetMiddle();
            const double yMid = y_.GetMiddle();
            const double zMid = z_.GetMiddle();

            return Eigen::Vector3d(xMid, yMid, zMid);
        }

    private:
        MiddleFilter x_;
        MiddleFilter y_;
        MiddleFilter z_;
    };
}

#endif