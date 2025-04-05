#ifndef FUSION_LINE_BASE_H
#define FUSION_LINE_BASE_H

#include <Eigen/Eigen>

namespace Fusion {
struct LineBase {
public:
    // using Ptr = std::shared_ptr<LineBase>;
    LineBase() = default;
    LineBase(const double c0, const double c1, const double c2, const double c3,
             const float start, const float end, const float prob)
        : c0_{c0}, c1_{c1}, c2_{c2}, c3_{c3},
          start_{start}, end_{end}, prob_{prob} {}
    ~LineBase() = default;
    
    inline double GetY(const double x) const
    {
        return c0_ + (c1_ * x) + (c2_ * x * x) + (c3_ * x * x * x);
    }

    inline Eigen::Vector2d GetCoord(const double x) const
    {
        const double y = GetY(x);
        return Eigen::Vector2d{x, y};
    }

    inline Eigen::Vector4d GetCofficient() const
    {
        return Eigen::Vector4d{c0_, c1_, c2_, c3_};
    }

    inline bool IsProValid() const
    { 
        return prob_ > 0.9;
    }

    std::vector<Eigen::Vector2d> GetSamplePts(uint32_t sampleNum, float sampleEnd, float sampleStart);
    void Polyfit(const std::vector<Eigen::Vector2d>& p);

private:
    bool FitToCubicCurve(const std::vector<Eigen::Vector2d>& p,  Eigen::Vector4d& state);
    bool FitToPrimaryCurve(const std::vector<Eigen::Vector2d>& p,  Eigen::Vector4d& state);

public:
    double c0_ = 0.0;
    double c1_ = 0.0;
    double c2_ = 0.0;
    double c3_ = 0.0;
    double c4_ = 0.0;
    float start_ = 0.0;
    float end_ = 0.0;
    float prob_ = 0.0;
};

} // Fusion 

#endif