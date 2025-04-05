
#include "line_base.h"

namespace Fusion {
void LineBase::Polyfit(const std::vector<Eigen::Vector2d>& p)
{
    Eigen::Vector4d state;
    state.setZero();
    size_t rows =  p.size();

    if (rows > 4U) {
        if (!FitToCubicCurve(p, state)) {
            FitToPrimaryCurve(p, state);
        }
    } else if (rows >= 2) {
        FitToPrimaryCurve(p, state);
    } else {
        return;
    }

    c0_ = state[0];
    c1_ = state[1];
    c2_ = state[2];
    c3_ = state[3];
}

bool LineBase::FitToCubicCurve(const std::vector<Eigen::Vector2d>& p,  Eigen::Vector4d& state)
{
    const size_t rows = p.size();
    Eigen::MatrixXd A;
    A.setZero();
    Eigen::VectorXd b;
    b.setZero();
    A.resize(rows, 4);
    b.resize(rows);
    for(size_t i = 0U; i < rows; ++i) {
        A.row(i) << 1, p[i].x(), (p[i].x() * p[i].x()), (p[i].x() * p[i].x() * p[i].x());
        b[i] = p[i].y();
    }

    Eigen::FullPivLU<Eigen::MatrixXd> lu(A);
    const size_t rankA = lu.rank();
    if (rankA >= 4) {
        state = (A.transpose() * A).ldlt().solve(A.transpose() * b);
        return true;
    }
    state.setZero();
    return false;
}

bool LineBase::FitToPrimaryCurve(const std::vector<Eigen::Vector2d>& p,  Eigen::Vector4d& state)
{
    //直接拟合成y=c0 + c1*x
    const size_t rows = p.size();
    Eigen::MatrixXd A;
    A.setZero();
    Eigen::VectorXd b;
    b.setZero();
    A.resize(rows, 2U);
    b.resize(rows);
    Eigen::Vector2d tempState;
    tempState.setZero();
    for(size_t i =0; i < rows; ++i) {
        A.row(i) << 1, p[i].x();
        b[i] = p[i].y();
    }
    Eigen::FullPivLU<Eigen::MatrixXd> lu(A);
    const size_t rankA = lu.rank();
    if(rankA >=2 ){
        tempState = (A.transpose()*A).ldlt().solve(A.transpose()*b);
        state << tempState(0), tempState(1), 0.0, 0.0;
        return true;
    }
    state.setZero();
    return false;
}

std::vector<Eigen::Vector2d> LineBase::GetSamplePts(uint32_t sampleNum, float sampleEnd, float sampleStart) 
{
    std::vector<Eigen::Vector2d> samplePts;
    samplePts.reserve(sampleNum);

    const float end = sampleEnd;
    const float start = sampleStart;
    const float gap = ((end - start) / static_cast<float>(sampleNum));
    for (uint32_t i = 0; i < sampleNum; ++i) {
        const double  x = static_cast<double>(start + static_cast<float>(i) * gap);
        samplePts.emplace_back(GetCoord(x));
    }
    return samplePts;
}
}