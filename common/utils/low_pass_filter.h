/*
 * @Description:
 * @Author: ChangJian
 * @Date: 2022-05-06 11:13:10
 * @LastEditTime: 2022-05-07 10:13:11
 * @LastEditors: JiangHeng
 */
#include <map>
#include <array>

namespace Fusion
{
    enum CutOffFreq
    {
        HZ_00 = 0U,
        HZ_05 = 1U,
        HZ_10 = 2U,
        HZ_15 = 3U,
        HZ_20 = 4U,
        HZ_25 = 5U,
        HZ_30 = 6U,
        HZ_35 = 7U,
        HZ_40 = 8U,
        HZ_45 = 9U,
    };

    class LowPassFilter
    {
        using Array4U = std::array<double, 4U>;
        struct ButterWorth3rdParam
        {
        public:
            ButterWorth3rdParam(const Array4U &out, const Array4U &in)
            {
                outputParam = out;
                inputParam = in;
            }
            ButterWorth3rdParam()
            {
                outputParam = {0.0, 0.0, 0.0, 0.0};
                inputParam = {1.0, 0.0, 0.0, 0.0};
            }

            ~ButterWorth3rdParam() = default;

            Array4U outputParam;
            Array4U inputParam;
        };

    public:
        LowPassFilter(const CutOffFreq &freq)
        {
            SetButterWorthParam();
            GetTargetParam(freq);
        }
        ~LowPassFilter() {}

        double Filt(const double rawdata)
        {
            double output = 0.0;
            for (size_t i = 1U; i < param_.inputParam.size(); ++i)
            {
                output += param_.inputParam[i] * inputStore_[i - 1];
                output -= param_.outputParam[i] * outputStore_[i - 1];
            }

            output += param_.inputParam[0] * rawdata;
            SetInput(rawdata);
            SetOutput(output);

            return output;
        }

    private:
        void SetButterWorthParam()
        {
            mapParam_[HZ_00] = ButterWorth3rdParam();
            mapParam_[HZ_05] = ButterWorth3rdParam({1.0, -2.37409474370935, 1.92935566909122, -0.532075368312092},
                                                   {0.00289819463372137, 0.00869458390116412, 0.00869458390116412, 0.00289819463372137});
            mapParam_[HZ_10] = ButterWorth3rdParam({1.0, -1.76004188034317, 1.18289326203783, -0.278059917634546},
                                                   {0.0180989330075144, 0.0542967990225433, 0.0542967990225433, 0.0180989330075144});
            mapParam_[HZ_15] = ButterWorth3rdParam({1.0, -1.16191748367173, 0.695942755789651, -0.137761301259893},
                                                   {0.0495329963572532, 0.148598989071760, 0.148598989071760, 0.0495329963572532});
            mapParam_[HZ_20] = ButterWorth3rdParam({1.0, -0.577240524806303, 0.421787048689562, -0.0562972364918427},
                                                   {0.0985311609239271, 0.295593482771781, 0.295593482771781, 0.0985311609239271});
            mapParam_[HZ_25] = ButterWorth3rdParam({1.0, -4.99600361081320e-16, 0.333333333333333, -1.85037170770859e-17},
                                                   {0.166666666666667, 0.500000000000000, 0.500000000000000, 0.166666666666667});
            mapParam_[HZ_30] = ButterWorth3rdParam({1.0, 0.577240524806302, 0.421787048689562, 0.0562972364918426},
                                                   {0.256915601248463, 0.770746803745390, 0.770746803745390, 0.256915601248463});
            mapParam_[HZ_35] = ButterWorth3rdParam({1.0, 1.16191748367173, 0.695942755789651, 0.137761301259893},
                                                   {0.374452692590160, 1.12335807777048, 1.12335807777048, 0.374452692590160});
            mapParam_[HZ_40] = ButterWorth3rdParam({1.0, 1.76004188034317, 1.18289326203783, 0.278059917634546},
                                                   {0.527624382501943, 1.58287314750583, 1.58287314750583, 0.527624382501943});
            mapParam_[HZ_45] = ButterWorth3rdParam({1.0, 2.37409474370935, 1.92935566909121, 0.532075368312092},
                                                   {0.729440722639082, 2.18832216791725, 2.18832216791725, 0.729440722639082});
        }

        void GetTargetParam(const CutOffFreq &freq)
        {
            if (mapParam_.find(freq) == mapParam_.end())
            {
                param_ = mapParam_[HZ_00];
            }
            else
            {
                param_ = mapParam_[freq];
            }
        }

        void SetOutput(double output)
        {
            outputStore_[2] = outputStore_[1];
            outputStore_[1] = outputStore_[0];
            outputStore_[0] = output;
        }
        void SetInput(double input)
        {
            inputStore_[2] = inputStore_[1];
            inputStore_[1] = inputStore_[0];
            inputStore_[0] = input;
        }

    private:
        std::map<CutOffFreq, ButterWorth3rdParam> mapParam_;
        ButterWorth3rdParam param_;
        std::array<double, 3U> outputStore_{0.0, 0.0, 0.0};
        std::array<double, 3U> inputStore_{0.0, 0.0, 0.0};
    };
}