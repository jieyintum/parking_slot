#include "pld_fusion/pld_fusion.h"

#include "utils/matrix_utils.h"

#include <iostream>

namespace Fusion
{
    namespace PSE
    {

        void PldFusion::slotMergeProcess(
            const std::list<FusionSlot::Ptr> pldSlots,
            const std::list<FusionSlot::Ptr> ussSlots,
            CSlotsList &mergedSlots,
            const Odometry poseOb)
        {
            if ((isParking_) && (mergedSlots.targetSlotLoc == FusionSlot::SlotLoc::UNKNOWN))
            {

                for (const auto &mergedSlot_ : mergedSlots.slotsList_)
                {
                    if (mergedSlot_->id == mergedSlots.targetSlotId)
                    {
                        mergedSlots.targetSlotLoc = mergedSlot_->locForUssSlot;
                        break;
                    }
                }
            }

            // std::cout<<"1"<<"\n";
            PldFusion::oldMatchedInfoProcess(pldSlots, ussSlots);
            // std::cout<<"2"<<"\n";
            PldFusion::findPartnerSlots(pldSlots, ussSlots);
            // std::cout<<"3"<<"\n";
            PldFusion::fusionProcess(pldSlots, ussSlots, mergedSlots);
            // std::cout<<"4"<<"\n";
            PldFusion::fusionInfoReset(pldSlots, ussSlots);
            // std::cout<<"4-0"<<"\n";

            for (auto &mergedSlot_ : mergedSlots.slotsList_)
            {
                mergedSlot_->isUpdatedThisCircle = false;

                PldFusion::CalculateSlotAngle(mergedSlot_);

                mergedSlot_->calcMiddlePoint(poseOb);

                if ((mergedSlot_->id == mergedSlots.targetSlotId) && (mergedSlots.targetSlotLoc != FusionSlot::SlotLoc::UNKNOWN))
                {
                    mergedSlot_->locForUssSlot = mergedSlots.targetSlotLoc;
                }
                else
                {
                    if (mergedSlot_->middlePoint.baseCorner[1] > 0)
                    {
                        // left
                        mergedSlot_->locForUssSlot = FusionSlot::SlotLoc::LEFT;
                    }
                    else
                    {
                        // right
                        mergedSlot_->locForUssSlot = FusionSlot::SlotLoc::RIGHT;
                    }
                }

                mergedSlot_->setIsParkable(debugParameter, poseOb);

                if (debugParameter.verticalAsRectangleSwitch == true)
                {

                    if (mergedSlot_->type == FusionSlot::Type::VERTICAL)
                    {
                        // std::cout<<"in vertical set"<<"\n";
                        mergedSlot_->initLines();
                        float length = 0.5 * (mergedSlot_->lines_[1]->getLength() + mergedSlot_->lines_[3]->getLength());

                        Eigen::Vector3d helpUV =
                            Fusion::turnUnitVectorFor90DegreeInAntiClockwiseInVehicleCoordinate(mergedSlot_->lines_[0]->getDefaultUnitVec());
                        mergedSlot_->corners[2].worldCorner = mergedSlot_->corners[1].worldCorner + helpUV * length;
                        mergedSlot_->corners[3].worldCorner = mergedSlot_->corners[0].worldCorner + helpUV * length;
                    }
                }
            }

            // std::cout<<"4-1"<<"\n";
        }

        void PldFusion::fusionProcess(
            const std::list<FusionSlot::Ptr> pldSlots,
            const std::list<FusionSlot::Ptr> ussSlots,
            CSlotsList &mergedSlots)
        {
            // std::cout<<"3 - 0"<<"\n";
            for (const auto &pldSlot_ : pldSlots)
            {
                FusionSlot::Ptr newMergedSlot(nullptr);
                bool l_isReplace_b(false);

                if (pldSlot_->alignInfo.lastResultSlot != nullptr)
                {
                    newMergedSlot = pldSlot_->alignInfo.lastResultSlot;
                    // std::cout<<"5"<<"\n";

                    mergedSlots.replace(newMergedSlot, pldSlot_);

                    if (pldSlot_->alignInfo.partnerSlot != nullptr)
                    {
                        pldSlot_->alignInfo.partnerSlot->alignInfo.resultSlot = newMergedSlot;
                    }
                    newMergedSlot->setAlignInfo(pldSlot_->alignInfo.partnerSlot, pldSlot_);
                    pldSlot_->alignInfo.resultSlot = newMergedSlot;
                }
                else
                {

                    // std::cout<<"6"<<"\n";

                    FusionSlot::Ptr &pldsPartnerSlot = pldSlot_->alignInfo.partnerSlot;
                    if (pldsPartnerSlot != nullptr)
                    {
                        // std::cout<<"11"<<"\n";
                        if ((pldsPartnerSlot->alignInfo.lastPartnerSlot == nullptr) && (pldsPartnerSlot->alignInfo.lastResultSlot != nullptr))
                        {
                            // std::cout<<"12"<<"\n";
                            newMergedSlot = pldsPartnerSlot->alignInfo.lastResultSlot;
                            mergedSlots.replace(newMergedSlot, pldSlot_);
                            l_isReplace_b = true;
                        }
                        else
                        {
                            newMergedSlot = std::make_shared<FusionSlot>(*pldSlot_);
                        }
                        pldsPartnerSlot->alignInfo.resultSlot = newMergedSlot;
                    }
                    else
                    {
                        // std::cout<<"13"<<"\n";
                        newMergedSlot = std::make_shared<FusionSlot>(*pldSlot_);
                    }
                    newMergedSlot->setAlignInfo(pldsPartnerSlot, pldSlot_);

                    pldSlot_->alignInfo.resultSlot = newMergedSlot;
                    if (l_isReplace_b == false)
                    {
                        mergedSlots.add(newMergedSlot);
                    }
                }
                if (newMergedSlot != nullptr)
                {
                    newMergedSlot->sensor = FusionSlot::ContributingSensor::PURE_VISION;
                }

                newMergedSlot->isUpdatedThisCircle = true;

                // std::cout<<"7"<<"\n";
            }
            // std::cout<<"3 - 1"<<"\n";
            for (const auto &mergedSlot_ : mergedSlots.slotsList_)
            {
                mergedSlot_->initLines();
            }
            // std::cout<<"3 -2"<<"\n";
            for (const auto &ussSlot_ : ussSlots)
            {
                if (ussSlot_->alignInfo.partnerSlot == nullptr)
                {
                    if (PldFusion::isConflictWithOtherSlot(mergedSlots.slotsList_, ussSlot_) == true)
                    {
                        // std::cout<<"conflict with others"<<"\n";
                        ussSlot_->alignInfo.resultSlot = nullptr;
                        continue;
                    }

                    FusionSlot::Ptr newMergedSlot(nullptr);
                    bool l_isReplace_b(false);
                    if (ussSlot_->alignInfo.lastResultSlot != nullptr)
                    {
                        if (ussSlot_->alignInfo.lastPartnerSlot != nullptr)
                        {
                            // lastResultSlot not usable, new slot
                            newMergedSlot = std::make_shared<FusionSlot>(*ussSlot_);
                        }
                        else
                        {
                            // lastResultSlot usable
                            newMergedSlot = ussSlot_->alignInfo.lastResultSlot;
                            mergedSlots.replace(newMergedSlot, ussSlot_);
                            l_isReplace_b = true;
                        }
                    }
                    else
                    {
                        // new uss slot
                        newMergedSlot = std::make_shared<FusionSlot>(*ussSlot_);
                    }

                    newMergedSlot->setAlignInfo(ussSlot_, nullptr);
                    ussSlot_->alignInfo.resultSlot = newMergedSlot;
                    if (l_isReplace_b == false)
                    {
                        mergedSlots.add(newMergedSlot);
                    }
                    if (newMergedSlot != nullptr)
                    {
                        newMergedSlot->sensor = FusionSlot::ContributingSensor::PURE_USS;
                    }
                    newMergedSlot->isUpdatedThisCircle = true;
                }
                else
                {
                    // do nothing, already in mergedSlot
                }
            }
            // std::cout<<"10"<<"\n";
            // find & delete not-exit merged slot
            for (auto iter = mergedSlots.slotsList_.begin(); iter != mergedSlots.slotsList_.end();)
            {
                if (!isMergeSlotExit(*iter, pldSlots, ussSlots))
                {
                    mergedSlots.deleteSlot(iter);
                }
                else
                {
                    ++iter;
                }
            }
        }

        bool PldFusion::isConflictWithOtherSlot(
            const std::list<FusionSlot::Ptr> mergedSlotsList,
            const FusionSlot::Ptr slot)
        {
            Eigen::Vector3d middlePoint = slot->middlePoint.worldCorner;

            for (const auto &mergedSlot_ : mergedSlotsList)
            {
                // if(Fusion::isPointInsideSlot(mergedSlot_->lines_, middlePoint) == true) {
                //     std::cout<<"is point inside slot: "<<true<<"\n";
                // }
                // else {
                //     std::cout<<"is point inside slot: "<<false<<"\n";
                // }

                if ((Fusion::isPointInsideSlot(mergedSlot_->lines_, middlePoint)) && (mergedSlot_->isUpdatedThisCircle == true))
                //&& (slot->alignInfo.lastResultSlot != mergedSlot_) )
                {
                    return true;
                }
            }
            return false;
        }

        bool PldFusion::isMergeSlotExit(
            const FusionSlot::Ptr mergedSlots,
            const std::list<FusionSlot::Ptr> pldSlots,
            const std::list<FusionSlot::Ptr> ussSlots)
        {
            bool l_isPldSlotExit_b(false);
            for (const auto &pldSlot_ : pldSlots)
            {
                if ((mergedSlots->alignInfo.pldSlot == pldSlot_) && (mergedSlots == pldSlot_->alignInfo.resultSlot))
                {
                    l_isPldSlotExit_b = true;
                    break;
                }
            }
            if (!l_isPldSlotExit_b)
            {
                mergedSlots->alignInfo.pldSlot = nullptr;
            }

            bool l_isUssSlotExit_b(false);
            for (const auto &ussSlot_ : ussSlots)
            {
                if ((mergedSlots->alignInfo.ussSlot == ussSlot_) && (mergedSlots == ussSlot_->alignInfo.resultSlot))
                {
                    l_isUssSlotExit_b = true;
                    break;
                }
            }
            if (!l_isUssSlotExit_b)
            {
                mergedSlots->alignInfo.ussSlot = nullptr;
            }

            return l_isPldSlotExit_b || l_isUssSlotExit_b;
        }

        void PldFusion::oldMatchedInfoProcess(
            const std::list<FusionSlot::Ptr> pldSlots,
            const std::list<FusionSlot::Ptr> ussSlots)
        {
            for (const auto &pldSlot_ : pldSlots)
            {
                if (pldSlot_->alignInfo.lastPartnerSlot == nullptr)
                {
                    continue;
                }
                oldMatchedInfoProcessForPld(pldSlot_, ussSlots);
            }

            for (const auto &ussSlot_ : ussSlots)
            {
                if (ussSlot_->alignInfo.lastPartnerSlot == nullptr)
                {
                    continue;
                }
                oldMatchedInfoProcessForPld(ussSlot_, pldSlots);
            }
        }

        void PldFusion::oldMatchedInfoProcessForPld(
            const FusionSlot::Ptr slotA,
            const std::list<FusionSlot::Ptr> slotsListB)
        {
            for (const auto &slotB_ : slotsListB)
            {
                if ((slotA->alignInfo.lastPartnerSlot == slotB_) && (slotB_->alignInfo.lastPartnerSlot == slotA))
                {
                    return;
                }
            }
            slotA->alignInfo.lastPartnerSlot == nullptr;
        }

        void PldFusion::findPartnerSlots(
            const std::list<FusionSlot::Ptr> pldSlots,
            const std::list<FusionSlot::Ptr> ussSlots)
        {
            for (const auto &pldSlot_ : pldSlots)
            {
                findPartnerSlotForPld(pldSlot_, ussSlots);
            }
        }

        void PldFusion::findPartnerSlotForPld(
            const FusionSlot::Ptr pldSlot,
            const std::list<FusionSlot::Ptr> ussSlots)
        {
            for (const auto &ussSlot_ : ussSlots)
            {
                for (int i = 0; i < 4; i++)
                {
                    const bool l_isTempDistanceValid = ((ussSlot_->corners[0].worldCorner - pldSlot->corners[i].worldCorner).norm() < 1.0) &&
                                                       ((ussSlot_->corners[1].worldCorner - pldSlot->corners[(i + 1) % 4].worldCorner).norm() < 1.0);
                    if ((l_isTempDistanceValid == true) && (ussSlot_->alignInfo.partnerSlot == nullptr))
                    {
                        pldSlot->alignInfo.partnerSlot = ussSlot_;
                        ussSlot_->alignInfo.partnerSlot = pldSlot;
                        return;
                    }
                }
            }
        }

        void PldFusion::fusionInfoReset(
            const std::list<FusionSlot::Ptr> pldSlots,
            const std::list<FusionSlot::Ptr> ussSlots)
        {
            for (const auto &pldSlot_ : pldSlots)
            {
                pldSlot_->resetAlignInfo();
            }

            for (const auto &ussSlot_ : ussSlots)
            {
                ussSlot_->resetAlignInfo();
            }
        }

    }
}