#ifndef ASTAR_H
#define ASTAR_H

#include <iostream>
#include <queue>
#include <vector>
#include <stack>
#include <algorithm>
#include "lane_fusion/lane_frame.h"

using namespace std;

namespace Fusion
{
    namespace PSE
    {
        class Astar
        {
        public:
            Astar();

            ~Astar();

            /**
             * @brief reset each point's index according new grid
             * @param input -> allPointIn
             * @param input -> sEIn
             * @param input -> rowMin
             * @param input -> colMin
             * @param output -> allPointIn
             * @param output -> sEIn
             * @return void
             */
            void ReSetGrid(std::vector<Point::Ptr> &allPointIn,
                           std::array<Point::Ptr, 2U> &sEIn,
                           const int &rowMin,
                           const int &colMin);

            /**
             * @brief Set map_'s accessible grid to 1 and unaccessible grid to 0
             * @param input -> rows
             * @param input -> cols
             * @param input -> allPointsIn
             * @param output -> map_
             * @return void
             */
            void InitAstar(const int &rows,
                           const int &cols,
                           std::vector<Point::Ptr> &allPointsIn);

            /**
             * @brief Search point
             * @param input -> rows
             * @param input -> cols
             * @param input -> allPointsIn
             * @param output -> map_
             * @return void
             */
            void Search(Point::Ptr &startPt, Point::Ptr &endPt);

            /**
             * @brief Get result
             * @param output -> output_
             * @return std::vector<Point::Ptr>
             */
            std::vector<Point::Ptr> GetPath();

            void PrintMap();

        private:
            /**
             * @brief Judging next node avaliable or not
             * @param input -> row
             * @param input -> col
             * @param input -> father
             * @param input -> G
             * @param output -> openList_
             * @return void
             */
            void CheckPoint(const int &row, const int &col, Point::Ptr &father, int G);

            /**
             * @brief along 8 direction to find path
             * @param input -> current
             * @param output -> current
             * @return void
             */
            void NextStep(Point::Ptr &currentPoint);

            /**
             * @brief Judging node is in list or not
             * @param input -> Nodelist
             * @param input -> row
             * @param input -> col
             * @param output -> int
             * @return -1 -> not in list, otherwise -> node in list
             */
            int IsInList(vector<Point::Ptr> &Nodelist, int row, int col);

            /**
             * @brief Calculate cost
             * @param input -> sNode
             * @param input -> eNode
             * @param input -> G
             * @param output -> sNode
             * @return void
             */
            void CalcCost(Point::Ptr &sNode, Point::Ptr &eNode, const int &G);

            /**
             * @brief Sort node by F
             * @param input -> n1
             * @param input -> n2
             * @param output -> bool
             * @return bool
             */
            static bool Compare(Point::Ptr &n1, Point::Ptr &n2);

            /**
             * @brief Judging node avaliable or not
             * @param input -> row
             * @param input -> col
             * @param output -> bool
             * @return true -> avaliable, false -> not avaliable
             */
            bool UnWalk(const int &row, const int &col);

            void PrintPath(Point::Ptr &current);

        private:
            std::vector<std::vector<int>> map_;
            vector<Point::Ptr> openList_;
            vector<Point::Ptr> closeList_;
            vector<Point::Ptr> output_;
            Point::Ptr endPt_;
            static const int WeightW_ = 10;
            static const int WeightWH_ = 14;
            int rowMax_;
            int colMax_;
        };
    }
}
#endif