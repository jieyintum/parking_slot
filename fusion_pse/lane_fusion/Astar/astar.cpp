#include "astar.h"

namespace Fusion
{
    namespace PSE
    {
        /*int map[101][101] =
                {
                        {0, 0, 0, 1, 0, 1, 0, 0, 0},
                        {0, 0, 0, 1, 0, 1, 0, 0, 0},
                        {0, 0, 0, 0, 0, 1, 0, 0, 0},
                        {0, 0, 0, 1, 0, 1, 0, 1, 0},
                        {0, 0, 0, 1, 0, 1, 0, 1, 0},
                        {0, 0, 0, 1, 0, 0, 0, 1, 0},
                        {0, 0, 0, 1, 0, 0, 0, 1, 0}
                };*/

        Astar::Astar()
        {
        }

        Astar::~Astar()
        {
        }

        void Astar::Search(Point::Ptr &startPos, Point::Ptr &endPos)
        {
            if (startPos->index.row < 0 || startPos->index.row > rowMax_ ||
                startPos->index.col < 0 || startPos->index.col > colMax_ ||
                endPos->index.row < 0 || endPos->index.row > rowMax_ ||
                endPos->index.col < 0 || endPos->index.col > colMax_)
            {
                return;
            }

            Point::Ptr current = std::make_shared<Point>();
            this->endPt_ = endPos;
            openList_.emplace_back(startPos);
            while (!openList_.empty())
            {
                current = openList_.front();
                if (current->index.row == endPos->index.row &&
                    current->index.col == endPos->index.col)
                {
                    output_.clear();
                    //            cout << "Find path, Grid is: " << endl;
                    //            PrintMap();
                    PrintPath(current);
                    //            std::cout << std::endl;
                    openList_.clear();
                    closeList_.clear();
                    break;
                }
                NextStep(current);
                closeList_.emplace_back(current);
                openList_.erase(openList_.begin());
                sort(openList_.begin(), openList_.end(), Compare);
            }
        }

        void Astar::CheckPoint(const int &row, const int &col, Point::Ptr &father, int G)
        {
            if (row < 0 || row > rowMax_ || col < 0 || col > colMax_)
                return;
            if (this->UnWalk(row, col))
                return;
            // if in clostList return
            if (IsInList(closeList_, row, col) != -1)
                return;
            int index;
            // if in openList, xxx
            if ((index = IsInList(openList_, row, col)) != -1)
            {
                Point::Ptr point = std::make_shared<Point>();
                point = openList_[index];
                if (point->G > father->G + G)
                {
                    point->parent = father;
                    point->G = father->G + G;
                    point->F = point->G + point->H;
                }
            }
            else
            {
                // if not in openLit
                Point::Ptr point = std::make_shared<Point>();
                point->index.row = row;
                point->index.col = col;
                point->parent = father;
                CalcCost(point, endPt_, G);
                openList_.emplace_back(point);
            }
        }

        void Astar::NextStep(Point::Ptr &current)
        {
            CheckPoint(current->index.row - 1, current->index.col, current, WeightW_);
            CheckPoint(current->index.row + 1, current->index.col, current, WeightW_);
            CheckPoint(current->index.row, current->index.col - 1, current, WeightW_);
            CheckPoint(current->index.row, current->index.col + 1, current, WeightW_);
            CheckPoint(current->index.row - 1, current->index.col + 1, current, WeightWH_);
            CheckPoint(current->index.row - 1, current->index.col - 1, current, WeightWH_);
            CheckPoint(current->index.row + 1, current->index.col - 1, current, WeightWH_);
            CheckPoint(current->index.row + 1, current->index.col + 1, current, WeightWH_);
        }

        int Astar::IsInList(vector<Point::Ptr> &Nodelist, int row, int col)
        {
            for (int i = 0; i < Nodelist.size(); i++)
            {
                if (Nodelist.at(i)->index.row == row &&
                    Nodelist.at(i)->index.col == col)
                {
                    return i;
                }
            }

            return -1;
        }

        void Astar::ReSetGrid(std::vector<Point::Ptr> &allPointIn,
                              std::array<Point::Ptr, 2U> &sEIn,
                              const int &rowMin,
                              const int &colMin)
        {
            for (auto &point : allPointIn)
            {
                point->index.row = point->index.row - rowMin;
                point->index.col = point->index.col - colMin;
            }

            for (auto &sE : sEIn)
            {
                sE->index.row = sE->index.row - rowMin;
                sE->index.col = sE->index.col - colMin;
            }

            //    std::cout << "Start index: " << sEIn[0]->index.row << " " << sEIn[0]->index.col << std::endl;
            //    std::cout << "End index: " << sEIn[1]->index.row << " " << sEIn[1]->index.col << std::endl;
        }

        void Astar::InitAstar(const int &rows,
                              const int &cols,
                              std::vector<Point::Ptr> &allPointsIn)
        {
            //    std::cout << "Boundary: " << rows << " " << cols << std::endl;
            /// set all grid to 1
            map_.clear();
            map_.resize(rows + 1);
            for (int i = 0; i < rows + 1; ++i)
            {
                map_[i].resize(cols + 1);
                for (int j = 0; j < cols + 1; ++j)
                {
                    map_[i][j] = 1;
                }
            }

            /// set can pass grid to 0
            for (const auto &point : allPointsIn)
            {
                map_[point->index.row][point->index.col] = 0;
            }

            /// sets boundary
            rowMax_ = rows;
            colMax_ = cols;
        }

        void Astar::CalcCost(Point::Ptr &sNode, Point::Ptr &eNode, const int &G)
        {
            int H = abs(sNode->index.row - eNode->index.col) * WeightW_ +
                    abs(sNode->index.col - eNode->index.col) * WeightW_;
            int currentg = sNode->parent->G + G;
            int F = currentg + H;
            sNode->F = F;
            sNode->H = H;
            sNode->G = currentg;
        }

        bool Astar::Compare(Point::Ptr &n1, Point::Ptr &n2)
        {
            // printf("%d,%d",n1->f,n2->f);
            return n1->F < n2->F;
        }

        bool Astar::UnWalk(const int &row, const int &col)
        {
            if (map_[row][col] == 1)
            {
                return true;
            }
            return false;
        }

        void Astar::PrintPath(Point::Ptr &current)
        {
            if (current->parent != nullptr)
            {
                PrintPath(current->parent);
            }
            output_.emplace_back(current);
            map_[current->index.row][current->index.col] = 6;
            //    printf("(%d,%d)", current->index.row, current->index.col);
        }

        void Astar::PrintMap()
        {
            for (int i = 0; i <= rowMax_; i++)
            {
                for (int j = 0; j <= colMax_; j++)
                {
                    //            if (map_[i][j] == 6) {
                    printf("%d ", map_[i][j]);
                    //            } else {
                    //                printf("%s", " ");
                    //            }
                }
                printf("\n");
            }
        }

        std::vector<Point::Ptr> Astar::GetPath()
        {
            if (!output_.empty())
            {
                return output_;
            }
            else
            {
                return {};
            }
        }
    }
}