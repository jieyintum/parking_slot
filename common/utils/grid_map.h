#ifndef FUSION_GRID_MAP_H_
#define FUSION_GRID_MAP_H_

#include <memory>
#include <vector>
#include <set>
#include <unordered_map>
#include <deque>
#include <cmath>

namespace Fusion {

struct Grid {
    using Ptr = std::shared_ptr<Grid>;

    Grid() = default;

    ~Grid() = default;

    uint32_t pointNum = 0;

    bool isFind = false;

    uint32_t nbrNum = 0;
};


struct GridIndex {
    GridIndex() = default;

    GridIndex(const int16_t _r, const int16_t _c) : row(_r), col(_c)
    {}

    GridIndex operator+(const GridIndex& other) const
    {
        return GridIndex(row + other.row, col + other.col);
    }

    bool operator<(const GridIndex& other) const
    {
        uint32_t ordered = (static_cast<uint32_t>(row) << 16) + static_cast<uint32_t>(col);
        uint32_t orderedOther = (static_cast<uint32_t>(other.row) << 16) + static_cast<uint32_t>(other.col);

        return ordered < orderedOther;
    }

    void operator+=(const GridIndex& other)
    {
        row += other.row;
        col += other.col;
    }

    int16_t row = 0;
    int16_t col = 0;
};

enum class SearchMethod {
    FOUR_NEIGHBOUR,
    EIGHT_NEIGHBOUR,
    ROW_NEIGHBOUR,
    COL_NEIGHBOUR
};

using Cluster = std::vector<GridIndex>;

template<typename T>
class GridMap {
public:

    GridMap(const float minX, const float minY, const float deltaX, const float deltaY, const float precision) :
            minX_(minX), minY_(minY), precision_(precision)
    {
        size_.row = static_cast<int16_t>(deltaY / precision) + 1;
        size_.col = static_cast<int16_t>(deltaX / precision) + 1;


        ///create grid map
        if (size_.row > 0 && size_.row < 3000 && size_.col > 0 && size_.col < 3000) {
            map_.reserve(size_.row);
            for (int i = 0 ; i < size_.row ; ++i) {
                std::vector<std::shared_ptr<T>> temp;
                temp.reserve(size_.col);
                for (int j = 0 ; j < size_.col ; ++j) {
                    temp.template emplace_back(std::make_shared<T>());
                }
                map_.template emplace_back(temp);
            }
        }

        ///search method
        searchNeibour_[SearchMethod::FOUR_NEIGHBOUR] = {{-1, 0},
                                                        {1,  0},
                                                        {0,  -1},
                                                        {0,  1}};
        searchNeibour_[SearchMethod::EIGHT_NEIGHBOUR] = {{-1, 0},
                                                         {1,  0},
                                                         {0,  -1},
                                                         {0,  1},
                                                         {-1, -1},
                                                         {-1, 1},
                                                         {1,  -1},
                                                         {1,  1}};
        searchNeibour_[SearchMethod::ROW_NEIGHBOUR] = {{0, -1},
                                                       {0, 1}};
        searchNeibour_[SearchMethod::COL_NEIGHBOUR] = {{-1, 0},
                                                       {1,  0}};
    }

    /**
    * @description: Check if a point is in this map.
    * @param {GridIndex&} index
    * @return bool
    */
    inline bool IsInRange(const GridIndex& index)
    {
        return (index.row >= 0) && (index.row < size_.row) && (index.col >= 0) && (index.col < size_.col);
    }

    inline bool InsertPoint(const float x, const float y)
    {
        const auto index = GetRowCol(x, y);
        if (IsInRange(index)) {
            ++map_[index.row][index.col]->pointNum;
            activateIndex_.insert(index);
            return true;
        }
        return false;
    }

    inline GridIndex GetRowCol(const float x, const float y)
    {
        ///function(floor(x)): The largest int, which not > x
        GridIndex index;
        index.row = std::floor((y - minY_) / precision_);
        index.col = std::floor((x - minX_) / precision_);

        return index;
    }

    inline void Reset()
    {
        for (const auto& index : activateIndex_) {
            map_[index.row][index.col] = std::make_shared<T>();
        }
        activateIndex_.clear();
    }

    std::vector<Cluster> Clustering(const SearchMethod& method)
    {
        /// load direction
        const auto& findNeighbours = searchNeibour_[method];
        std::vector<Cluster> clusters;
        for (const auto& index : activateIndex_) {
            if (map_[index.row][index.col]->isFind == true) {
                continue;
            }
            clusters.push_back(FindNeighbour(findNeighbours, index));
        }
        return clusters;
    }

    int16_t GetRow() const
    {
        return this->size_.row;
    }

    int16_t GelCol() const
    {
        return this->size_.col;
    }

    void SetLowerBound(const float minX, const float minY) 
    {
        this->minX_ = minX;
        this->minY_ = minY;
    }

private:
    Cluster FindNeighbour(const std::vector<GridIndex>& findNeighbours,
                          const GridIndex& seedIndex)
    {
        Cluster cluster;
        std::deque<GridIndex> indexDeq;
        indexDeq.push_back(seedIndex);
        cluster.push_back(seedIndex);

        while (!indexDeq.empty()) {
            const auto& indexFront = indexDeq.front();
            map_[indexFront.row][indexFront.col]->isFind = true;
            uint32_t nbrNum = 0;
            ///find grid along with direction
            for (const auto& neighbour : findNeighbours) {
                const auto index = indexFront + neighbour;
                if (IsInRange(index)) {
                    if (map_[index.row][index.col]->pointNum != 0) {
                        ++nbrNum;
                    }
                    /// if this direction's next grid's status is false
                    /// and this grid has at least one slot, turn it to true
                    if (map_[index.row][index.col]->pointNum != 0 &&
                        map_[index.row][index.col]->isFind == false) {
                        map_[index.row][index.col]->isFind = true;
                        indexDeq.template emplace_back(index);
                        cluster.template emplace_back(index);
                    }
                }

            }
            map_[indexFront.row][indexFront.col]->nbrNum = nbrNum;
            indexDeq.pop_front();
        }
        return cluster;
    }

protected:
    std::vector<std::vector<std::shared_ptr<T>>> map_;
    ///set container
    std::set<GridIndex> activateIndex_;
    GridIndex size_;
private:
    float minX_;
    float minY_;
    const float precision_;
    std::unordered_map<SearchMethod, std::vector<GridIndex>> searchNeibour_;
};

}
#endif