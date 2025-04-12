#ifndef OVERLAP_CHECKER_H
#define OVERLAP_CHECKER_H

#include <vector>

namespace StaticObjectMerge
{
  class OverlapChecker
  {
  public:
    OverlapChecker() = default;
    virtual ~OverlapChecker() = default;

    template <typename POLYGON1, typename POLYGON2, typename POINT>
    static bool SATOverlap(const POLYGON1 &polygen1, const POLYGON2 &polygen2)
    {
      if (polygen1.empty() || polygen2.empty())
      {
        return false;
      }

      if (IsSame(polygen1, polygen2))
      {
        return true;
      }

      std::vector<POINT> edges;
      const size_t size = polygen1.size() + polygen2.size();
      edges.reserve(size);
      GetEdgesFromVertices(polygen1, edges);
      GetEdgesFromVertices(polygen2, edges);
      for (size_t idx = 0U; idx < size; idx++)
      {
        POINT direction = edges[idx];
        direction = Perp(direction);
        const POINT proj1 = Projection(polygen1, direction);
        const POINT proj2 = Projection(polygen2, direction);
        if (!IsOverlap(proj1, proj2))
        {
          return false;
        }
      }
      return true;
    }

  private:
    template <typename POINT>
    static inline POINT Perp(const POINT &v)
    {
      return POINT(v[1], -v[0]);
    }

    template <typename POINT>
    static inline POINT Minuspoints(const POINT &p1, const POINT &p2)
    {
      return POINT(p1[0] - p2[0], p1[1] - p2[1]);
    }

    template <typename POINT>
    static inline float MultPoint(const POINT &a, const POINT &b)
    {
      return a[0] * b[0] + a[1] * b[1];
    }

    template <typename POLYGON1, typename POLYGON2>
    static inline bool IsSame(const POLYGON1 &polygen1, const POLYGON2 &polygen2)
    {
      if (polygen1.size() == polygen2.size())
      {
        for (size_t i = 0; i < polygen1.size(); ++i)
        {
          if (std::abs(polygen1[i](0) - polygen2[i](0)) > 1e-5 || std::abs(polygen1[i](1) - polygen2[i](1)) > 1e-5)
          {
            return false;
          }
        }
        return true;
      }
      return false;
    }

    template <typename POLYGON, typename POINT>
    static POINT Projection(const POLYGON &polygen, const POINT &axis)
    {
      auto min_tmp = MultPoint(polygen[0], axis);
      auto max_tmp = min_tmp;
      for (size_t i = 1U; i < polygen.size(); i++)
      {
        const auto p = MultPoint(polygen[i], axis);
        min_tmp = p < min_tmp ? p : min_tmp;
        max_tmp = p > max_tmp ? p : max_tmp;
      }

      return POINT(min_tmp, max_tmp);
    }

    template <typename POLYGON, typename POINT>
    static void GetEdgesFromVertices(const POLYGON &vertices,
                                     std::vector<POINT> &edges)
    {
      POINT tmp_edges;
      for (size_t i = 0U; i < vertices.size() - 1U; i++)
      {
        tmp_edges = Minuspoints(vertices[i + 1U], vertices[i]);
        edges.emplace_back(tmp_edges);
      }
      if (vertices.size() > 2)
      {
        tmp_edges = Minuspoints(vertices[0], vertices[vertices.size() - 1U]);
        edges.emplace_back(tmp_edges);
      }
    }

    template <typename POINT>
    static inline bool Contains(const float a, const POINT &b)
    {
      return (a >= b[0]) && (a <= b[1]);
    }

    template <typename POINT>
    static bool IsOverlap(const POINT &a, const POINT &b)
    {
      if (Contains(a[0], b))
      {
        return true;
      }
      if (Contains(a[1], b))
      {
        return true;
      }
      if (Contains(b[0], a))
      {
        return true;
      }
      if (Contains(b[1], a))
      {
        return true;
      }
      return false;
    }
  };
}

#endif