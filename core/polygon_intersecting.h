#pragma once

#include "polygon.h"
#include <vector>

namespace polygon{
    // points ordered by the un clock order, 逆时针
    template <class T>
    struct MonotoneChain{
        std::vector<point2d_t<T>*> points_chain;
        bool GetMidEdge(extreme_edge_t<T>& midedge, MonotoneChain<T>& monochan_before, MonotoneChain<T>& monochain_after);
    };
 
    // may use the zig-zag method to find the bridge first 
    // the inserct can be caled by the bridge fastly
    // has same corner case: same line coincide, or inserct at the polygon extreme_points.

    // change the problem change to 单调链的相交问题。
    // 1. decide a main direction
    // 2. split polygon to two link
    template <class T>
    class PolygonIntersecting{
        public:
            PolygonIntersecting(Polygon<T>* pgf, Polygon<T>* pgs):polygon_firstptr_(pgf), polygon_secondptr_(pgs){}
            void CalBridge();
            void SplitPolygon2MonotoneChain();// can get four chain, than use these four chain to cal if has interface

            void GetFirstMonoChain(std::vector<point2d_t<T>*>& leftchain, std::vector<point2d_t<T>*>& rightchain);

            void GetFirstMonoChainXbase(std::vector<point2d_t<T>*>& leftchain, std::vector<point2d_t<T>*>& rightchain);
            void GetSecondMonoChainXbase(std::vector<point2d_t<T>*>& leftchain, std::vector<point2d_t<T>*>& rightchain);
            void GetFirstMonoChainYbase(std::vector<point2d_t<T>*>& leftchain, std::vector<point2d_t<T>*>& rightchain);
            void GetSecondMonoChainYbase(std::vector<point2d_t<T>*>& leftchain, std::vector<point2d_t<T>*>& rightchain);
        private:
            // decide a main direction, to split polygon. just use ox(_|_)oy
            // split ordered:
            // 1. if down border has point in same line, alls push into after chain
            // 2. if up border has point in same line, alls push into before chain
            bool xory_major_ = false;

            MonotoneChain<T> leftfirst_monochain_xbase_;
            MonotoneChain<T> rightfirst_monochain_xbase_;
            MonotoneChain<T> leftsecond_monochain_xbase_;
            MonotoneChain<T> rightsecond_monochain_xbase_;

            MonotoneChain<T> leftfirst_monochain_ybase_;
            MonotoneChain<T> rightfirst_monochain_ybase_;
            MonotoneChain<T> leftsecond_monochain_ybase_;
            MonotoneChain<T> rightsecond_monochain_ybase_;

            Polygon<T>* polygon_firstptr_ = nullptr;
            Polygon<T>* polygon_secondptr_ = nullptr;

            bool IsSmallerorBiggerX(polygon::Polygon<T>* ptr, const bool& isclockwise, const int& last_count, int& cur_count); // can change the curcount to avoid all points in a line .
            bool IsSmallerorBiggerXLock(polygon::Polygon<T>* ptr, const bool& isclockwise, const int& last_count, int& cur_count);

            bool IsSmallerorBiggerY(polygon::Polygon<T>* ptr, const bool& isclockwise, const int& last_count, int& cur_count); // can change the curcount to avoid all points in a line .
            bool IsSmallerorBiggerYLock(polygon::Polygon<T>* ptr, const bool& isclockwise, const int& last_count, int& cur_count);

    };
}

#include "polygon_intersecting.cpp"