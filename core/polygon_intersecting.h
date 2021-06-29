#pragma once

#include "polygon.h"
#include <vector>

namespace polygon{
    // points ordered by the un clock order, 逆时针
    template <class T>
    struct MonotoneChain{
        std::vector<point2d_t<T>*> points_chain;
        int monochainid = 0;
        bool GetMidEdge(extreme_edge_t<T>& midedge, MonotoneChain<T>& monochan_before, MonotoneChain<T>& monochain_after);
        bool GetEdge(int index, extreme_edge_t<T>& exedge){// start is 0
            if(index > points_chain.size()-2){
                return false;
            }
            exedge.p_start = points_chain[index];
            exedge.p_end = points_chain[index+1];
        }
        MonotoneChain() = default;
        MonotoneChain(int mcid):monochainid(mcid){}
    };
 

    template <class T>
    bool IsLineIntersecting(extreme_edge_t<T>& segment0, extreme_edge_t<T>&segment1){
        bool flag_endpoint0 = false, flag_endpoint1 = false;
        if(segment0.p_start->Equal(segment1.p_start) || segment0.p_start->Equal(segment1.p_end)){
            printf("segment0 is intersecting with segment1 at endpoint start.\n");
            flag_endpoint0 = true;
        }
        if(segment0.p_end->Equal(segment1.p_start) || segment0.p_end->Equal(segment1.p_end)){
            printf("segment0 is intersecting with segment1 at endpoint end.\n");
            flag_endpoint1 = true;
        }
        if(flag_endpoint0 && flag_endpoint1){
            printf("two line is all matched.\n");
            return true;
        }
        else if(flag_endpoint0 || flag_endpoint1){
            return true;
        }
        
        bool seg0_seg1s = ToLeftTest(segment0.p_start, segment0.p_end, segment1.p_start);
        bool seg0_seg1e = ToLeftTest(segment0.p_start, segment0.p_end, segment1.p_end);
        bool flag0 = false;
        if(seg0_seg1s ^ seg0_seg1e){
            flag0 = true;
        }
        else{
            return false;
        }

        bool seg1_seg0s = ToLeftTest(segment1.p_start, segment1.p_end, segment0.p_start);
        bool seg1_seg0e = ToLeftTest(segment1.p_start, segment1.p_end, segment0.p_end);
        if(seg1_seg0s ^ seg1_seg0e && flag0){
            return true;
        }
        else{
            return false;
        }
        // return false;
    }

    // may use the zig-zag method to find the bridge first 
    // the inserct can be caled by the bridge fastly
    // has same corner case: same line coincide, or inserct at the polygon extreme_points.

    // change the problem change to 单调链的相交问题。
    // 1. decide a main direction
    // 2. split polygon to two link
    template <class T>
    struct linepoint2d_t{
        linepoint2d_t() = default;

        linepoint2d_t(const bool& updatetag){
            if(updatetag)
                ++counterLP;
        }
        linepoint2d_t(const int& monochaintype, const bool& updatetag){
            monochainid = monochaintype;
            if(updatetag)
                ++counterLP;
        }
        /**
         * @brief monochain type has 0, 1, 2, 3, 4, 5, 6, 7
         * 0 is monochain type is first_leftmonochain_xbase
         * 1 is monochain type is first_rightmonochain_xbase
         * 2 is monochain type is second_leftmonochain_xbase
         * 3 is monochain type is second rightmonochain_xbase
         * 
         * 4 is monochain type is first leftmonochain_ybase
         * 5 is monochain type is first_rightmonochain_ybase
         * 6 is monochain type is second_leftmonochain_ybase
         * 7 is monochain type is second_rightmonochain_ybase
        */
        linepoint2d_t(point2d_t<T>* ptr, int monochaintype, const bool& il, const bool& updatetag = true){
            point2dptr = ptr;
            tag = counterLP;
            isleft = il;
            monochainid = monochaintype;
            if(updatetag)
                ++counterLP;
        }
        point2d_t<T>* point2dptr = nullptr;
        bool isleft = true;
        unsigned long long tag = 0ull;
        unsigned long long monochainid;
        private:
            static std::atomic_ullong counterLP;// 条直线有关于当前vector中排列的索引序号，设置成tag，多个点可以是一个tag    
    };
    template <class T>
    std::atomic_ullong linepoint2d_t<T>::counterLP = {0};

    template <class T>
    bool Compare_Xmin_minypre(const linepoint2d_t<T>& lp0, const linepoint2d_t<T>& lp1){
        bool flag = false;
        if(lp0.point2dptr->x == lp1.point2dptr->x){
            flag = true;
        }
        else if(lp0.point2dptr->x == lp1.point2dptr->x){
            if(lp0.point2dptr->y < lp1.point2dptr->y)
                flag = true;
            else
                flag = false;
        }
        else {
            flag = false;
        }
        return flag;
    }

    template <class T>
    bool Compare_Ymin_minxpre(const linepoint2d_t<T>& lp0, const linepoint2d_t<T>& lp1){
        bool flag = false;
        if(lp0.point2dptr->y == lp1.point2dptr->y){
            flag = true;
        }
        else if(lp0.point2dptr->y == lp1.point2dptr->y){
            if(lp0.point2dptr->x < lp1.point2dptr->x)
                flag = true;
            else
                flag = false;
        }
        else {
            flag = false;
        }
        return flag;
    }
    // just between two convex polygon.
    template <class T>
    class PolygonIntersecting{
        public:
            PolygonIntersecting(Polygon<T>* pgf, Polygon<T>* pgs):polygon_firstptr_(pgf), polygon_secondptr_(pgs){
                leftfirst_monochain_xbase_.monochainid = 0;
                rightfirst_monochain_xbase_.monochainid = 1;
                leftsecond_monochain_xbase_.monochainid = 2;
                rightsecond_monochain_xbase_.monochainid = 3;

                leftfirst_monochain_ybase_.monochainid = 4;
                rightfirst_monochain_ybase_.monochainid = 5;
                leftsecond_monochain_ybase_.monochainid = 6;
                rightsecond_monochain_ybase_.monochainid = 7;
            }
            void CalBridge();
            void SplitPolygon2MonotoneChain();// can get four chain, than use these four chain to cal if has interface
            void GetFirstMonoChain(std::vector<point2d_t<T>*>& leftchain, std::vector<point2d_t<T>*>& rightchain);

            void GetFirstMonoChainXbase(std::vector<point2d_t<T>*>& leftchain, std::vector<point2d_t<T>*>& rightchain);
            void GetSecondMonoChainXbase(std::vector<point2d_t<T>*>& leftchain, std::vector<point2d_t<T>*>& rightchain);
            void GetFirstMonoChainYbase(std::vector<point2d_t<T>*>& leftchain, std::vector<point2d_t<T>*>& rightchain);
            void GetSecondMonoChainYbase(std::vector<point2d_t<T>*>& leftchain, std::vector<point2d_t<T>*>& rightchain);

            bool HasIntersection(const bool& usexorybase = false);
            void CostructIntersectionPolygon(const MonotoneChain<T>& inter);
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

            void OrderLineEndPoints();
            bool PotentionIntersection(const std::vector<linepoint2d_t<T>>& sorted_extremelinepoint2d_hor, const std::vector<linepoint2d_t<T>>& sorted_extremelinepoint2d_vel);
            // bool PotentionIntersection(const MonotoneChain<T>& monochain0, const MonotoneChain<T>& monochain1, const std::vector<linepoint2d_t<T>>& sorted_extremelinepoint2d_hor, const std::vector<linepoint2d_t<T>>& sorted_extremelinepoint2d_vel);
            std::vector<linepoint2d_t<T>> sorted_extremelinepoint2d_xbase_hor_, sorted_extremelinepoint2d_xbase_vel_;
            std::vector<linepoint2d_t<T>> sorted_extremelinepoint2d_ybase_hor_, sorted_extermelinepoint2d_ybase_vel_; 

            bool HasIntersection_Xbase();
            bool HasIntersection_Ybase();
    };
}

#include "polygon_intersecting.cpp"