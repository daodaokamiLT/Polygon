#include "polygon_intersecting.h"
#include <limits>

#define debug false
#define waitkey false
#if debug
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#endif
/***
 * 
 * 需要解决那种交在定点上，或者重合的问题
 * 
 * 
 * 
 * 
*/
namespace polygon
{
    template <class T>
    bool MonotoneChain<T>::GetMidEdge(extreme_edge_t<T> &midedge, MonotoneChain<T> &monochan_before, MonotoneChain<T> &monochain_after)
    {
        int size = points_chain.size();
        PolygonPrintf("points chain size is %d.\n", size);
        if (size == 1)
        {
            PolygonPrintf("error cannot create a edge and the chain never be 1 point, because that cannot create a line.\n");
            exit(-1);
        }
        if (size == 2)
        {
            midedge.p_start = points_chain[0];
            midedge.p_end = points_chain[1];
            return true;
        }
        // 边数比点数少1
        int mid = (int)(size - 1) / 2;
        PolygonPrintf("mid is %d.\n", mid);
        bool issucc = this->GetEdge(mid, midedge);
        if (!issucc)
        {
            PolygonPrintf("error, failed GetEdge.\n");
            exit(-1);
        }
        // 按照这个chain的顺序来排列的
        if (mid != 0)
        {
            for (int i = 0; i <= mid; ++i)
            {
                monochan_before.points_chain.emplace_back(points_chain[i]);
            }
        }
        if (mid + 1 != size - 1)
        {
            for (int i = mid + 1; i < size; ++i)
            {
                monochain_after.points_chain.emplace_back(points_chain[i]);
            }
        }
        return issucc;
    }

    template <class T>
    bool MonotoneChain<T>::GetMidEdge(extreme_edge_t<T> &midedge, MonotoneChain<T> &monochan_before, MonotoneChain<T> &monochain_after, int& before_start_index, int& after_start_index)
    {
        int size = points_chain.size();
        PolygonPrintf("points chain size is %d.\n", size);
        if (size == 1)
        {
            PolygonPrintf("error cannot create a edge and the chain never be 1 point, because that cannot create a line.\n");
            exit(-1);
        }
        if (size == 2)
        {
            midedge.p_start = points_chain[0];
            midedge.p_end = points_chain[1];
            return true;
        }
        // 边数比点数少1
        int mid = (int)(size - 1) / 2;
        PolygonPrintf("mid is %d.\n", mid);
        bool issucc = this->GetEdge(mid, midedge);
        if (!issucc)
        {
            PolygonPrintf("error, failed GetEdge.\n");
            exit(-1);
        }
        // 按照这个chain的顺序来排列的
        if (mid != 0)
        {
            before_start_index = 0;
            for (int i = 0; i <= mid; ++i)
            {
                monochan_before.points_chain.emplace_back(points_chain[i]);
            }
        }
        if (mid + 1 != size - 1)
        {
            after_start_index = mid+1;
            for (int i = mid + 1; i < size; ++i)
            {
                monochain_after.points_chain.emplace_back(points_chain[i]);
            }
        }
        return issucc;
    }

    template <class T>
    void findextreme(MonotoneChain<T> &monochain, linepoint2d_t<T> &p_left, linepoint2d_t<T> &p_right, linepoint2d_t<T> &p_up, linepoint2d_t<T> &p_down)
    {
        T min_x = std::numeric_limits<T>::max(), max_x = std::numeric_limits<T>::min(), min_y = std::numeric_limits<T>::max(), max_y = std::numeric_limits<T>::min();
        size_t minx_index = 0, maxx_index = 0, miny_index = 0, maxy_index = 0;
        for (size_t i = 0; i < monochain.points_chain.size(); ++i)
        {
            // std::cout<<monochain.points_chain[i]->x<<", "<<monochain.points_chain[i]->y<<std::endl;
            if (min_x > monochain.points_chain[i]->x)
            {
                min_x = monochain.points_chain[i]->x;
                minx_index = i;
            }
            if (min_y > monochain.points_chain[i]->y)
            {
                min_y = monochain.points_chain[i]->y;
                miny_index = i;
            }
            if (max_x < monochain.points_chain[i]->x)
            {
                max_x = monochain.points_chain[i]->x;
                maxx_index = i;
            }
            if (max_y < monochain.points_chain[i]->y)
            {
                max_y = monochain.points_chain[i]->y;
                maxy_index = i;
            }
        }
        p_left.isleft = true;
        p_left.point2dptr = monochain.points_chain[minx_index];
        // std::cout<<"maxx_index "<<maxx_index<<" maxx is "<<max_x<<std::endl;
        p_right.isleft = false;
        p_right.point2dptr = monochain.points_chain[maxx_index];

        p_up.isleft = true;
        p_up.point2dptr = monochain.points_chain[miny_index];

        p_down.isleft = false;
        p_down.point2dptr = monochain.points_chain[maxy_index];
    }
    /**
     * 
     * 若x 有的投影是相同的，则y小的优先加入
     * 若y 有的投影是相同的，则x小的优先加入
     * 
     * **/
    template <class T>
    void PolygonIntersecting<T>::OrderLineEndPoints()
    {
        // first polygon
        linepoint2d_t<T> linepoint2d_fromL1st_xbase_leftest(0, 0), linepoint2d_fromL1st_xbase_rightest(0, 0), linepoint2d_fromL1st_xbase_topest(0, 0), linepoint2d_fromL1st_xbase_bottomest(0, 0);
        findextreme(leftfirst_monochain_xbase_, linepoint2d_fromL1st_xbase_leftest, linepoint2d_fromL1st_xbase_rightest, linepoint2d_fromL1st_xbase_topest, linepoint2d_fromL1st_xbase_bottomest);

        // std::cout<<"linepoint2d_fromL1st tag: "<<linepoint2d_fromL1st_xbase_leftest.tag<<", "<<linepoint2d_fromL1st_xbase_rightest.tag<<", "<<linepoint2d_fromL1st_xbase_topest.tag<<", "<<linepoint2d_fromL1st_xbase_bottomest.tag<<std::endl;
        linepoint2d_t<T> linepoint2d_fromR1st_xbase_leftest(1, 1), linepoint2d_fromR1st_xbase_rightest(1, 1), linepoint2d_fromR1st_xbase_topest(1, 1), linepoint2d_fromR1st_xbase_bottomest(1, 1);
        findextreme(rightfirst_monochain_xbase_, linepoint2d_fromR1st_xbase_leftest, linepoint2d_fromR1st_xbase_rightest, linepoint2d_fromR1st_xbase_topest, linepoint2d_fromR1st_xbase_bottomest);
        // std::cout<<"linepoint2d_fromL1st tag: "<<linepoint2d_fromR1st_xbase_leftest.tag<<", "<<linepoint2d_fromR1st_xbase_rightest.tag<<", "<<linepoint2d_fromR1st_xbase_topest.tag<<", "<<linepoint2d_fromR1st_xbase_bottomest.tag<<std::endl;

        // check
        if (linepoint2d_fromL1st_xbase_leftest.point2dptr->x != linepoint2d_fromR1st_xbase_leftest.point2dptr->x || linepoint2d_fromL1st_xbase_rightest.point2dptr->x != linepoint2d_fromR1st_xbase_rightest.point2dptr->x)
        {
            PolygonPrintf("error, the first extreme points get error, same polygon leftest and rightest x is not same. left %lf %lf right %lf %lf.\n", linepoint2d_fromL1st_xbase_leftest.point2dptr->x, linepoint2d_fromR1st_xbase_leftest.point2dptr->x,
                          linepoint2d_fromL1st_xbase_rightest.point2dptr->x, linepoint2d_fromR1st_xbase_rightest.point2dptr->x);
            exit(-1);
        }

        // second polygon
        linepoint2d_t<T> linepoint2d_fromL2nd_xbase_leftest(2, 2), linepoint2d_fromL2nd_xbase_rightest(2, 2), linepoint2d_fromL2nd_xbase_topest(2, 2), linepoint2d_fromL2nd_xbase_bottomest(2, 2);
        findextreme(leftsecond_monochain_xbase_, linepoint2d_fromL2nd_xbase_leftest, linepoint2d_fromL2nd_xbase_rightest, linepoint2d_fromL2nd_xbase_topest, linepoint2d_fromL2nd_xbase_bottomest);
        // std::cout<<"linepoint2d_fromL2nd tag: "<<linepoint2d_fromL2nd_xbase_leftest.tag<<", "<<linepoint2d_fromL2nd_xbase_rightest.tag<<", "<<linepoint2d_fromL2nd_xbase_topest.tag<<", "<<linepoint2d_fromL2nd_xbase_bottomest.tag<<std::endl;

        linepoint2d_t<T> linepoint2d_fromR2nd_xbase_leftest(3, 3), linepoint2d_fromR2nd_xbase_rightest(3, 3), linepoint2d_fromR2nd_xbase_topest(3, 3), linepoint2d_fromR2nd_xbase_bottomest(3, 3);
        findextreme(rightsecond_monochain_xbase_, linepoint2d_fromR2nd_xbase_leftest, linepoint2d_fromR2nd_xbase_rightest, linepoint2d_fromR2nd_xbase_topest, linepoint2d_fromR2nd_xbase_bottomest);
        // std::cout<<"linepoint2d_fromR2nd tag: "<<linepoint2d_fromR2nd_xbase_leftest.tag<<", "<<linepoint2d_fromR2nd_xbase_rightest.tag<<", "<<linepoint2d_fromR2nd_xbase_topest.tag<<", "<<linepoint2d_fromR2nd_xbase_bottomest.tag<<std::endl;

        if (linepoint2d_fromL2nd_xbase_leftest.point2dptr->x != linepoint2d_fromR2nd_xbase_leftest.point2dptr->x || linepoint2d_fromL2nd_xbase_rightest.point2dptr->x != linepoint2d_fromR2nd_xbase_rightest.point2dptr->x)
        {
            PolygonPrintf("error, the second extreme points get error, same polygon leftest and rightest x is not same.\n");
            exit(-1);
        }

        // ----- ybase
        linepoint2d_t<T> linepoint2d_fromL1st_ybase_leftest(4, 4), linepoint2d_fromL1st_ybase_rightest(4, 4), linepoint2d_fromL1st_ybase_topest(4, 4), linepoint2d_fromL1st_ybase_bottomest(4, 4);
        findextreme(leftfirst_monochain_ybase_, linepoint2d_fromL1st_ybase_leftest, linepoint2d_fromL1st_ybase_rightest, linepoint2d_fromL1st_ybase_topest, linepoint2d_fromL1st_ybase_bottomest);
        // std::cout<<"linepoint2d_fromL1st tag: "<<linepoint2d_fromL1st_ybase_leftest.tag<<", "<<linepoint2d_fromL1st_ybase_rightest.tag<<", "<<linepoint2d_fromL1st_ybase_topest.tag<<", "<<linepoint2d_fromL1st_ybase_bottomest.tag<<std::endl;

        linepoint2d_t<T> linepoint2d_fromR1st_ybase_leftest(5, 5), linepoint2d_fromR1st_ybase_rightest(5, 5), linepoint2d_fromR1st_ybase_topest(5, 5), linepoint2d_fromR1st_ybase_bottomest(5, 5);
        findextreme(rightfirst_monochain_ybase_, linepoint2d_fromR1st_ybase_leftest, linepoint2d_fromR1st_ybase_rightest, linepoint2d_fromR1st_ybase_topest, linepoint2d_fromR1st_ybase_bottomest);
        // std::cout<<"linepoint2d_fromR1st tag: "<<linepoint2d_fromR1st_ybase_leftest.tag<<", "<<linepoint2d_fromR1st_ybase_rightest.tag<<", "<<linepoint2d_fromR1st_ybase_topest.tag<<", "<<linepoint2d_fromR1st_ybase_bottomest.tag<<std::endl;

        if (linepoint2d_fromL1st_ybase_topest.point2dptr->y != linepoint2d_fromR1st_ybase_topest.point2dptr->y || linepoint2d_fromL1st_ybase_bottomest.point2dptr->y != linepoint2d_fromR1st_ybase_bottomest.point2dptr->y)
        {
            PolygonPrintf("error, the first extreme points get error, same polygon topest and bottomest y is not same.\n");
            PolygonPrintf("linepoint2d_fromL1st_ybase_topest.point2dptr->y %lf != %lf linepoint2d_fromR1st_ybase_topest.point2dptr->y || linepoint2d_fromL1st_ybase_bottomest.point2dptr->y %lf != %lf linepoint2d_fromR1st_ybase_bottomest.point2dptr->y.\n", linepoint2d_fromL1st_ybase_topest.point2dptr->y, linepoint2d_fromR1st_ybase_topest.point2dptr->y,
                          linepoint2d_fromL1st_ybase_bottomest.point2dptr->y, linepoint2d_fromR1st_ybase_bottomest.point2dptr->y);
            exit(-1);
        }

        // ----- second ybase
        linepoint2d_t<T> linepoint2d_fromL2nd_ybase_leftest(6, 6), linepoint2d_fromL2nd_ybase_rightest(6, 6), linepoint2d_fromL2nd_ybase_topest(6, 6), linepoint2d_fromL2nd_ybase_bottomest(6, 6);
        findextreme(leftsecond_monochain_ybase_, linepoint2d_fromL2nd_ybase_leftest, linepoint2d_fromL2nd_ybase_rightest, linepoint2d_fromL2nd_ybase_topest, linepoint2d_fromL2nd_ybase_bottomest);
        // std::cout<<"linepoint2d_fromL2nd tag: "<<linepoint2d_fromL2nd_ybase_leftest.tag<<", "<<linepoint2d_fromL2nd_ybase_rightest.tag<<", "<<linepoint2d_fromL2nd_ybase_topest.tag<<", "<<linepoint2d_fromL2nd_ybase_bottomest.tag<<std::endl;

        linepoint2d_t<T> linepoint2d_fromR2nd_ybase_leftest(7, 7), linepoint2d_fromR2nd_ybase_rightest(7, 7), linepoint2d_fromR2nd_ybase_topest(7, 7), linepoint2d_fromR2nd_ybase_bottomest(7, 7);
        findextreme(rightsecond_monochain_ybase_, linepoint2d_fromR2nd_ybase_leftest, linepoint2d_fromR2nd_ybase_rightest, linepoint2d_fromR2nd_ybase_topest, linepoint2d_fromR2nd_ybase_bottomest);
        // std::cout<<"linepoint2d_fromR2nd tag: "<<linepoint2d_fromR2nd_ybase_leftest.tag<<", "<<linepoint2d_fromR2nd_ybase_rightest.tag<<", "<<linepoint2d_fromR2nd_ybase_topest.tag<<", "<<linepoint2d_fromR2nd_ybase_bottomest.tag<<std::endl;

        if (linepoint2d_fromL2nd_ybase_topest.point2dptr->y != linepoint2d_fromR2nd_ybase_topest.point2dptr->y || linepoint2d_fromL2nd_ybase_bottomest.point2dptr->y != linepoint2d_fromR2nd_ybase_bottomest.point2dptr->y)
        {
            PolygonPrintf("error, the second extreme points get error, same polygon topest and bottomest y is not same.\n");
            exit(-1);
        }

        sorted_extremelinepoint2d_xbase_hor_.clear();
        sorted_extremelinepoint2d_xbase_hor_.emplace_back(linepoint2d_fromL1st_xbase_leftest);
        sorted_extremelinepoint2d_xbase_hor_.emplace_back(linepoint2d_fromL1st_xbase_rightest);
        sorted_extremelinepoint2d_xbase_hor_.emplace_back(linepoint2d_fromL2nd_xbase_leftest);
        sorted_extremelinepoint2d_xbase_hor_.emplace_back(linepoint2d_fromL2nd_xbase_rightest);
        std::sort(sorted_extremelinepoint2d_xbase_hor_.begin(), sorted_extremelinepoint2d_xbase_hor_.end(), Compare_Xmin_minypre<T>);
        // for(auto expoint: sorted_extremelinepoint2d_xbase_hor_){
        //     std::cout<<"expoint: "<<expoint.point2dptr->x<<" "<<expoint.point2dptr->y<<std::endl;
        // }

        sorted_extremelinepoint2d_xbase_vel00_.clear();
        sorted_extremelinepoint2d_xbase_vel00_.emplace_back(linepoint2d_fromL1st_xbase_topest);
        sorted_extremelinepoint2d_xbase_vel00_.emplace_back(linepoint2d_fromL1st_xbase_bottomest);
        sorted_extremelinepoint2d_xbase_vel00_.emplace_back(linepoint2d_fromL2nd_xbase_topest);
        sorted_extremelinepoint2d_xbase_vel00_.emplace_back(linepoint2d_fromL2nd_xbase_bottomest);
        std::sort(sorted_extremelinepoint2d_xbase_vel00_.begin(), sorted_extremelinepoint2d_xbase_vel00_.end(), Compare_Ymin_minxpre<T>);

        sorted_extremelinepoint2d_xbase_vel01_.clear();
        sorted_extremelinepoint2d_xbase_vel01_.emplace_back(linepoint2d_fromL1st_xbase_topest);
        sorted_extremelinepoint2d_xbase_vel01_.emplace_back(linepoint2d_fromL1st_xbase_bottomest);
        sorted_extremelinepoint2d_xbase_vel01_.emplace_back(linepoint2d_fromR2nd_xbase_topest);
        sorted_extremelinepoint2d_xbase_vel01_.emplace_back(linepoint2d_fromR2nd_xbase_bottomest);
        std::sort(sorted_extremelinepoint2d_xbase_vel01_.begin(), sorted_extremelinepoint2d_xbase_vel01_.end(), Compare_Ymin_minxpre<T>);

        sorted_extremelinepoint2d_xbase_vel10_.clear();
        sorted_extremelinepoint2d_xbase_vel10_.emplace_back(linepoint2d_fromR1st_xbase_topest);
        sorted_extremelinepoint2d_xbase_vel10_.emplace_back(linepoint2d_fromR1st_xbase_bottomest);
        sorted_extremelinepoint2d_xbase_vel10_.emplace_back(linepoint2d_fromL2nd_xbase_topest);
        sorted_extremelinepoint2d_xbase_vel10_.emplace_back(linepoint2d_fromL2nd_xbase_bottomest);
        std::sort(sorted_extremelinepoint2d_xbase_vel10_.begin(), sorted_extremelinepoint2d_xbase_vel10_.end(), Compare_Ymin_minxpre<T>);

        sorted_extremelinepoint2d_xbase_vel11_.clear();
        sorted_extremelinepoint2d_xbase_vel11_.emplace_back(linepoint2d_fromR1st_xbase_topest);
        sorted_extremelinepoint2d_xbase_vel11_.emplace_back(linepoint2d_fromR1st_xbase_bottomest);
        sorted_extremelinepoint2d_xbase_vel11_.emplace_back(linepoint2d_fromR2nd_xbase_topest);
        sorted_extremelinepoint2d_xbase_vel11_.emplace_back(linepoint2d_fromR2nd_xbase_bottomest);
        std::sort(sorted_extremelinepoint2d_xbase_vel11_.begin(), sorted_extremelinepoint2d_xbase_vel11_.end(), Compare_Ymin_minxpre<T>);

        sorted_extermelinepoint2d_ybase_vel_.clear();
        sorted_extermelinepoint2d_ybase_vel_.emplace_back(linepoint2d_fromL1st_ybase_topest);
        sorted_extermelinepoint2d_ybase_vel_.emplace_back(linepoint2d_fromL1st_ybase_bottomest);
        sorted_extermelinepoint2d_ybase_vel_.emplace_back(linepoint2d_fromL2nd_ybase_topest);
        sorted_extermelinepoint2d_ybase_vel_.emplace_back(linepoint2d_fromL2nd_ybase_bottomest);
        std::sort(sorted_extermelinepoint2d_ybase_vel_.begin(), sorted_extermelinepoint2d_ybase_vel_.end(), Compare_Ymin_minxpre<T>);

        sorted_extremelinepoint2d_ybase_hor00_.clear();
        sorted_extremelinepoint2d_ybase_hor00_.emplace_back(linepoint2d_fromL1st_ybase_leftest);
        sorted_extremelinepoint2d_ybase_hor00_.emplace_back(linepoint2d_fromL1st_ybase_rightest);
        sorted_extremelinepoint2d_ybase_hor00_.emplace_back(linepoint2d_fromL2nd_ybase_leftest);
        sorted_extremelinepoint2d_ybase_hor00_.emplace_back(linepoint2d_fromL2nd_ybase_rightest);
        std::sort(sorted_extremelinepoint2d_ybase_hor00_.begin(), sorted_extremelinepoint2d_ybase_hor00_.end(), Compare_Xmin_minypre<T>);

        sorted_extremelinepoint2d_ybase_hor01_.clear();
        sorted_extremelinepoint2d_ybase_hor01_.emplace_back(linepoint2d_fromL1st_ybase_leftest);
        sorted_extremelinepoint2d_ybase_hor01_.emplace_back(linepoint2d_fromL1st_ybase_rightest);
        sorted_extremelinepoint2d_ybase_hor01_.emplace_back(linepoint2d_fromR2nd_ybase_leftest);
        sorted_extremelinepoint2d_ybase_hor01_.emplace_back(linepoint2d_fromR2nd_ybase_rightest);
        std::sort(sorted_extremelinepoint2d_ybase_hor01_.begin(), sorted_extremelinepoint2d_ybase_hor01_.end(), Compare_Xmin_minypre<T>);

        sorted_extremelinepoint2d_ybase_hor10_.clear();
        sorted_extremelinepoint2d_ybase_hor10_.emplace_back(linepoint2d_fromR1st_ybase_leftest);
        sorted_extremelinepoint2d_ybase_hor10_.emplace_back(linepoint2d_fromR1st_ybase_rightest);
        sorted_extremelinepoint2d_ybase_hor10_.emplace_back(linepoint2d_fromL2nd_ybase_leftest);
        sorted_extremelinepoint2d_ybase_hor10_.emplace_back(linepoint2d_fromL2nd_ybase_rightest);
        std::sort(sorted_extremelinepoint2d_ybase_hor10_.begin(), sorted_extremelinepoint2d_ybase_hor10_.end(), Compare_Xmin_minypre<T>);

        sorted_extremelinepoint2d_ybase_hor11_.clear();
        sorted_extremelinepoint2d_ybase_hor11_.emplace_back(linepoint2d_fromR1st_ybase_leftest);
        sorted_extremelinepoint2d_ybase_hor11_.emplace_back(linepoint2d_fromR1st_ybase_rightest);
        sorted_extremelinepoint2d_ybase_hor11_.emplace_back(linepoint2d_fromR2nd_ybase_leftest);
        sorted_extremelinepoint2d_ybase_hor11_.emplace_back(linepoint2d_fromR2nd_ybase_rightest);
        std::sort(sorted_extremelinepoint2d_ybase_hor11_.begin(), sorted_extremelinepoint2d_ybase_hor11_.end(), Compare_Xmin_minypre<T>);

        // 在四个队列中，进行排序查看是否有可能的重合部分， 若有则进行递归求解。
    }
    /**
     * @brief 不处理同一个polygon的相交问题。
     * 
     * **/
    /*// template <class T>
    // bool PolygonIntersecting<T>::PotentionIntersection(const MonotoneChain<T>& monochain0, const MonotoneChain<T>& monochain1, const std::vector<linepoint2d_t<T>>& sorted_extremelinepoint2d_hor, const std::vector<linepoint2d_t<T>>& sorted_extremelinepoint2d_vel){
    //     // monochain has
    //     if(monochain0.points_chain.size() < 2 || sorted_extremelinepoint2d_hor.size() != 4 || sorted_extremelinepoint2d_vel.size() != 4){
    //         PolygonPrintf("error, monochain cannot < 2 or sorted size not equalwith 4.\n");
    //         exit(-1);
    //     }
    //     if(monochain0.monochainid == monochain1.monochainid){
    //         PolygonPrintf("error, cannot create by the same monochain.\n");
    //         exit(-1);
    //     }

    //     bool possible = PotentionIntersection(sorted_extremelinepoint2d_hor, sorted_extremelinepoint2d_vel);
    //     if(!possible){
    //         return false;
    //     }
    //     // set possible 
    //     return possible;
    // }*/

    // we think the monochain

    template <class T>
    bool PolygonIntersecting<T>::PotentionIntersection(const MonotoneChain<T> &monochain0, const MonotoneChain<T> &monochain1)
    {
        if (monochain0.points_chain.size() == 0 || monochain1.points_chain.size() == 0)
        {
            return false;
        }
        if (monochain0.points_chain.size() == 1 || monochain1.points_chain.size() == 1)
        {
            PolygonPrintf("error, monochain points size cannot be 1 that cannot create a line.\n");
            exit(-1);
        }
        T minx0 = std::numeric_limits<T>::max(), miny0 = std::numeric_limits<T>::max(),
          maxx0 = std::numeric_limits<T>::min(), maxy0 = std::numeric_limits<T>::min();
        T minx1 = std::numeric_limits<T>::max(), miny1 = std::numeric_limits<T>::max(),
          maxx1 = std::numeric_limits<T>::min(), maxy1 = std::numeric_limits<T>::min();

        for (auto elem : monochain0.points_chain)
        {
            if (minx0 > elem->x)
                minx0 = elem->x;
            if (miny0 > elem->y)
                miny0 = elem->y;
            if (maxx0 < elem->x)
                maxx0 = elem->x;
            if (maxy0 < elem->y)
                maxy0 = elem->y;
        }
        for (auto elem : monochain1.points_chain)
        {
            if (minx1 > elem->x)
                minx1 = elem->x;
            if (miny1 > elem->y)
                miny1 = elem->y;
            if (maxx1 < elem->x)
                maxx1 = elem->x;
            if (maxy1 < elem->y)
                maxy1 = elem->y;
        }
        bool possiblex = IsRangeMixed(minx0, maxx0, minx1, maxx1);
        bool possibley = IsRangeMixed(miny0, maxy0, miny1, maxy1);

        return (possiblex && possibley);
    }

    template <class T>
    bool PolygonIntersecting<T>::PotentionIntersection(const std::vector<linepoint2d_t<T>> &sorted_extremelinepoint2d_hor, const std::vector<linepoint2d_t<T>> &sorted_extremelinepoint2d_vel)
    { // sort 肯定是偶数的
        // monochain has
        if (sorted_extremelinepoint2d_hor.size() != 4 || sorted_extremelinepoint2d_vel.size() != 4)
        {
            PolygonPrintf("please init valid sorted extremelinepoints, before use this method.\n");
            exit(-1);
        }
        // //  不一定有 isleft
        // if(!sorted_extremelinepoint2d_hor[0].isleft || !sorted_extremelinepoint2d_vel[0].isleft){
        //     PolygonPrintf("the fist element must be tag = left.\n");
        //     exit(-1);
        // }
        bool possible_xbase = false;
        // 永远都是两条边，四个点
        if (sorted_extremelinepoint2d_hor[0].tag == sorted_extremelinepoint2d_hor[1].tag &&
            sorted_extremelinepoint2d_hor[0].monochainid == sorted_extremelinepoint2d_hor[1].monochainid)
        {
            if (sorted_extremelinepoint2d_hor[2].tag != sorted_extremelinepoint2d_hor[3].tag ||
                sorted_extremelinepoint2d_hor[2].monochainid != sorted_extremelinepoint2d_hor[3].monochainid)
            {
                PolygonPrintf("error, sorted extremelinepoints format is not ture.\n");
                exit(-1);
            }
            possible_xbase = false;
        }
        else if (sorted_extremelinepoint2d_hor[0].tag != sorted_extremelinepoint2d_hor[1].tag &&
                 sorted_extremelinepoint2d_hor[0].monochainid != sorted_extremelinepoint2d_hor[1].monochainid)
        {
            if (sorted_extremelinepoint2d_hor[0].tag != sorted_extremelinepoint2d_hor[2].tag && sorted_extremelinepoint2d_hor[0].tag != sorted_extremelinepoint2d_hor[3].tag)
            {
                PolygonPrintf("error, sorted extremelinepoints format is not ture.\n");
                exit(-1);
            }
            if (sorted_extremelinepoint2d_hor[1].tag != sorted_extremelinepoint2d_hor[2].tag && sorted_extremelinepoint2d_hor[1].tag != sorted_extremelinepoint2d_hor[3].tag)
            {
                PolygonPrintf("error, sorted extremelinepoints format is not ture.\n");
                exit(-1);
            }
            possible_xbase = true;
        } // else is all false.
        // second judgement
        bool possible_ybase = false;
        if (sorted_extremelinepoint2d_vel[0].tag == sorted_extremelinepoint2d_vel[1].tag &&
            sorted_extremelinepoint2d_vel[0].monochainid == sorted_extremelinepoint2d_vel[1].monochainid)
        {
            if (sorted_extremelinepoint2d_vel[2].tag != sorted_extremelinepoint2d_vel[3].tag ||
                sorted_extremelinepoint2d_vel[2].monochainid != sorted_extremelinepoint2d_vel[3].monochainid)
            {
                PolygonPrintf("error, sorted extremelinepoints format is not ture.\n");
                exit(-1);
            }
            possible_ybase = false;
        }
        else if (sorted_extremelinepoint2d_vel[0].tag != sorted_extremelinepoint2d_vel[1].tag &&
                 sorted_extremelinepoint2d_vel[0].monochainid != sorted_extremelinepoint2d_vel[1].monochainid)
        {
            if (sorted_extremelinepoint2d_vel[0].tag != sorted_extremelinepoint2d_vel[2].tag && sorted_extremelinepoint2d_vel[0].tag != sorted_extremelinepoint2d_vel[3].tag)
            {
                PolygonPrintf("error, sorted extremelinepoints format is not ture.\n");
                exit(-1);
            }
            if (sorted_extremelinepoint2d_vel[1].tag != sorted_extremelinepoint2d_vel[2].tag && sorted_extremelinepoint2d_vel[1].tag != sorted_extremelinepoint2d_vel[3].tag)
            {
                PolygonPrintf("error, sorted extremelinepoints format is not ture.\n");
                exit(-1);
            }
            possible_ybase = true;
        }                                        // else is all false.
        return possible_xbase && possible_ybase; // 只有1 1 时才是可能有交点的.
    }

    template <class T>
    bool PolygonIntersecting<T>::PotentionIntersection(const std::vector<linepoint2d_t<T>> &sorted_extremelinepoint2d)
    {
        if (sorted_extremelinepoint2d.size() != 4)
        {
            PolygonPrintf("error, the sorted extremeline points is not equal with 4.\n");
            exit(-1);
        }
        if (sorted_extremelinepoint2d[0].monochainid != sorted_extremelinepoint2d[1].monochainid && sorted_extremelinepoint2d[0].tag != sorted_extremelinepoint2d[1].tag)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    /***
     * 
     * @brief 逆时针方向，作为遍历的方法，（是按照starpolygon创建时，所有extreme point is ordered by the unclockwise）
     * 
     * 
     * 
     * **/
    // when init cur_count == lastcount, 0 is smaller 1 is bigger
    template <class T>
    bool PolygonIntersecting<T>::IsSmallerorBiggerX(polygon::Polygon<T> *ptr, const bool &isclockwise, const int &last_count, int &cur_count)
    {
        if (isclockwise)
        {
            --cur_count;
            if (cur_count < 0)
                cur_count = ptr->SizeOfExtremePoints() - 1;
        }
        else
        {
            ++cur_count;
            if (cur_count == ptr->SizeOfExtremePoints())
                cur_count = 0;
        }
        if (ptr->GetExIndexX(cur_count) < ptr->GetExIndexX(last_count))
        {
            return false;
        }
        else if (ptr->GetExIndexX(cur_count) > ptr->GetExIndexX(last_count))
        {
            return true;
        }
        return IsSmallerorBiggerX(ptr, isclockwise, last_count, cur_count);
    }

    template <class T>
    bool PolygonIntersecting<T>::IsSmallerorBiggerXLock(polygon::Polygon<T> *ptr, const bool &isclockwise, const int &last_count, int &cur_count)
    {
        if (isclockwise)
        {
            --cur_count;
            if (cur_count < 0)
            {
                PolygonPrintf("error, cannot find result.\n");
                cur_count = -1;
                return false;
            }
        }
        else
        {
            ++cur_count;
            if (cur_count == (int)(ptr->SizeOfExtremePoints()))
            {
                PolygonPrintf("error, cannot find result.\n");
                cur_count = (int)(ptr->SizeOfExtremePoints());
                return false;
            }
        }
        if (ptr->GetExIndexX(cur_count) < ptr->GetExIndexX(last_count))
        {
            return false;
        }
        else if (ptr->GetExIndexX(cur_count) > ptr->GetExIndexX(last_count))
        {
            return true;
        }
        return IsSmallerorBiggerXLock(ptr, isclockwise, last_count, cur_count);
    }

    // when init cur_count == lastcount, 0 is smaller 1 is bigger
    template <class T>
    bool PolygonIntersecting<T>::IsSmallerorBiggerY(polygon::Polygon<T> *ptr, const bool &isclockwise, const int &last_count, int &cur_count)
    {
        if (isclockwise)
        {
            --cur_count;
            if (cur_count < 0)
                cur_count = ptr->SizeOfExtremePoints() - 1;
        }
        else
        {
            ++cur_count;
            if (cur_count == ptr->SizeOfExtremePoints())
                cur_count = 0;
        }
        if (ptr->GetExIndexY(cur_count) < ptr->GetExIndexY(last_count))
        {
            return false;
        }
        else if (ptr->GetExIndexY(cur_count) > ptr->GetExIndexY(last_count))
        {
            return true;
        }
        return IsSmallerorBiggerY(ptr, isclockwise, last_count, cur_count);
    }

    template <class T>
    bool PolygonIntersecting<T>::IsSmallerorBiggerYLock(polygon::Polygon<T> *ptr, const bool &isclockwise, const int &last_count, int &cur_count)
    {
        if (isclockwise)
        {
            --cur_count;
            if (cur_count < 0)
            {
                PolygonPrintf("error, cannot find result.\n");
                cur_count = -1;
                return false;
            }
        }
        else
        {
            ++cur_count;
            if (cur_count == (int)(ptr->SizeOfExtremePoints()))
            {
                PolygonPrintf("error, cannot find result.\n");
                cur_count = (int)(ptr->SizeOfExtremePoints());
                return false;
            }
        }
        // std::cout<<ptr->GetExIndexY(cur_count)<<", "<<ptr->GetExIndexY(last_count)<<std::endl;
        if (ptr->GetExIndexY(cur_count) < ptr->GetExIndexY(last_count))
        {
            return false;
        }
        else if (ptr->GetExIndexY(cur_count) > ptr->GetExIndexY(last_count))
        {
            return true;
        }
        PolygonPrintf("run in 2.\n");
        return IsSmallerorBiggerYLock(ptr, isclockwise, last_count, cur_count);
    }

    template <class T>
    void PolygonIntersecting<T>::SplitPolygon2MonotoneChain()
    {
        leftfirst_monochain_xbase_.points_chain.clear();
        leftfirst_monochain_ybase_.points_chain.clear();
        rightfirst_monochain_xbase_.points_chain.clear();
        rightfirst_monochain_ybase_.points_chain.clear();

        // use the extreme edges to split not the extreme poitns !!!!
        std::vector<extreme_edge_t<T>> exedges0;
        polygon_firstptr_->GetExtremeEdges(exedges0);
        std::vector<point2d_t<T> *> expoints_first;

        expoints_first.emplace_back(exedges0[0].p_start);
        expoints_first.emplace_back(exedges0[0].p_end);
        // PolygonPrintf("exedges0 %d (%lf %f) - (%lf %lf).\n", 0, exedges0[0].p_start->x, exedges0[0].p_start->y, exedges0[0].p_end->x, exedges0[0].p_end->y);
        for (int i = 1; i < exedges0.size() - 1; ++i)
        {
            // PolygonPrintf("exedges0 %d (%lf %f) - (%lf %lf).\n", i, exedges0[i].p_start->x, exedges0[i].p_start->y, exedges0[i].p_end->x, exedges0[i].p_end->y);
            if (!expoints_first[i]->Equal(exedges0[i].p_start))
            {
                PolygonPrintf("edge cannot match with points order.\n");
                exit(-1);
            }
            expoints_first.emplace_back(exedges0[i].p_end);
        }
        /***=============           first start X major           =================**/
        point2d_t<T> *minx0_expoint = nullptr, *maxx0_expoint = nullptr, *miny0_expoint = nullptr, *maxy0_expoint = nullptr;
        size_t minx0_index, maxx0_index, miny0_index, maxy0_index;
        size_t start_idx = 0, end_idx = expoints_first.size() - 1, endthreshold = expoints_first.size(); // 永远在数值上都是差一步的距离 start_idx - 1 = end_idx
        // is unclockwise to visited these points
        bool findminfirst = false;
        int firstDiff_Index = 0;
        if (expoints_first[1]->x < expoints_first[0]->x)
        {
            findminfirst = true;
        }
        else if (expoints_first[1]->x > expoints_first[0]->x)
        { //-----1
            findminfirst = false;
        }
        else
        {
            bool issmallerorbigger = IsSmallerorBiggerXLock(polygon_firstptr_, false, 0, firstDiff_Index);
            if (issmallerorbigger)
            {
                findminfirst = false;
            }
            else
            {
                findminfirst = true;
            }
        }
        if (firstDiff_Index == 0)
        {
            firstDiff_Index = 1;
        }
        ++firstDiff_Index;
        if (firstDiff_Index == endthreshold)
        {
            printf("run in firstDiff_index == endthreshold.\n");
            // minfirst 是第一次找到的是minx， 然后从min->max, 那么优先的是right monochain
            if(findminfirst){
                index_rightfirst_inrightfirstchain_xbase_ = 0;
                index_leftfirst_inleftfirstchain_xbase_ = firstDiff_Index-2;
            }
            else{
                index_leftfirst_inleftfirstchain_xbase_ = 0;
                index_rightfirst_inrightfirstchain_xbase_ = firstDiff_Index-2;
            }
            for (int i = 0; i < firstDiff_Index - 1; ++i)
            {
                if (findminfirst)
                {
                    rightfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[i]);
                }
                else
                {
                    leftfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[i]);
                }
            }
            if (findminfirst)
            {
                leftfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[firstDiff_Index - 2]);
                leftfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[firstDiff_Index - 1]);
                leftfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[0]);
            }
            else
            {
                rightfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[firstDiff_Index - 2]);
                rightfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[firstDiff_Index - 1]);
                rightfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[0]);
            }
        }

        // 定理： 一个循环有序队列中，最大值和最小值的index 不可能都在端点, 除非一条直线的情况加一个点的情况
        for (int i = 0; i < expoints_first.size(); ++i)
        {
            PolygonPrintf("export first %d is %lf %lf.\n", i, expoints_first[i]->x, expoints_first[i]->y);
        }
        for (int i = firstDiff_Index; i < endthreshold; ++i)
        { //must can be find minest or maxest x points, but should
            if (findminfirst)
            {
                if (expoints_first[i]->x > expoints_first[i - 1]->x)
                {
                    // 当出现第一个小于的值时，找到最大值。接下来，将所当前最大值和之后所有递减的push into leftfirst_monochain
                    int temp = i - 2;
                    if (temp < 0)
                    {
                        temp = endthreshold - 1;
                    }
                    PolygonPrintf("minx0_expoint %d-1 is %lf %lf.\n", i - 1, expoints_first[temp]->x, expoints_first[temp]->y);
                    PolygonPrintf("minx0_expoint %d is %lf %lf.\n", i - 1, expoints_first[i - 1]->x, expoints_first[i - 1]->y);
                    PolygonPrintf("minx0_expoint %d+1 is %lf %lf.\n", i - 1, expoints_first[i]->x, expoints_first[i]->y);
                    
                    minx0_index = i - 1;
                    minx0_expoint = expoints_first[minx0_index];

                    index_rightfirst_inrightfirstchain_xbase_ = minx0_index;

                    int count = minx0_index;
                    int next_count = count + 1;
                    if (next_count == endthreshold)
                        next_count = 0;
                    while (expoints_first[next_count]->x >= expoints_first[count]->x)
                    {
                        rightfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[count]);
                        ++count;
                        if (count == endthreshold)
                            count = 0;
                        next_count = count + 1;
                        if (next_count == endthreshold)
                            next_count = 0;
                    }
                    
                    // 把min 也放进来
                    rightfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[count]);
                    leftfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[count]);

                    index_leftfirst_inleftfirstchain_xbase_ = count;

#if debug
                    temp = count - 1;
                    if (temp < 0)
                    {
                        temp = endthreshold - 1;
                    }
                    int temp1 = count + 1;
                    if (temp1 >= endthreshold)
                    {
                        temp1 = 0;
                    }
                    PolygonPrintf("maxx0_expoint %d-1 is %lf %lf.\n", count, expoints_first[temp]->x, expoints_first[temp]->y);
                    PolygonPrintf("maxx0_expoint %d is %lf %lf.\n", count, expoints_first[count]->x, expoints_first[count]->y);
                    PolygonPrintf("maxx0_expoint %d+1 is %lf %lf.\n", count, expoints_first[temp1]->x, expoints_first[temp1]->y);
#endif
                    // maxx0_index = next_count;
                    maxx0_index = count;
                    maxx0_expoint = expoints_first[maxx0_index];
                    count = maxx0_index + 1;
                    if (count == endthreshold)
                    {
                        count = 0;
                    }
                    next_count = minx0_index;
                    while (count != next_count)
                    {
                        leftfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[count]);
                        ++count;
                        if (count == endthreshold)
                            count = 0;
                    }
                    leftfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[count]);
                    break;
                }
            }
            else
            {
                // find max
                if (expoints_first[i]->x < expoints_first[i - 1]->x)
                {
                    // 找到最小值，接下来，把所有当前最小值之后递增的pushinto rightfirst_monochain
                    maxx0_index = i - 1;
                    maxx0_expoint = expoints_first[maxx0_index];

                    index_leftfirst_inleftfirstchain_xbase_ = maxx0_index;

                    int count = maxx0_index;
                    int next_count = count + 1;
                    if (next_count == endthreshold)
                        next_count = 0;
                    while (expoints_first[next_count]->x <= expoints_first[count]->x)
                    {
                        leftfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[count]);
                        ++count;
                        if (count == endthreshold)
                            count = 0;
                        next_count = count + 1;
                        if (next_count == endthreshold)
                            next_count = 0;
                    }
                    leftfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[count]);
                    rightfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[count]);

                    index_rightfirst_inrightfirstchain_xbase_ = count;

                    // minx0_index = next_count;
                    minx0_index = count;
                    minx0_expoint = expoints_first[minx0_index];
                    count = minx0_index + 1;
                    if (count == endthreshold)
                    {
                        count = 0;
                    }
                    next_count = maxx0_index;
                    while (count != next_count)
                    {
                        rightfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[count]);
                        ++count;
                        if (count == endthreshold)
                            count = 0;
                    }
                    rightfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[count]);
                    break;
                }
            }
            if (i == endthreshold - 1)
            {
                // 最后一个结果还未break， 则说明第一个是minx， the last is maxx
                index_leftfirst_inleftfirstchain_xbase_ = endthreshold -1;
                index_rightfirst_inrightfirstchain_xbase_ = 0;

                rightfirst_monochain_xbase_.points_chain.assign(expoints_first.begin(), expoints_first.end());
                leftfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[endthreshold - 1]);
                leftfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[0]);
                break;
            }
        }

        //***------------          first start Y major            -----------------**/
        findminfirst = false;
        firstDiff_Index = 0;
        if (expoints_first[1]->y < expoints_first[0]->y)
        {
            findminfirst = true;
        }
        else if (expoints_first[1]->y > expoints_first[0]->y)
        { //-----1
            findminfirst = false;
        }
        else
        {
            bool issmallerorbigger = IsSmallerorBiggerYLock(polygon_firstptr_, false, 0, firstDiff_Index);
            if (issmallerorbigger)
            {
                findminfirst = false;
            }
            else
            {
                findminfirst = true;
            }
        }
        if (firstDiff_Index == 0)
        {
            firstDiff_Index = 1;
        }
        ++firstDiff_Index;
        if (firstDiff_Index == endthreshold)
        {
            PolygonPrintf("run in here firstDiff_index = endthreshold.\n");
            // minfirst 是第一次找到的是minx， 然后从min->max, 那么优先的是right monochain
            if(findminfirst){
                index_rightfirst_inrightfirstchain_ybase_ = 0;
                index_leftfirst_inleftfirstchain_ybase_ = firstDiff_Index-2;
            }
            else{
                index_leftfirst_inleftfirstchain_ybase_ = 0;
                index_rightfirst_inrightfirstchain_ybase_ = firstDiff_Index-2;
            }
            for (int i = 0; i < firstDiff_Index - 1; ++i)
            {
                if (findminfirst)
                {
                    rightfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[i]);
                }
                else
                {
                    leftfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[i]);
                }
            }
            if (findminfirst)
            {
                leftfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[firstDiff_Index - 2]);
                leftfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[firstDiff_Index - 1]);
                leftfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[0]);
            }
            else
            {
                rightfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[firstDiff_Index - 2]);
                rightfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[firstDiff_Index - 1]);
                rightfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[0]);
            }
        }
        // 定理： 一个循环有序队列中，最大值和最小值的index 不可能都在端点, 除非一条直线的情况加一个点的情况
        /**
         * 
         * @brief 问题描述：当出现start 和 end 是极小值或者极大值的时候，没有push 任何记过进入segments
         *        会出现问题，是xbase 和 ybase 都会存在的问题！！！！
         * 
         * **/ 
        for (int i = firstDiff_Index; i < endthreshold; ++i)
        { //must can be find minest or maxest x points, but should
            PolygonPrintf("run in here in for.\n");
            if (findminfirst)
            {
                PolygonPrintf("run in findmindyfirst.\n");
                if (expoints_first[i]->y > expoints_first[i - 1]->y)
                {
                    // 当出现第一个小于的值时，找到最大值。接下来，将所当前最大值和之后所有递减的push into leftfirst_monochain
                    miny0_index = i - 1;
                    miny0_expoint = expoints_first[miny0_index];
                    
                    index_rightfirst_inrightfirstchain_ybase_ = miny0_index;
                    
                    int count = miny0_index;
                    int next_count = count + 1;
                    if (next_count == endthreshold)
                        next_count = 0;
                    while (expoints_first[next_count]->y >= expoints_first[count]->y)
                    {
                        rightfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[count]);
                        ++count;
                        if (count == endthreshold)
                            count = 0;
                        next_count = count + 1;
                        if (next_count == endthreshold)
                            next_count = 0;
                    }
                    // 把min 也放进来
                    rightfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[count]);
                    leftfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[count]);

                    index_leftfirst_inleftfirstchain_ybase_ = count;

                    // maxy0_index = next_count;
                    maxy0_index = count;
                    maxy0_expoint = expoints_first[maxy0_index];
                    count = maxy0_index + 1;
                    if (count == endthreshold)
                    {
                        count = 0;
                    }
                    next_count = miny0_index;
                    while (count != next_count)
                    {
                        leftfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[count]);
                        ++count;
                        if (count == endthreshold)
                            count = 0;
                    }
                    leftfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[count]);
                    break;
                }
            }
            else
            {
                // find max
                PolygonPrintf("run in findmaxdyfirst.\n");
                PolygonPrintf("expoints i(%d) and i-1(%d) is %lf %lf.\n", i, i-1, expoints_first[i]->y, expoints_first[i-1]->y);
                if (expoints_first[i]->y < expoints_first[i - 1]->y)
                {
                    // 找到最小值，接下来，把所有当前最小值之后递增的pushinto rightfirst_monochain
                    maxy0_index = i - 1;
                    maxy0_expoint = expoints_first[maxy0_index];

                    index_leftfirst_inleftfirstchain_ybase_ = maxy0_index;

                    int count = maxy0_index;
                    int next_count = count + 1;
                    if (next_count == endthreshold)
                        next_count = 0;
                    while (expoints_first[next_count]->y <= expoints_first[count]->y)
                    {
                        PolygonPrintf("run in push data into leftfirst ybase.\n");
                        leftfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[count]);
                        ++count;
                        if (count == endthreshold)
                            count = 0;
                        next_count = count + 1;
                        if (next_count == endthreshold)
                            next_count = 0;
                    }
                    leftfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[count]);
                    rightfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[count]);
                    
                    index_rightfirst_inrightfirstchain_ybase_ = count;

                    // miny0_index = next_count;
                    miny0_index = count;
                    miny0_expoint = expoints_first[miny0_index];
                    count = miny0_index + 1;
                    if (count == endthreshold)
                    {
                        count = 0;
                    }
                    next_count = maxy0_index;
                    while (count != next_count)
                    {
                        rightfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[count]);
                        ++count;
                        if (count == endthreshold)
                            count = 0;
                    }
                    rightfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[count]);
                    break;
                }
            }
            if (i == endthreshold - 1)
            {
                // 最后一个结果还未break， 则说明第一个是miny， the last is maxy
                index_leftfirst_inleftfirstchain_ybase_ = endthreshold-1;
                index_rightfirst_inrightfirstchain_ybase_ = 0;
                rightfirst_monochain_ybase_.points_chain.assign(expoints_first.begin(), expoints_first.end());
                leftfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[endthreshold - 1]);
                leftfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[0]);
                break;
            }
        }

        // ##########################################################################

        /***=============          second start X major           =================**/
        std::vector<extreme_edge_t<T>> exedges1;
        polygon_secondptr_->GetExtremeEdges(exedges1);
        std::vector<point2d_t<T> *> expoints_second;
        expoints_second.emplace_back(exedges1[0].p_start);
        expoints_second.emplace_back(exedges1[0].p_end);
        // PolygonPrintf("exedges1 %d (%lf %f) - (%lf %lf).\n", 0, exedges1[0].p_start->x, exedges1[0].p_start->y, exedges1[0].p_end->x, exedges1[0].p_end->y);
        for (int i = 1; i < exedges1.size() - 1; ++i)
        {
            // PolygonPrintf("exedges1 %d (%lf %f) - (%lf %lf).\n", i, exedges1[i].p_start->x, exedges1[i].p_start->y, exedges1[i].p_end->x, exedges1[i].p_end->y);
            if (!expoints_second[i]->Equal(exedges1[i].p_start))
            {
                PolygonPrintf("edge cannot match with points order.\n");
                exit(-1);
            }
            expoints_second.emplace_back(exedges1[i].p_end);
        }
        /***=============           first start X major           =================**/
        point2d_t<T> *minx1_expoint = nullptr, *maxx1_expoint = nullptr, *miny1_expoint = nullptr, *maxy1_expoint = nullptr;
        int minx1_index, maxx1_index, miny1_index, maxy1_index;
        start_idx = 0, end_idx = expoints_second.size() - 1, endthreshold = expoints_second.size(); // 永远在数值上都是差一步的距离 start_idx - 1 = end_idx
        // is unclockwise to visited these points
        findminfirst = false;
        firstDiff_Index = 0;
        if (expoints_second[1]->x < expoints_second[0]->x)
        {
            findminfirst = true;
        }
        else if (expoints_second[1]->x > expoints_second[0]->x)
        { //-----1
            findminfirst = false;
        }
        else
        {
            bool issmallerorbigger = IsSmallerorBiggerYLock(polygon_firstptr_, false, 0, firstDiff_Index);
            if (issmallerorbigger)
            {
                findminfirst = false;
            }
            else
            {
                findminfirst = true;
            }
        }
        if (firstDiff_Index == 0)
        {
            firstDiff_Index = 1;
        }
        ++firstDiff_Index;
        if (firstDiff_Index == endthreshold)
        {
            // minfirst 是第一次找到的是minx， 然后从min->max, 那么优先的是right monochain
            printf("run in firstDiff_index == endthreshold.\n");
            if(findminfirst){
                index_rightfirst_inrightsecondchain_xbase_ = 0;
                index_leftfirst_inleftsecondchain_xbase_ = firstDiff_Index-2;
            }
            else{
                index_leftfirst_inleftsecondchain_xbase_ = 0;
                index_rightfirst_inrightsecondchain_xbase_ = firstDiff_Index-2;
            }
            for (int i = 0; i < firstDiff_Index - 1; ++i)
            {
                if (findminfirst)
                {
                    rightsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[i]);
                }
                else
                {
                    leftsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[i]);
                }
            }
            if (findminfirst)
            {
                leftsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[firstDiff_Index - 2]);
                leftsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[firstDiff_Index - 1]);
                leftsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[0]);
            }
            else
            {
                rightsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[firstDiff_Index - 2]);
                rightsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[firstDiff_Index - 1]);
                rightsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[0]);
            }
        }
        // 定理： 一个循环有序队列中，最大值和最小值的index 不可能都在端点, 除非一条直线的情况加一个点的情况
        for (int i = firstDiff_Index; i < endthreshold; ++i)
        { //must can be find minest or maxest x points, but should
            printf("run in firstDiff for. %d.\n", firstDiff_Index);
            if (findminfirst)
            {
                printf("run in findminfirst.\n");
                if (expoints_second[i]->x > expoints_second[i - 1]->x)
                {
                    // 当出现第一个小于的值时，找到最大值。接下来，将所当前最大值和之后所有递减的push into leftsecond_monochain
                    minx1_index = i - 1;
                    minx1_expoint = expoints_second[minx1_index];

                    index_rightfirst_inrightsecondchain_xbase_ = minx1_index;

                    int count = minx1_index;
                    int next_count = count + 1;
                    if (next_count == endthreshold)
                        next_count = 0;
                    while (expoints_second[next_count]->x >= expoints_second[count]->x)
                    {
                        rightsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[count]);
                        ++count;
                        if (count == endthreshold)
                            count = 0;
                        next_count = count + 1;
                        if (next_count == endthreshold)
                            next_count = 0;
                    }
                    // 把min 也放进来
                    rightsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[count]);
                    leftsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[count]);

                    index_leftfirst_inleftsecondchain_xbase_ = count;

                    // maxx1_index = next_count;
                    maxx1_index = count;
                    maxx1_expoint = expoints_second[maxx1_index];
                    count = maxx1_index + 1;
                    if (count == endthreshold)
                        count = 0;
                    next_count = minx1_index;
                    while (count != next_count)
                    {
                        leftsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[count]);
                        ++count;
                        if (count == endthreshold)
                            count = 0;
                    }
                    leftsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[count]);
                    break;
                }
            }
            else
            {
                // find max
                printf("run in findmaxfirst.\n");
                if (expoints_second[i]->x < expoints_second[i - 1]->x)
                {
                    // 找到最小值，接下来，把所有当前最小值之后递增的pushinto rightsecond_monochain
                    printf(" --- i is %d.\n", i);
                    maxx1_index = i - 1;
                    maxx1_expoint = expoints_second[maxx1_index];

                    index_leftfirst_inleftsecondchain_xbase_ = maxx1_index;


                    int count = maxx1_index;
                    int next_count = count + 1;
                    if (next_count == endthreshold)
                        next_count = 0;
                    while (expoints_second[next_count]->x <= expoints_second[count]->x)
                    {
                        leftsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[count]);
                        ++count;
                        if (count == endthreshold)
                            count = 0;
                        next_count = count + 1;
                        if (next_count == endthreshold)
                            next_count = 0;
                    }
                    leftsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[count]);
                    rightsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[count]);

                    index_rightfirst_inrightsecondchain_xbase_ = count;

                    minx1_index = count;
                    minx1_expoint = expoints_second[minx1_index];
                    count = minx1_index + 1;
                    if (count == endthreshold)
                    {
                        count = 0;
                    }
                    next_count = maxx1_index;
                    while (count != next_count)
                    {
                        rightsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[count]);
                        ++count;
                        if (count == endthreshold)
                            count = 0;
                    }
                    rightsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[count]);
                    break;
                }
            }
            if (i == endthreshold - 1)
            {
                // 最后一个结果还未break， 则说明第一个是miny， the last is maxy
                index_leftfirst_inleftsecondchain_xbase_ = endthreshold-1;
                index_rightfirst_inrightsecondchain_xbase_ = 0;
                rightsecond_monochain_xbase_.points_chain.assign(expoints_second.begin(), expoints_second.end());
                leftsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[endthreshold - 1]);
                leftsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[0]);
                break;
            }
        }

        //***------------          first start Y major            -----------------**/
        findminfirst = false;
        firstDiff_Index = 0;
        if (expoints_second[1]->y < expoints_second[0]->y)
        {
            findminfirst = true;
        }
        else if (expoints_second[1]->y > expoints_second[0]->y)
        { //-----1
            findminfirst = false;
        }
        else
        {
            bool issmallerorbigger = IsSmallerorBiggerYLock(polygon_firstptr_, false, 0, firstDiff_Index);
            if (issmallerorbigger)
            {
                findminfirst = false;
            }
            else
            {
                findminfirst = true;
            }
        }
        if (firstDiff_Index == 0)
        {
            firstDiff_Index = 1;
        }
        ++firstDiff_Index;
        if (firstDiff_Index == endthreshold)
        {
            PolygonPrintf("run in here, firstDiff_index == endthreshold.\n");
            // minfirst 是第一次找到的是minx， 然后从min->max, 那么优先的是right monochain
            if(findminfirst){
                index_rightfirst_inrightsecondchain_ybase_ = 0;
                index_leftfirst_inleftsecondchain_ybase_ = firstDiff_Index-2;
            }
            else{
                index_leftfirst_inleftsecondchain_ybase_ = 0;
                index_rightfirst_inrightsecondchain_ybase_ = firstDiff_Index-2;
            }
            for (int i = 0; i < firstDiff_Index - 1; ++i)
            {
                if (findminfirst)
                {
                    rightsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[i]);
                }
                else
                {
                    leftsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[i]);
                }
            }
            if (findminfirst)
            {
                leftsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[firstDiff_Index - 2]);
                leftsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[firstDiff_Index - 1]);
                leftsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[0]);
            }
            else
            {
                rightsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[firstDiff_Index - 2]);
                rightsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[firstDiff_Index - 1]);
                rightsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[0]);
            }
        }
        // 定理： 一个循环有序队列中，最大值和最小值的index 不可能都在端点, 除非一条直线的情况加一个点的情况
        for (int i = firstDiff_Index; i < endthreshold; ++i)
        { //must can be find minest or maxest x points, but should
            PolygonPrintf("run in here in for.\n");
            if (findminfirst)
            {
                if (expoints_second[i]->y > expoints_second[i - 1]->y)
                {
                    // 当出现第一个小于的值时，找到最大值。接下来，将所当前最大值和之后所有递减的push into leftsecond_monochain
                    miny1_index = i - 1;
                    miny1_expoint = expoints_second[miny1_index];

                    index_rightfirst_inrightsecondchain_ybase_ = miny1_index;

                    int count = miny1_index;
                    int next_count = count + 1;
                    if (next_count == endthreshold)
                    {
                        next_count = 0;
                    }
                    while (expoints_second[next_count]->y >= expoints_second[count]->y)
                    {
                        rightsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[count]);
                        ++count;
                        if (count == endthreshold)
                            count = 0;
                        next_count = count + 1;
                        if (next_count == endthreshold)
                            next_count = 0;
                    }
                    // 把min 也放进来
                    rightsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[count]);
                    leftsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[count]);

                    index_leftfirst_inleftsecondchain_ybase_ = count;

                    // maxy1_index = next_count;
                    maxy1_index = count;
                    maxy1_expoint = expoints_second[maxy1_index];
                    count = maxy1_index + 1;
                    if (count == endthreshold)
                    {
                        count = 0;
                    }
                    next_count = miny1_index;
                    while (count != next_count)
                    {
                        leftsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[count]);
                        ++count;
                        if (count == endthreshold)
                            count = 0;
                    }
                    leftsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[count]);
                    break;
                }
            }
            else
            {
                // find max
                if (expoints_second[i]->y < expoints_second[i - 1]->y)
                {
                    // 找到最小值，接下来，把所有当前最小值之后递增的pushinto rightsecond_monochain
                    maxy1_index = i - 1;
                    maxy1_expoint = expoints_second[maxy1_index];

                    index_leftfirst_inleftsecondchain_ybase_ = maxy1_index;


                    int count = maxy1_index;
                    int next_count = count + 1;

                    if (next_count == endthreshold)
                    {
                        next_count = 0;
                    }
                    while (expoints_second[next_count]->y <= expoints_second[count]->y)
                    {
                        leftsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[count]);
                        ++count;
                        if (count == endthreshold)
                            count = 0;
                        next_count = count + 1;
                        if (next_count == endthreshold)
                            next_count = 0;
                    }
                    leftsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[count]);
                    rightsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[count]);

                    index_rightfirst_inrightsecondchain_ybase_ = count;

                    // miny1_index = next_count;
                    miny1_index = count;
                    miny1_expoint = expoints_second[miny1_index];
                    count = miny1_index + 1;
                    if (count == endthreshold)
                    {
                        count = 0;
                    }
                    next_count = maxy1_index;
                    while (count != next_count)
                    {
                        rightsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[count]);
                        ++count;
                        if (count == endthreshold)
                            count = 0;
                    }
                    rightsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[count]);
                    break;
                }
            }
            if (i == endthreshold - 1)
            {
                index_leftfirst_inleftsecondchain_ybase_ = endthreshold-1;
                index_rightfirst_inrightsecondchain_ybase_ = 0;
                rightsecond_monochain_ybase_.points_chain.assign(expoints_second.begin(), expoints_second.end());
                leftsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[endthreshold - 1]);
                leftsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[0]);
                break;
            }
        }
        printf("index %d %d %d %d.\n", index_leftfirst_inleftfirstchain_xbase_, index_leftfirst_inleftsecondchain_xbase_, index_rightfirst_inrightfirstchain_xbase_, index_rightfirst_inrightsecondchain_xbase_);
        PolygonPrintf("polygon xbase extreme points size is %d, leftchain size is %d, rightchain size is %d.\n", (int)polygon_firstptr_->SizeOfExtremePoints(), (int)leftfirst_monochain_xbase_.points_chain.size(), (int)rightfirst_monochain_xbase_.points_chain.size());
        PolygonPrintf("polygon ybase extreme points size is %d, leftchain size is %d, rightchain size is %d.\n", (int)polygon_firstptr_->SizeOfExtremePoints(), (int)leftfirst_monochain_ybase_.points_chain.size(), (int)rightfirst_monochain_ybase_.points_chain.size());
        PolygonPrintf("polygon xbase second extreme points size is %d, leftchain size is %d, rightchain size is %d.\n", (int)polygon_secondptr_->SizeOfExtremePoints(), (int)leftsecond_monochain_xbase_.points_chain.size(), (int)rightsecond_monochain_xbase_.points_chain.size());
        PolygonPrintf("polygon ybase second extreme points size is %d, leftchain size is %d, rightchain size is %d.\n", (int)polygon_secondptr_->SizeOfExtremePoints(), (int)leftsecond_monochain_ybase_.points_chain.size(), (int)rightsecond_monochain_ybase_.points_chain.size());
    }

    template <class T>
    void PolygonIntersecting<T>::GetFirstMonoChain(std::vector<point2d_t<T> *> &leftchain, std::vector<point2d_t<T> *> &rightchain)
    {
        leftchain.clear();
        rightchain.clear();
        if (xory_major_)
        {
            leftchain.assign(leftfirst_monochain_ybase_.points_chain.begin(), leftfirst_monochain_ybase_.points_chain.end());
            rightchain.assign(rightfirst_monochain_ybase_.points_chain.begin(), rightfirst_monochain_ybase_.points_chain.end());
        }
        else
        {
            leftchain.assign(leftfirst_monochain_xbase_.points_chain.begin(), leftfirst_monochain_xbase_.points_chain.end());
            rightchain.assign(rightfirst_monochain_xbase_.points_chain.begin(), rightfirst_monochain_xbase_.points_chain.end());
        }
    }

    template <class T>
    void PolygonIntersecting<T>::GetFirstMonoChainXbase(std::vector<point2d_t<T> *> &leftchain, std::vector<point2d_t<T> *> &rightchain)
    {
        leftchain.clear();
        rightchain.clear();
        leftchain.assign(leftfirst_monochain_xbase_.points_chain.begin(), leftfirst_monochain_xbase_.points_chain.end());
        rightchain.assign(rightfirst_monochain_xbase_.points_chain.begin(), rightfirst_monochain_xbase_.points_chain.end());
    }

    template <class T>
    void PolygonIntersecting<T>::GetSecondMonoChainXbase(std::vector<point2d_t<T> *> &leftchain, std::vector<point2d_t<T> *> &rightchain)
    {
        leftchain.clear();
        rightchain.clear();
        leftchain.assign(leftsecond_monochain_xbase_.points_chain.begin(), leftsecond_monochain_xbase_.points_chain.end());
        rightchain.assign(rightsecond_monochain_xbase_.points_chain.begin(), rightsecond_monochain_xbase_.points_chain.end());
    }

    template <class T>
    void PolygonIntersecting<T>::GetFirstMonoChainYbase(std::vector<point2d_t<T> *> &leftchain, std::vector<point2d_t<T> *> &rightchain)
    {
        leftchain.clear();
        rightchain.clear();
        leftchain.assign(leftfirst_monochain_ybase_.points_chain.begin(), leftfirst_monochain_ybase_.points_chain.end());
        rightchain.assign(rightfirst_monochain_ybase_.points_chain.begin(), rightfirst_monochain_ybase_.points_chain.end());
    }

    template <class T>
    void PolygonIntersecting<T>::GetSecondMonoChainYbase(std::vector<point2d_t<T> *> &leftchain, std::vector<point2d_t<T> *> &rightchain)
    {
        leftchain.clear();
        rightchain.clear();
        leftchain.assign(leftsecond_monochain_ybase_.points_chain.begin(), leftsecond_monochain_ybase_.points_chain.end());
        rightchain.assign(rightsecond_monochain_ybase_.points_chain.begin(), rightsecond_monochain_ybase_.points_chain.end());
    }

    template <class T>
    bool PolygonIntersecting<T>::HasIntersection(const bool &usexorybase)
    {
        // 自己选择 // 0 x  12 y
        OrderLineEndPoints();
        potention_intersection_tags_.clear();
        bool possible = false;
        if (usexorybase)
        {
            possible = HasIntersection_Ybase();
        }
        else
        {
            possible = HasIntersection_Xbase();
        }
        return possible;
    }

    template <class T>
    bool PolygonIntersecting<T>::HasIntersection_Xbase()
    {
        if (sorted_extremelinepoint2d_xbase_hor_.size() != 4 || (sorted_extremelinepoint2d_xbase_vel00_.size() != 4 && sorted_extremelinepoint2d_xbase_vel01_.size() != 4 && sorted_extremelinepoint2d_xbase_vel10_.size() != 4 && sorted_extremelinepoint2d_xbase_vel11_.size() != 4))
        {
            PolygonPrintf("error, hasn't OrderLinePoints.\n");
            exit(-1);
        }
        // for (auto pxbase_hor : sorted_extremelinepoint2d_xbase_hor_)
        // {
        //     std::cout<<"sorted: "<<pxbase_hor.point2dptr->x<<", "<<pxbase_hor.point2dptr->y<<", tag "<<pxbase_hor.tag<<", monochainid "<<pxbase_hor.monochainid<<std::endl;
        // }
        bool possible = false;
        if (PotentionIntersection(sorted_extremelinepoint2d_xbase_hor_))
        {
            // just need one possible that canbe has intersection.
            bool flag = false;
            if (PotentionIntersection(sorted_extremelinepoint2d_xbase_vel00_))
            {
                PolygonPrintf("check in xbase 000.\n");
                potention_intersection_tags_.emplace_back("000");
                flag = true;
            }
            if (PotentionIntersection(sorted_extremelinepoint2d_xbase_vel01_))
            {
                PolygonPrintf("check in xbase 001.\n");
                potention_intersection_tags_.emplace_back("001");
                flag = true;
            }
            if (PotentionIntersection(sorted_extremelinepoint2d_xbase_vel10_))
            {
                PolygonPrintf("check in xbase 010.\n");
                potention_intersection_tags_.emplace_back("010");
                flag = true;
            }
            if (PotentionIntersection(sorted_extremelinepoint2d_xbase_vel11_))
            {
                PolygonPrintf("check in xbase 011.\n");
                potention_intersection_tags_.emplace_back("011");
                flag = true;
            }
            if(flag)
                possible = flag;
        }
        return possible;
    }

    template <class T>
    bool PolygonIntersecting<T>::HasIntersection_Ybase()
    {
        if ((sorted_extremelinepoint2d_ybase_hor00_.size() != 4 && sorted_extremelinepoint2d_ybase_hor01_.size() != 4 && sorted_extremelinepoint2d_ybase_hor10_.size() != 4 &&
             sorted_extremelinepoint2d_ybase_hor11_.size() != 4) ||
            sorted_extermelinepoint2d_ybase_vel_.size() != 4)
        {
            PolygonPrintf("error, hasn't OrderLinePoints.\n");
            exit(-1);
        }
        bool possible = false;
        if (PotentionIntersection(sorted_extermelinepoint2d_ybase_vel_))
        {
            bool flag = false;
            if (PotentionIntersection(sorted_extremelinepoint2d_ybase_hor00_))
            {
                potention_intersection_tags_.emplace_back("100");
                flag = true;
            }
            if (PotentionIntersection(sorted_extremelinepoint2d_ybase_hor01_))
            {
                potention_intersection_tags_.emplace_back("101");
                flag = true;
            }
            if (PotentionIntersection(sorted_extremelinepoint2d_ybase_hor10_))
            {
                potention_intersection_tags_.emplace_back("110");
                flag = true;
            }
            if (PotentionIntersection(sorted_extremelinepoint2d_ybase_hor11_))
            {
                potention_intersection_tags_.emplace_back("111");
                flag = true;
            }
            if(flag)
                possible = flag;
        }
        return possible;
    }

    template <class T>
    void PolygonIntersecting<T>::CalIntersectionBetweenTwoConvexPolygon()
    {
        if (potention_intersection_tags_.empty())
        {
            PolygonPrintf("potention intersection is empty. cannot cal the ConvexPolygon.\n");
            return;
        }

        for (auto tag : potention_intersection_tags_)
        {
            PolygonPrintf("potention intersectin tag is %s.\n", tag.c_str());
            if (tag == "000")
            {
                /* code */
                PolygonPrintf("run at %s.\n", tag.c_str());
                printf("000, start index is %d %d.\n", index_leftfirst_inleftfirstchain_xbase_, index_leftfirst_inleftsecondchain_xbase_);
                CalIntersectionBetweenTwoMonochainLine(leftfirst_monochain_xbase_, leftsecond_monochain_xbase_, index_leftfirst_inleftfirstchain_xbase_, index_leftfirst_inleftsecondchain_xbase_);
            }
            if (tag == "001")
            {
                /* code */
                PolygonPrintf("run at %s.\n", tag.c_str());
                printf("001, start index is %d %d.\n", index_leftfirst_inleftfirstchain_xbase_, index_rightfirst_inrightsecondchain_xbase_);
                CalIntersectionBetweenTwoMonochainLine(leftfirst_monochain_xbase_, rightsecond_monochain_xbase_, index_leftfirst_inleftfirstchain_xbase_, index_rightfirst_inrightsecondchain_xbase_);
            }
            if (tag == "010")
            {
                /* code */
                PolygonPrintf("run at %s.\n", tag.c_str());
                printf("010, start index is %d %d.\n", index_rightfirst_inrightfirstchain_xbase_, index_leftfirst_inleftsecondchain_xbase_);
                CalIntersectionBetweenTwoMonochainLine(rightfirst_monochain_xbase_, leftsecond_monochain_xbase_, index_rightfirst_inrightfirstchain_xbase_, index_leftfirst_inleftsecondchain_xbase_);
            }
            if (tag == "011")
            {
                /* code */
                PolygonPrintf("run at %s.\n", tag.c_str());
                printf("011, start index is %d %d.\n", index_rightfirst_inrightfirstchain_xbase_, index_rightfirst_inrightsecondchain_xbase_);
                CalIntersectionBetweenTwoMonochainLine(rightfirst_monochain_xbase_, rightsecond_monochain_xbase_, index_rightfirst_inrightfirstchain_xbase_, index_rightfirst_inrightsecondchain_xbase_);
            }
            if (tag == "100")
            {
                /* code */
                PolygonPrintf("run at %s.\n", tag.c_str());
                CalIntersectionBetweenTwoMonochainLine(leftfirst_monochain_ybase_, leftsecond_monochain_ybase_, index_leftfirst_inleftfirstchain_ybase_, index_leftfirst_inleftsecondchain_ybase_);
            }
            if (tag == "101")
            {
                /* code */
                PolygonPrintf("run at %s.\n", tag.c_str());
                CalIntersectionBetweenTwoMonochainLine(leftfirst_monochain_ybase_, rightsecond_monochain_ybase_, index_leftfirst_inleftfirstchain_ybase_, index_rightfirst_inrightsecondchain_ybase_);
            }
            if (tag == "110")
            {
                /* code */
                PolygonPrintf("run at %s.\n", tag.c_str());
                CalIntersectionBetweenTwoMonochainLine(rightfirst_monochain_ybase_, leftsecond_monochain_ybase_, index_rightfirst_inrightfirstchain_ybase_, index_leftfirst_inleftsecondchain_ybase_);
            }
            if (tag == "111")
            {
                /* code */
                PolygonPrintf("run at %s.\n", tag.c_str());
                CalIntersectionBetweenTwoMonochainLine(rightfirst_monochain_ybase_, rightsecond_monochain_ybase_, index_rightfirst_inrightfirstchain_ybase_, index_rightfirst_inrightsecondchain_ybase_);
            }
        }
    }

    template <class T>
    void PolygonIntersecting<T>::CalIntersectionBetweenTwoMonochainLine(MonotoneChain<T> &chain0, MonotoneChain<T> &chain1, int start0, int start1)
    {
#if debug
        PolygonPrintf("chain0 points: ...\n");
        for (int i = 0; i < chain0.points_chain.size(); ++i)
        {
            PolygonPrintf("chain0 is %lf %lf.\n", chain0.points_chain[i]->x, chain0.points_chain[i]->y);
        }
        PolygonPrintf("chain1 points: ...\n");
        for (int i = 0; i < chain1.points_chain.size(); ++i)
        {
            PolygonPrintf("chain1 is %lf %lf.\n", chain1.points_chain[i]->x, chain1.points_chain[i]->y);
        }
#endif
        if (chain0.points_chain.size() <= 4 && chain1.points_chain.size() <= 4)
        { // 都只剩下最多三条边的时候
            // 判断当前是否存在交点，并计算交点位置
            PolygonPrintf("run in the chain edge size <= 3 and point size <= 4 %d %d.\n", (int)chain0.points_chain.size(), (int)chain1.points_chain.size());
            if (chain0.points_chain.size() == 0 || chain1.points_chain.size() == 0)
            {
                return; // no interesction.
            }
            if (chain0.points_chain.size() == 1 || chain1.points_chain.size() == 1)
            {
                PolygonPrintf("error, these chain points size cannot be 1 ...\n");
                exit(-1);
            }
#if debug
            cv::Mat image5(cv::Size(1200, 600), CV_8UC1, cv::Scalar(0));
            for (int i = 0; i < chain0.points_chain.size() - 1; ++i)
            {
                cv::line(image5, cv::Point2d(50 + 10 * chain0.points_chain[i]->x, 550 - 10 * chain0.points_chain[i]->y), cv::Point2d(50 + 10 * chain0.points_chain[i + 1]->x, 550 - 10 * chain0.points_chain[i + 1]->y), 255, 1);
            }
            for (int i = 0; i < chain1.points_chain.size() - 1; ++i)
            {
                cv::line(image5, cv::Point2d(50 + 10 * chain1.points_chain[i]->x, 550 - 10 * chain1.points_chain[i]->y), cv::Point2d(50 + 10 * chain1.points_chain[i + 1]->x, 550 - 10 * chain1.points_chain[i + 1]->y), 255, 1);
            }
            cv::imshow("debug image debug", image5);
#if waitkey
            cv::waitKey(0);
#else
            cv::waitKey(1);
#endif

#endif
            for (int p0 = 0; p0 < chain0.points_chain.size() - 1; ++p0)
            {
                extreme_edge_t<T> midedge0;
                midedge0.p_start = chain0.points_chain[p0];
                midedge0.p_end = chain0.points_chain[p0 + 1];
                for (int p1 = 0; p1 < chain1.points_chain.size() - 1; ++p1)
                {
                    extreme_edge_t<T> midedge1;
                    midedge1.p_start = chain1.points_chain[p1];
                    midedge1.p_end = chain1.points_chain[p1 + 1];

                    if (IsLineIntersecting(midedge0, midedge1))
                    {
                        // has ， find the point and save into vector
                        printf("line has intersection. edge is %d %d+++++++++++++++++++++++++++++++++++ \n", start0+p0, start1+p1);
                        point2d_t<T> *inter = new point2d_t<T>(0, 0);
                        bool issucc = CalRealIntersectionPosition(midedge0, midedge1, inter);
                        if (issucc)
                        {
                            extremeedge_index_t indexes;
                            indexes.first_index = start0+p0;
                            indexes.second_index = start1+p1;
                            intersections_.emplace_back(std::make_pair(indexes, inter));
                        }
                        else
                        {
                            delete inter;
                        }
                    }
                }
            }
            return;
        }
        else
        {
            // 几种状态，需要分别判断，分支递归或者直接判断结果
            // no intersection
            // PolygonPrintf("run in the chain points intersection.\n");
            MonotoneChain<T> leftchain0, rightchain0, leftchain1, rightchain1;
            extreme_edge_t<T> midedge0, midedge1;
            // PolygonPrintf("before split chain0_1 size is %d %d.\n", (int)chain0.points_chain.size(), (int)chain1.points_chain.size());
            int left0_start=-1, right0_start=-1, left1_start= -1, right1_start = -1;
            chain0.GetMidEdge(midedge0, leftchain0, rightchain0, left0_start, right0_start);
            chain1.GetMidEdge(midedge1, leftchain1, rightchain1, left1_start, right1_start);
            printf("left right0 start is %d, %d rightchain size %d.\n", left0_start, right0_start, (int)rightchain0.points_chain.size());
            printf("left right1 start is %d, %d rightchain size %d.\n", left1_start, right1_start, (int)rightchain1.points_chain.size());
            /**
             * @brief @todo
             * 下方还需要更新right0 和 right1 
             * 可能还要 - 1
             * 
            */
            (chain1.points_chain.size() - 1) / 2;
            if (midedge0.p_end == nullptr || midedge1.p_end == nullptr)
            {
                return;
            }
#if debug
            cv::Mat image(cv::Size(1200, 600), CV_8UC1, cv::Scalar(0));
            cv::line(image, cv::Point2d(50 + 10 * midedge0.p_start->x, 550 - 10 * midedge0.p_start->y), cv::Point2d(50 + 10 * midedge0.p_end->x, 550 - 10 * midedge0.p_end->y), 255, 1);
            cv::line(image, cv::Point2d(50 + 10 * midedge1.p_start->x, 550 - 10 * midedge1.p_start->y), cv::Point2d(50 + 10 * midedge1.p_end->x, 550 - 10 * midedge1.p_end->y), 255, 1);
            cv::imshow("debug image mid", image);
#endif
            if (IsLineIntersecting(midedge0, midedge1))
            {
                // PolygonPrintf("line has intersecting...\n");
                point2d_t<T> *inter = new point2d_t<T>(0, 0);
                bool issucc = CalRealIntersectionPosition(midedge0, midedge1, inter);

                if (issucc)
                {
                    printf("line has intersection, edge is %d %d===============================.\n", start0+(int)((chain0.points_chain.size() - 1) / 2), start1+(int)((chain1.points_chain.size() - 1) / 2));
                    extremeedge_index_t indexes; // 在这个位置的点和后面一个点之间的边会存在一个交点
                    indexes.first_index = start0 + (chain0.points_chain.size() - 1) / 2;
                    indexes.second_index = start1 + (chain1.points_chain.size() - 1) / 2;
                    intersections_.emplace_back(std::make_pair(indexes, inter));
                }
                else
                {
                    delete inter;
                    // 没有相交，则把mid的端点，加入left 和 right中。
                }
            }
            else
            {
                // PolygonPrintf("0 insert before chain size is %d.\n", (int)rightchain0.points_chain.size());
                if (rightchain0.points_chain.empty())
                {
                    rightchain0.points_chain.insert(rightchain0.points_chain.begin(), midedge0.p_start);
                    rightchain0.points_chain.insert(rightchain0.points_chain.begin(), midedge0.p_end);
                    right0_start = 0;
                }
                else{
                    rightchain0.points_chain.insert(rightchain0.points_chain.begin(), midedge0.p_start);
                    --right0_start;
                }
        
                // PolygonPrintf("0 insert after chain size is %d.\n", (int)rightchain0.points_chain.size());
                // rightchain0.points_chain.insert(rightchain0.points_chain.begin(), midedge0.p_end);
                // PolygonPrintf("0 insert after chain size is %d.\n", (int)rightchain0.points_chain.size());
                // PolygonPrintf("1 insert before chain size is %d.\n", (int)rightchain1.points_chain.size());
                if (rightchain1.points_chain.empty())
                {
                    
                    rightchain1.points_chain.insert(rightchain1.points_chain.begin(), midedge1.p_start);
                    rightchain1.points_chain.insert(rightchain1.points_chain.begin(), midedge1.p_end);
                    right1_start = 0;
                }    
                else{
                    rightchain1.points_chain.insert(rightchain1.points_chain.begin(), midedge1.p_start);
                    --right1_start;
                }
                
                // PolygonPrintf("1 insert before chain size is %d.\n", (int)rightchain1.points_chain.size());
                // rightchain1.points_chain.insert(rightchain1.points_chain.begin(), midedge1.p_end);
                // PolygonPrintf("1 insert before chain size is %d.\n", (int)rightchain1.points_chain.size());
            }
#if debug
#if waitkey
            cv::waitKey(0);

#else
            cv::waitKey(1);
#endif 
            cv::Mat image0(cv::Size(1200, 600), CV_8UC1, cv::Scalar(0));
            // PolygonPrintf("run in 00 show. leftchain0.points_chain size is %d.\n", (int)leftchain0.points_chain.size());
            if (!leftchain0.points_chain.empty())
                for (int i = 0; i < leftchain0.points_chain.size() - 1; ++i)
                {
                    cv::line(image0, cv::Point2d(50 + 10 * leftchain0.points_chain[i]->x, 550 - 10 * leftchain0.points_chain[i]->y), cv::Point2d(50 + 10 * leftchain0.points_chain[i + 1]->x, 550 - 10 * leftchain0.points_chain[i + 1]->y), 255, 1);
                }
            // PolygonPrintf("run in 00 show. leftchain1.points_chain size is %d.\n", (int)leftchain1.points_chain.size());
            if (!leftchain1.points_chain.empty())
                for (int i = 0; i < leftchain1.points_chain.size() - 1; ++i)
                {
                    cv::line(image0, cv::Point2d(50 + 10 * leftchain1.points_chain[i]->x, 550 - 10 * leftchain1.points_chain[i]->y), cv::Point2d(50 + 10 * leftchain1.points_chain[i + 1]->x, 550 - 10 * leftchain1.points_chain[i + 1]->y), 255, 1);
                }
            // PolygonPrintf("run end 00 show.\n");
            cv::imshow("debug image", image0);
#endif
            if (PotentionIntersection(leftchain0, leftchain1))
            {
                int tempsize = intersections_.size();
                printf("00, new start left right index is %d %d.\n", start0+left0_start, start1+left1_start);
                CalIntersectionBetweenTwoMonochainLine(leftchain0, leftchain1, start0+left0_start, start1+left1_start);
                if (tempsize == intersections_.size())
                {
                    PolygonPrintf("intersection not increase, no new intersection.\n");
                }
                PolygonPrintf("accept this 00 result.\n");
            }
            else
            {
                PolygonPrintf("rejust this 00 result.\n");
            }

#if debug
#if waitkey
            cv::waitKey(0);
#else
            cv::waitKey(1);
#endif 
            // PolygonPrintf("run in 01 show.\n");
            cv::Mat image1(cv::Size(1200, 600), CV_8UC1, cv::Scalar(0));
            // PolygonPrintf("run in 01 show. leftchain0.points_chain size is %d.\n", (int)leftchain0.points_chain.size());
            if (!leftchain0.points_chain.empty())
                for (int i = 0; i < leftchain0.points_chain.size() - 1; ++i)
                {
                    cv::line(image1, cv::Point2d(50 + 10 * leftchain0.points_chain[i]->x, 550 - 10 * leftchain0.points_chain[i]->y), cv::Point2d(50 + 10 * leftchain0.points_chain[i + 1]->x, 550 - 10 * leftchain0.points_chain[i + 1]->y), 255, 1);
                }
            // PolygonPrintf("run in 01 show. rightchain1.points_chain size is %d.\n", (int)rightchain1.points_chain.size());
            if (!rightchain1.points_chain.empty())
                for (int i = 0; i < rightchain1.points_chain.size() - 1; ++i)
                {
                    cv::line(image1, cv::Point2d(50 + 10 * rightchain1.points_chain[i]->x, 550 - 10 * rightchain1.points_chain[i]->y), cv::Point2d(50 + 10 * rightchain1.points_chain[i + 1]->x, 550 - 10 * rightchain1.points_chain[i + 1]->y), 255, 1);
                }
            // PolygonPrintf("run end 01 show.\n");
            cv::imshow("debug image", image1);
#endif

            if (PotentionIntersection(leftchain0, rightchain1))
            {
                printf("01, new start left right index is %d %d.\n", start0+left0_start, start1+right1_start);
                CalIntersectionBetweenTwoMonochainLine(leftchain0, rightchain1, start0+left0_start, start1+right1_start);
                PolygonPrintf("accept this 01 result.\n");
            }
            else
            {
                PolygonPrintf("rejust this 01 result.\n");
            }

#if debug
#if waitkey
            cv::waitKey(0);
#else
            cv::waitKey(1);
#endif      // PolygonPrintf("run in 10 show.\n");
            cv::Mat image2(cv::Size(1200, 600), CV_8UC1, cv::Scalar(0));
            // PolygonPrintf("run in 10 show. rightchain0.points_chain size is %d.\n", (int)rightchain0.points_chain.size());
            if (!rightchain0.points_chain.empty())
                for (int i = 0; i < rightchain0.points_chain.size() - 1; ++i)
                {
                    cv::line(image2, cv::Point2d(50 + 10 * rightchain0.points_chain[i]->x, 550 - 10 * rightchain0.points_chain[i]->y), cv::Point2d(50 + 10 * rightchain0.points_chain[i + 1]->x, 550 - 10 * rightchain0.points_chain[i + 1]->y), 255, 1);
                }
            // PolygonPrintf("run in 10 show. leftchain0.points_chain size is %d.\n", (int)leftchain1.points_chain.size());
            if (!leftchain1.points_chain.empty())
                for (int i = 0; i < leftchain1.points_chain.size() - 1; ++i)
                {
                    cv::line(image2, cv::Point2d(50 + 10 * leftchain1.points_chain[i]->x, 550 - 10 * leftchain1.points_chain[i]->y), cv::Point2d(50 + 10 * leftchain1.points_chain[i + 1]->x, 550 - 10 * leftchain1.points_chain[i + 1]->y), 255, 1);
                }
            // PolygonPrintf("run end 10 show.\n");
            cv::imshow("debug image", image2);

#endif

            if (PotentionIntersection(rightchain0, leftchain1))
            {
                printf("10, new start left right index is %d %d.\n", start0+right0_start, start1+right1_start);
                CalIntersectionBetweenTwoMonochainLine(rightchain0, leftchain1, start0+right0_start, start1+left1_start);
                PolygonPrintf("accept this 10 result.\n");
            }
            else
            {
                PolygonPrintf("rejust this 10 result.\n");
            }
#if debug
#if waitkey
            cv::waitKey(0);
#else
            cv::waitKey(1);
#endif 
            // PolygonPrintf("run in 11 show.\n");
            cv::Mat image3(cv::Size(1200, 600), CV_8UC1, cv::Scalar(0));
            // PolygonPrintf("run in 11 show. rightchain0.points_chain size is %d.\n", (int)rightchain0.points_chain.size());
            if (!rightchain0.points_chain.empty())
                for (int i = 0; i < rightchain0.points_chain.size() - 1; ++i)
                {
                    cv::line(image3, cv::Point2d(50 + 10 * rightchain0.points_chain[i]->x, 550 - 10 * rightchain0.points_chain[i]->y), cv::Point2d(50 + 10 * rightchain0.points_chain[i + 1]->x, 550 - 10 * rightchain0.points_chain[i + 1]->y), 255, 1);
                }
            // PolygonPrintf("run in 11 show. rightchain1.points_chain size is %d.\n", (int)rightchain1.points_chain.size());
            if (!rightchain1.points_chain.empty())
                for (int i = 0; i < rightchain1.points_chain.size() - 1; ++i)
                {
                    cv::line(image3, cv::Point2d(50 + 10 * rightchain1.points_chain[i]->x, 550 - 10 * rightchain1.points_chain[i]->y), cv::Point2d(50 + 10 * rightchain1.points_chain[i + 1]->x, 550 - 10 * rightchain1.points_chain[i + 1]->y), 255, 1);
                }
            // PolygonPrintf("run end 11 show.\n");
            cv::imshow("debug image", image3);
#endif
            if (PotentionIntersection(rightchain0, rightchain1))
            {
                printf("11, new start left right index is %d %d.\n", start0+right0_start, start1+right1_start);
                CalIntersectionBetweenTwoMonochainLine(rightchain0, rightchain1, start0+right0_start, start1+right1_start);
                PolygonPrintf("accept this 11 result.\n");
            }
            else
            {
                PolygonPrintf("rejust this 11 result.\n");
            }
#if debug
#if waitkey
            cv::waitKey(0);
#else
            cv::waitKey(1);
#endif
#endif
            return;
        }
    }
    template <class T>
    bool CalRealIntersectionPosition(const extreme_edge_t<T> &edge0, const extreme_edge_t<T> &edge1, point2d_t<T> *inter)
    {
        const point2d_t<T> *pa = edge0.p_start;
        const point2d_t<T> *pb = edge0.p_end;
        const point2d_t<T> *pc = edge1.p_start;
        const point2d_t<T> *pd = edge1.p_end;

        // PolygonPrintf("pa %lf %lf.\n", pa->x, pa->y);
        // PolygonPrintf("pb %lf %lf.\n", pb->x, pb->y);
        // PolygonPrintf("pc %lf %lf.\n", pc->x, pc->y);
        // PolygonPrintf("pd %lf %lf.\n", pd->x, pd->y);
        // |x_ab, -x_cd||k|    |x_ac|
        // |y_ab, -y_cd||l| =  |y_ac|
        Eigen::Matrix2d A = Eigen::Matrix2d::Zero();
        A(0, 0) = pb->x - pa->x;
        A(0, 1) = -1 * (pd->x - pc->x);
        A(1, 0) = pb->y - pa->y;
        A(1, 1) = -1 * (pd->y - pc->y);

        if (A.determinant() == 0)
        {
            // 就没有可用的结果。
            PolygonPrintf("A determinant is == zero.\n");
            return false;
        }

        Eigen::Vector2d b(pc->x - pa->x, pc->y - pa->y);
        Eigen::Vector2d x = A.fullPivLu().solve(b);

        // std::cout << "x is: " << x.transpose() << std::endl;
        if (x[0] >= -1e-9 && x[0] - 1 <= 1e-9 && x[1] >= -1e-9 && x[1] - 1 <= 1e-9)
        {
            PolygonPrintf("has intersection.\n");
            inter->x = pa->x + x[0] * (pb->x - pa->x);
            inter->y = pa->y + x[0] * (pb->y - pa->y);
            return true;
        }
        else
        {
            PolygonPrintf("no intersection.\n");
            return false;
        }
    }

    template <class T>
    exedgeindex_node_t<T>* FindNodeInVec(exedgeindex_node_t<T> container[], int containersize, int exedgeidx, point2d_t<T>* p){
        printf("findnode in vec idx is %d.\n", exedgeidx);
        if(exedgeidx < 0 || exedgeidx >= containersize){
            printf("error, exedgeidx is outof bounder.\n");
            exit(-1);
        }
        if(!container[exedgeidx].index == exedgeidx){
            printf("cur containter may not match your exedgeidx.\n");
            exit(-1);
            return nullptr;
        }
        exedgeindex_node_t<T>* ptr = &container[exedgeidx];
        printf("ptr index and exedgeidx is %d %d.\n", ptr->index, exedgeidx);
        while(ptr->index == exedgeidx){
            printf("cur ptr and next point is %lf %lf , %lf %lf.\n", ptr->point->x, ptr->point->y, ptr->next->point->x, ptr->next->point->y);
            if(ptr->point->Equal(p)){
                return ptr;
            }
            if(!ptr->next){
                printf("error, ptr->next is nullptr.\n");
                exit(-1);
            }
            ptr = ptr->next;
        }
    }

    template <class T>
    void PolygonIntersecting<T>::ReconstructionIntersectionPolygon(){
        // we know the intersected edge in two polygon.
        // construction begin in the first polygon edge.
        // edge is points size in a polygon // make a circle, the last point link the first point
        
        if(intersections_.empty() || intersections_.size() == 1){
            printf("error, two polygon cannot intersect.\n");
            return;// get intersect_polygon is nullptr, means no intersections.
        }
        int firstpoints_size = polygon_firstptr_->SizeOfExtremePoints();
        int secondpoints_size = polygon_secondptr_->SizeOfExtremePoints();
        
        exedgeindex_node_t<T> vec_firstpoints[polygon_firstptr_->SizeOfExtremePoints()];
        exedgeindex_node_t<T> vec_secondpoints[polygon_secondptr_->SizeOfExtremePoints()];

        exedgeindex_node_t<T>* ptr = &vec_firstpoints[0];
        
        for(int i=0; i<polygon_firstptr_->SizeOfExtremePoints()-1; ++i){
            ptr->index = i;
            ptr->isinter = false;
            ptr->next = &vec_firstpoints[i+1];
            ptr->point = polygon_firstptr_->GetExtremePoint(i);
            ptr = ptr->next;
        }

        ptr->index = polygon_firstptr_->SizeOfExtremePoints()-1;
        ptr->isinter = false;
        ptr->next = &vec_firstpoints[0];
        ptr->point = polygon_firstptr_->GetExtremePoint(polygon_firstptr_->SizeOfExtremePoints()-1);
        
        ptr = &vec_secondpoints[0];
        for(int i=0; i<polygon_secondptr_->SizeOfExtremePoints()-1; ++i){
            ptr->index = i;
            ptr->isinter = false;
            ptr->next = &vec_secondpoints[i+1];
            ptr->point = polygon_secondptr_->GetExtremePoint(i);
            ptr = ptr->next;
        }
        ptr->index = polygon_secondptr_->SizeOfExtremePoints()-1;
        ptr->isinter = false;
        ptr->next = &vec_secondpoints[0];
        ptr->point = polygon_secondptr_->GetExtremePoint(polygon_secondptr_->SizeOfExtremePoints()-1);

    #if !debug 
        cv::Mat img00(cv::Size(1200, 600), CV_8UC1, cv::Scalar(0));
        for(int i=0; i<polygon_firstptr_->SizeOfExtremePoints()-1; ++i){
            cv::line(img00, cv::Point2d(50+10*vec_firstpoints[i].point->x, 550-10*vec_firstpoints[i].point->y), cv::Point2d(50+10*vec_firstpoints[i+1].point->x, 550-10*vec_firstpoints[i+1].point->y), 255, 2);
            cv::imshow("img00", img00);
            // cv::waitKey(0);
        }
        cv::line(img00, cv::Point2d(50+10*vec_firstpoints[polygon_firstptr_->SizeOfExtremePoints()-1].point->x, 550-10*vec_firstpoints[polygon_firstptr_->SizeOfExtremePoints()-1].point->y), 
                        cv::Point2d(50+10*vec_firstpoints[0].point->x, 550-10*vec_firstpoints[0].point->y), 255, 2);
        cv::imshow("img00", img00);                        
        // cv::waitKey(0);
        

        // cv::Mat img11(cv::Size(1200, 600), CV_8UC1, cv::Scalar(0));
        for(int i=0; i<polygon_secondptr_->SizeOfExtremePoints()-1; ++i){
            cv::line(img00, cv::Point2d(50+10*vec_secondpoints[i].point->x, 550-10*vec_secondpoints[i].point->y), cv::Point2d(50+10*vec_secondpoints[i+1].point->x, 550-10*vec_secondpoints[i+1].point->y), 255, 2);
            cv::imshow("img00", img00);
            // cv::waitKey(0);
        }
        cv::line(img00, cv::Point2d(50+10*vec_secondpoints[polygon_secondptr_->SizeOfExtremePoints()-1].point->x, 550-10*vec_secondpoints[polygon_secondptr_->SizeOfExtremePoints()-1].point->y), 
                        cv::Point2d(50+10*vec_secondpoints[0].point->x, 550-10*vec_secondpoints[0].point->y), 255, 2);
        cv::imshow("img00", img00);                        
        cv::waitKey(0);
        
    #endif

        // insert intersections
        exedgeindex_node_t<T> vec_firstintersect_points[intersections_.size()];
        exedgeindex_node_t<T> vec_secondintersect_points[intersections_.size()];

        for(int i=0; i<intersections_.size(); ++i){
            vec_firstintersect_points[i].index = intersections_[i].first.first_index;
            vec_firstintersect_points[i].point = intersections_[i].second;
            vec_firstintersect_points[i].isinter = true;
            vec_firstintersect_points[i].matched_index = intersections_[i].first.second_index;
            // add into point extremeedge
            if(vec_firstpoints[intersections_[i].first.first_index].next!=nullptr){
                if(vec_firstpoints[intersections_[i].first.first_index].next->isinter && vec_firstpoints[intersections_[i].first.first_index].next->next != nullptr){
                    if(vec_firstpoints[intersections_[i].first.first_index].next->next->isinter && vec_firstpoints[intersections_[i].first.first_index].next->next->next != nullptr){
                        if(vec_firstpoints[intersections_[i].first.first_index].next->next->next->isinter){
                            printf("error, one edge intersected more than 2 points.\n");
                            #if debug
                            cv::waitKey(0);
                            #endif
                            exit(-1);
                        }
                    }
                }

                // if the element is in the end.
                point2d_t<T>* p0 = vec_firstpoints[intersections_[i].first.first_index].point;
                point2d_t<T>* p1 = vec_firstpoints[intersections_[i].first.first_index].next->point;
                point2d_t<T>* p2 = vec_firstintersect_points[i].point;
                double dis0 = std::pow(p1->x-p0->x, 2)+std::pow(p1->y-p0->y, 2);
                double dis1 = std::pow(p2->x-p0->x, 2)+std::pow(p2->y-p0->y, 2);
                if(dis0 < dis1){
                    auto temp = vec_firstpoints[intersections_[i].first.first_index].next->next;
                    vec_firstpoints[intersections_[i].first.first_index].next->next = &vec_firstintersect_points[i];
                    vec_firstintersect_points[i].next = temp;
                }
                else if(dis0 > dis1){
                    auto temp = vec_firstpoints[intersections_[i].first.first_index].next;
                    vec_firstpoints[intersections_[i].first.first_index].next = &vec_firstintersect_points[i];
                    vec_firstintersect_points[i].next = temp;
                }
                else{
                    printf("error, dis0 cannot equal with dis1.\n");
                    exit(-1);
                }
            }
            else{
                printf("0error, never caused this type.\n");
                exit(-1);
                auto temp = vec_firstpoints[intersections_[i].first.first_index].next;
                vec_firstpoints[intersections_[i].first.first_index].next = &vec_firstintersect_points[i];
                vec_firstintersect_points[i].next = temp;
            }

            vec_secondintersect_points[i].index = intersections_[i].first.second_index;
            vec_secondintersect_points[i].point = intersections_[i].second;
            vec_secondintersect_points[i].isinter = true;
            vec_secondintersect_points[i].matched_index = intersections_[i].first.first_index;

            // add into point extremeedge
            if(vec_secondpoints[intersections_[i].first.second_index].next!= nullptr){
                if(vec_secondpoints[intersections_[i].first.second_index].next->isinter && vec_firstpoints[intersections_[i].first.second_index].next->next != nullptr){
                    if(vec_secondpoints[intersections_[i].first.second_index].next->next->isinter && vec_firstpoints[intersections_[i].first.second_index].next->next->next != nullptr){
                        if(vec_secondpoints[intersections_[i].first.second_index].next->next->next->isinter){
                            printf("error, one edge intersected more than 2 points.\n");
                            #if debug
                            cv::waitKey(0);
                            #endif
                            exit(-1);
                        }
                    }
                }
                // if the element is in the end.
                // if the element is in the end.
                point2d_t<T>* p0 = vec_secondpoints[intersections_[i].first.second_index].point;
                point2d_t<T>* p1 = vec_secondpoints[intersections_[i].first.second_index].next->point;
                point2d_t<T>* p2 = vec_secondintersect_points[i].point;
                double dis0 = std::pow(p1->x-p0->x, 2)+std::pow(p1->y-p0->y, 2);
                double dis1 = std::pow(p2->x-p0->x, 2)+std::pow(p2->y-p0->y, 2);
                if(dis0 < dis1){
                    auto temp = vec_secondpoints[intersections_[i].first.second_index].next->next;
                    vec_secondpoints[intersections_[i].first.second_index].next->next = &vec_secondintersect_points[i];
                    vec_secondintersect_points[i].next = temp;
                }
                else if(dis0 > dis1){
                    auto temp = vec_secondpoints[intersections_[i].first.second_index].next;
                    vec_secondpoints[intersections_[i].first.second_index].next = &vec_secondintersect_points[i];
                    vec_secondintersect_points[i].next = temp;
                }
                else{
                    printf("error, dis0 cannot equal with dis1.\n");
                    exit(-1);
                }
                
            }
            else{
                printf("1error, never caused this type.\n");
                exit(-1);
                auto temp = vec_secondpoints[intersections_[i].first.second_index].next;
                vec_secondpoints[intersections_[i].first.second_index].next = &vec_secondintersect_points[i];
                vec_secondintersect_points[i].next = temp;
            }
        }
        
#if debug
        // draw the link of the add intersecs polygon., 目前这个push方式有问题
        cv::Mat img0(cv::Size(1200, 600), CV_8UC1, cv::Scalar(0));
        exedgeindex_node_t<T>* root = &vec_firstpoints[0];
        cv::line(img0, cv::Point2d(50+10*root->point->x, 550-10*root->point->y), cv::Point2d(50+10*root->next->point->x, 550-10*root->next->point->y), 255, 2);
        cv::imshow("img0", img0);
        cv::waitKey(0);
        root = root->next;
        while(root != &vec_firstpoints[0]){
            cv::line(img0, cv::Point2d(50+10*root->point->x, 550-10*root->point->y), cv::Point2d(50+10*root->next->point->x, 550-10*root->next->point->y), 255, 2);
            root = root->next;
            cv::imshow("img0", img0);
            cv::waitKey(0);
        }
        cv::imshow("img0", img0);
        cv::waitKey(0);

        cv::Mat img1(cv::Size(1200, 600), CV_8UC1, cv::Scalar(0));
        exedgeindex_node_t<T>* root1 = &vec_secondpoints[0];
        cv::line(img1, cv::Point2d(50+10*root1->point->x, 550-10*root1->point->y), cv::Point2d(50+10*root1->next->point->x, 550-10*root1->next->point->y), 255, 2);
        root1 = root1->next;
        cv::imshow("img1", img1);
        cv::waitKey(0);
        while(root1 != &vec_secondpoints[0]){
            cv::line(img1, cv::Point2d(50+10*root1->point->x, 550-10*root1->point->y), cv::Point2d(50+10*root1->next->point->x, 550-10*root1->next->point->y), 255, 2);
            root1 = root1->next;
            cv::imshow("img1", img1);
            cv::waitKey(0);
        }
        cv::imshow("img1", img1);
        cv::waitKey(0);
#endif
        bool infirstchain = true;
        std::vector<point2d_t<T>*> reinterpolygon_points;
        if (vec_firstpoints[intersections_[0].first.first_index].next == nullptr || !vec_firstpoints[intersections_[0].first.first_index].next->isinter)
        {
            printf("intersections_[0].first.first_index is %d. first polygon size is %d.\n", intersections_[0].first.first_index, (int)polygon_firstptr_->SizeOfExtremePoints());
            printf("error, the first intersection line next shouldn't be nullptr.\n");
            exit(-1);
        }

        exedgeindex_node_t<T>* node_ptr = vec_firstpoints[intersections_[0].first.first_index].next;
        exedgeindex_node_t<T>* root_ptr = node_ptr;
        reinterpolygon_points.emplace_back(node_ptr->point);
        printf("cur node index is %d. matched is %d.\n", node_ptr->index, node_ptr->matched_index);
        printf("emplace pid %lf, %lf.===========================\n", node_ptr->point->x, node_ptr->point->y);

        if(polygon_secondptr_->InPolygon(vec_firstpoints[intersections_[0].first.first_index].point)){
            printf("first node is in secondpolygon.\n");
            infirstchain = false;
        }
        if(!infirstchain){// 端点的前端，在polygon 外面，则下一次仍然可以加进去
            node_ptr = FindNodeInVec(vec_secondpoints, secondpoints_size, node_ptr->matched_index, node_ptr->point);
            printf("find intersect in second polygon edge id %d.\n", node_ptr->index);
        }
        if(!node_ptr->next){
            printf("error, node_ptr is null.\n");
            exit(-1);
        }
        node_ptr = node_ptr->next;
        // 不论是在first chain 还是 second chain 都转移到下一个位置
        // 需要一个功能，查找某一点，在链表中的位置，input: index, points(use equal.)
        while(node_ptr != root_ptr){
            reinterpolygon_points.emplace_back(node_ptr->point);
            printf("emplace pid %lf, %lf.===========================\n", node_ptr->point->x, node_ptr->point->y);

            if (infirstchain)
            {
                if(node_ptr->isinter){
                    node_ptr = FindNodeInVec(vec_secondpoints, secondpoints_size, node_ptr->matched_index, node_ptr->point);
                    infirstchain = false;
                }
            }
            else
            {
                if(node_ptr->isinter){
                    node_ptr = FindNodeInVec(vec_firstpoints, firstpoints_size, node_ptr->matched_index, node_ptr->point);
                    infirstchain = true;
                }
            }
            if(node_ptr->next == nullptr){
                printf("error node_ptr never == nullptr.\n");
                exit(-1);
            }
            node_ptr = node_ptr->next;
        }
#if !debug

            printf("repolgon points size is %d.\n", (int)reinterpolygon_points.size());
            cv::Mat img(cv::Size(1200, 600), CV_8UC1, cv::Scalar(0));
            for(int i=0; i<reinterpolygon_points.size()-1; ++i){
                cv::line(img, cv::Point2d(50+10*reinterpolygon_points[i]->x, 550-10*reinterpolygon_points[i]->y), cv::Point2d(50+10*reinterpolygon_points[i+1]->x, 550-10*reinterpolygon_points[i+1]->y), cv::Scalar(255), 1);
                cv::imshow("img repolygon", img);
                cv::waitKey(0);
            }
            cv::line(img, cv::Point2d(50+10*reinterpolygon_points[reinterpolygon_points.size()-1]->x, 550-10*reinterpolygon_points[reinterpolygon_points.size()-1]->y), cv::Point2d(50+10*reinterpolygon_points[0]->x, 550-10*reinterpolygon_points[0]->y), cv::Scalar(255), 1);
            cv::imshow("img repolygon", img);
            cv::waitKey(0);

#endif
    }

    // 排序单个边与其他边相交的情况， 更优秀的做法是相交算完了，然后直接加入到polygon中
    template <class T>
    bool compare_dis(const std::pair<exedgeindex_node_t<T>*, double> elem0, const std::pair<exedgeindex_node_t<T>*, double> elem1){
        return elem0.second < elem1.second;
    }

    template <class T>
    void PolygonIntersecting<T>::SortedFirst_polygonandIntersection(exedgeindex_node_t<T> vec[], const int& size){
        for(int i=0; i<size; ++i){
            if(vec[i].next != nullptr){
                Sorted_polygonandIntersection(polygon_firstptr_->GetExtremePoint(i), vec[i]);
            }
            continue;
        }
    }
    template <class T>
    void PolygonIntersecting<T>::SortedSecond_polygonandIntersection(exedgeindex_node_t<T> vec[], const int& size){
        for(int i=0; i<size; ++i){
            if(vec[i].next != nullptr){
                Sorted_polygonandIntersection(polygon_secondptr_->GetExtremePoint(i), vec[i]);
            }
            continue;
        }
    }

    template <class T>
    void PolygonIntersecting<T>::Sorted_polygonandIntersection(point2d_t<T>* p0, exedgeindex_node_t<T>& root){
        // list order， how to sort the list, compare the dis with the start of the exedge
        std::vector<std::pair<exedgeindex_node_t<T>*, double>> exedgeindex_vec;
        exedgeindex_node_t<T>* rptr = &root;
        
        while(rptr != nullptr){
            double dis = std::pow(intersections_[rptr->intersect_index].second->x - p0->x, 2)+std::pow(intersections_[rptr->intersect_index].second->y - p0->y, 2);
            exedgeindex_vec.emplace_back(std::make_pair(rptr, dis));
            rptr = rptr->next;
        }
        std::sort(exedgeindex_vec.begin(), exedgeindex_vec.end(), compare_dis);

        rptr = &root;
        for(int i=0; exedgeindex_vec.size()-1; ++i){
            rptr->index = exedgeindex_vec[i].first->index;
            rptr->intersect_index = exedgeindex_vec[i].first->intersect_index;
            rptr->next = exedgeindex_vec[i+1].first;
            rptr = rptr->next;
        }
    }
}


/**
 * while (true)
        {
            printf("run in here.\n");
            if(infirstchain){
                exedgeindex_node_t<T>* curnode = &vec_firstpoints[counter_first];
                
                for(int i=0; i< counter_first_intersected; ++i)
                    curnode = curnode->next;

                if(reinterpolygon_points.size() >= 2){
                    if (curnode->point->id == reinterpolygon_points[0]->id)
                    {
                        printf("find the same point break.\n");
                        break;
                    }
                }
                reinterpolygon_points.emplace_back(curnode->point);
                printf("emplace pid %lld.===========================\n", curnode->point->id);
                if(curnode->isinter){
                    printf("in is inter first.\n");
                    printf("counter second %d %d in is inter first.\n", counter_second, counter_second_intersected);
                    counter_second_intersected = 2;// 直接跳到下一个交点上或者顶点上
                    counter_second = curnode->matched_index;
                    infirstchain = false;
                    printf("after counter second %d %d in is inter first.\n", counter_second, counter_second_intersected);
                }
                else{
                    printf("in isn't inter first.\n");
                    printf("counter first %d %d in is inter first.\n", counter_first, counter_first_intersected);
                    if(counter_first_intersected == 2){
                        ++counter_first;
                        if(counter_first == polygon_firstptr_->SizeOfExtremePoints()){
                            counter_first = 0;
                        }    
                    }
                    ++counter_first;
                    counter_first_intersected = 0;
                    if(counter_first == polygon_firstptr_->SizeOfExtremePoints()){
                        counter_first = 0;
                    }
                    printf("after counter first %d %d in is inter first.\n", counter_first, counter_first_intersected);
                }
            }
            else{
                exedgeindex_node_t<T>* curnode = &vec_secondpoints[counter_second];
                for(int i=0; i<counter_second_intersected; ++i){
                    curnode = curnode->next;
                }
                reinterpolygon_points.emplace_back(curnode->point);
                printf("emplace pid %lld.===========================\n", curnode->point->id);
                if(curnode->isinter)
                {
                    printf("in is inter second.\n");
                    printf("counter first %d %d in is inter second.\n", counter_first, counter_first_intersected);
                    counter_first_intersected = 2;
                    counter_first = curnode->matched_index;
                    infirstchain = true;
                    printf("after counter first %d %d in is inter second.\n", counter_first, counter_first_intersected);
                }
                else
                {
                    printf("in isn't inter second.\n");
                    printf("counter second %d %d in is inter second.\n", counter_second, counter_second_intersected);
                    if(counter_second_intersected == 2){
                        ++counter_second;
                        if(counter_second == polygon_secondptr_->SizeOfExtremePoints()){
                            counter_second = 0;
                        }
                    }
                    ++counter_second;
                    counter_second_intersected = 0;
                    if(counter_second == polygon_secondptr_->SizeOfExtremePoints()){
                        counter_second = 0;
                    }
                    printf("after counter second %d %d in is inter second.\n", counter_second, counter_second_intersected);
                }
            }
            cv::waitKey(0);
        }
 * 
 * */ 