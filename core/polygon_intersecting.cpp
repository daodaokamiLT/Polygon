#include "polygon_intersecting.h"
#include <limits>

namespace polygon{


    template <class T>
    bool MonotoneChain<T>::GetMidEdge(extreme_edge_t<T>& midedge, MonotoneChain<T>& monochan_before, MonotoneChain<T>& monochain_after){
        int size = points_chain.size();
        if(size == 1){
            //printf("error cannot create a edge.\n");
            return false;
        }
        if(size == 2){
            midedge.p_start = points_chain[0];
            midedge.p_end = points_chain[1];
            return false;
        }
        int mid = (int)size/2;
        midedge.p_start = points_chain[mid-1];
        midedge.p_end = points_chain[mid];
        for(int i=0; i<=mid-1; ++i){
            monochan_before.emplace_back(points_chain[i]);
        }
        for(int i=mid; i<size; ++i){
            monochain_after.emplace_back(points_chain[i]);
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
    bool PolygonIntersecting<T>::isSmallerorBiger(const polygon::Polygon<T>* ptr, const bool& isclockwise, const int& last_count, int& cur_count){
        if (isclockwise)
        {
            --count_ex;
            if (count_ex < 0)
                count_ex = expoints_first.size() - 1;
        }
        else
        {
            ++count_ex;
            if (count_ex == expoints_first.size())
                count_ex = 0;
        }
        if(ptr->GetExIndexX(cur_count) < ptr->GetExIndexX(last_count)){
            return false;
        }
        else if(ptr->GetExIndexX(cur_count) > ptr->GetExIndexX(last_count)){
            return true;
        }
        return isSmallerorBiger(isclockwise, last_count, cur_count);
    }
    template <class T>
    void PolygonIntersecting<T>::SplitPolygon2MonotoneChain(){
        T minx0 = std::numeric_limits<T>::max(); T miny0=std::numeric_limits<T>::max();
        T maxx0 = std::numeric_limits<T>::min(); T maxy0=std::numeric_limits<T>::min();
        point2d_t<T>* minx0_expoint = nullptr, *maxx0_expoint = nullptr, *miny0_expoint = nullptr, *maxy0_expoint = nullptr;
        size_t minx0_index, maxx0_index, miny0_index, maxy0_index;
        std::vector<point2d_t<T>*> expoints_first;
        polygon_firstptr_->GetExtremePoints(expoints_first);   
        printf("polygon extreme points size is %d, leftchain size is %d, rightchain size is %d.\n", (int)polygon_firstptr_->SizeOfExtremePoints(), (int)leftfirst_monochain_.points_chain.size(), (int)rightfirst_monochain_.points_chain.size());
        // 使用循环队列最快
        FunctionalVectorPtr<point2d_t<T>> functionalvec(expoints_first.size());
        
    }

    template <class T>
    void PolygonIntersecting<T>::GetFirstMonoChain(std::vector<point2d_t<T>*>& leftchain, std::vector<point2d_t<T>*>& rightchain){
        leftchain.clear(); rightchain.clear();
        leftchain.assign(leftfirst_monochain_.points_chain.begin(), leftfirst_monochain_.points_chain.end());
        rightchain.assign(rightfirst_monochain_.points_chain.begin(), rightfirst_monochain_.points_chain.end());
    }

    template <class T>
    void PolygonIntersecting<T>::GetSecondMonoChain(std::vector<point2d_t<T>*>& leftchain, std::vector<point2d_t<T>*>& rightchain){
        leftchain.clear(); rightchain.clear();
        leftchain.assign(leftsecond_monochain_.points_chain.begin(), leftsecond_monochain_.points_chain.end());
        rightchain.assign(rightsecond_monochain_.points_chain.begin(), rightsecond_monochain_.points_chain.end());
    }
}
