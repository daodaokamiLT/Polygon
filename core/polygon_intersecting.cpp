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
    bool PolygonIntersecting<T>::isSmallerorBiggerX(polygon::Polygon<T>* ptr, const bool& isclockwise, const int& last_count, int& cur_count){
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
        if(ptr->GetExIndexX(cur_count) < ptr->GetExIndexX(last_count)){
            return false;
        }
        else if(ptr->GetExIndexX(cur_count) > ptr->GetExIndexX(last_count)){
            return true;
        }
        return isSmallerorBiggerX(isclockwise, last_count, cur_count);
    }

    template <class T>
    bool PolygonIntersecting<T>::isSmallerorBiggerXLock(polygon::Polygon<T>* ptr, const bool& isclockwise, const int& last_count, int& cur_count){
        if (isclockwise)
        {
            --cur_count;
            if(cur_count < 0){
                printf("error, cannot find result.\n");
                cur_count = -1;
                return false;
            }
        }
        else
        {
            ++cur_count;
            if(cur_count == (int)(ptr->SizeOfExtremePoints())){
                printf("error, cannot find result.\n");
                cur_count = (int)(ptr->SizeOfExtremePoints());
                return false;
            }
        }
        if(ptr->GetExIndexX(cur_count) < ptr->GetExIndexX(last_count)){
            return false;
        }
        else if(ptr->GetExIndexX(cur_count) > ptr->GetExIndexX(last_count)){
            return true;
        }
        return isSmallerorBiggerXLock(ptr, isclockwise, last_count, cur_count);
    }

    template <class T>
    void PolygonIntersecting<T>::SplitPolygon2MonotoneChain(){
        point2d_t<T>* minx0_expoint = nullptr, *maxx0_expoint = nullptr, *miny0_expoint = nullptr, *maxy0_expoint = nullptr;
        size_t minx0_index, maxx0_index, miny0_index, maxy0_index;
        std::vector<point2d_t<T>*> expoints_first;
        polygon_firstptr_->GetExtremePoints(expoints_first); 
        size_t start_idx = 0, end_idx = expoints_first.size()-1, endthreshold = expoints_first.size();// 永远在数值上都是差一步的距离 start_idx - 1 = end_idx
        // is unclockwise to visited these points
        bool isclockwise = false;
        int firstDiff_Index = 0;
        if(expoints_first[1]->x < expoints_first[0]->x){      //-----0
            isclockwise = false;// 往逆时针方向 x --> 变大 find maxx_x
        }
        else if(expoints_first[1]->x > expoints_first[0]->x){ //-----1
            isclockwise = true; // 往逆时针方向是变小 find minx_x
        }
        else{
            /***
             * @brief it cannot caused that all x is same.
             * 
            */
            // decide min or max, 逆时针方向下，找到第一个x不一样的结果
            bool issmallerorbigger = isSmallerorBiggerXLock(polygon_firstptr_, isclockwise, 0, firstDiff_Index);
            if(issmallerorbigger){// 0 is same as ----- 0
                isclockwise = false;
            }
            else{// 1 is same as ----- 1
                isclockwise = true;
            }
        }
        if(firstDiff_Index==0){
            firstDiff_Index = 1;
        }
        ++firstDiff_Index;
        if(firstDiff_Index == endthreshold){
            // the polygon is just a line with a out point. at the end of the expoints.
            for(int i=0; i<firstDiff_Index-1; ++i){
                if(isclockwise){ // like | .
                    leftfirst_monochain_.points_chain.emplace_back(expoints_first[i]);                    
                }
                else{
                    rightfirst_monochain_.points_chain.emplace_back(expoints_first[i]);
                }
            }
            if(isclockwise){
                rightfirst_monochain_.points_chain.emplace_back(expoints_first[firstDiff_Index-2]);
                rightfirst_monochain_.points_chain.emplace_back(expoints_first[firstDiff_Index-1]);
                rightfirst_monochain_.points_chain.emplace_back(expoints_first[0]);
            }
            else{
                leftfirst_monochain_.points_chain.emplace_back(expoints_first[firstDiff_Index-2]);
                leftfirst_monochain_.points_chain.emplace_back(expoints_first[firstDiff_Index-1]);
                leftfirst_monochain_.points_chain.emplace_back(expoints_first[0]);
            }
        }
        // 定理： 一个循环有序队列中，最大值和最小值的index 不可能都在端点, 除非一条直线的情况加一个点的情况
        for(int i=firstDiff_Index; i<endthreshold; ++i){//must can be find minest or maxest x points, but should 
            if(isclockwise){
                printf("run in here 0.\n");
                if(expoints_first[i]->x < expoints_first[i-1]->x){
                    // 当出现第一个小于的值时，找到最大值。接下来，将所当前最大值和之后所有递减的push into leftfirst_monochain
                    maxx0_index = i-1;
                    printf("run in here 1 max is %d.\n", (int)maxx0_index);
                    maxx0_expoint = expoints_first[maxx0_index];
                    int count = maxx0_index;
                    int next_count = count+1;
                    if(next_count == endthreshold) next_count = 0;
                    while(expoints_first[next_count]->x <= expoints_first[count]->x){
                        leftfirst_monochain_.points_chain.emplace_back(expoints_first[count]);
                        ++count;
                        if(count == endthreshold) count = 0;
                        next_count = count + 1;
                        if(next_count == endthreshold) next_count = 0;
                    }
                    // 把min 也放进来
                    leftfirst_monochain_.points_chain.emplace_back(expoints_first[count]);
                    rightfirst_monochain_.points_chain.emplace_back(expoints_first[count]);
                    minx0_index = next_count;
                    printf("run in here2 min is %d.\n", (int)minx0_index);
                    minx0_expoint = expoints_first[minx0_index];
                    count = minx0_index; next_count = maxx0_index;
                    while(count != next_count){
                        rightfirst_monochain_.points_chain.emplace_back(expoints_first[count]);
                        ++count;
                        if(count == endthreshold) count = 0;
                    }
                    rightfirst_monochain_.points_chain.emplace_back(expoints_first[count]);
                    break;
                }
            }
            else{
                if(expoints_first[i]->x > expoints_first[i-1]->x){
                    // 找到最小值，接下来，把所有当前最小值之后递增的pushinto rightfirst_monochain
                    minx0_index = i-1;
                    minx0_expoint = expoints_first[minx0_index];
                    int count = minx0_index;
                    int next_count = count + 1;
                    if(next_count == endthreshold) next_count = 0;
                    while(expoints_first[next_count]->x >= expoints_first[count]->x){
                        rightfirst_monochain_.points_chain.emplace_back(expoints_first[count]);
                        ++count;
                        if(count == endthreshold) count = 0;
                        next_count = count + 1;
                        if(next_count == endthreshold) next_count = 0;
                    }
                    rightfirst_monochain_.points_chain.emplace_back(expoints_first[count]);
                    leftfirst_monochain_.points_chain.emplace_back(expoints_first[count]);
                    maxx0_index = next_count;
                    maxx0_expoint = expoints_first[maxx0_index];
                    count = maxx0_index; next_count = minx0_index;
                    while(count != next_count){
                        leftfirst_monochain_.points_chain.emplace_back(expoints_first[count]);
                        ++count;
                        if(count == endthreshold) count = 0;
                    }
                    leftfirst_monochain_.points_chain.emplace_back(expoints_first[count]);
                    break;
                }
            }
        }

        // resplit for y
        isclockwise = false;




        printf("polygon extreme points size is %d, leftchain size is %d, rightchain size is %d.\n", (int)polygon_firstptr_->SizeOfExtremePoints(), (int)leftfirst_monochain_.points_chain.size(), (int)rightfirst_monochain_.points_chain.size());
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
