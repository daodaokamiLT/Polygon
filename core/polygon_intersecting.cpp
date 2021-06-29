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
        // 边数比点数少1
        int mid = (int)(size-1)/2;
        GetEdge(midedge, midedge);
        for(int i=0; i<=mid; ++i){
            monochan_before.emplace_back(points_chain[i]);
        }
        for(int i=mid+1; i<size; ++i){
            monochain_after.emplace_back(points_chain[i]);
        }
    }
    template <class T>
    void findextreme(MonotoneChain<T>& monochain, linepoint2d_t<T>& p_left, linepoint2d_t<T>& p_right, linepoint2d_t<T>& p_up, linepoint2d_t<T>& p_down){
        T min_x = std::numeric_limits<T>::max(), max_x = std::numeric_limits<T>::min(), min_y = std::numeric_limits<T>::max(), max_y = std::numeric_limits<T>::min();
        size_t minx_index = 0, maxx_index = 0, miny_index = 0, maxy_index = 0; 
        for(size_t i=0; i<monochain.points_chain.size();++i){
            if(min_x > monochain.points_chain[i]->x){
                min_x = monochain.points_chain[i]->x;
                minx_index = i;
            }
            if(min_y > monochain.points_chain[i]->y){
                min_y = monochain.points_chain[i]->y;
                miny_index = i;
            }
            if(max_x < monochain.points_chain[i]->x){
                max_x = monochain.points_chain[i]->x;
                maxx_index = i;
            }
            if(max_y < monochain.points_chain[i]->x){
                max_y = monochain.points_chain[i]->x;
                maxy_index = i;
            }   
        }
        p_left.isleft = true;
        p_left.point2dptr = monochain.points_chain[minx_index];

        p_right.isleft = false;
        p_right.point2dptr = monochain.points_chain[maxx_index];

        p_top.isleft = true;
        p_right.point2dptr = monochain.points_chain[miny_index];

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
    void PolygonIntersecting<T>::OrderLineEndPoints(){
        // first polygon
        linepoint2d_t<T> linepoint2d_fromL1st_xbase_leftest, linepoint2d_fromL1st_xbase_rightest, linepoint2d_fromL1st_xbase_topest, linepoint2d_fromL1st_xbase_bottomest;
        findextreme(leftfirst_monochain_xbase_, linepoint2d_fromL1st_xbase_leftest, linepoint2d_fromL1st_xbase_rightest, linepoint2d_fromL1st_xbase_topest, linepoint2d_fromL1st_xbase_bottomest);
        linepoint2d_t<T>::UpdateMonoChainId();
        
        linepoint2d_t<T> linepoint2d_fromR1st_xbase_leftest, linepoint2d_fromR1st_xbase_rightest, linepoint2d_fromR1st_xbase_topest, linepoint2d_fromR1st_xbase_bottomest;
        findextreme(rightfirst_monochain_xbase_, linepoint2d_fromR1st_xbase_leftest, linepoint2d_fromR1st_xbase_rightest, linepoint2d_fromR1st_xbase_topest, linepoint2d_fromR1st_xbase_bottomest);
        linepoint2d_t<T>::UpdateMonoChainId();
        // ----- ybase
        linepoint2d_t<T> linepoint2d_fromL1st_ybase_leftest, linepoint2d_fromL1st_ybase_rightest, linepoint2d_fromL1st_ybase_topest, linepoint2d_fromL1st_ybase_bottomest;
        findextreme(leftfirst_monochain_ybase_, linepoint2d_fromL1st_ybase_leftest, linepoint2d_fromL1st_ybase_rightest, linepoint2d_fromL1st_ybase_topest, linepoint2d_fromL1st_ybase_bottomest);
        linepoint2d_t<T>::UpdateMonoChainId();

        linepoint2d_t<T> linepoint2d_fromR1st_ybase_leftest, linepoint2d_fromR1st_ybase_rightest, linepoint2d_fromR1st_ybase_topest, linepoint2d_fromR1st_ybase_bottomest;
        findextreme(rightfirst_monochain_ybase_, linepoint2d_fromR1st_ybase_leftest, linepoint2d_fromR1st_ybase_rightest, linepoint2d_fromR1st_ybase_topest, linepoint2d_fromR1st_ybase_bottomest);
        linepoint2d_t<T>::UpdateMonoChainId();
        // second polygon
        linepoint2d_t<T> linepoint2d_fromL2nd_xbase_leftest, linepoint2d_fromL2nd_xbase_rightest, linepoint2d_fromL2nd_xbase_topest, linepoint2d_fromL2nd_xbase_bottomest;
        findextreme(leftsecond_monochain_xbase_, linepoint2d_fromL2nd_xbase_leftest, linepoint2d_fromL2nd_xbase_rightest, linepoint2d_fromL2nd_xbase_topest, linepoint2d_fromL2nd_xbase_bottomest);
        linepoint2d_t<T>::UpdateMonoChainId();

        linepoint2d_t<T> linepoint2d_fromR2nd_xbase_leftest, linepoint2d_fromR2nd_xbase_rightest, linepoint2d_fromR2nd_xbase_topest, linepoint2d_fromR2nd_xbase_bottomest;
        findextreme(rightsecond_monochain_xbase_, linepoint2d_fromR2nd_xbase_leftest, linepoint2d_fromR2nd_xbase_rightest, linepoint2d_fromR2nd_xbase_topest, linepoint2d_fromR2nd_xbase_bottomest);
        linepoint2d_t<T>::UpdateMonoChainId();
        // ----- second ybase
        linepoint2d_t<T> linepoint2d_fromL2nd_ybase_leftest, linepoint2d_fromL2nd_ybase_rightest, linepoint2d_fromL2nd_ybase_topest, linepoint2d_fromL2nd_ybase_bottomest;
        findextreme(leftsecond_monochain_ybase_, linepoint2d_fromL2nd_ybase_leftest, linepoint2d_fromL2nd_ybase_rightest, linepoint2d_fromL2nd_ybase_topest, linepoint2d_fromL2nd_ybase_bottomest);
        linepoint2d_t<T>::UpdateMonoChainId();

        linepoint2d_t<T> linepoint2d_fromR2nd_ybase_leftest, linepoint2d_fromR2nd_ybase_rightest, linepoint2d_fromR2nd_ybase_topest, linepoint2d_fromR2nd_ybase_bottomest;
        findextreme(rightsecond_monochain_ybase_, linepoint2d_fromR2nd_ybase_leftest, linepoint2d_fromR2nd_ybase_rightest, linepoint2d_fromR2nd_ybase_topest, linepoint2d_fromR2nd_ybase_bottomest);
        linepoint2d_t<T>::UpdateMonoChainId();

        sorted_extremelinepoint2d_xbase_hor_.clear();
        sorted_extremelinepoint2d_xbase_hor_.emplace_back(linepoint2d_fromL1st_xbase_leftest);
        sorted_extremelinepoint2d_xbase_hor_.emplace_back(linepoint2d_fromL1st_xbase_rightest);
        sorted_extremelinepoint2d_xbase_hor_.emplace_back(linepoint2d_fromR1st_xbase_leftest);
        sorted_extremelinepoint2d_xbase_hor_.emplace_back(linepoint2d_fromR1st_xbase_rightest);
        sorted_extremelinepoint2d_xbase_hor_.emplace_back(linepoint2d_fromL2nd_xbase_leftest);
        sorted_extremelinepoint2d_xbase_hor_.emplace_back(linepoint2d_fromL2nd_xbase_rightest);
        sorted_extremelinepoint2d_xbase_hor_.emplace_back(linepoint2d_fromR2nd_xbase_leftest);
        sorted_extremelinepoint2d_xbase_hor_.emplace_back(linepoint2d_fromR2nd_xbase_rightest);

        sorted_extremelinepoint2d_xbase_vel_.clear();
        sorted_extremelinepoint2d_xbase_vel_.emplace_back(linepoint2d_fromL1st_xbase_topest);
        sorted_extremelinepoint2d_xbase_vel_.emplace_back(linepoint2d_fromL1st_xbase_bottomest);
        sorted_extremelinepoint2d_xbase_vel_.emplace_back(linepoint2d_fromR1st_xbase_topest);
        sorted_extremelinepoint2d_xbase_vel_.emplace_back(linepoint2d_fromR1st_xbase_bottomest);
        sorted_extremelinepoint2d_xbase_vel_.emplace_back(linepoint2d_fromL2nd_xbase_topest);
        sorted_extremelinepoint2d_xbase_vel_.emplace_back(linepoint2d_fromL2nd_xbase_bottomest);
        sorted_extremelinepoint2d_xbase_vel_.emplace_back(linepoint2d_fromR2nd_xbase_topest);
        sorted_extremelinepoint2d_xbase_vel_.emplace_back(linepoint2d_fromR2nd_xbase_bottomest);

        sorted_extremelinepoint2d_ybase_hor_.clear();
        sorted_extremelinepoint2d_ybase_hor_.emplace_back(linepoint2d_fromL1st_ybase_leftest);
        sorted_extremelinepoint2d_ybase_hor_.emplace_back(linepoint2d_fromL1st_ybase_rightest);
        sorted_extremelinepoint2d_ybase_hor_.emplace_back(linepoint2d_fromR1st_ybase_leftest);
        sorted_extremelinepoint2d_ybase_hor_.emplace_back(linepoint2d_fromR1st_ybase_rightest);
        sorted_extremelinepoint2d_ybase_hor_.emplace_back(linepoint2d_fromL2nd_ybase_leftest);
        sorted_extremelinepoint2d_ybase_hor_.emplace_back(linepoint2d_fromL2nd_ybase_rightest);
        sorted_extremelinepoint2d_ybase_hor_.emplace_back(linepoint2d_fromR2nd_ybase_leftest);
        sorted_extremelinepoint2d_ybase_hor_.emplace_back(linepoint2d_fromR2nd_ybase_rightest);

        sorted_extremelinepoint2d_ybase_vel_.clear();
        sorted_extremelinepoint2d_ybase_vel_.emplace_back(linepoint2d_fromL1st_ybase_topest);
        sorted_extremelinepoint2d_ybase_vel_.emplace_back(linepoint2d_fromL1st_ybase_bottomest);
        sorted_extremelinepoint2d_ybase_vel_.emplace_back(linepoint2d_fromR1st_ybase_topest);
        sorted_extremelinepoint2d_ybase_vel_.emplace_back(linepoint2d_fromR1st_ybase_bottomest);
        sorted_extremelinepoint2d_ybase_vel_.emplace_back(linepoint2d_fromL2nd_ybase_topest);
        sorted_extremelinepoint2d_ybase_vel_.emplace_back(linepoint2d_fromL2nd_ybase_bottomest);
        sorted_extremelinepoint2d_ybase_vel_.emplace_back(linepoint2d_fromR2nd_ybase_topest);
        sorted_extremelinepoint2d_ybase_vel_.emplace_back(linepoint2d_fromR2nd_ybase_bottomest);

        // 在四个队列中，进行排序查看是否有可能的重合部分， 若有则进行递归求解。
        
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
    bool PolygonIntersecting<T>::IsSmallerorBiggerX(polygon::Polygon<T>* ptr, const bool& isclockwise, const int& last_count, int& cur_count){
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
        return IsSmallerorBiggerX(ptr, isclockwise, last_count, cur_count);
    }

    template <class T>
    bool PolygonIntersecting<T>::IsSmallerorBiggerXLock(polygon::Polygon<T>* ptr, const bool& isclockwise, const int& last_count, int& cur_count){
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
        return IsSmallerorBiggerXLock(ptr, isclockwise, last_count, cur_count);
    }

// when init cur_count == lastcount, 0 is smaller 1 is bigger
    template <class T>
    bool PolygonIntersecting<T>::IsSmallerorBiggerY(polygon::Polygon<T>* ptr, const bool& isclockwise, const int& last_count, int& cur_count){
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
        if(ptr->GetExIndexY(cur_count) < ptr->GetExIndexY(last_count)){
            return false;
        }
        else if(ptr->GetExIndexY(cur_count) > ptr->GetExIndexY(last_count)){
            return true;
        }
        return IsSmallerorBiggerY(ptr, isclockwise, last_count, cur_count);
    }

    template <class T>
    bool PolygonIntersecting<T>::IsSmallerorBiggerYLock(polygon::Polygon<T>* ptr, const bool& isclockwise, const int& last_count, int& cur_count){
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
        std::cout<<ptr->GetExIndexY(cur_count)<<", "<<ptr->GetExIndexY(last_count)<<std::endl;
        if(ptr->GetExIndexY(cur_count) < ptr->GetExIndexY(last_count)){
            return false;
        }
        else if(ptr->GetExIndexY(cur_count) > ptr->GetExIndexY(last_count)){
            return true;
        }
        printf("run in 2.\n");
        return IsSmallerorBiggerYLock(ptr, isclockwise, last_count, cur_count);
    }


    template <class T>
    void PolygonIntersecting<T>::SplitPolygon2MonotoneChain(){
        leftfirst_monochain_xbase_.points_chain.clear();
        leftfirst_monochain_ybase_.points_chain.clear();
        rightfirst_monochain_xbase_.points_chain.clear();
        rightfirst_monochain_ybase_.points_chain.clear();

        // use the extreme edges to split not the extreme poitns !!!!
        std::vector<extreme_edge_t<T>> exedges0;
        polygon_firstptr_->GetExtremeEdges(exedges0);
        std::vector<point2d_t<T>*> expoints_first;
        expoints_first.emplace_back(exedges0[0].p_start);
        expoints_first.emplace_back(exedges0[0].p_end);
        // printf("exedges0 %d (%lf %f) - (%lf %lf).\n", 0, exedges0[0].p_start->x, exedges0[0].p_start->y, exedges0[0].p_end->x, exedges0[0].p_end->y);
        for(int i=1; i<exedges0.size(); ++i){
            // printf("exedges0 %d (%lf %f) - (%lf %lf).\n", i, exedges0[i].p_start->x, exedges0[i].p_start->y, exedges0[i].p_end->x, exedges0[i].p_end->y);
            if(!expoints_first[i]->Equal(exedges0[i].p_start)){
                printf("edge cannot match with points order.\n");
                exit(-1);
            }
            expoints_first.emplace_back(exedges0[i].p_end);
        }
        /***=============           first start X major           =================**/
        point2d_t<T>* minx0_expoint = nullptr, *maxx0_expoint = nullptr, *miny0_expoint = nullptr, *maxy0_expoint = nullptr;
        size_t minx0_index, maxx0_index, miny0_index, maxy0_index;
        size_t start_idx = 0, end_idx = expoints_first.size()-1, endthreshold = expoints_first.size();// 永远在数值上都是差一步的距离 start_idx - 1 = end_idx
        // is unclockwise to visited these points
        bool findminfirst = false;
        int firstDiff_Index = 0;
        if(expoints_first[1]->x < expoints_first[0]->x){
            findminfirst = true;
        }
        else if(expoints_first[1]->x > expoints_first[0]->x){ //-----1
            findminfirst = false;
        }
        else{
            bool issmallerorbigger = IsSmallerorBiggerXLock(polygon_firstptr_, false, 0, firstDiff_Index);
            if(issmallerorbigger){
                findminfirst = false;
            }
            else{
                findminfirst = true;
            }
        }
        if(firstDiff_Index==0){
            firstDiff_Index = 1;
        }
        ++firstDiff_Index;
        if(firstDiff_Index == endthreshold){
            // minfirst 是第一次找到的是minx， 然后从min->max, 那么优先的是right monochain
            for(int i=0; i<firstDiff_Index-1; ++i){
                if(findminfirst){
                    leftfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[i]);                    
                }
                else{
                    rightfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[i]);
                }
            }
            if(findminfirst){
                rightfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[firstDiff_Index-2]);
                rightfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[firstDiff_Index-1]);
                rightfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[0]);
            }
            else{
                leftfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[firstDiff_Index-2]);
                leftfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[firstDiff_Index-1]);
                leftfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[0]);
            }
        }
        // 定理： 一个循环有序队列中，最大值和最小值的index 不可能都在端点, 除非一条直线的情况加一个点的情况
        for(int i=firstDiff_Index; i<endthreshold; ++i){//must can be find minest or maxest x points, but should 
            if(findminfirst){
                if(expoints_first[i]->x > expoints_first[i-1]->x){
                    // 当出现第一个小于的值时，找到最大值。接下来，将所当前最大值和之后所有递减的push into leftfirst_monochain
                    minx0_index = i-1;
                    minx0_expoint = expoints_first[minx0_index];
                    int count = minx0_index;
                    int next_count = count+1;
                    if(next_count == endthreshold) next_count = 0;
                    while(expoints_first[next_count]->x >= expoints_first[count]->x){
                        rightfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[count]);
                        ++count;
                        if(count == endthreshold) count = 0;
                        next_count = count + 1;
                        if(next_count == endthreshold) next_count = 0;
                    }
                    // 把min 也放进来
                    rightfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[count]);
                    leftfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[count]);
                    maxx0_index = next_count;
                    maxx0_expoint = expoints_first[maxx0_index];
                    count = maxx0_index; next_count = minx0_index;
                    while(count != next_count){
                        leftfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[count]);
                        ++count;
                        if(count == endthreshold) count = 0;
                    }
                    leftfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[count]);
                    break;
                }
            }
            else{
                // find max
                if(expoints_first[i]->x < expoints_first[i-1]->x){
                    // 找到最小值，接下来，把所有当前最小值之后递增的pushinto rightfirst_monochain
                    maxx0_index = i-1;
                    maxx0_expoint = expoints_first[maxx0_index];
                    int count = maxx0_index;
                    int next_count = count + 1;
                    if(next_count == endthreshold) next_count = 0;
                    while(expoints_first[next_count]->x <= expoints_first[count]->x){
                        leftfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[count]);
                        ++count;
                        if(count == endthreshold) count = 0;
                        next_count = count + 1;
                        if(next_count == endthreshold) next_count = 0;
                    }
                    leftfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[count]);
                    rightfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[count]);

                    minx0_index = next_count;
                    minx0_expoint = expoints_first[minx0_index];
                    count = minx0_index; next_count = maxx0_index;
                    while(count != next_count){
                        rightfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[count]);
                        ++count;
                        if(count == endthreshold) count = 0;
                    }
                    rightfirst_monochain_xbase_.points_chain.emplace_back(expoints_first[count]);
                    break;
                }
            }
        }

        //***------------          first start Y major            -----------------**/
        findminfirst = false;
        firstDiff_Index = 0;
        if(expoints_first[1]->y < expoints_first[0]->y){
            findminfirst = true;
        }
        else if(expoints_first[1]->y > expoints_first[0]->y){ //-----1
            findminfirst = false;
        }
        else{
            bool issmallerorbigger = IsSmallerorBiggerYLock(polygon_firstptr_, false, 0, firstDiff_Index);
            if(issmallerorbigger){
                findminfirst = false;
            }
            else{
                findminfirst = true;
            }
        }
        if(firstDiff_Index==0){
            firstDiff_Index = 1;
        }
        ++firstDiff_Index;
        if(firstDiff_Index == endthreshold){
            // minfirst 是第一次找到的是minx， 然后从min->max, 那么优先的是right monochain
            for(int i=0; i<firstDiff_Index-1; ++i){
                if(findminfirst){
                    leftfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[i]);                    
                }
                else{
                    rightfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[i]);
                }
            }
            if(findminfirst){
                rightfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[firstDiff_Index-2]);
                rightfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[firstDiff_Index-1]);
                rightfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[0]);
            }
            else{
                leftfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[firstDiff_Index-2]);
                leftfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[firstDiff_Index-1]);
                leftfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[0]);
            }
        }
        // 定理： 一个循环有序队列中，最大值和最小值的index 不可能都在端点, 除非一条直线的情况加一个点的情况
        for(int i=firstDiff_Index; i<endthreshold; ++i){//must can be find minest or maxest x points, but should 
            if(findminfirst){
                if(expoints_first[i]->y > expoints_first[i-1]->y){
                    // 当出现第一个小于的值时，找到最大值。接下来，将所当前最大值和之后所有递减的push into leftfirst_monochain
                    miny0_index = i-1;
                    miny0_expoint = expoints_first[miny0_index];
                    int count = miny0_index;
                    int next_count = count+1;
                    if(next_count == endthreshold) next_count = 0;
                    while(expoints_first[next_count]->y >= expoints_first[count]->y){
                        rightfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[count]);
                        ++count;
                        if(count == endthreshold) count = 0;
                        next_count = count + 1;
                        if(next_count == endthreshold) next_count = 0;
                    }
                    // 把min 也放进来
                    rightfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[count]);
                    leftfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[count]);
                    maxy0_index = next_count;
                    maxy0_expoint = expoints_first[maxy0_index];
                    count = maxy0_index; next_count = miny0_index;
                    while(count != next_count){
                        leftfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[count]);
                        ++count;
                        if(count == endthreshold) count = 0;
                    }
                    leftfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[count]);
                    break;
                }
            }
            else{
                // find max
                if(expoints_first[i]->y < expoints_first[i-1]->y){
                    // 找到最小值，接下来，把所有当前最小值之后递增的pushinto rightfirst_monochain
                    maxy0_index = i-1;
                    maxy0_expoint = expoints_first[maxy0_index];
                    int count = maxy0_index;
                    int next_count = count + 1;
                    if(next_count == endthreshold) next_count = 0;
                    while(expoints_first[next_count]->y <= expoints_first[count]->y){
                        leftfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[count]);
                        ++count;
                        if(count == endthreshold) count = 0;
                        next_count = count + 1;
                        if(next_count == endthreshold) next_count = 0;
                    }
                    leftfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[count]);
                    rightfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[count]);

                    miny0_index = next_count;
                    miny0_expoint = expoints_first[miny0_index];
                    count = miny0_index; next_count = maxy0_index;
                    while(count != next_count){
                        rightfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[count]);
                        ++count;
                        if(count == endthreshold) count = 0;
                    }
                    rightfirst_monochain_ybase_.points_chain.emplace_back(expoints_first[count]);
                    break;
                }
            }
        }


        // ##########################################################################



        /***=============          second start X major           =================**/
        std::vector<extreme_edge_t<T>> exedges1;
        polygon_secondptr_->GetExtremeEdges(exedges1);
        std::vector<point2d_t<T>*> expoints_second;
        expoints_second.emplace_back(exedges1[0].p_start);
        expoints_second.emplace_back(exedges1[0].p_end);
        // printf("exedges1 %d (%lf %f) - (%lf %lf).\n", 0, exedges1[0].p_start->x, exedges1[0].p_start->y, exedges1[0].p_end->x, exedges1[0].p_end->y);
        for(int i=1; i<exedges1.size(); ++i){
            // printf("exedges1 %d (%lf %f) - (%lf %lf).\n", i, exedges1[i].p_start->x, exedges1[i].p_start->y, exedges1[i].p_end->x, exedges1[i].p_end->y);
            if(!expoints_second[i]->Equal(exedges1[i].p_start)){
                printf("edge cannot match with points order.\n");
                exit(-1);
            }
            expoints_second.emplace_back(exedges1[i].p_end);
        }
    
        /***=============           first start X major           =================**/
        point2d_t<T>* minx1_expoint = nullptr, *maxx1_expoint = nullptr, *miny1_expoint = nullptr, *maxy1_expoint = nullptr;
        size_t minx1_index, maxx1_index, miny1_index, maxy1_index;
        start_idx = 0, end_idx = expoints_second.size()-1, endthreshold = expoints_second.size();// 永远在数值上都是差一步的距离 start_idx - 1 = end_idx
        // is unclockwise to visited these points
        findminfirst = false;
        firstDiff_Index = 0;
        if(expoints_second[1]->x < expoints_second[0]->x){
            findminfirst = true;
        }
        else if(expoints_second[1]->x > expoints_second[0]->x){ //-----1
            findminfirst = false;
        }
        else{
            bool issmallerorbigger = IsSmallerorBiggerYLock(polygon_firstptr_, false, 0, firstDiff_Index);
            if(issmallerorbigger){
                findminfirst = false;
            }
            else{
                findminfirst = true;
            }
        }
        if(firstDiff_Index==0){
            firstDiff_Index = 1;
        }
        ++firstDiff_Index;
        if(firstDiff_Index == endthreshold){
            // minfirst 是第一次找到的是minx， 然后从min->max, 那么优先的是right monochain
            for(int i=0; i<firstDiff_Index-1; ++i){
                if(findminfirst){
                    leftsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[i]);                    
                }
                else{
                    rightsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[i]);
                }
            }
            if(findminfirst){
                rightsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[firstDiff_Index-2]);
                rightsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[firstDiff_Index-1]);
                rightsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[0]);
            }
            else{
                leftsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[firstDiff_Index-2]);
                leftsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[firstDiff_Index-1]);
                leftsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[0]);
            }
        }
        // 定理： 一个循环有序队列中，最大值和最小值的index 不可能都在端点, 除非一条直线的情况加一个点的情况
        for(int i=firstDiff_Index; i<endthreshold; ++i){//must can be find minest or maxest x points, but should 
            if(findminfirst){
                if(expoints_second[i]->x > expoints_second[i-1]->x){
                    // 当出现第一个小于的值时，找到最大值。接下来，将所当前最大值和之后所有递减的push into leftsecond_monochain
                    minx1_index = i-1;
                    minx1_expoint = expoints_second[minx1_index];
                    int count = minx1_index;
                    int next_count = count+1;
                    if(next_count == endthreshold) next_count = 0;
                    while(expoints_second[next_count]->x >= expoints_second[count]->x){
                        rightsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[count]);
                        ++count;
                        if(count == endthreshold) count = 0;
                        next_count = count + 1;
                        if(next_count == endthreshold) next_count = 0;
                    }
                    // 把min 也放进来
                    rightsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[count]);
                    leftsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[count]);
                    maxx1_index = next_count;
                    maxx1_expoint = expoints_second[maxx1_index];
                    count = maxx1_index; next_count = minx1_index;
                    while(count != next_count){
                        leftsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[count]);
                        ++count;
                        if(count == endthreshold) count = 0;
                    }
                    leftsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[count]);
                    break;
                }
            }
            else{
                // find max
                if(expoints_second[i]->x < expoints_second[i-1]->x){
                    // 找到最小值，接下来，把所有当前最小值之后递增的pushinto rightsecond_monochain
                    maxx1_index = i-1;
                    maxx1_expoint = expoints_second[maxx1_index];
                    int count = maxx1_index;
                    int next_count = count + 1;
                    if(next_count == endthreshold) next_count = 0;
                    while(expoints_second[next_count]->x <= expoints_second[count]->x){
                        leftsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[count]);
                        ++count;
                        if(count == endthreshold) count = 0;
                        next_count = count + 1;
                        if(next_count == endthreshold) next_count = 0;
                    }
                    leftsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[count]);
                    rightsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[count]);

                    minx1_index = next_count;
                    minx1_expoint = expoints_second[minx1_index];
                    count = minx1_index; next_count = maxx1_index;
                    while(count != next_count){
                        rightsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[count]);
                        ++count;
                        if(count == endthreshold) count = 0;
                    }
                    rightsecond_monochain_xbase_.points_chain.emplace_back(expoints_second[count]);
                    break;
                }
            }
        }

        //***------------          first start Y major            -----------------**/
        findminfirst = false;
        firstDiff_Index = 0;
        if(expoints_second[1]->y < expoints_second[0]->y){
            findminfirst = true;
        }
        else if(expoints_second[1]->y > expoints_second[0]->y){ //-----1
            findminfirst = false;
        }
        else{
            bool issmallerorbigger = IsSmallerorBiggerYLock(polygon_firstptr_, false, 0, firstDiff_Index);
            if(issmallerorbigger){
                findminfirst = false;
            }
            else{
                findminfirst = true;
            }
        }
        if(firstDiff_Index==0){
            firstDiff_Index = 1;
        }
        ++firstDiff_Index;
        if(firstDiff_Index == endthreshold){
            // minfirst 是第一次找到的是minx， 然后从min->max, 那么优先的是right monochain
            for(int i=0; i<firstDiff_Index-1; ++i){
                if(findminfirst){
                    leftsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[i]);                    
                }
                else{
                    rightsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[i]);
                }
            }
            if(findminfirst){
                rightsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[firstDiff_Index-2]);
                rightsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[firstDiff_Index-1]);
                rightsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[0]);
            }
            else{
                leftsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[firstDiff_Index-2]);
                leftsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[firstDiff_Index-1]);
                leftsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[0]);
            }
        }
        // 定理： 一个循环有序队列中，最大值和最小值的index 不可能都在端点, 除非一条直线的情况加一个点的情况
        for(int i=firstDiff_Index; i<endthreshold; ++i){//must can be find minest or maxest x points, but should 
            if(findminfirst){
                if(expoints_second[i]->y > expoints_second[i-1]->y){
                    // 当出现第一个小于的值时，找到最大值。接下来，将所当前最大值和之后所有递减的push into leftsecond_monochain
                    miny1_index = i-1;
                    miny1_expoint = expoints_second[miny1_index];
                    int count = miny1_index;
                    int next_count = count+1;
                    if(next_count == endthreshold) next_count = 0;
                    while(expoints_second[next_count]->y >= expoints_second[count]->y){
                        rightsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[count]);
                        ++count;
                        if(count == endthreshold) count = 0;
                        next_count = count + 1;
                        if(next_count == endthreshold) next_count = 0;
                    }
                    // 把min 也放进来
                    rightsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[count]);
                    leftsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[count]);
                    maxy1_index = next_count;
                    maxy1_expoint = expoints_second[maxy1_index];
                    count = maxy1_index; next_count = miny1_index;
                    while(count != next_count){
                        leftsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[count]);
                        ++count;
                        if(count == endthreshold) count = 0;
                    }
                    leftsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[count]);
                    break;
                }
            }
            else{
                // find max
                if(expoints_second[i]->y < expoints_second[i-1]->y){
                    // 找到最小值，接下来，把所有当前最小值之后递增的pushinto rightsecond_monochain
                    maxy1_index = i-1;
                    maxy1_expoint = expoints_second[maxy1_index];
                    int count = maxy1_index;
                    int next_count = count + 1;
                    if(next_count == endthreshold) next_count = 0;
                    while(expoints_second[next_count]->y <= expoints_second[count]->y){
                        leftsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[count]);
                        ++count;
                        if(count == endthreshold) count = 0;
                        next_count = count + 1;
                        if(next_count == endthreshold) next_count = 0;
                    }
                    leftsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[count]);
                    rightsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[count]);

                    miny1_index = next_count;
                    miny1_expoint = expoints_second[miny1_index];
                    count = miny1_index; next_count = maxy1_index;
                    while(count != next_count){
                        rightsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[count]);
                        ++count;
                        if(count == endthreshold) count = 0;
                    }
                    rightsecond_monochain_ybase_.points_chain.emplace_back(expoints_second[count]);
                    break;
                }
            }
        }

        printf("polygon xbase extreme points size is %d, leftchain size is %d, rightchain size is %d.\n", (int)polygon_firstptr_->SizeOfExtremePoints(), (int)leftfirst_monochain_xbase_.points_chain.size(), (int)rightfirst_monochain_xbase_.points_chain.size());
        printf("polygon ybase extreme points size is %d, leftchain size is %d, rightchain size is %d.\n", (int)polygon_firstptr_->SizeOfExtremePoints(), (int)leftfirst_monochain_ybase_.points_chain.size(), (int)rightfirst_monochain_ybase_.points_chain.size());
        printf("polygon xbase second extreme points size is %d, leftchain size is %d, rightchain size is %d.\n", (int)polygon_secondptr_->SizeOfExtremePoints(), (int)leftsecond_monochain_xbase_.points_chain.size(), (int)rightsecond_monochain_xbase_.points_chain.size());
        printf("polygon ybase second extreme points size is %d, leftchain size is %d, rightchain size is %d.\n", (int)polygon_secondptr_->SizeOfExtremePoints(), (int)leftsecond_monochain_ybase_.points_chain.size(), (int)rightsecond_monochain_ybase_.points_chain.size());
    }

    template <class T>
    void PolygonIntersecting<T>::GetFirstMonoChain(std::vector<point2d_t<T>*>& leftchain, std::vector<point2d_t<T>*>& rightchain){
        leftchain.clear(); rightchain.clear();
        if(xory_major_){
            leftchain.assign(leftfirst_monochain_ybase_.points_chain.begin(), leftfirst_monochain_ybase_.points_chain.end());
            rightchain.assign(rightfirst_monochain_ybase_.points_chain.begin(), rightfirst_monochain_ybase_.points_chain.end());
        }
        else{
            leftchain.assign(leftfirst_monochain_xbase_.points_chain.begin(), leftfirst_monochain_xbase_.points_chain.end());
            rightchain.assign(rightfirst_monochain_xbase_.points_chain.begin(), rightfirst_monochain_xbase_.points_chain.end());
        }
        
    }

    template <class T>
    void PolygonIntersecting<T>::GetFirstMonoChainXbase(std::vector<point2d_t<T>*>& leftchain, std::vector<point2d_t<T>*>& rightchain){
        leftchain.clear(); rightchain.clear();
        leftchain.assign(leftfirst_monochain_xbase_.points_chain.begin(), leftfirst_monochain_xbase_.points_chain.end());
        rightchain.assign(rightfirst_monochain_xbase_.points_chain.begin(), rightfirst_monochain_xbase_.points_chain.end());
    }

    template <class T>
    void PolygonIntersecting<T>::GetSecondMonoChainXbase(std::vector<point2d_t<T>*>& leftchain, std::vector<point2d_t<T>*>& rightchain){
        leftchain.clear(); rightchain.clear();
        leftchain.assign(leftsecond_monochain_xbase_.points_chain.begin(), leftsecond_monochain_xbase_.points_chain.end());
        rightchain.assign(rightsecond_monochain_xbase_.points_chain.begin(), rightsecond_monochain_xbase_.points_chain.end());
    }

    
    template <class T>
    void PolygonIntersecting<T>::GetFirstMonoChainYbase(std::vector<point2d_t<T>*>& leftchain, std::vector<point2d_t<T>*>& rightchain){
        leftchain.clear(); rightchain.clear();
        leftchain.assign(leftfirst_monochain_ybase_.points_chain.begin(), leftfirst_monochain_ybase_.points_chain.end());
        rightchain.assign(rightfirst_monochain_ybase_.points_chain.begin(), rightfirst_monochain_ybase_.points_chain.end());
    }

    template <class T>
    void PolygonIntersecting<T>::GetSecondMonoChainYbase(std::vector<point2d_t<T>*>& leftchain, std::vector<point2d_t<T>*>& rightchain){
        leftchain.clear(); rightchain.clear();
        leftchain.assign(leftsecond_monochain_ybase_.points_chain.begin(), leftsecond_monochain_ybase_.points_chain.end());
        rightchain.assign(rightsecond_monochain_ybase_.points_chain.begin(), rightsecond_monochain_ybase_.points_chain.end());
    }
}