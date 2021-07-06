#include "polygon_intersecting.h"
#include <limits>
/***
 * 
 * 需要解决那种交在定点上，或者重合的问题
 * 
 * 
 * 
 * 
*/

namespace polygon{
    template <class T>
    bool MonotoneChain<T>::GetMidEdge(extreme_edge_t<T>& midedge, MonotoneChain<T>& monochan_before, MonotoneChain<T>& monochain_after){
        
        int size = points_chain.size();
        printf("points chain size is %d.\n", size);
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
        printf("mid is %d.\n", mid);
        bool issucc= this->GetEdge(mid, midedge);
        if(!issucc){
            printf("error, failed GetEdge.\n");
            exit(-1);
        }
        for(int i=0; i<=mid; ++i){
            monochan_before.points_chain.emplace_back(points_chain[i]);
        }
        for(int i=mid+1; i<size; ++i){
            monochain_after.points_chain.emplace_back(points_chain[i]);
        }
        return issucc;
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
            if(max_y < monochain.points_chain[i]->y){
                max_y = monochain.points_chain[i]->y;
                maxy_index = i;
            }   
        }
        p_left.isleft = true;
        p_left.point2dptr = monochain.points_chain[minx_index];

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
    void PolygonIntersecting<T>::OrderLineEndPoints(){
        // first polygon
        linepoint2d_t<T> linepoint2d_fromL1st_xbase_leftest(0, 0), linepoint2d_fromL1st_xbase_rightest(0, 0), linepoint2d_fromL1st_xbase_topest(0, 0), linepoint2d_fromL1st_xbase_bottomest(0, 0);
        findextreme(leftfirst_monochain_xbase_, linepoint2d_fromL1st_xbase_leftest, linepoint2d_fromL1st_xbase_rightest, linepoint2d_fromL1st_xbase_topest, linepoint2d_fromL1st_xbase_bottomest);
        
        std::cout<<"linepoint2d_fromL1st tag: "<<linepoint2d_fromL1st_xbase_leftest.tag<<", "<<linepoint2d_fromL1st_xbase_rightest.tag<<", "<<linepoint2d_fromL1st_xbase_topest.tag<<", "<<linepoint2d_fromL1st_xbase_bottomest.tag<<std::endl;
        linepoint2d_t<T> linepoint2d_fromR1st_xbase_leftest(1, 1), linepoint2d_fromR1st_xbase_rightest(1, 1), linepoint2d_fromR1st_xbase_topest(1, 1), linepoint2d_fromR1st_xbase_bottomest(1, 1);
        findextreme(rightfirst_monochain_xbase_, linepoint2d_fromR1st_xbase_leftest, linepoint2d_fromR1st_xbase_rightest, linepoint2d_fromR1st_xbase_topest, linepoint2d_fromR1st_xbase_bottomest);
        std::cout<<"linepoint2d_fromL1st tag: "<<linepoint2d_fromR1st_xbase_leftest.tag<<", "<<linepoint2d_fromR1st_xbase_rightest.tag<<", "<<linepoint2d_fromR1st_xbase_topest.tag<<", "<<linepoint2d_fromR1st_xbase_bottomest.tag<<std::endl;
        
        // check
        if(linepoint2d_fromL1st_xbase_leftest.point2dptr->x != linepoint2d_fromR1st_xbase_leftest.point2dptr->x || linepoint2d_fromL1st_xbase_rightest.point2dptr->x != linepoint2d_fromR1st_xbase_rightest.point2dptr->x){
            printf("error, the first extreme points get error, same polygon leftest and rightest x is not same.\n");
            exit(-1);
        }

        // second polygon
        linepoint2d_t<T> linepoint2d_fromL2nd_xbase_leftest(2, 2), linepoint2d_fromL2nd_xbase_rightest(2, 2), linepoint2d_fromL2nd_xbase_topest(2, 2), linepoint2d_fromL2nd_xbase_bottomest(2, 2);
        findextreme(leftsecond_monochain_xbase_, linepoint2d_fromL2nd_xbase_leftest, linepoint2d_fromL2nd_xbase_rightest, linepoint2d_fromL2nd_xbase_topest, linepoint2d_fromL2nd_xbase_bottomest);
        std::cout<<"linepoint2d_fromL2nd tag: "<<linepoint2d_fromL2nd_xbase_leftest.tag<<", "<<linepoint2d_fromL2nd_xbase_rightest.tag<<", "<<linepoint2d_fromL2nd_xbase_topest.tag<<", "<<linepoint2d_fromL2nd_xbase_bottomest.tag<<std::endl;
        

        linepoint2d_t<T> linepoint2d_fromR2nd_xbase_leftest(3, 3), linepoint2d_fromR2nd_xbase_rightest(3, 3), linepoint2d_fromR2nd_xbase_topest(3, 3), linepoint2d_fromR2nd_xbase_bottomest(3, 3);
        findextreme(rightsecond_monochain_xbase_, linepoint2d_fromR2nd_xbase_leftest, linepoint2d_fromR2nd_xbase_rightest, linepoint2d_fromR2nd_xbase_topest, linepoint2d_fromR2nd_xbase_bottomest);
        std::cout<<"linepoint2d_fromR2nd tag: "<<linepoint2d_fromR2nd_xbase_leftest.tag<<", "<<linepoint2d_fromR2nd_xbase_rightest.tag<<", "<<linepoint2d_fromR2nd_xbase_topest.tag<<", "<<linepoint2d_fromR2nd_xbase_bottomest.tag<<std::endl;
        
        if(linepoint2d_fromL2nd_xbase_leftest.point2dptr->x != linepoint2d_fromR2nd_xbase_leftest.point2dptr->x || linepoint2d_fromL2nd_xbase_rightest.point2dptr->x != linepoint2d_fromR2nd_xbase_rightest.point2dptr->x){
            printf("error, the second extreme points get error, same polygon leftest and rightest x is not same.\n");
            exit(-1);
        }
        
        // ----- ybase
        linepoint2d_t<T> linepoint2d_fromL1st_ybase_leftest(4, 4), linepoint2d_fromL1st_ybase_rightest(4, 4), linepoint2d_fromL1st_ybase_topest(4, 4), linepoint2d_fromL1st_ybase_bottomest(4, 4);
        findextreme(leftfirst_monochain_ybase_, linepoint2d_fromL1st_ybase_leftest, linepoint2d_fromL1st_ybase_rightest, linepoint2d_fromL1st_ybase_topest, linepoint2d_fromL1st_ybase_bottomest);
        std::cout<<"linepoint2d_fromL1st tag: "<<linepoint2d_fromL1st_ybase_leftest.tag<<", "<<linepoint2d_fromL1st_ybase_rightest.tag<<", "<<linepoint2d_fromL1st_ybase_topest.tag<<", "<<linepoint2d_fromL1st_ybase_bottomest.tag<<std::endl;
        

        linepoint2d_t<T> linepoint2d_fromR1st_ybase_leftest(5, 5), linepoint2d_fromR1st_ybase_rightest(5, 5), linepoint2d_fromR1st_ybase_topest(5, 5), linepoint2d_fromR1st_ybase_bottomest(5, 5);
        findextreme(rightfirst_monochain_ybase_, linepoint2d_fromR1st_ybase_leftest, linepoint2d_fromR1st_ybase_rightest, linepoint2d_fromR1st_ybase_topest, linepoint2d_fromR1st_ybase_bottomest);
        std::cout<<"linepoint2d_fromR1st tag: "<<linepoint2d_fromR1st_ybase_leftest.tag<<", "<<linepoint2d_fromR1st_ybase_rightest.tag<<", "<<linepoint2d_fromR1st_ybase_topest.tag<<", "<<linepoint2d_fromR1st_ybase_bottomest.tag<<std::endl;
        
        if(linepoint2d_fromL1st_ybase_topest.point2dptr->y != linepoint2d_fromR1st_ybase_topest.point2dptr->y || linepoint2d_fromL1st_ybase_bottomest.point2dptr->y != linepoint2d_fromR1st_ybase_bottomest.point2dptr->y){
            printf("error, the first extreme points get error, same polygon topest and bottomest y is not same.\n");
            printf("linepoint2d_fromL1st_ybase_topest.point2dptr->y %lf != %lf linepoint2d_fromR1st_ybase_topest.point2dptr->y || linepoint2d_fromL1st_ybase_bottomest.point2dptr->y %lf != %lf linepoint2d_fromR1st_ybase_bottomest.point2dptr->y.\n", linepoint2d_fromL1st_ybase_topest.point2dptr->y, linepoint2d_fromR1st_ybase_topest.point2dptr->y,
                                        linepoint2d_fromL1st_ybase_bottomest.point2dptr->y, linepoint2d_fromR1st_ybase_bottomest.point2dptr->y);
            exit(-1);
        }

        // ----- second ybase
        linepoint2d_t<T> linepoint2d_fromL2nd_ybase_leftest(6, 6), linepoint2d_fromL2nd_ybase_rightest(6, 6), linepoint2d_fromL2nd_ybase_topest(6, 6), linepoint2d_fromL2nd_ybase_bottomest(6, 6);
        findextreme(leftsecond_monochain_ybase_, linepoint2d_fromL2nd_ybase_leftest, linepoint2d_fromL2nd_ybase_rightest, linepoint2d_fromL2nd_ybase_topest, linepoint2d_fromL2nd_ybase_bottomest);
        std::cout<<"linepoint2d_fromL2nd tag: "<<linepoint2d_fromL2nd_ybase_leftest.tag<<", "<<linepoint2d_fromL2nd_ybase_rightest.tag<<", "<<linepoint2d_fromL2nd_ybase_topest.tag<<", "<<linepoint2d_fromL2nd_ybase_bottomest.tag<<std::endl;
        
        linepoint2d_t<T> linepoint2d_fromR2nd_ybase_leftest(7, 7), linepoint2d_fromR2nd_ybase_rightest(7, 7), linepoint2d_fromR2nd_ybase_topest(7, 7), linepoint2d_fromR2nd_ybase_bottomest(7, 7);
        findextreme(rightsecond_monochain_ybase_, linepoint2d_fromR2nd_ybase_leftest, linepoint2d_fromR2nd_ybase_rightest, linepoint2d_fromR2nd_ybase_topest, linepoint2d_fromR2nd_ybase_bottomest);
        std::cout<<"linepoint2d_fromR2nd tag: "<<linepoint2d_fromR2nd_ybase_leftest.tag<<", "<<linepoint2d_fromR2nd_ybase_rightest.tag<<", "<<linepoint2d_fromR2nd_ybase_topest.tag<<", "<<linepoint2d_fromR2nd_ybase_bottomest.tag<<std::endl;
        
        if(linepoint2d_fromL2nd_ybase_topest.point2dptr->y != linepoint2d_fromR2nd_ybase_topest.point2dptr->y || linepoint2d_fromL2nd_ybase_bottomest.point2dptr->y != linepoint2d_fromR2nd_ybase_bottomest.point2dptr->y){
            printf("error, the second extreme points get error, same polygon topest and bottomest y is not same.\n");
            exit(-1);
        }
    
        
        sorted_extremelinepoint2d_xbase_hor_.clear();
        sorted_extremelinepoint2d_xbase_hor_.emplace_back(linepoint2d_fromL1st_xbase_leftest);
        sorted_extremelinepoint2d_xbase_hor_.emplace_back(linepoint2d_fromL1st_xbase_rightest);
        sorted_extremelinepoint2d_xbase_hor_.emplace_back(linepoint2d_fromL2nd_xbase_leftest);
        sorted_extremelinepoint2d_xbase_hor_.emplace_back(linepoint2d_fromL2nd_xbase_rightest);
        std::sort(sorted_extremelinepoint2d_xbase_hor_.begin(), sorted_extremelinepoint2d_xbase_hor_.end(), Compare_Xmin_minypre<T>);
        for(auto expoint: sorted_extremelinepoint2d_xbase_hor_){
            std::cout<<"expoint: "<<expoint.point2dptr->x<<" "<<expoint.point2dptr->y<<std::endl;
        }
        
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
    //         printf("error, monochain cannot < 2 or sorted size not equalwith 4.\n");
    //         exit(-1);
    //     }
    //     if(monochain0.monochainid == monochain1.monochainid){
    //         printf("error, cannot create by the same monochain.\n");
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
    bool PolygonIntersecting<T>::PotentionIntersection(const std::vector<linepoint2d_t<T>>& sorted_extremelinepoint2d_hor, const std::vector<linepoint2d_t<T>>& sorted_extremelinepoint2d_vel){ // sort 肯定是偶数的
        // monochain has
        if(sorted_extremelinepoint2d_hor.size() != 4 || sorted_extremelinepoint2d_vel.size() != 4){
            printf("please init valid sorted extremelinepoints, before use this method.\n");
            exit(-1);
        }
        // //  不一定有 isleft 
        // if(!sorted_extremelinepoint2d_hor[0].isleft || !sorted_extremelinepoint2d_vel[0].isleft){
        //     printf("the fist element must be tag = left.\n");
        //     exit(-1);
        // }
        bool possible_xbase = false;
        // 永远都是两条边，四个点
        if(sorted_extremelinepoint2d_hor[0].tag == sorted_extremelinepoint2d_hor[1].tag &&
                sorted_extremelinepoint2d_hor[0].monochainid == sorted_extremelinepoint2d_hor[1].monochainid){
            if(sorted_extremelinepoint2d_hor[2].tag != sorted_extremelinepoint2d_hor[3].tag ||
                sorted_extremelinepoint2d_hor[2].monochainid != sorted_extremelinepoint2d_hor[3].monochainid){
                printf("error, sorted extremelinepoints format is not ture.\n");
                exit(-1);
            }
            possible_xbase = false;
        }
        else if(sorted_extremelinepoint2d_hor[0].tag != sorted_extremelinepoint2d_hor[1].tag && 
                sorted_extremelinepoint2d_hor[0].monochainid != sorted_extremelinepoint2d_hor[1].monochainid){
            if(sorted_extremelinepoint2d_hor[0].tag != sorted_extremelinepoint2d_hor[2].tag && sorted_extremelinepoint2d_hor[0].tag != sorted_extremelinepoint2d_hor[3].tag){
                printf("error, sorted extremelinepoints format is not ture.\n");
                exit(-1);
            }
            if(sorted_extremelinepoint2d_hor[1].tag != sorted_extremelinepoint2d_hor[2].tag && sorted_extremelinepoint2d_hor[1].tag != sorted_extremelinepoint2d_hor[3].tag){
                printf("error, sorted extremelinepoints format is not ture.\n");
                exit(-1);
            }
            possible_xbase = true;
        }// else is all false.
        // second judgement
        bool possible_ybase = false;
        if(sorted_extremelinepoint2d_vel[0].tag == sorted_extremelinepoint2d_vel[1].tag &&
                sorted_extremelinepoint2d_vel[0].monochainid == sorted_extremelinepoint2d_vel[1].monochainid){
            if(sorted_extremelinepoint2d_vel[2].tag != sorted_extremelinepoint2d_vel[3].tag ||
                sorted_extremelinepoint2d_vel[2].monochainid != sorted_extremelinepoint2d_vel[3].monochainid){
                printf("error, sorted extremelinepoints format is not ture.\n");
                exit(-1);
            }
            possible_ybase = false;
        }
        else if(sorted_extremelinepoint2d_vel[0].tag != sorted_extremelinepoint2d_vel[1].tag && 
                sorted_extremelinepoint2d_vel[0].monochainid != sorted_extremelinepoint2d_vel[1].monochainid){
            if(sorted_extremelinepoint2d_vel[0].tag != sorted_extremelinepoint2d_vel[2].tag && sorted_extremelinepoint2d_vel[0].tag != sorted_extremelinepoint2d_vel[3].tag){
                printf("error, sorted extremelinepoints format is not ture.\n");
                exit(-1);
            }
            if(sorted_extremelinepoint2d_vel[1].tag != sorted_extremelinepoint2d_vel[2].tag && sorted_extremelinepoint2d_vel[1].tag != sorted_extremelinepoint2d_vel[3].tag){
                printf("error, sorted extremelinepoints format is not ture.\n");
                exit(-1);
            }
            possible_ybase = true;
        }// else is all false.
        return possible_xbase && possible_ybase; // 只有1 1 时才是可能有交点的.
    }
    
    
    template <class T>
    bool PolygonIntersecting<T>::PotentionIntersection(const std::vector<linepoint2d_t<T>>& sorted_extremelinepoint2d){
        if(sorted_extremelinepoint2d.size() != 4){
            printf("error, the sorted extremeline points is not equal with 4.\n");
            exit(-1);
        }
        if(sorted_extremelinepoint2d[0].monochainid != sorted_extremelinepoint2d[1].monochainid && sorted_extremelinepoint2d[0].tag != sorted_extremelinepoint2d[1].tag){
            return true;
        }
        else{
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

    template <class T>
    bool PolygonIntersecting<T>::HasIntersection(const bool& usexorybase){
        // 自己选择 // 0 x  12 y
        OrderLineEndPoints();
        potention_intersection_tags_.clear();
        bool possible = false;
        if(usexorybase){
            possible = HasIntersection_Ybase();
        }
        else{
            possible = HasIntersection_Xbase();
        }
        return possible;
    }

    template <class T>
    bool PolygonIntersecting<T>::HasIntersection_Xbase(){
        if(sorted_extremelinepoint2d_xbase_hor_.size()!=4 || (sorted_extremelinepoint2d_xbase_vel00_.size() != 4 && sorted_extremelinepoint2d_xbase_vel01_.size() != 4 
                    && sorted_extremelinepoint2d_xbase_vel10_.size() != 4 && sorted_extremelinepoint2d_xbase_vel11_.size() != 4)){
            printf("error, hasn't OrderLinePoints.\n");
            exit(-1);
        }
        for(auto pxbase_hor : sorted_extremelinepoint2d_xbase_hor_){
            std::cout<<"sorted: "<<pxbase_hor.point2dptr->x<<", "<<pxbase_hor.point2dptr->y<<", tag "<<pxbase_hor.tag<<", monochainid "<<pxbase_hor.monochainid<<std::endl;
        }
        if(PotentionIntersection(sorted_extremelinepoint2d_xbase_hor_)){
            // just need one possible that canbe has intersection.
            bool flag = false;
            if(PotentionIntersection(sorted_extremelinepoint2d_xbase_vel00_)){
                printf("check in xbase 000.\n");
                potention_intersection_tags_.emplace_back("000");
                flag = true;
            }
            if(PotentionIntersection(sorted_extremelinepoint2d_xbase_vel01_)){
                printf("check in xbase 001.\n");
                potention_intersection_tags_.emplace_back("001");
                flag = true;
            }
            if(PotentionIntersection(sorted_extremelinepoint2d_xbase_vel10_)){
                printf("check in xbase 010.\n");
                potention_intersection_tags_.emplace_back("010");
                flag = true;
            }
            if(PotentionIntersection(sorted_extremelinepoint2d_xbase_vel11_)){
                printf("check in xbase 011.\n");
                potention_intersection_tags_.emplace_back("011");
                flag = true;
            }
            return flag;
        }
        else{
            return false;
        }
    }


    template <class T>
    bool PolygonIntersecting<T>::HasIntersection_Ybase(){
        if((sorted_extremelinepoint2d_ybase_hor00_.size() != 4 && sorted_extremelinepoint2d_ybase_hor01_.size() != 4 && sorted_extremelinepoint2d_ybase_hor10_.size() != 4 && 
                    sorted_extremelinepoint2d_ybase_hor11_.size() != 4) || sorted_extermelinepoint2d_ybase_vel_.size() != 4){
            printf("error, hasn't OrderLinePoints.\n");
            exit(-1);
        }
        bool possible = false;
        if(PotentionIntersection(sorted_extermelinepoint2d_ybase_vel_)){
            bool flag = false;
            if(PotentionIntersection(sorted_extremelinepoint2d_ybase_hor00_)){
                potention_intersection_tags_.emplace_back("100");
                flag = true;
            }
            if(PotentionIntersection(sorted_extremelinepoint2d_ybase_hor01_)){
                potention_intersection_tags_.emplace_back("101");
                flag = true;
            }
            if(PotentionIntersection(sorted_extremelinepoint2d_ybase_hor10_)){
                potention_intersection_tags_.emplace_back("110");
                flag = true;
            }
            if(PotentionIntersection(sorted_extremelinepoint2d_ybase_hor11_)){
                potention_intersection_tags_.emplace_back("111");
                flag = true;
            }
        }       
    }

    template <class T>
    void PolygonIntersecting<T>::CalIntersectionBetweenTwoConvexPolygon(){
        if(potention_intersection_tags_.empty()){
            printf("potention intersection is empty. cannot cal the ConvexPolygon.\n" );
            return ;
        }
        
        for(auto tag : potention_intersection_tags_){
            printf("potention intersectin tag is %s.\n", tag.c_str());
            if (tag == "000")
            {
                /* code */
                printf("run at %s.\n", tag.c_str());
                CalIntersectionBetweenTwoMonochainLine(leftfirst_monochain_xbase_, leftsecond_monochain_xbase_, 0, 0);
            }
            if (tag == "001")
            {
                /* code */
                printf("run at %s.\n", tag.c_str());
                CalIntersectionBetweenTwoMonochainLine(leftfirst_monochain_xbase_, rightsecond_monochain_xbase_, 0, 0);
            }
            if (tag == "010")
            {
                /* code */
                printf("run at %s.\n", tag.c_str());
                CalIntersectionBetweenTwoMonochainLine(rightfirst_monochain_xbase_, leftsecond_monochain_xbase_, 0, 0);
            }
            if (tag == "011")
            {
                /* code */
                printf("run at %s.\n", tag.c_str());
                CalIntersectionBetweenTwoMonochainLine(rightfirst_monochain_xbase_, rightsecond_monochain_xbase_, 0, 0);
            }
            if (tag == "100")
            {
                /* code */
                printf("run at %s.\n", tag.c_str());
                CalIntersectionBetweenTwoMonochainLine(leftfirst_monochain_ybase_, leftsecond_monochain_ybase_, 0, 0);
            }
            if (tag == "101")
            {
                /* code */
                printf("run at %s.\n", tag.c_str());
                CalIntersectionBetweenTwoMonochainLine(leftfirst_monochain_ybase_, rightsecond_monochain_ybase_, 0, 0);
            }
            if (tag == "110")
            {
                /* code */
                printf("run at %s.\n", tag.c_str());
                CalIntersectionBetweenTwoMonochainLine(rightfirst_monochain_ybase_, leftsecond_monochain_ybase_, 0, 0);
            }
            if (tag == "111")
            {
                /* code */
                printf("run at %s.\n", tag.c_str());
                CalIntersectionBetweenTwoMonochainLine(rightfirst_monochain_ybase_, rightsecond_monochain_ybase_, 0, 0);
            }
        }
    }
    
    template <class T>
    void PolygonIntersecting<T>::CalIntersectionBetweenTwoMonochainLine(MonotoneChain<T>& chain0, MonotoneChain<T>& chain1, int start0, int start1){
        if(chain0.points_chain.size() < 3 && chain1.points_chain.size() < 3){// 都只剩下一条边的时候
            // 判断当前是否存在交点，并计算交点位置
            printf("run in the chain points size < 3.\n");
            extreme_edge_t<T> midedge0, midedge1;
            midedge0.p_start = chain0.points_chain[0];
            midedge0.p_end = chain0.points_chain[1];
            midedge1.p_start = chain1.points_chain[0];
            midedge1.p_end = chain1.points_chain[1];
            
            if(IsLineIntersecting(midedge0, midedge1)){
                // has ， find the point and save into vector
                point2d_t<T>* inter = new point2d_t<T>(0 ,0); 
                bool issucc = CalRealIntersectionPosition(midedge0, midedge1, inter);
                if(issucc){
                    extremeedge_index_t indexes; 
                    indexes.first_index = start0;
                    indexes.second_index = start1;
                    intersections_.emplace_back(std::make_pair(indexes, inter));
                }
                else{
                    delete inter;
                }
            }
        }
        else{
            // 几种状态，需要分别判断，分支递归或者直接判断结果
            // no intersection
            printf("run in the chain points intersection.\n");
            MonotoneChain<T> leftchain0, rightchain0, leftchain1, rightchain1;
            extreme_edge_t<T> midedge0, midedge1;
            chain0.GetMidEdge(midedge0, leftchain0, rightchain0);
            chain1.GetMidEdge(midedge1, leftchain1, rightchain1);
            if(IsLineIntersecting(midedge0, midedge1)){
                printf("line intersecting...\n");
                point2d_t<T>* inter = new point2d_t<T>(0 ,0); 
                bool issucc = CalRealIntersectionPosition(midedge0, midedge1, inter);
                if(issucc){
                    extremeedge_index_t indexes; // 在这个位置的点和后面一个点之间的边会存在一个交点
                    indexes.first_index = start0+(chain0.points_chain.size()-1)/2;
                    indexes.second_index = start1+(chain1.points_chain.size()-1)/2;
                    intersections_.emplace_back(std::make_pair(indexes, inter));
                }
                else{
                    delete inter;
                }
            }
            std::vector<linepoint2d_t<T>> leftchainlinepoints0, rightchainlinepoints0, leftchainlinepoints1, rightchainlinepoints1;
            for(int i=0; i<leftchain0.points_chain.size(); ++i){
                leftchainlinepoints0.emplace_back(linepoint2d_t<T>(leftchain0.points_chain[i], 0, true, false));
            }
            
            for(int i=0; i<rightchain0.points_chain.size(); ++i){
                rightchainlinepoints0.emplace_back(linepoint2d_t<T>(rightchain0.points_chain[i], 1, false, false));
            }
            
            for(int i=0; i<leftchain1.points_chain.size(); ++i){
                leftchainlinepoints1.emplace_back(linepoint2d_t<T>(leftchain1.points_chain[i], 2, true, false));
            }
            
            for(int i=0; i<rightchain1.points_chain.size(); ++i){
                rightchainlinepoints1.emplace_back(linepoint2d_t<T>(rightchain1.points_chain[i], 3, false, false));
            }
            printf("run rePotentionIntersections intersections. leftchainlinepoints0,1 is (%d, %d).\n", (int)leftchainlinepoints0.size(), (int)leftchainlinepoints1.size());
            
            // 这里的问题是： 这里是两个chain之间可能性的比较，要自己创建sorted points
            if (PotentionIntersection(leftchainlinepoints0, leftchainlinepoints1))
                CalIntersectionBetweenTwoMonochainLine(leftchain0, leftchain1, 0, 0);
            if (PotentionIntersection(leftchainlinepoints0, rightchainlinepoints1))
                CalIntersectionBetweenTwoMonochainLine(leftchain0, rightchain1, 0, 0);
            if (PotentionIntersection(rightchainlinepoints0, leftchainlinepoints1))
                CalIntersectionBetweenTwoMonochainLine(rightchain0, leftchain1, 0, 0);
            if (PotentionIntersection(rightchainlinepoints1, rightchainlinepoints1))
                CalIntersectionBetweenTwoMonochainLine(rightchain0, rightchain1, 0, 0);
            return;
        }

    }
    template <class T>
    bool CalRealIntersectionPosition(const extreme_edge_t<T>& edge0, const extreme_edge_t<T>& edge1, point2d_t<T>* inter){
        const point2d_t<T>* pa = edge0.p_start;
        const point2d_t<T>* pb = edge0.p_end;
        const point2d_t<T>* pc = edge1.p_start;
        const point2d_t<T>* pd = edge1.p_end;

        Eigen::Matrix2d A = Eigen::Matrix2d::Zero();
        A(0, 0) = pb->x - pa->x; A(0, 1) = pc->x - pd->x;
        A(1, 0) = pb->y - pa->y; A(1, 1) = pc->y - pd->y;
        if(A.determinant() <= 0){
            // 就没有可用的结果。
            return false;
        }

        Eigen::Vector2d b = Eigen::Vector2d::Zero();
        b[0] = pc->x + pd->x - pa->x - pb->x;
        b[1] = pc->y + pd->y - pa->y - pb->y;
        // 因为规模小所以没有所谓的速度差问题
        Eigen::Vector2d x = A.fullPivLu().solve(b);
        // 判断是否有解先
        double l = (1+x[0])/2.;// x[0] = 2l-1
        double k = (1+x[1])/2.;// x[1] = 2k-1
        if(l >= 0 && l <= 1 && k>=0 && k<=1){
            inter->x = (pa->x+(pc->x-pa->x + pd->x-pa->x)+x[1]*(pd->x-pc->x))/2;
            inter->y = (pa->y+(pc->y-pa->y + pd->y-pa->y)+x[1]*(pd->y-pc->y))/2;
            return true;
        }
        return false;
    }
}